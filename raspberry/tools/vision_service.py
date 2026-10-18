#!/usr/bin/env python3
"""
vision_service.py - on-demand object recognition, stereo distance and text reading (OCR) for the Quest XR app.

Requests come over ws-teleop (wss://127.0.0.1:8557, relayed from the app):
    {"type": "vision_request", "action": "analyze" | "point", "req": <id>}
  analyze: detection + stereo distance + OCR of the target            (~1.5-2 s on the Pi 5 CPU)
  point:   detection + stereo distance only, result with "aim": true  (~0.6 s); ws-teleop plans the arm motion
           towards the target (head_assist_gimbal.request_point) and asks for an "analyze" when it gets there
           (the text is read from close up).
Reply: {"type": "vision_result", "req", "action", "ok", "why"?, "aim"?, "zoom", "objects": [...],
        "target": {"label", "label_it", "conf", "box": [x0, y0, x1, y1] | null, "center": [u, v], "dist_mm",
                   "cam_xyz_mm": [X right, Y down, Z forward] (cam0 frame, zoom-1 geometry),
                   "dx", "dy" (cam1 - cam0 at the target, STREAM px: what the viewer uses for the right eye)},
        "texts": [{"text", "score", "box"}], "ms": {...}}
All image coordinates are cam0 STREAM pixels (800 x 450 at the current sensor zoom) of the analysed keyframe.

Target = the detected object under the crosshair (image centre of cam0): the smallest box that contains it, else
the nearest box centre within NEAR_PX; without a detection the point under the crosshair itself ("oggetto").
Distance: the collimator's template matching (stereo_collimator.py) at the target pixel, several points of the
box, median; true disparity in zoom-1 px = dx_stream / zoom + cam1 crop offset, Z = f * B / (dx_inf - d).

Models (raspberry/models): YOLO11n COCO (ONNX, exported with ultralytics) and RapidOCR PP-OCRv3 (bundled in the
rapidocr_onnxruntime wheel). Runs in raspberry/.venv-vision (system site packages: PyAV, numpy), Nice 10, so
the camera encoders keep priority.
"""
from __future__ import annotations

import argparse
import json
import math
import ssl
import sys
import threading
import time
from pathlib import Path

import av
import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stereo_collimator as sc  # noqa: E402  (matching helpers, calibration, constants, websockets import)

RASPBERRY = Path(__file__).resolve().parents[1]
YOLO_PATH = RASPBERRY / "models" / "yolo11n.onnx"
W, H = sc.W, sc.H
CX, CY = W / 2.0, H / 2.0
CONF_MIN = 0.30
NMS_IOU = 0.45
NEAR_PX = 110.0
OCR_MIN_SCORE = 0.55
FRAME_WAIT_S = 0.8          # wait this long for a keyframe pair newer than the request

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]
COCO_IT = ["persona", "bicicletta", "auto", "moto", "aereo", "autobus", "treno", "camion", "barca", "semaforo",
           "idrante", "stop", "parchimetro", "panchina", "uccello", "gatto", "cane", "cavallo", "pecora", "mucca",
           "elefante", "orso", "zebra", "giraffa", "zaino", "ombrello", "borsa", "cravatta", "valigia", "frisbee",
           "sci", "snowboard", "palla", "aquilone", "mazza", "guantone", "skateboard", "surf",
           "racchetta", "bottiglia", "calice", "tazza", "forchetta", "coltello", "cucchiaio", "ciotola", "banana",
           "mela", "panino", "arancia", "broccolo", "carota", "hot dog", "pizza", "ciambella", "torta", "sedia",
           "divano", "pianta", "letto", "tavolo", "WC", "TV", "portatile", "mouse", "telecomando", "tastiera",
           "telefono", "microonde", "forno", "tostapane", "lavandino", "frigorifero", "libro", "orologio",
           "vaso", "forbici", "peluche", "phon", "spazzolino"]


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ---------------------------------------------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------------------------------------------

class Reader(threading.Thread):
    """Latest keyframe of one RTSP path: grey half resolution (matching) and, for cam0, the BGR full frame."""

    def __init__(self, url: str, color: bool):
        super().__init__(daemon=True)
        self.url, self.color = url, color
        self.lock = threading.Lock()
        self.half = None
        self.bgr = None
        self.t = 0.0

    def run(self):
        while True:
            try:
                c = av.open(self.url, options={"rtsp_transport": "udp", "fflags": "nobuffer", "flags": "low_delay"},
                            timeout=5)
                s = c.streams.video[0]
                s.codec_context.skip_frame = "NONKEY"
                log("reading", self.url)
                for f in c.decode(s):
                    bgr = f.to_ndarray(format="bgr24")[:H, :W] if self.color else None
                    y = (cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr is not None
                         else f.to_ndarray(format="gray")[:H, :W]).astype(np.float32)
                    half = 0.25 * (y[0::2, 0::2] + y[1::2, 0::2] + y[0::2, 1::2] + y[1::2, 1::2])
                    with self.lock:
                        self.half, self.bgr, self.t = half, bgr, time.monotonic()
                c.close()
            except Exception as e:
                log("reader", self.url, "error:", e)
            time.sleep(1.0)

    def latest(self):
        with self.lock:
            return self.half, self.bgr, self.t


# ---------------------------------------------------------------------------------------------------------------
# detection (YOLO11n, COCO)
# ---------------------------------------------------------------------------------------------------------------

class Detector:
    def __init__(self, path: Path, threads: int):
        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(str(path), sess_options=so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.size = int(self.sess.get_inputs()[0].shape[2])

    def __call__(self, bgr: np.ndarray) -> list[dict]:
        h, w = bgr.shape[:2]
        n = self.size
        s = n / max(w, h)
        nw, nh = int(round(w * s)), int(round(h * s))
        top = (n - nh) // 2
        pad = np.full((n, n, 3), 114, np.uint8)
        pad[top:top + nh, :nw] = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = pad[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.inp: x})[0][0]          # (84, 8400): cx, cy, w, h, 80 class scores
        scores = out[4:].T
        cls = scores.argmax(1)
        conf = scores[np.arange(scores.shape[0]), cls]
        keep = conf >= CONF_MIN
        if not np.any(keep):
            return []
        b, cls, conf = out[:4].T[keep], cls[keep], conf[keep]
        x0 = (b[:, 0] - b[:, 2] / 2) / s
        y0 = (b[:, 1] - top - b[:, 3] / 2) / s
        bw, bh = b[:, 2] / s, b[:, 3] / s
        res = []
        for c in np.unique(cls):
            idx = np.nonzero(cls == c)[0]
            boxes = [[float(x0[i]), float(y0[i]), float(bw[i]), float(bh[i])] for i in idx]
            sel = cv2.dnn.NMSBoxes(boxes, [float(conf[i]) for i in idx], CONF_MIN, NMS_IOU)
            for k in np.array(sel).reshape(-1):
                i = idx[int(k)]
                bx = [max(0.0, float(x0[i])), max(0.0, float(y0[i])),
                      min(float(w), float(x0[i] + bw[i])), min(float(h), float(y0[i] + bh[i]))]
                res.append({"label": COCO[int(c)], "label_it": COCO_IT[int(c)], "conf": round(float(conf[i]), 3),
                            "box": [round(v, 1) for v in bx]})
        res.sort(key=lambda d: -d["conf"])
        return res


def pick_target(objs: list[dict]):
    """The object under the crosshair: smallest box containing it, else the nearest box centre within NEAR_PX."""
    inside = [o for o in objs if o["box"][0] - 6 <= CX <= o["box"][2] + 6 and o["box"][1] - 6 <= CY <= o["box"][3] + 6]
    if inside:
        return min(inside, key=lambda o: (o["box"][2] - o["box"][0]) * (o["box"][3] - o["box"][1]))
    best, bd = None, NEAR_PX
    for o in objs:
        d = math.hypot((o["box"][0] + o["box"][2]) / 2 - CX, (o["box"][1] + o["box"][3]) / 2 - CY)
        if d < bd:
            best, bd = o, d
    return best


# ---------------------------------------------------------------------------------------------------------------
# stereo at an arbitrary pixel (stereo_collimator.match_size, template centred on (px, py) instead of the centre)
# ---------------------------------------------------------------------------------------------------------------

def match_at(left: np.ndarray, right: np.ndarray, px: float, py: float, zoom: float):
    h, w = left.shape
    cx, cy = int(round(px / 2)), int(round(py / 2))
    for tpl in sc.TPLS:
        hf = tpl // 2
        if cx - hf < 0 or cy - hf < 0 or cx + hf > w or cy + hf > h:
            continue
        t = left[cy - hf: cy + hf, cx - hf: cx + hf]
        if float(t.std()) < sc.MIN_TEXTURE:
            continue
        dyr = int(round(sc.DY_MAX * max(1.0, zoom)))
        y0, y1 = max(0, cy - hf - dyr), min(h, cy + hf + dyr)
        x0, x1 = max(0, cx - hf + int(sc.DX_MIN * zoom)), min(w, cx + hf + int(sc.DX_MAX * zoom))
        if x1 - x0 <= tpl or y1 - y0 <= tpl:
            continue
        m = sc.ncc(right[y0:y1, x0:x1], t)
        iy, ix = np.unravel_index(int(np.argmax(m)), m.shape)
        best = float(m[iy, ix])
        masked = m.copy()
        masked[max(0, iy - 3): iy + 4, max(0, ix - 3): ix + 4] = -1
        second = float(masked.max())
        on_edge = iy == 0 or iy == m.shape[0] - 1 or ix == 0 or ix == m.shape[1] - 1
        if best < sc.MIN_SCORE or best - second < sc.MIN_MARGIN or on_edge:
            continue
        fx = sc.subpixel(m[iy, ix - 1], m[iy, ix], m[iy, ix + 1]) if 0 < ix < m.shape[1] - 1 else 0.0
        fy = sc.subpixel(m[iy - 1, ix], m[iy, ix], m[iy + 1, ix]) if 0 < iy < m.shape[0] - 1 else 0.0
        dx = (x0 + ix + fx + hf) - cx
        dy = (y0 + iy + fy + hf) - cy
        return 2.0 * dx, 2.0 * dy, best
    return None


def read_zoom_state():
    """(zoom, cam1 offset [ox, oy] zoom-1 px) from cam_stream.py's state files, (1, [0, 0]) when unknown."""
    z, off = 1.0, [0.0, 0.0]
    try:
        st = json.loads(Path(sc.CAM_ZOOM_PATH).read_text())
        if time.time() - float(st.get("ts", 0)) < 3.0:
            z = float(st.get("zoom", 1.0))
    except Exception:
        pass
    try:
        st = json.loads(Path(sc.CAM1_STATE_PATH).read_text())
        if time.time() - float(st.get("ts", 0)) < 3.0:
            off = [float(v) for v in st.get("offset", [0.0, 0.0])[:2]]
    except Exception:
        pass
    return max(1.0, z), off


# ---------------------------------------------------------------------------------------------------------------
# service
# ---------------------------------------------------------------------------------------------------------------

class Service:
    def __init__(self, args):
        self.args = args
        self.readers = [Reader(f"{args.rtsp}/cam0", True), Reader(f"{args.rtsp}/cam1", False)]
        for r in self.readers:
            r.start()
        self.det = Detector(YOLO_PATH, args.threads)
        from rapidocr_onnxruntime import RapidOCR
        self.ocr = RapidOCR()
        self.calib = sc.load_calib()
        self.last_col = {"dist_mm": None, "t": 0.0}
        self.queue: list[dict] = []
        self.cv = threading.Condition()
        log(f"YOLO {YOLO_PATH.name} {self.det.size}px, f {sc.F_PX:.1f} px, dx_inf {self.calib['dx_inf']:.1f} px")

    # -- frames
    def frames_after(self, t_req: float):
        """Newest cam0/cam1 keyframes, waiting up to FRAME_WAIT_S for a pair taken after the request."""
        deadline = time.monotonic() + FRAME_WAIT_S
        while True:
            l, bgr, tl = self.readers[0].latest()
            r, _, tr = self.readers[1].latest()
            if l is not None and r is not None and tl >= t_req and tr >= t_req - 0.05:
                return l, bgr, r, tl
            if time.monotonic() > deadline:
                return (l, bgr, r, tl) if l is not None and r is not None else None
            time.sleep(0.02)

    # -- one request
    def process(self, req: dict) -> dict:
        action = req.get("action", "analyze")
        out = {"type": "vision_result", "req": req.get("req"), "action": action, "ok": False}
        t0 = time.monotonic()
        fr = self.frames_after(req.get("_t", t0))
        if fr is None:
            out["why"] = "nessun video"
            return out
        left, bgr, right, tf = fr
        zoom, off = read_zoom_state()
        out["zoom"] = round(zoom, 3)
        t1 = time.monotonic()
        objs = self.det(bgr)
        t2 = time.monotonic()
        tgt = pick_target(objs)
        out["objects"] = [dict(o, target=(o is tgt)) for o in objs[:8]]
        if tgt is not None:
            bx = tgt["box"]
            u, v = (bx[0] + bx[2]) / 2.0, (bx[1] + bx[3]) / 2.0
            target = {"label": tgt["label"], "label_it": tgt["label_it"], "conf": tgt["conf"], "box": bx}
            # several points of the box (centre first): plain surfaces have no texture at the centre
            qw, qh = (bx[2] - bx[0]) * 0.22, (bx[3] - bx[1]) * 0.22
            pts = [(u, v), (u - qw, v), (u + qw, v), (u, v - qh), (u, v + qh)]
        else:
            u, v = CX, CY
            target = {"label": None, "label_it": "oggetto", "conf": None, "box": None}
            pts = [(CX, CY), (CX - 30, CY), (CX + 30, CY), (CX, CY - 30), (CX, CY + 30)]
        target["center"] = [round(u, 1), round(v, 1)]
        meas = []
        for px, py in pts:
            m = match_at(left, right, px, py, zoom)
            if m is not None:
                meas.append(m)
        t3 = time.monotonic()
        dist = None
        if meas:
            dxs = sorted(m[0] for m in meas)
            dys = sorted(m[1] for m in meas)
            dx_s, dy_s = dxs[len(dxs) // 2], dys[len(dys) // 2]
            d_true = dx_s / zoom + off[0]                      # zoom-1 px
            d = self.calib["dx_inf"] - d_true
            if d > 0.5:
                dist = sc.F_PX * sc.BASELINE_MM / d
            target["dx"], target["dy"] = round(dx_s, 1), round(dy_s, 1)
        if dist is None and tgt is None and self.last_col["dist_mm"] and time.monotonic() - self.last_col["t"] < 1.5:
            dist = self.last_col["dist_mm"]                    # crosshair point: the collimator's lock
        if dist is not None:
            target["dist_mm"] = round(dist, 0)
            # optical axis of cam0 through the stream centre; zoom-1 px = (stream - centre) / zoom
            x = (u - CX) / zoom * dist / sc.F_PX
            y = (v - CY) / zoom * dist / sc.F_PX
            target["cam_xyz_mm"] = [round(x, 1), round(y, 1), round(dist, 1)]
        out["target"] = target
        out["ok"] = dist is not None or tgt is not None
        if dist is None:
            out["why"] = "distanza non misurabile (superficie senza dettagli?)"
        if action == "point":
            out["aim"] = dist is not None
        else:
            out["texts"] = self.read_text(bgr, target["box"], u, v)
        t4 = time.monotonic()
        out["ms"] = {"wait": round((t1 - t0) * 1000), "detect": round((t2 - t1) * 1000),
                     "stereo": round((t3 - t2) * 1000), "ocr": round((t4 - t3) * 1000)}
        out["frame_age_ms"] = round((t4 - tf) * 1000)
        return out

    def read_text(self, bgr, box, u, v) -> list[dict]:
        if box is not None:
            x0, y0, x1, y1 = box
            mx, my = (x1 - x0) * 0.12 + 8, (y1 - y0) * 0.12 + 8
            x0, y0, x1, y1 = max(0, x0 - mx), max(0, y0 - my), min(W, x1 + mx), min(H, y1 + my)
        else:
            x0, y0, x1, y1 = max(0, u - 220), max(0, v - 120), min(W, u + 220), min(H, v + 120)
        x0, y0, x1, y1 = (int(round(t)) for t in (x0, y0, x1, y1))
        crop = bgr[y0:y1, x0:x1]
        if crop.size == 0:
            return []
        k = 2.0 if crop.shape[1] < 320 else 1.0                # small crops: upscale, OCR likes ~20+ px letters
        if k != 1.0:
            crop = cv2.resize(crop, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        try:
            res, _ = self.ocr(crop)
        except Exception as e:
            log("ocr error:", e)
            return []
        texts = []
        for pts, txt, score in res or []:
            score = float(score)
            txt = str(txt).strip()
            if score < OCR_MIN_SCORE or len(txt) < 2:
                continue
            p = np.asarray(pts, dtype=float) / k
            texts.append({"text": txt, "score": round(score, 2),
                          "box": [round(float(p[:, 0].min()) + x0, 1), round(float(p[:, 1].min()) + y0, 1),
                                  round(float(p[:, 0].max()) + x0, 1), round(float(p[:, 1].max()) + y0, 1)]})
        texts.sort(key=lambda t: (round(t["box"][1] / 20), t["box"][0]))
        return texts

    # -- ws
    def run(self):
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        threading.Thread(target=self.worker, daemon=True).start()
        while True:
            try:
                ws = sc.connect(self.args.ws, ssl=ctx, open_timeout=5, max_size=2 ** 22)
                self.ws = ws
                log("connected to ws-teleop")
                for raw in ws:
                    try:
                        m = json.loads(raw)
                    except Exception:
                        continue
                    if not isinstance(m, dict):
                        continue
                    if m.get("type") == "stereo_collimation" and m.get("lock") and m.get("dist_mm"):
                        self.last_col = {"dist_mm": float(m["dist_mm"]), "t": time.monotonic()}
                    elif m.get("type") == "vision_request":
                        m["_t"] = time.monotonic()
                        with self.cv:
                            self.queue = [m]                         # only the newest request matters
                            self.cv.notify()
            except Exception as e:
                log("ws-teleop:", e)
            self.ws = None
            time.sleep(2)

    def worker(self):
        while True:
            with self.cv:
                while not self.queue:
                    self.cv.wait()
                req = self.queue.pop(0)
            try:
                res = self.process(req)
            except Exception as e:
                log("process error:", repr(e))
                res = {"type": "vision_result", "req": req.get("req"), "action": req.get("action"), "ok": False,
                       "why": f"errore: {e}"}
            t = res.get("target") or {}
            log(f"{res.get('action')} req {res.get('req')}: {t.get('label_it')} conf {t.get('conf')} "
                f"dist {t.get('dist_mm')} texts {[x['text'] for x in res.get('texts', [])]} ms {res.get('ms')}")
            ws = getattr(self, "ws", None)
            if ws is not None:
                try:
                    ws.send(json.dumps(res))
                except Exception as e:
                    log("send:", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", default="wss://127.0.0.1:8557")
    ap.add_argument("--rtsp", default="rtsp://127.0.0.1:8554")
    ap.add_argument("--threads", type=int, default=2, help="onnxruntime threads for YOLO (the cameras keep priority)")
    ap.add_argument("--test", action="store_true", help="analyse once, print the result and exit")
    args = ap.parse_args()
    svc = Service(args)
    if args.test:
        time.sleep(3)
        print(json.dumps(svc.process({"action": "analyze", "req": "test", "_t": time.monotonic()}), indent=1))
        return
    svc.run()


if __name__ == "__main__":
    main()
