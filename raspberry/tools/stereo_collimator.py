#!/usr/bin/env python3
"""
stereo_collimator.py - automatic crosshair collimation for the VR viewers.

What it does
  The left-eye crosshair sits at the centre of cam0. This service finds, ~4.5 times a second, where that same
  point of the scene appears in cam1 (normalised cross-correlation of the central patch) and broadcasts the
  offset through ws-teleop:

    {"type": "stereo_collimation", "dx": px, "dy": px, "score": 0..1, "lock": bool,
     "dist_mm": float|null, "dx_inf": px, "ts": unix}

  dx / dy are in cam1 video pixels (800x450), relative to the image centre. A viewer that draws the right-eye
  crosshair at (centre + dx, centre + dy) sees the two crosshairs fuse ON the object, at its real depth, at any
  digital zoom (the viewer multiplies the offset by its zoom).

Cost
  Only H.264 keyframes are decoded (IDR every 20 frames -> ~4.5 fps per camera at 90 fps capture); the two
  cameras are frame-synchronised and their keyframes arrive within one frame (~11 ms) of each other. About
  10 % of one core on the Pi 5.

Distance
  Pinhole stereo: Z = f * B / d with d = dx_inf - dx (pixels). f comes from the real horizontal FOV of the
  90 fps sensor mode (ScalerCrop 3072x1728 of the 4608x2592 IMX708 -> 46.8 deg on 800 px -> f ~ 924 px),
  B = baseline between the camera centres (63 mm). dx_inf is the offset of a point at infinity (0 for perfectly
  parallel cameras); calibrate it by putting a textured target at a known distance under the crosshair and
  sending {"type": "stereo_calibrate", "distance_mm": 1000} over ws-teleop (or --calibrate-mm on the command line).
  Stored in config_runtime/video/stereo_calib.json.

Runs with the system python (PyAV + numpy from Debian); websockets is taken from the project venv.
Service: raspberry/systemd/jonny5-stereo-collimator.service.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import ssl
import sys
import threading
import time
from pathlib import Path

import av
import numpy as np

RASPBERRY = Path(__file__).resolve().parents[1]
# websockets lives in the project venv (same Python 3.13 ABI); append so the system numpy/av keep priority
for sp in glob.glob(str(RASPBERRY / ".venv" / "lib" / "python3*" / "site-packages")):
    if sp not in sys.path:
        sys.path.append(sp)
from websockets.sync.client import connect  # noqa: E402

CALIB_PATH = RASPBERRY / "config_runtime" / "video" / "stereo_calib.json"

W, H = 800, 450
HFOV_DEG = 46.8                      # 66 deg lens on the 3072x1728 sensor crop of the 90/120 fps mode
F_PX = (W / 2) / math.tan(math.radians(HFOV_DEG / 2))
BASELINE_MM = 63.0

# matching, at half resolution (400x225)
TPLS = (32, 48, 64, 96)              # template sides, half res (64..192 px full res): the smallest one with enough
                                     # texture is used (a flat surface under the crosshair -> look a bit wider)
DX_MIN, DX_MAX = -210, 40            # search range in half-res pixels: near objects shift LEFT in cam1
DY_MAX = 12
MIN_TEXTURE = 4.0                    # template std (grey levels) below which there is nothing to match
MIN_SCORE = 0.65
MIN_MARGIN = 0.02                    # best peak must beat the best peak elsewhere by this much (repetitive textures)
# Continuity: when two places are almost equally good (drawers, shelves...), take the one next to the last lock
# if it is within CONT_PX (full-res) of it and its score is within CONT_BAND of the best.
CONT_PX = 16.0
CONT_BAND = 0.06
CONT_MAX_AGE_S = 3.0                 # was 1 s: one missed keyframe (~0.2 s) dropped the continuity, then it stayed
                                     # unlocked on repetitive / smooth targets
# Vertical offset between the cameras (mechanical, independent of the distance): learned from the locks and sent
# while searching, so the right crosshair / convergence stays aligned vertically (the stored calibration said
# -0.3 px, the locks measure about -10 px after the cameras moved).
DY_LEARN_N = 25
DY_LEARN_MIN = 5
DY_SAVE_DELTA = 1.0
DY_SAVE_EVERY_S = 60.0
# Sensor zoom (tools/cam_stream.py, centred ScalerCrop): the stream pixels are zoom x the zoom-1 pixels. All the
# geometry (history, dy learning, dx_inf, distance, calibration) is kept in ZOOM-1 pixels; the messages are in
# stream pixels (what the viewer draws). While the zoom moves the matching is skipped and the last lock is
# rescaled (a centred crop scales the offsets from the centre by the zoom ratio).
CAM_ZOOM_PATH = "/dev/shm/jonny5_cam0_zoom.json"
ZOOM_SETTLE_S = 0.35


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ---------------------------------------------------------------------------------------------------------------
# keyframe readers
# ---------------------------------------------------------------------------------------------------------------

class KeyframeReader(threading.Thread):
    """Keeps the latest decoded keyframe (grey, half resolution) of one RTSP path, reconnecting forever."""

    def __init__(self, url: str):
        super().__init__(daemon=True)
        self.url = url
        self.lock = threading.Lock()
        self.frame = None
        self.t = 0.0
        self.seq = 0

    def run(self):
        while True:
            try:
                c = av.open(self.url, options={"rtsp_transport": "udp", "fflags": "nobuffer",
                                               "flags": "low_delay"}, timeout=5)
                s = c.streams.video[0]
                s.codec_context.skip_frame = "NONKEY"
                log("reading", self.url)
                for f in c.decode(s):
                    y = f.to_ndarray(format="gray")[:H, :W].astype(np.float32)
                    half = 0.25 * (y[0::2, 0::2] + y[1::2, 0::2] + y[0::2, 1::2] + y[1::2, 1::2])
                    with self.lock:
                        self.frame = half
                        self.t = time.monotonic()
                        self.seq += 1
                c.close()
            except Exception as e:  # MediaMTX restart (refocus), network...
                log("reader", self.url, "error:", e)
            time.sleep(1.0)

    def latest(self):
        with self.lock:
            return self.frame, self.t, self.seq


# ---------------------------------------------------------------------------------------------------------------
# matching
# ---------------------------------------------------------------------------------------------------------------

def box_sum(a: np.ndarray, kh: int, kw: int) -> np.ndarray:
    """Sum over every kh x kw window (valid positions), via an integral image."""
    ii = np.pad(a, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    return ii[kh:, kw:] - ii[:-kh, kw:] - ii[kh:, :-kw] + ii[:-kh, :-kw]


def ncc(search: np.ndarray, tpl: np.ndarray) -> np.ndarray:
    """Normalised cross-correlation of tpl over every valid position of search (TM_CCOEFF_NORMED)."""
    th, tw = tpl.shape
    t = tpl - tpl.mean()
    tn = math.sqrt(float((t * t).sum())) or 1.0
    sh, sw = search.shape
    fh, fw = sh + th, sw + tw
    corr = np.fft.irfft2(np.fft.rfft2(search, (fh, fw)) * np.conj(np.fft.rfft2(t, (fh, fw))), (fh, fw))
    corr = corr[: sh - th + 1, : sw - tw + 1]
    n = th * tw
    s1 = box_sum(search, th, tw)
    s2 = box_sum(search * search, th, tw)
    var = np.maximum(s2 - s1 * s1 / n, 1e-6)
    return corr / (np.sqrt(var) * tn)


def subpixel(v_m, v_0, v_p) -> float:
    den = v_m - 2 * v_0 + v_p
    return 0.0 if abs(den) < 1e-9 else 0.5 * (v_m - v_p) / den


def match(left: np.ndarray, right: np.ndarray, prev_dx=None, zoom: float = 1.0):
    """Returns (dx, dy, score, ok, why, window_px_full) in FULL-resolution pixels, cam1 relative to cam0 centre.
    Tries the template sides in TPLS from the smallest; the first one with enough texture whose match passes the
    score/uniqueness checks wins."""
    last = (None, None, 0.0, False, "flat", 0)
    for tpl in TPLS:
        r = match_size(left, right, tpl, prev_dx, zoom)
        if r[4] == "flat":
            continue
        if r[3]:
            return r
        last = r
    return last


def match_size(left: np.ndarray, right: np.ndarray, TPL: int, prev_dx=None, zoom: float = 1.0):
    h, w = left.shape
    cy, cx = h // 2, w // 2
    t = left[cy - TPL // 2: cy + TPL // 2, cx - TPL // 2: cx + TPL // 2]
    if float(t.std()) < MIN_TEXTURE:
        return None, None, 0.0, False, "flat", 2 * TPL
    y0, y1 = max(0, cy - TPL // 2 - DY_MAX), min(h, cy + TPL // 2 + DY_MAX)
    # the disparity range grows with the sensor zoom; the image border limits it anyway
    x0, x1 = max(0, cx - TPL // 2 + int(DX_MIN * zoom)), min(w, cx + TPL // 2 + int(DX_MAX * zoom))
    sr = right[y0:y1, x0:x1]
    m = ncc(sr, t)
    iy, ix = np.unravel_index(int(np.argmax(m)), m.shape)
    best = float(m[iy, ix])
    masked = m.copy()
    masked[max(0, iy - 3): iy + 4, max(0, ix - 3): ix + 4] = -1
    second = float(masked.max())
    why_ok = ""
    if best >= MIN_SCORE and best - second < MIN_MARGIN and prev_dx is not None:  # prev_dx: stream px
        # ambiguous: among the near-best places pick the one closest to the previous lock
        cy_, cx_ = np.nonzero(m >= best - CONT_BAND)
        cand_dx = 2.0 * ((x0 + cx_ + TPL / 2) - cx)
        k = int(np.argmin(np.abs(cand_dx - prev_dx)))
        if abs(cand_dx[k] - prev_dx) <= CONT_PX * zoom:
            iy, ix = int(cy_[k]), int(cx_[k])
            best = float(m[iy, ix])
            second = best - MIN_MARGIN  # accepted by continuity
            why_ok = "continuity"
    fx = subpixel(m[iy, ix - 1], m[iy, ix], m[iy, ix + 1]) if 0 < ix < m.shape[1] - 1 else 0.0
    fy = subpixel(m[iy - 1, ix], m[iy, ix], m[iy + 1, ix]) if 0 < iy < m.shape[0] - 1 else 0.0
    # template top-left in right = (x0 + ix, y0 + iy); its centre relative to the image centre
    dx = (x0 + ix + fx + TPL / 2) - cx
    dy = (y0 + iy + fy + TPL / 2) - cy
    # a peak on the border of the search window is a truncated / wrong match (the real one is outside)
    on_edge = iy == 0 or iy == m.shape[0] - 1 or ix == 0 or ix == m.shape[1] - 1
    ok = best >= MIN_SCORE and best - second >= MIN_MARGIN and not on_edge
    why = why_ok if ok else ("edge" if on_edge else ("low score" if best < MIN_SCORE else "ambiguous"))
    return 2.0 * dx, 2.0 * dy, best, ok, why, 2 * TPL


# ---------------------------------------------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------------------------------------------

def load_calib() -> dict:
    try:
        c = json.loads(CALIB_PATH.read_text())
        return {"dx_inf": float(c.get("dx_inf_px", 0.0)), "dy_off": float(c.get("dy_off_px", 0.0))}
    except Exception:
        return {"dx_inf": 0.0, "dy_off": 0.0}


def save_calib(dx_inf: float, dy_off: float, distance_mm: float, dx_meas: float):
    CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
    CALIB_PATH.write_text(json.dumps({
        "dx_inf_px": round(dx_inf, 2), "dy_off_px": round(dy_off, 2),
        "f_px": round(F_PX, 1), "hfov_deg": HFOV_DEG, "baseline_mm": BASELINE_MM,
        "calibrated_with_mm": distance_mm, "dx_measured_px": round(dx_meas, 2),
        "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }, indent=2))


# ---------------------------------------------------------------------------------------------------------------
# main loop
# ---------------------------------------------------------------------------------------------------------------

def save_dy_off(dy_off: float):
    try:
        c = json.loads(CALIB_PATH.read_text())
    except Exception:
        c = {}
    c["dy_off_px"] = round(dy_off, 2)
    c["dy_learned_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    try:
        CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
        CALIB_PATH.write_text(json.dumps(c, indent=2))
    except Exception as e:
        log(f"cannot save dy_off: {e}")


class CamZoom:
    """Applied sensor zoom of the cameras (cam_stream.py state file); 1 when the file is missing / stale
    (rpicam-vid capture). stable = not moving for ZOOM_SETTLE_S."""

    def __init__(self):
        self.z = 1.0
        self.changed_at = 0.0

    def read(self):
        z, target = 1.0, 1.0
        try:
            with open(CAM_ZOOM_PATH) as f:
                st = json.load(f)
            if time.time() - float(st.get("ts", 0)) < 3.0:
                z = float(st.get("zoom", 1.0))
                target = float(st.get("target", z))
        except Exception:
            pass
        now = time.monotonic()
        if abs(z - self.z) > 0.002:
            self.changed_at = now
        self.z = z
        stable = abs(z - target) < 0.005 and now - self.changed_at > ZOOM_SETTLE_S
        return z, stable


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtsp", default="rtsp://127.0.0.1:8554")
    ap.add_argument("--ws", default="wss://127.0.0.1:8557")
    ap.add_argument("--calibrate-mm", type=float, default=0.0,
                    help="calibrate dx_inf with a target at this distance under the crosshair, then exit")
    ap.add_argument("--print", action="store_true", help="also print every measurement")
    args = ap.parse_args()

    readers = [KeyframeReader(f"{args.rtsp}/cam0"), KeyframeReader(f"{args.rtsp}/cam1")]
    for r in readers:
        r.start()
    calib = load_calib()
    log(f"f = {F_PX:.1f} px, baseline {BASELINE_MM} mm, dx_inf {calib['dx_inf']:.1f} px")

    calib_request = {"mm": args.calibrate_mm}
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    hist: list = []        # last good measurements (dx, dy) for a small median
    last_lock = (0.0, -1e9)  # (dx, monotonic time) of the last locked measurement
    dy_hist = []
    dy_saved = calib['dy_off']
    dy_saved_at = -1e9
    last_seq = (-1, -1)
    ws = None
    calib_samples: list = []
    camzoom = CamZoom()
    last_out = None       # (mdx1, mdy1, dist) of the last lock, zoom-1 px, for the rescaled hold while zooming

    def ws_reader(sock):
        try:
            for raw in sock:
                try:
                    m = json.loads(raw)
                except Exception:
                    continue
                if isinstance(m, dict) and m.get("type") == "stereo_calibrate":
                    mm = float(m.get("distance_mm") or 0)
                    if mm > 0:
                        log(f"calibration requested at {mm:.0f} mm")
                        calib_request["mm"] = mm
        except Exception:
            pass

    while True:
        if ws is None and not args.calibrate_mm:
            try:
                ws = connect(args.ws, ssl=ctx, open_timeout=5, max_size=2 ** 22)
                threading.Thread(target=ws_reader, args=(ws,), daemon=True).start()
                log("connected to ws-teleop")
            except Exception as e:
                log("ws-teleop not reachable:", e)
                ws = None
                time.sleep(2)
                continue

        (fl, tl, sl), (fr, tr, srq) = readers[0].latest(), readers[1].latest()
        if fl is None or fr is None or (sl, srq) == last_seq or abs(tl - tr) > 0.03 or time.monotonic() - max(tl, tr) > 0.5:
            time.sleep(0.02)
            continue
        last_seq = (sl, srq)
        z, z_stable = camzoom.read()
        if not z_stable:
            # zoom moving: these keyframes mix crops, do not match; send the last lock rescaled to this zoom
            msg = {"type": "stereo_collimation", "score": 0.0, "dx_inf": round(calib["dx_inf"] * z, 2), "win": 0,
                   "ts": round(time.time(), 3), "zoom": round(z, 3)}
            if last_out is not None and time.monotonic() - last_lock[1] < CONT_MAX_AGE_S:
                msg.update({"lock": True, "dx": round(last_out[0] * z, 2), "dy": round(last_out[1] * z, 2),
                            "dist_mm": round(last_out[2], 1) if last_out[2] else None, "why": "zoom"})
                last_lock = (last_lock[0], time.monotonic())
            else:
                msg.update({"lock": False, "dx": round(calib["dx_inf"] * z, 2), "dy": round(calib["dy_off"] * z, 2),
                            "dist_mm": None, "why": "zoom"})
            if ws is not None:
                try:
                    ws.send(json.dumps(msg))
                except Exception:
                    try:
                        ws.close()
                    except Exception:
                        pass
                    ws = None
            continue
        prev = last_lock[0] * z if time.monotonic() - last_lock[1] < CONT_MAX_AGE_S else None
        dx, dy, score, ok, why, win = match(fl, fr, prev, z)
        if ok:
            dx, dy = dx / z, dy / z          # zoom-1 pixels from here on

        msg = {"type": "stereo_collimation", "score": round(score, 3), "dx_inf": round(calib["dx_inf"] * z, 2),
               "win": win, "ts": round(time.time(), 3), "zoom": round(z, 3)}
        if ok:
            last_lock = (dx, time.monotonic())
            dy_hist = (dy_hist + [dy])[-DY_LEARN_N:]
            if len(dy_hist) >= DY_LEARN_MIN:
                dy_learn = float(np.median(dy_hist))
                if (abs(dy_learn - dy_saved) > DY_SAVE_DELTA
                        and time.monotonic() - dy_saved_at > DY_SAVE_EVERY_S):
                    save_dy_off(dy_learn)
                    log(f"vertical offset learned: dy_off {dy_saved:.1f} -> {dy_learn:.1f} px")
                    dy_saved, dy_saved_at = dy_learn, time.monotonic()
                calib["dy_off"] = dy_learn
            hist = (hist + [(dx, dy)])[-3:]
            mdx = float(np.median([h[0] for h in hist]))
            mdy = float(np.median([h[1] for h in hist]))
            d = calib["dx_inf"] - mdx
            dist = F_PX * BASELINE_MM / d if d > 0.5 else None
            last_out = (mdx, mdy, dist)
            msg.update({"lock": True, "dx": round(mdx * z, 2), "dy": round(mdy * z, 2),
                        "dist_mm": round(dist, 1) if dist else None})
            if why:
                msg["why"] = why
            if calib_request["mm"] > 0:
                calib_samples.append((mdx, mdy))
                if len(calib_samples) >= 9:
                    cdx = float(np.median([c[0] for c in calib_samples]))
                    cdy = float(np.median([c[1] for c in calib_samples]))
                    mm = calib_request["mm"]
                    dx_inf = cdx + F_PX * BASELINE_MM / mm
                    save_calib(dx_inf, cdy, mm, cdx)
                    calib = {"dx_inf": dx_inf, "dy_off": cdy}
                    log(f"calibrated: dx at {mm:.0f} mm = {cdx:.1f} px -> dx_inf = {dx_inf:.1f} px, dy {cdy:.1f} px "
                        f"-> {CALIB_PATH}")
                    calib_request["mm"] = 0
                    calib_samples = []
                    if args.calibrate_mm:
                        return
        else:
            hist = []
            msg.update({"lock": False, "dx": round(calib["dx_inf"] * z, 2), "dy": round(calib["dy_off"] * z, 2),
                        "dist_mm": None, "why": why})

        if args.print or (int(time.time()) % 10 == 0 and sl % 5 == 0):
            log(json.dumps(msg))
        if ws is not None:
            try:
                ws.send(json.dumps(msg))
            except Exception as e:
                log("ws send failed:", e)
                try:
                    ws.close()
                except Exception:
                    pass
                ws = None


if __name__ == "__main__":
    main()
