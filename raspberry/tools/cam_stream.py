#!/usr/bin/env python3
"""
cam_stream.py - one Camera Module 3 -> H.264 (x264, low latency) -> RTSP publish to MediaMTX, with a LIVE sensor
zoom (ScalerCrop) controlled over UDP. Replaces rpicam-vid in scripts/video_capture.sh (CAPTURE=picamera2).

Why: rpicam-vid cannot change the crop while streaming. The Quest app zoomed digitally into the 800x450 stream
(zoom 3x = 270x150 real pixels). The 90/120 fps sensor mode reads 1536x864 (2x2 binned 3072x1728 field), so a
centred ScalerCrop gives up to ~1.9x of REAL detail at the same 800x450 / 90 fps, and beyond that the ISP upscales
a sharper source than the app could (and the bitrate is spent on the zoomed region).

Same stream as rpicam-vid (video_capture.sh): WxH @ FPS, x264 preset ultrafast + tune zerolatency, baseline,
CBR-ish (VBV), IDR every INTRA frames, SPS/PPS repeated, denoise off, fixed exposure / gain / AWB / lens from the
script's probe, libcamera frame sync (cam0 server, cam1 client).

Zoom control: UDP 127.0.0.1:(CTL_PORT_BASE + camera), JSON {"zoom": z} (1 .. ZOOM_MAX) and/or
{"offset": [ox, oy]}: the crop centre moved by (ox, oy) ZOOM-1 stream pixels (sensor convergence: the stereo
collimator moves cam1's crop onto the object under cam0's crosshair, so at high zoom the object stays in view and
the residual disparity is small). The offset is clamped to the sensor area (no margin at 1x). The crop eases towards
the target (time constant ZOOM_TAU_S) so both cameras move smoothly together. The applied zoom is written to
/dev/shm/jonny5_cam{N}_zoom.json ({"zoom", "target", "ts"}) for the collimator and ws-teleop.

Brightness: UDP {"ev": stops} (EV_MIN .. EV_MAX). With the fixed probed exposure (--shutter/--gain, both cameras
matched) the gain is scaled by 2^ev: below gain 1 the shutter gets shorter, above the sensor's analogue maximum
the shutter gets longer up to the frame period and the rest is requested as gain (the ISP adds it as digital
gain). Both cameras receive the same ev, so the eyes stay matched. With auto exposure (--shutter 0) the ev goes to
libcamera's ExposureValue. The applied exposure is logged once the sensor reports it.

High-resolution snapshot for the OCR (tools/vision_service.py, 2026-10-18): with --hires (default) the ISP main
output is the sensor mode size (1536x864, the most detail available at 90 fps, 1.92x the stream) and the video is
encoded from the lores output (WxH, same crop, so nothing changes in the stream). UDP {"snap": {"id": n,
"box": [x0, y0, x1, y1]}} (stream px) -> the grey (Y) crop of that box from the next main frame is saved to
/dev/shm/jonny5_cam{N}_snap.npy and the sender gets {"snap_done": n, "path", "scale", "x0", "y0"} (main px).
UDP {"still": {"id": n, "box": [...]}}: the same from a FULL-RESOLUTION still (4608x2592, ~2x the snapshot, 3.8x
the stream at zoom 1): the camera switches to the still mode for one frame (focus, exposure and white balance
frozen at the current values) and back. The encoder only gets no frames meanwhile (about 1 s): the RTSP session
stays up; the sensor sync and the zoom / brightness are restored afterwards. Reply {"still_done": n, ...} with the
offsets / scale relative to the stream as for the snapshot, plus "ms".
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import threading
import time


import av  # noqa: F401  (PyAV, used by the encoder)
import numpy as np
from libcamera import controls
from picamera2 import Picamera2
from picamera2.encoders import LibavH264Encoder
from picamera2.outputs import PyavOutput
from picamera2.request import MappedArray

CTL_PORT_BASE = 5600
ZOOM_MAX = 6.0
ZOOM_TAU_S = 0.15
EV_MIN, EV_MAX = -2.0, 2.0
ANALOGUE_GAIN_MAX = 16.0     # IMX708
GAIN_REQUEST_MAX = 64.0      # beyond the analogue maximum the ISP applies digital gain
STATE_DIR = "/dev/shm"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class LowLatencyH264(LibavH264Encoder):  # noqa: D101
    """Picamera2's libav x264 encoder with the rpicam-vid --low-latency settings: slice threads (no frame-thread
    latency), baseline, level, VBV so the rate is really capped."""

    def __init__(self, *a, level="4.2", vbv_frames=10, **kw):
        super().__init__(*a, **kw)
        self.level = level
        self.vbv_frames = vbv_frames
        self.lat_ms = []
        self.lat_stats = None
        self.stamp_on = False   # glass-to-glass measurement: capture time written into the frame

    def _start(self):
        super()._start()
        cc = self._stream.codec_context
        cc.thread_type = av.codec.context.ThreadType.SLICE
        cc.thread_count = 2
        if self.level:
            cc.options["level"] = self.level
        cc.options["forced-idr"] = "1"   # a forced keyframe is a real IDR (the collimator decodes only those)
        params = ["scenecut=0"]   # keyframes only where we ask (aligned across the two cameras)
        if self.bitrate:
            kbps = int(self.bitrate / 1000)
            buf = max(50, int(kbps * self.vbv_frames / max(1, self.framerate)))
            params += [f"vbv-maxrate={kbps}", f"vbv-bufsize={buf}"]
        cc.options["x264-params"] = ":".join(params)

    def _yuv420_frame(self, arr):
        """Copy the YUV420 buffer (stride 896 for 800 px) plane by plane into a frame of the exact size.
        av.VideoFrame.from_numpy_buffer turns the 675 rows of an 800x450 buffer into a 448-row frame: the U / V
        planes were read 2 luma rows early (green / yellow garbage in the top 8 rows, chroma 4 rows off, stream
        encoded at 800x448). 0.15 ms per frame."""
        if self._av_input_format != "yuv420p":
            return av.VideoFrame.from_numpy_buffer(arr, format=self._av_input_format, width=self.width)
        w, h = self.width, self.height
        stride = arr.shape[1]
        ch, cs = (h + 1) // 2, stride // 2
        fl = arr.reshape(-1)
        y = fl[:stride * h].reshape(h, stride)[:, :w]
        u = fl[stride * h:stride * h + cs * ch].reshape(ch, cs)[:, :w // 2]
        v = fl[stride * h + cs * ch:stride * h + 2 * cs * ch].reshape(ch, cs)[:, :w // 2]
        frame = av.VideoFrame(w, h, "yuv420p")
        for plane, data in zip(frame.planes, (y, u, v)):
            dst = np.frombuffer(plane, np.uint8).reshape(plane.height, plane.line_size)
            dst[:, :data.shape[1]] = data
        return frame

    STAMP_BITS = 24
    STAMP_BS = 16

    def _draw_stamp(self, frame, sensor_ns):
        """Top-left block row: capture time = SensorTimestamp (CLOCK_MONOTONIC) as epoch ms & 0xFFFFFF, 24 bits LSB
        first + white / black guard, 16 px blocks (stamped_h264_source.py format). Read in the Quest screenshot
        by tools/g2g_quest.py together with the app's display-time row."""
        epoch_ms = ((sensor_ns + (time.time_ns() - time.monotonic_ns())) // 1000000) & 0xFFFFFF
        bs = self.STAMP_BS
        yp, up, vp = frame.planes
        y = np.frombuffer(yp, np.uint8).reshape(yp.height, yp.line_size)
        u = np.frombuffer(up, np.uint8).reshape(up.height, up.line_size)
        v = np.frombuffer(vp, np.uint8).reshape(vp.height, vp.line_size)
        for i in range(self.STAMP_BITS + 2):
            on = ((epoch_ms >> i) & 1) if i < self.STAMP_BITS else (i == self.STAMP_BITS)
            y[0:bs, i * bs:(i + 1) * bs] = 235 if on else 16
        u[0:bs // 2, 0:(self.STAMP_BITS + 2) * bs // 2] = 128
        v[0:bs // 2, 0:(self.STAMP_BITS + 2) * bs // 2] = 128

    def _encode(self, stream, request):
        # Same as LibavH264Encoder._encode, but a forced keyframe uses PictureType.I: Picamera2 0.3.34 assigns
        # the string "I", which PyAV 14 rejects (TypeError) and kills the capture.
        if request.stream_map.get(stream) is None:
            return   # full-resolution still in progress (no lores stream in that mode): skip, the video resumes after
        request.acquire()
        self._request_release_queue.append(request)
        timestamp_us = self._timestamp(request)
        sensor_ns = request.get_metadata().get("SensorTimestamp")
        with MappedArray(request, stream) as m:
            frame = self._yuv420_frame(m.array)
            if self.stamp_on and sensor_ns:
                self._draw_stamp(frame, sensor_ns)
            frame.pts = timestamp_us
            if self._key_frames_requested > self._key_frames_generated:
                self._key_frames_generated = self._key_frames_requested
                frame.pict_type = av.video.frame.PictureType.I
            for packet in self._stream.encode(frame):
                self._lasttimestamp = (time.monotonic_ns(), packet.pts)
                self.outputframe(bytes(packet), packet.is_keyframe, timestamp=packet.pts, packet=packet)
            if sensor_ns:
                # sensor timestamp (CLOCK_MONOTONIC, frame start) -> packet handed to the RTSP muxer
                self.lat_ms.append((time.monotonic_ns() - sensor_ns) / 1e6)
                if len(self.lat_ms) >= 900:      # ~10 s at 90 fps
                    v = sorted(self.lat_ms)
                    self.lat_stats = {"p50": round(v[len(v) // 2], 1), "p95": round(v[int(len(v) * 0.95)], 1),
                                      "max": round(v[-1], 1)}
                    log(f"sensor -> RTSP mux latency ms {self.lat_stats}")
                    self.lat_ms = []
        while len(self._request_release_queue) > self._request_release_delay:
            self._request_release_queue.popleft().release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, required=True)
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=450)
    ap.add_argument("--fps", type=float, default=90.0)
    ap.add_argument("--bitrate", type=int, default=3000000)
    ap.add_argument("--intra", type=int, default=20)
    ap.add_argument("--sensor", default="1536x864", help="sensor mode output size")
    ap.add_argument("--shutter", type=int, default=0, help="us; 0 = auto exposure")
    ap.add_argument("--gain", type=float, default=0.0)
    ap.add_argument("--awbgains", default="", help="r,b; empty = auto white balance")
    ap.add_argument("--lens-position", type=float, default=None, help="dioptres (with --autofocus-mode manual)")
    # rpicam-vid style options passed through by video_capture.sh
    ap.add_argument("--autofocus-mode", choices=("default", "manual", "auto", "continuous"), default="default")
    ap.add_argument("--autofocus-range", default="normal")
    ap.add_argument("--roi", default="", help="ignored: the zoom is driven over UDP")
    ap.add_argument("--sync", choices=("server", "client", "off"), default="off")
    ap.add_argument("--zoom", type=float, default=1.0, help="initial zoom")
    ap.add_argument("--hires", type=int, default=1, help="1: main = sensor size for snapshots, video from lores")
    ap.add_argument("-o", "--output", required=True, help="rtsp://127.0.0.1:8554/camN")
    args = ap.parse_args()

    cam = args.camera
    sw, sh = (int(v) for v in args.sensor.lower().split("x"))
    picam2 = Picamera2(cam)
    ctrl = {"FrameRate": args.fps, "NoiseReductionMode": controls.draft.NoiseReductionModeEnum.Off}
    if args.shutter > 0:
        ctrl.update({"AeEnable": False, "ExposureTime": args.shutter, "AnalogueGain": max(1.0, args.gain)})
    if args.awbgains:
        r, b = (float(v) for v in args.awbgains.split(","))
        ctrl.update({"AwbEnable": False, "ColourGains": (r, b)})
    if args.lens_position is not None:
        ctrl.update({"AfMode": controls.AfModeEnum.Manual, "LensPosition": args.lens_position})
    elif args.autofocus_mode in ("continuous", "default"):
        # rpicam-vid's "default" is continuous AF too (what the stream used before this program)
        ctrl.update({"AfMode": controls.AfModeEnum.Continuous})
    elif args.autofocus_mode == "auto":
        ctrl.update({"AfMode": controls.AfModeEnum.Auto})
    if args.sync != "off":
        ctrl["SyncMode"] = (controls.rpi.SyncModeEnum.Server if args.sync == "server"
                            else controls.rpi.SyncModeEnum.Client)
    hires = bool(args.hires) and (sw, sh) != (args.width, args.height)
    if hires:
        streams = {"main": {"size": (sw, sh), "format": "YUV420"},
                   "lores": {"size": (args.width, args.height), "format": "YUV420"}}
    else:
        streams = {"main": {"size": (args.width, args.height), "format": "YUV420"}}
    cfg = picam2.create_video_configuration(
        **streams,
        sensor={"output_size": (sw, sh), "bit_depth": 10},
        controls=ctrl, buffer_count=6)
    picam2.configure(cfg)

    # crop limits of this sensor mode (sensor pixel array coordinates)
    smin, smax, _sdef = picam2.camera_controls["ScalerCrop"]
    full = smax  # (x, y, w, h)
    log(f"cam{cam}: sensor mode {picam2.camera_configuration()['sensor']}, crop max {full}, min {smin}")
    mc = picam2.camera_configuration()["main"]
    log(f"cam{cam}: main stream {mc.get('size')} {mc.get('format')} stride {mc.get('stride')} framesize {mc.get('framesize')}")

    k = full[2] / float(args.width)   # sensor pixels per zoom-1 stream pixel

    def crop_for(z: float, off=(0.0, 0.0)):
        """ScalerCrop for zoom z with the centre moved by off (zoom-1 px); also returns the offset actually
        applied after clamping to the sensor area."""
        z = max(1.0, min(ZOOM_MAX, z))
        w = int(round(full[2] / z)) & ~1
        h = int(round(w * args.height / args.width)) & ~1
        h = min(h, full[3])
        cx = full[0] + full[2] / 2.0 + off[0] * k
        cy = full[1] + full[3] / 2.0 + off[1] * k
        x = int(round(cx - w / 2.0))
        y = int(round(cy - h / 2.0))
        x = max(full[0], min(full[0] + full[2] - w, x)) & ~1
        y = max(full[1], min(full[1] + full[3] - h, y)) & ~1
        applied = ((x + w / 2.0 - (full[0] + full[2] / 2.0)) / k, (y + h / 2.0 - (full[1] + full[3] / 2.0)) / k)
        return (x, y, w, h), applied

    state = {"target": max(1.0, min(ZOOM_MAX, args.zoom)), "zoom": max(1.0, min(ZOOM_MAX, args.zoom)),
             "off": [0.0, 0.0], "off_t": [0.0, 0.0], "off_applied": [0.0, 0.0]}
    crop, state["off_applied"] = crop_for(state["zoom"])
    picam2.set_controls({"ScalerCrop": crop})

    # Keyframes ALIGNED across the two cameras: the stereo collimator decodes only keyframes and pairs them by
    # time. With a frame counter per encoder (iperiod) the two cameras' IDRs fell on different frames (the
    # encoders start at different times) and matching pairs were rare. The synchronised sensors share the
    # frame timestamps, so the GOP index is taken from SensorTimestamp and an IDR is forced when it changes.
    enc = LowLatencyH264(bitrate=args.bitrate, repeat=True, iperiod=None, framerate=int(round(args.fps)),
                         profile="baseline")
    period_ns = 1e9 / args.fps
    gop = {"last": None}

    def align_keyframes(request):
        ts = request.get_metadata().get("SensorTimestamp")
        if ts is None:
            return
        state["sensor_ts"] = (int(ts), time.time())   # for the sync check (phase between the two cameras)
        state["md"] = request.get_metadata()
        if expo.get("log_after") and time.monotonic() > expo["log_after"]:
            md = request.get_metadata()
            expo["log_after"] = None
            log(f"cam{cam}: ev {expo['ev']:+.2f} -> shutter {md.get('ExposureTime')} us, analogue gain "
                f"{md.get('AnalogueGain', 0):.2f}, digital gain {md.get('DigitalGain', 0):.2f}")
        g = int(round(ts / period_ns)) // max(1, args.intra)
        if g != gop["last"]:
            gop["last"] = g
            enc.force_key_frame()

    # ---- brightness (UDP {"ev": stops}), see the module docstring
    shutter_max = int(1e6 / args.fps) - 400
    expo = {"ev": 0.0, "log_after": None}

    def apply_ev(ev: float):
        ev = max(EV_MIN, min(EV_MAX, ev))
        expo["ev"] = ev
        if args.shutter > 0:
            shutter = float(args.shutter)
            gain = max(1.0, args.gain) * (2.0 ** ev)
            if gain < 1.0:                               # darker than the probe at gain 1: shorter shutter
                shutter *= gain
                gain = 1.0
            elif gain > ANALOGUE_GAIN_MAX and shutter < shutter_max:
                s2 = min(float(shutter_max), shutter * gain / ANALOGUE_GAIN_MAX)
                gain *= shutter / s2
                shutter = s2
            gain = min(GAIN_REQUEST_MAX, gain)
            picam2.set_controls({"ExposureTime": max(50, int(round(shutter))), "AnalogueGain": float(gain)})
            log(f"cam{cam}: ev {ev:+.2f} -> request shutter {int(shutter)} us, gain {gain:.2f}")
        else:
            picam2.set_controls({"ExposureValue": float(ev)})
            log(f"cam{cam}: ev {ev:+.2f} (auto exposure, ExposureValue)")
        expo["log_after"] = time.monotonic() + 0.5

    # ---- high-resolution snapshot (UDP {"snap": {...}}), taken from the next main frame in the camera thread
    snap = {"req": None}
    snap_path = os.path.join(STATE_DIR, f"jonny5_cam{cam}_snap.npy")
    snap_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def take_snapshot(request):
        rq = snap["req"]
        if rq is None:
            return
        snap["req"] = None
        sid, box, addr = rq
        try:
            mw, mh = (sw, sh) if hires else (args.width, args.height)
            k = mw / float(args.width)
            x0 = max(0, int((box[0]) * k)) & ~1
            y0 = max(0, int((box[1]) * k)) & ~1
            x1 = min(mw, int(math.ceil(box[2] * k)))
            y1 = min(mh, int(math.ceil(box[3] * k)))
            with MappedArray(request, "main") as m:
                crop = np.array(m.array[y0:y1, x0:x1], dtype=np.uint8)   # Y plane rows come first
            np.save(snap_path, crop)
            snap_sock.sendto(json.dumps({"snap_done": sid, "path": snap_path, "scale": k, "x0": x0, "y0": y0,
                                         "w": int(crop.shape[1]), "h": int(crop.shape[0])}).encode(), addr)
        except Exception as e:
            log(f"cam{cam}: snapshot failed {e!r}")
            try:
                snap_sock.sendto(json.dumps({"snap_done": sid, "error": str(e)}).encode(), addr)
            except OSError:
                pass

    full_w, full_h = picam2.sensor_resolution

    def take_still(sid, box, addr):
        """Full-resolution grey crop of box (stream px), see the module docstring."""
        t0 = time.monotonic()
        md = state.get("md") or {}
        crop_now = state.get("crop_applied") or crop
        # NoiseReductionMode Off as in the video: the still mode default (temporal denoise) aborts the PiSP back end
        # ("TDN output not enabled when TDN enabled") when switching from the video mode
        ctrl_s = {"AeEnable": False, "AwbEnable": False, "NoiseReductionMode": controls.draft.NoiseReductionModeEnum.Off,
                  "ExposureTime": int(md.get("ExposureTime", args.shutter or 10000)),
                  "AnalogueGain": float(md.get("AnalogueGain", max(1.0, args.gain))),
                  "ScalerCrop": (0, 0, full_w, full_h)}
        if md.get("ColourGains"):
            ctrl_s["ColourGains"] = tuple(md["ColourGains"])
        if md.get("LensPosition") is not None:
            ctrl_s.update({"AfMode": controls.AfModeEnum.Manual, "LensPosition": float(md["LensPosition"])})
        still_cfg = picam2.create_still_configuration(main={"size": (full_w, full_h), "format": "YUV420"},
                                                      buffer_count=1, transform=cfg["transform"], controls=ctrl_s)
        try:
            arr = picam2.switch_mode_and_capture_array(still_cfg, "main")
        except Exception as e:
            log(f"cam{cam}: still failed {e!r}")
            snap_sock.sendto(json.dumps({"still_done": sid, "error": str(e)}).encode(), addr)
            return
        t1 = time.monotonic()
        # back in the video mode: zoom crop, brightness and focus mode as before the still
        picam2.set_controls({"ScalerCrop": crop_now})
        if args.lens_position is None and args.autofocus_mode in ("continuous", "default"):
            picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        if abs(expo["ev"]) > 1e-3:
            apply_ev(expo["ev"])
        # stream px -> sensor px: the video crop (centred for cam0, same flip as the still)
        k = crop_now[2] / float(args.width)
        ax = (full_w - crop_now[2]) / 2.0 + (crop_now[0] - (full_w - crop_now[2]) / 2.0)
        ay = (full_h - crop_now[3]) / 2.0 + (crop_now[1] - (full_h - crop_now[3]) / 2.0)
        x0 = max(0, int(ax + box[0] * k)) & ~1
        y0 = max(0, int(ay + box[1] * k)) & ~1
        x1 = min(full_w, int(math.ceil(ax + box[2] * k)))
        y1 = min(full_h, int(math.ceil(ay + box[3] * k)))
        g = np.array(arr[y0:y1, x0:x1], dtype=np.uint8)
        np.save(snap_path, g)
        ms = round((time.monotonic() - t0) * 1000)
        log(f"cam{cam}: full-resolution still {g.shape[1]}x{g.shape[0]} (x{k:.2f}), mode switch {round((t1 - t0) * 1000)} ms,"
            f" total {ms} ms")
        snap_sock.sendto(json.dumps({"still_done": sid, "path": snap_path, "scale": k, "x0": x0 - ax, "y0": y0 - ay,
                                     "w": int(g.shape[1]), "h": int(g.shape[0]), "ms": ms}).encode(), addr)

    def pre_callback(request):
        align_keyframes(request)
        take_snapshot(request)

    picam2.pre_callback = pre_callback
    if args.sync != "off":
        enc.sync_enable = True       # encode only once the cameras report SyncReady
    # RTP over UDP like rpicam-vid: MediaMTX is configured with rtspTransports [udp] (TCP -> 461 -> EPROTONOSUPPORT)
    out = PyavOutput(args.output, format="rtsp")
    fatal = {"err": None}

    def on_output_error(e):
        # PyavOutput closes the container silently on a mux error: log it and exit, video_capture.sh restarts
        fatal["err"] = e
        log(f"cam{cam}: output error {e!r}")

    out.error_callback = on_output_error
    picam2.start_encoder(enc, out, name="lores" if hires else "main")
    picam2.start()
    log(f"cam{cam}: streaming {args.width}x{args.height} @ {args.fps} fps, {args.bitrate} bit/s, IDR {args.intra},"
        f" sync {args.sync} -> {args.output}")

    # ---- zoom control (UDP) + smoothing loop
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", CTL_PORT_BASE + cam))
    sock.settimeout(0.02)
    state_path = os.path.join(STATE_DIR, f"jonny5_cam{cam}_zoom.json")
    tmp_path = state_path + ".tmp"
    stop = threading.Event()

    def write_state():
        try:
            with open(tmp_path, "w") as f:
                json.dump({"zoom": round(state["zoom"], 4), "target": round(state["target"], 4),
                           "offset": [round(v, 2) for v in state["off_applied"]],
                           "offset_target": [round(v, 2) for v in state["off_t"]],
                           "sensor_ts": state.get("sensor_ts"),
                           "ev": round(expo["ev"], 3),
                           "ts": round(time.time(), 3)}, f)
            os.replace(tmp_path, state_path)
        except OSError:
            pass

    write_state()
    last = time.monotonic()
    last_applied = state["zoom"]
    last_off = [0.0, 0.0]
    last_write = 0.0
    try:
        while not stop.is_set():
            if fatal["err"] is not None:
                raise SystemExit(2)
            try:
                data, addr = sock.recvfrom(512)
                msg = json.loads(data.decode("utf-8", "replace"))
                if isinstance(msg.get("still"), dict) and cam == 0:
                    b = [float(v) for v in msg["still"].get("box", [0, 0, args.width, args.height])[:4]]
                    take_still(msg["still"].get("id"), b, addr)
                if isinstance(msg.get("snap"), dict):
                    b = [float(v) for v in msg["snap"].get("box", [0, 0, args.width, args.height])[:4]]
                    snap["req"] = (msg["snap"].get("id"), b, addr)
                if "zoom" in msg:
                    z = float(msg["zoom"])
                    if math.isfinite(z):
                        state["target"] = max(1.0, min(ZOOM_MAX, z))
                if "ev" in msg:
                    ev = float(msg["ev"])
                    if math.isfinite(ev) and abs(ev - expo["ev"]) > 1e-3:
                        apply_ev(ev)
                if "stamp" in msg:
                    enc.stamp_on = bool(msg["stamp"])
                    log(f"cam{cam}: capture stamp {'ON' if enc.stamp_on else 'OFF'}")
                if "offset" in msg:
                    ox, oy = (float(v) for v in msg["offset"][:2])
                    if math.isfinite(ox) and math.isfinite(oy):
                        state["off_t"] = [max(-400.0, min(400.0, ox)), max(-225.0, min(225.0, oy))]
            except socket.timeout:
                pass
            except (ValueError, OSError):
                pass
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            a = 1.0 - math.exp(-dt / ZOOM_TAU_S)
            # ease in log space: the same speed at 1x and 4x
            lz = math.log(state["zoom"]) + (math.log(state["target"]) - math.log(state["zoom"])) * a
            state["zoom"] = math.exp(lz)
            if abs(state["zoom"] - state["target"]) < 0.002:
                state["zoom"] = state["target"]
            for i in (0, 1):
                state["off"][i] += (state["off_t"][i] - state["off"][i]) * a
                if abs(state["off"][i] - state["off_t"][i]) < 0.05:
                    state["off"][i] = state["off_t"][i]
            if (abs(state["zoom"] - last_applied) >= 0.002
                    or max(abs(state["off"][i] - last_off[i]) for i in (0, 1)) >= 0.05):
                crop, state["off_applied"] = crop_for(state["zoom"], state["off"])
                picam2.set_controls({"ScalerCrop": crop})
                state["crop_applied"] = crop
                last_applied = state["zoom"]
                last_off = list(state["off"])
            if now - last_write > 0.1:
                write_state()
                last_write = now
    except KeyboardInterrupt:
        pass
    finally:
        try:
            picam2.stop_encoder()
        except Exception:
            pass
        picam2.stop()
        picam2.close()


if __name__ == "__main__":
    main()
