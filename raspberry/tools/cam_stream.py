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
# libcamera frame sync (cam0 server -> cam1 client) travels as UDP multicast (239.255.255.250:10000). Stand-alone
# (no PC on eth0) the Pi has no default route, the server cannot send and the client cannot even join the group,
# so cam1 never encoded a frame (2026-10-19). jonny5-sync-route.service now routes that group on lo; this is the
# safety net: after SYNC_TIMEOUT_S without a frame the encoder stops waiting for SyncReady (unsynchronised stereo
# is far better than one eye).
SYNC_TIMEOUT_S = 10.0
GAIN_REQUEST_MAX = 64.0      # beyond the analogue maximum the ISP applies digital gain
STATE_DIR = "/dev/shm"
# Stereo matching (2026-10-19): cam0 is the MASTER (auto exposure / white balance, one-shot AF on request), cam1
# FOLLOWS it (--follow 0): it copies the master's shutter, gains and colour gains from the master's state file (so
# both eyes always have the same brightness and colour, also outdoors when the light changes) and after each AF
# request both cameras focus on the same window; cam1 cross-checks its lens position against the master's
# (+ the learned lens_delta of the two actuators) and falls back to the master's when its own AF went elsewhere
# (the old boot probe left cam0 at 4.9 dioptres and cam1 at 0.9 for the whole session: one blurred eye at zoom).
# Calibration file (lens_delta, gain_ratio, colour_ratio), learned at runtime:
MATCH_CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config_runtime", "camera",
                                "stereo_match.json")
META_WRITE_S = 0.05          # master state file rate (the follower polls it)
# The two modules' LensPosition scales differ (measured 2026-10-19 on a 2 m scene: cam0 sharpest at 3.0
# "dioptres", cam1 at 0.5): each camera keeps its OWN one-shot AF result; lens_delta (cam1 - cam0) is learned
# from the pairs where both focused and is only used as the fallback when cam1's AF fails.
# Measured 2026-10-19 (lens sweeps vs the Laplacian sharpness of the crosshair region): cam0's PDAF lands on its
# sharpness peak (3.05-3.16 vs peak 3.2), cam1's PDAF is biased by +0.8 (1.30-1.34 vs peak 0.5). So only the
# master runs the PDAF; both cameras then REFINE by contrast (a short lens sweep around the start position on the
# live frames) and the follower starts its sweep at master + lens_delta (delta learned from the refined pairs).
LENS_DELTA_ALPHA = 0.25      # learning rate of lens_delta from refined pairs
AF_SETTLE_S = 3.0            # max wait for the PDAF after a trigger
AF_MIN_S = 0.5               # a terminal AfState this soon after the trigger is the previous one (stale)
REFINE_SPAN = 0.3            # contrast refinement: +-span around the start, REFINE_N steps
REFINE_N = 5
REFINE_SETTLE_S = 0.12       # lens settle before measuring a step
REFINE_FRAMES = 3            # frames averaged per step
REFINE_MIN_RATIO = 1.08      # max/min sharpness across the steps below this: no peak (flat / dark), keep the start
REFINE_MAX_EXT = 4           # extra sweeps when the best is at the border
# Stereo seed: the collimator writes its lock to STEREO_STATE_PATH; zoomed in (> LEARN_MAX_ZOOM) the master
# skips the PDAF (at 6x it landed 1.5 dioptres off) and refines from 1000 / dist_mm + master_offset, the
# learned offset of its LensPosition scale (cam0 reads ~2.5 for a true 0.5), measured at low zoom.
STEREO_STATE_PATH = "/dev/shm/jonny5_stereo.json"
STEREO_SEED_MAX_AGE_S = 2.0
MASTER_OFFSET_ALPHA = 0.25
REFINE_MIN_SHARP = 12.0      # best step below this (after the zoom downsampling): no usable detail, keep the start
REFINE_REGION_FRAC = 0.5     # sharpness region: central fraction of the stream
LEARN_MAX_ZOOM = 2.0         # lens_delta is learned only at low zoom (at 6x a 25 % window is 128 sensor px)
AF_WINDOW_MIN_FRAC = 0.25    # PDAF window never smaller than this fraction of the FULL field (zoomed in, the
                             # crop is tiny: at 6x the PDAF landed 1.5 dioptres off)
AF_WINDOW_FRAC = 0.25        # AF window: central fraction of the crop (the crosshair)


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
        self.frames = 0         # frames handed to the encoder (sync watchdog)

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
        self.frames += 1
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
    ap.add_argument("--follow", type=int, default=-1,
                    help="follow this camera's exposure / white balance / focus (stereo slave), see MATCH_CALIB_PATH")
    ap.add_argument("--match-calib", default=MATCH_CALIB_PATH)
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
    follow = int(args.follow)
    if follow >= 0:
        log(f"cam{cam}: stereo follower of cam{follow} (exposure, white balance, focus cross-check)")
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

    refine = {"active": False, "collect": False, "acc": []}   # filled in below (contrast refinement)

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

    def measure_sharpness(request):
        if not refine["collect"]:
            return
        try:
            with MappedArray(request, "lores" if hires else "main") as m:
                y = m.array[:args.height, :args.width]
                hh, ww = args.height, args.width
                f0, f1 = (1.0 - REFINE_REGION_FRAC) / 2.0, (1.0 + REFINE_REGION_FRAC) / 2.0
                c = np.asarray(y[int(hh * f0): int(hh * f1), int(ww * f0): int(ww * f1)], dtype=np.float32)
            k = max(1, int(round(state["zoom"])))
            if k > 1:
                ch_, cw_ = (c.shape[0] // k) * k, (c.shape[1] // k) * k
                c = c[:ch_, :cw_].reshape(ch_ // k, k, cw_ // k, k).mean(axis=(1, 3))
            lap = c[:-2, 1:-1] + c[2:, 1:-1] + c[1:-1, :-2] + c[1:-1, 2:] - 4.0 * c[1:-1, 1:-1]
            refine["acc"].append(float(lap.var()))
        except Exception:
            pass

    def pre_callback(request):
        align_keyframes(request)
        take_snapshot(request)
        measure_sharpness(request)

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

    def meta_now():
        md = state.get("md") or {}
        cg = md.get("ColourGains")
        return {"shutter": int(md.get("ExposureTime", 0) or 0),
                "gain": round(float(md.get("AnalogueGain", 0.0) or 0.0), 4),
                "dgain": round(float(md.get("DigitalGain", 1.0) or 1.0), 4),
                "cg": [round(float(v), 4) for v in cg] if cg else None,
                "lens": None if md.get("LensPosition") is None else round(float(md["LensPosition"]), 3),
                "af_state": int(md.get("AfState", 0) or 0),
                "ct": int(md.get("ColourTemperature", 0) or 0),
                "lux": round(float(md.get("Lux", 0.0) or 0.0), 1),
                "af_req": af["seq"], "af_done": af.get("done_seq", 0), "af_lens": af.get("lens_done")}

    def write_state():
        try:
            with open(tmp_path, "w") as f:
                json.dump({"zoom": round(state["zoom"], 4), "target": round(state["target"], 4),
                           "offset": [round(v, 2) for v in state["off_applied"]],
                           "offset_target": [round(v, 2) for v in state["off_t"]],
                           "sensor_ts": state.get("sensor_ts"),
                           "ev": round(expo["ev"], 3),
                           "meta": meta_now(),
                           "follow": follow,
                           "match": match_calib if follow >= 0 else None,
                           "ts": round(time.time(), 3)}, f)
            os.replace(tmp_path, state_path)
        except OSError:
            pass

    # ---- one-shot autofocus on request (UDP {"af": 1} or {"af": {"window": [x, y, w, h]}} in stream px):
    # both cameras get the same request from ws-teleop / the stereo collimator and focus on the same window
    # (default: the central AF_WINDOW_FRAC of the current crop = the crosshair).
    af = {"seq": 0, "t": 0.0, "mode": args.autofocus_mode}

    scm = picam2.camera_properties.get("ScalerCropMaximum")

    def stereo_seed():
        """1000 / dist_mm of the collimator's fresh lock (true dioptres), or None."""
        try:
            with open(STEREO_STATE_PATH) as f:
                st_ = json.load(f)
            if st_.get("lock") and st_.get("dist_mm") and time.time() - float(st_.get("ts", 0)) < STEREO_SEED_MAX_AGE_S:
                return 1000.0 / max(100.0, float(st_["dist_mm"]))
        except (OSError, ValueError, TypeError):
            pass
        return None

    def af_trigger(window=None):
        crop_now = state.get("crop_applied") or crop_for(state["zoom"], state["off"])[0]
        cx0, cy0, cw, ch = crop_now
        opts = window if isinstance(window, dict) else {}
        seed = stereo_seed()
        af["seed"] = seed
        if follow < 0 and state["zoom"] > LEARN_MAX_ZOOM and seed is not None and not opts.get("pdaf"):
            # zoomed in: no PDAF, refine by contrast from the stereo distance
            af["seq"] += 1
            af["t"] = time.monotonic()
            af["pdaf_pending"] = False
            start = seed + float(match_calib.get("master_offset", 0.0))
            log(f"cam{cam}: AF #{af['seq']} from the stereo seed {seed:.2f} + offset "
                f"{float(match_calib.get('master_offset', 0.0)):+.2f} = {start:.2f} (zoom {state['zoom']:.1f}, no PDAF)")
            start_refine(start, master_af_done)
            return
        if isinstance(window, dict):
            window = window.get("window")
        if window and len(window) >= 4:
            kx, ky = cw / float(args.width), ch / float(args.height)
            wx = int(cx0 + float(window[0]) * kx)
            wy = int(cy0 + float(window[1]) * ky)
            ww = max(32, int(float(window[2]) * kx))
            wh = max(32, int(float(window[3]) * ky))
        else:
            ww = int(max(cw * AF_WINDOW_FRAC, full[2] * AF_WINDOW_MIN_FRAC))
            wh = int(max(ch * AF_WINDOW_FRAC, full[3] * AF_WINDOW_MIN_FRAC))
            wx, wy = int(cx0 + cw / 2 - ww / 2), int(cy0 + ch / 2 - wh / 2)
            wx = max(int(full[0]), min(int(full[0] + full[2]) - ww, wx))
            wy = max(int(full[1]), min(int(full[1] + full[3]) - wh, wy))
        if scm and not opts.get("sensor_window") and opts.get("relative", True):
            # libcamera: AfWindows are pixels WITHIN the ScalerCropMaximum rectangle (relative to its origin)
            wx, wy = wx - int(scm[0]), wy - int(scm[1])
        ctl_af = {"AfMode": controls.AfModeEnum.Auto, "AfTrigger": controls.AfTriggerEnum.Start}
        if opts.get("metering") == "auto":
            ctl_af["AfMetering"] = controls.AfMeteringEnum.Auto
        else:
            if opts.get("sensor_window"):
                wx, wy, ww, wh = (int(v) for v in opts["sensor_window"][:4])
            ctl_af.update({"AfMetering": controls.AfMeteringEnum.Windows, "AfWindows": [(wx, wy, ww, wh)]})
        rng = opts.get("range")
        if rng in ("normal", "macro", "full"):
            ctl_af["AfRange"] = {"normal": controls.AfRangeEnum.Normal, "macro": controls.AfRangeEnum.Macro,
                                 "full": controls.AfRangeEnum.Full}[rng]
        if opts.get("speed") in ("normal", "fast"):
            ctl_af["AfSpeed"] = (controls.AfSpeedEnum.Fast if opts["speed"] == "fast"
                                 else controls.AfSpeedEnum.Normal)
        try:
            picam2.set_controls(ctl_af)
            af["mode"] = "auto"
            af["seq"] += 1
            af["t"] = time.monotonic()
            af["pdaf_pending"] = follow < 0      # the master refines by contrast once the PDAF settles
            log(f"cam{cam}: AF #{af['seq']} " + ("metering auto" if "AfWindows" not in ctl_af
                                                 else f"window {(wx, wy, ww, wh)}")
                + (f" range {rng}" if rng else "") + f" (ScalerCropMaximum {scm})")
        except Exception as e:
            log(f"cam{cam}: AF trigger failed {e!r}")

    def lens_set(d: float):
        try:
            picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": float(d)})
            af["mode"] = "manual"
        except Exception as e:
            log(f"cam{cam}: lens set failed {e!r}")

    # ---- contrast refinement: short lens sweep around a start position, measured on the live frames
    refine.update({"steps": [], "i": 0, "res": [], "t_set": 0.0, "centre": 0.0, "ext": 0, "cb": None, "t0": 0.0})

    def start_refine(centre: float, cb=None, ext: int = 0):
        centre = max(0.0, min(12.0, float(centre)))
        steps = [max(0.0, min(12.0, centre + REFINE_SPAN * (2.0 * k / (REFINE_N - 1) - 1.0))) for k in range(REFINE_N)]
        refine.update({"active": True, "collect": False, "acc": [], "steps": steps, "i": 0, "res": [],
                       "centre": centre, "ext": ext, "cb": cb, "t0": time.monotonic()})
        lens_set(steps[0])
        refine["t_set"] = time.monotonic()

    def refine_tick():
        if not refine["active"]:
            return
        now_r = time.monotonic()
        if now_r - refine["t_set"] < REFINE_SETTLE_S:
            return
        refine["collect"] = True
        if len(refine["acc"]) < REFINE_FRAMES:
            if now_r - refine["t_set"] > 1.0:       # no frames (encoder stalled?): give up
                refine.update({"active": False, "collect": False})
                lens_set(refine["centre"])
                log(f"cam{cam}: refine aborted (no frames)")
            return
        refine["collect"] = False
        refine["res"].append(float(np.mean(refine["acc"][:REFINE_FRAMES])))
        refine["acc"] = []
        refine["i"] += 1
        if refine["i"] < len(refine["steps"]):
            lens_set(refine["steps"][refine["i"]])
            refine["t_set"] = now_r
            return
        # done: pick the best step, parabolic sub-step, border -> extend the sweep
        res, steps = refine["res"], refine["steps"]
        refine["active"] = False
        b = int(np.argmax(res))
        ratio = max(res) / max(1e-6, min(res))
        if ratio < REFINE_MIN_RATIO or max(res) < REFINE_MIN_SHARP:
            lens_set(refine["centre"])
            log(f"cam{cam}: refine: {'flat' if ratio < REFINE_MIN_RATIO else 'no detail'} "
                f"({[round(v) for v in res]}), lens {refine['centre']:.2f} kept")
            if refine["cb"]:
                refine["cb"](refine["centre"], False, res)
            return
        if b in (0, len(res) - 1) and refine["ext"] >= REFINE_MAX_EXT:
            # still at the border after the extensions (or at the lens range limit): not a real peak
            lens_set(steps[b])
            log(f"cam{cam}: refine: no peak within range ({[round(v) for v in res]}), lens {steps[b]:.2f}")
            if refine["cb"]:
                refine["cb"](steps[b], False, res)
            return
        if b in (0, len(res) - 1) and refine["ext"] < REFINE_MAX_EXT:
            log(f"cam{cam}: refine: best at the border ({[round(v) for v in res]}), extending")
            start_refine(steps[b], refine["cb"], refine["ext"] + 1)
            return
        best = steps[b]
        if 0 < b < len(res) - 1:
            d = (res[b - 1] - res[b + 1]) / (2.0 * (res[b - 1] - 2.0 * res[b] + res[b + 1]) or 1e-6)
            if abs(d) < 1.0:
                best = steps[b] + d * (steps[1] - steps[0])
        lens_set(best)
        log(f"cam{cam}: refine: lens {best:.2f} (start {refine['centre']:.2f}, {[round(v) for v in res]}, "
            f"{round((time.monotonic() - refine['t0']) * 1000)} ms)")
        if refine["cb"]:
            refine["cb"](best, True, res)

    def master_af_done(best, ok, res):
        af["lens_done"] = round(float(best), 3)
        af["done_seq"] = af["seq"]
        seed = af.get("seed")
        if ok and seed is not None and state["zoom"] <= LEARN_MAX_ZOOM:
            off_old = float(match_calib.get("master_offset", 0.0))
            n = int(match_calib.get("master_offset_n", 0))
            pair = float(best) - float(seed)
            off_new = pair if n == 0 else off_old + MASTER_OFFSET_ALPHA * (pair - off_old)
            match_calib["master_offset"] = round(off_new, 3)
            match_calib["master_offset_n"] = n + 1
            calib_dirty["dirty"] = True
            log(f"cam{cam}: lens {best:.2f} vs stereo {seed:.2f} dioptres: master_offset -> {off_new:+.2f} (n={n + 1})")

    def master_tick():
        """After the PDAF settles (or times out) refine by contrast, then publish af_done for the follower."""
        if follow >= 0 or not af.get("pdaf_pending"):
            return
        md = state.get("md") or {}
        st_af = int(md.get("AfState", 0) or 0)
        dt_af = time.monotonic() - af["t"]
        if (dt_af >= AF_MIN_S and st_af in (2, 3)) or dt_af > AF_SETTLE_S:
            af["pdaf_pending"] = False
            lp = md.get("LensPosition")
            if lp is None:
                af["done_seq"] = af["seq"]
                return
            log(f"cam{cam}: PDAF {'focused' if st_af == 2 else 'failed/timeout'} at {float(lp):.2f}, refining")
            start_refine(float(lp), master_af_done)

    # ---- stereo follower (cam1): copy the master's exposure / colour, cross-check the focus
    match_calib = {"lens_delta": 0.0, "lens_delta_n": 0, "gain_ratio": 1.0, "colour_ratio": [1.0, 1.0],
                   "master_offset": 0.0, "master_offset_n": 0}
    try:
        with open(args.match_calib) as f:
            match_calib.update({k: v for k, v in json.load(f).items() if k in match_calib})
    except (OSError, ValueError):
        pass
    calib_dirty = {"t": 0.0, "dirty": False}

    def save_match_calib():
        try:
            os.makedirs(os.path.dirname(args.match_calib), exist_ok=True)
            tmp = args.match_calib + ".tmp"
            try:
                with open(args.match_calib) as f:
                    cur = json.load(f)
            except (OSError, ValueError):
                cur = {}
            mine = ("master_offset", "master_offset_n") if follow < 0 else \
                ("lens_delta", "lens_delta_n", "gain_ratio", "colour_ratio")
            cur.update({k: match_calib[k] for k in mine})
            cur["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            with open(tmp, "w") as f:
                json.dump(cur, f, indent=2)
            os.replace(tmp, args.match_calib)
            calib_dirty["dirty"] = False
            log(f"cam{cam}: stereo match calibration saved {match_calib}")
        except OSError as e:
            log(f"cam{cam}: calibration save failed {e!r}")

    def follower_af_done(ml):
        def cb(best, ok, res):
            if ok and state["zoom"] > LEARN_MAX_ZOOM:
                log(f"cam{cam}: focus {best:.2f} = master {ml:.2f} {best - ml:+.2f} (zoom {state['zoom']:.1f}: not learned)")
            elif ok:
                d_old = float(match_calib["lens_delta"])
                n = int(match_calib.get("lens_delta_n", 0))
                pair = float(best) - float(ml)
                d_new = pair if n == 0 else d_old + LENS_DELTA_ALPHA * (pair - d_old)
                match_calib["lens_delta"] = round(d_new, 3)
                match_calib["lens_delta_n"] = n + 1
                calib_dirty["dirty"] = True
                log(f"cam{cam}: focus {best:.2f} = master {ml:.2f} {pair:+.2f} (lens_delta -> {d_new:+.2f}, n={n + 1})")
            else:
                log(f"cam{cam}: focus kept at master {ml:.2f} {float(match_calib['lens_delta']):+.2f} (no contrast peak)")
        return cb

    def follow_loop():
        master_path = os.path.join(STATE_DIR, f"jonny5_cam{follow}_zoom.json")
        last_m = None          # last applied (shutter, gain, cg)
        last_mtime = 0.0
        manual_on = False
        last_done = None
        while not stop.is_set():
            time.sleep(0.02)
            try:
                mt = os.path.getmtime(master_path)
                if mt == last_mtime:
                    continue
                last_mtime = mt
                with open(master_path) as f:
                    m = (json.load(f).get("meta") or {})
            except (OSError, ValueError):
                continue
            if not m.get("shutter") or not m.get("gain"):
                continue
            # exposure + colour: the master's values, scaled by the learned sensor-to-sensor ratios
            gain = float(m["gain"]) * float(m.get("dgain", 1.0)) * float(match_calib["gain_ratio"])
            gain = max(1.0, min(GAIN_REQUEST_MAX, gain))
            cg = m.get("cg")
            ctl = {}
            if not manual_on:
                ctl.update({"AeEnable": False, "AwbEnable": False})
            cur = (int(m["shutter"]), round(gain, 3), tuple(round(float(v), 3) for v in cg) if cg else None)
            if cur != last_m:
                ctl.update({"ExposureTime": max(50, int(m["shutter"])), "AnalogueGain": float(gain)})
                if cg:
                    cr = match_calib["colour_ratio"]
                    ctl["ColourGains"] = (float(cg[0]) * float(cr[0]), float(cg[1]) * float(cr[1]))
                try:
                    picam2.set_controls(ctl)
                    if not manual_on:
                        manual_on = True
                        log(f"cam{cam}: following cam{follow}: shutter {cur[0]} us gain {cur[1]} cg {cur[2]}")
                except Exception as e:
                    log(f"cam{cam}: follow set failed {e!r}")
                last_m = cur
            # focus: when the master has finished (PDAF + refinement) start our contrast refinement from
            # master + lens_delta (our own PDAF is biased, see the header)
            done = m.get("af_done")
            if last_done is None:
                last_done = done                       # the counter as found at start is not a new focus
            elif done != last_done:
                last_done = done
                ml = m.get("af_lens")
                if ml is not None and not refine["active"]:
                    start_refine(float(ml) + float(match_calib["lens_delta"]), follower_af_done(float(ml)))
            if calib_dirty["dirty"] and time.monotonic() - calib_dirty["t"] > 30.0:
                calib_dirty["t"] = time.monotonic()
                save_match_calib()

    if follow >= 0:
        threading.Thread(target=follow_loop, name="follow", daemon=True).start()

    write_state()
    last = time.monotonic()
    t_start = last
    sync_warned = False
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
                        if follow >= 0:
                            expo["ev"] = ev     # the follower copies the master's resulting exposure
                        else:
                            apply_ev(ev)
                if "af" in msg:
                    if follow >= 0:
                        af["seq"] += 1
                        af["t"] = time.monotonic()      # the master's af_done drives our refinement
                    else:
                        af_trigger(msg["af"] if isinstance(msg["af"], dict) else None)
                if "lens" in msg and math.isfinite(float(msg["lens"])):
                    lens_set(float(msg["lens"]))
                    log(f"cam{cam}: lens {float(msg['lens']):.2f} (manual)")
                if "gain_ratio" in msg and follow >= 0:
                    g = float(msg["gain_ratio"])
                    if math.isfinite(g) and 0.5 <= g <= 2.0 and abs(g - match_calib["gain_ratio"]) > 1e-3:
                        match_calib["gain_ratio"] = round(g, 4)
                        calib_dirty["dirty"] = True
                if "colour_ratio" in msg and follow >= 0:
                    r, b = (float(v) for v in msg["colour_ratio"][:2])
                    if all(math.isfinite(v) and 0.5 <= v <= 2.0 for v in (r, b)):
                        match_calib["colour_ratio"] = [round(r, 4), round(b, 4)]
                        calib_dirty["dirty"] = True
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
            if not sync_warned and enc.sync_enable and enc.frames == 0 and now - t_start > SYNC_TIMEOUT_S:
                sync_warned = True
                enc.sync_enable = False
                log(f"cam{cam}: no frame sync after {SYNC_TIMEOUT_S:.0f} s (no multicast route?): streaming UNSYNCHRONISED")
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
            master_tick()
            refine_tick()
            if follow < 0 and calib_dirty["dirty"] and now - calib_dirty["t"] > 30.0:
                calib_dirty["t"] = now
                save_match_calib()
            if now - last_write > META_WRITE_S:
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
