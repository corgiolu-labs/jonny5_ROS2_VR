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

Zoom control: UDP 127.0.0.1:(CTL_PORT_BASE + camera), JSON {"zoom": z} (1 .. ZOOM_MAX). The crop eases towards
the target (time constant ZOOM_TAU_S) so both cameras move smoothly together. The applied zoom is written to
/dev/shm/jonny5_cam{N}_zoom.json ({"zoom", "target", "ts"}) for the collimator and ws-teleop.
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
from libcamera import controls
from picamera2 import Picamera2
from picamera2.encoders import LibavH264Encoder
from picamera2.outputs import PyavOutput
from picamera2.request import MappedArray

CTL_PORT_BASE = 5600
ZOOM_MAX = 6.0
ZOOM_TAU_S = 0.15
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

    def _encode(self, stream, request):
        # Same as LibavH264Encoder._encode, but a forced keyframe uses PictureType.I: Picamera2 0.3.34 assigns
        # the string "I", which PyAV 14 rejects (TypeError) and kills the capture.
        request.acquire()
        self._request_release_queue.append(request)
        timestamp_us = self._timestamp(request)
        with MappedArray(request, stream) as m:
            frame = av.VideoFrame.from_numpy_buffer(m.array, format=self._av_input_format, width=self.width)
            frame.pts = timestamp_us
            if self._key_frames_requested > self._key_frames_generated:
                self._key_frames_generated = self._key_frames_requested
                frame.pict_type = av.video.frame.PictureType.I
            for packet in self._stream.encode(frame):
                self._lasttimestamp = (time.monotonic_ns(), packet.pts)
                self.outputframe(bytes(packet), packet.is_keyframe, timestamp=packet.pts, packet=packet)
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
    cfg = picam2.create_video_configuration(
        main={"size": (args.width, args.height), "format": "YUV420"},
        sensor={"output_size": (sw, sh), "bit_depth": 10},
        controls=ctrl, buffer_count=6)
    picam2.configure(cfg)

    # crop limits of this sensor mode (sensor pixel array coordinates)
    smin, smax, _sdef = picam2.camera_controls["ScalerCrop"]
    full = smax  # (x, y, w, h)
    log(f"cam{cam}: sensor mode {picam2.camera_configuration()['sensor']}, crop max {full}, min {smin}")

    def crop_for(z: float):
        z = max(1.0, min(ZOOM_MAX, z))
        w = int(round(full[2] / z)) & ~1
        h = int(round(w * args.height / args.width)) & ~1
        h = min(h, full[3])
        x = full[0] + (full[2] - w) // 2
        y = full[1] + (full[3] - h) // 2
        return (x & ~1, y & ~1, w, h)

    state = {"target": max(1.0, min(ZOOM_MAX, args.zoom)), "zoom": max(1.0, min(ZOOM_MAX, args.zoom))}
    picam2.set_controls({"ScalerCrop": crop_for(state["zoom"])})

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
        g = int(round(ts / period_ns)) // max(1, args.intra)
        if g != gop["last"]:
            gop["last"] = g
            enc.force_key_frame()

    picam2.pre_callback = align_keyframes
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
    picam2.start_encoder(enc, out)
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
                           "ts": round(time.time(), 3)}, f)
            os.replace(tmp_path, state_path)
        except OSError:
            pass

    write_state()
    last = time.monotonic()
    last_applied = state["zoom"]
    last_write = 0.0
    try:
        while not stop.is_set():
            if fatal["err"] is not None:
                raise SystemExit(2)
            try:
                data, _ = sock.recvfrom(512)
                msg = json.loads(data.decode("utf-8", "replace"))
                if "zoom" in msg:
                    z = float(msg["zoom"])
                    if math.isfinite(z):
                        state["target"] = max(1.0, min(ZOOM_MAX, z))
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
            if abs(state["zoom"] - last_applied) >= 0.002:
                picam2.set_controls({"ScalerCrop": crop_for(state["zoom"])})
                last_applied = state["zoom"]
            if now - last_write > 0.1 or state["zoom"] != last_applied:
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
