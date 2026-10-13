#!/usr/bin/env python3
"""stamped_h264_source.py — sorgente H264 *timbrata* per il glass-to-glass WebRTC.

Cattura con picamera2, disegna lo stesso timbro temporale di stamped_mjpeg_source.py
(24 blocchi + 2 guard, tempo di cattura in Pi epoch ms low-24bit), poi passa i frame
raw a ffmpeg che li encoda in **H264 software** (libx264, zerolatency — il Pi 5 non ha
encoder H264 HW, quindi anche MediaMTX usa software: libx264 è rappresentativo) e li
**pubblica in RTSP** su una path MediaMTX di test (cam_g2g), che MediaMTX serve in
WebRTC/WHEP. Il browser legge il timbro dai frame del <video> e calcola la latenza.

Uso: python3 stamped_h264_source.py WIDTH HEIGHT FPS [BITRATE] [URL]
  URL rtsp://...  -> publish su MediaMTX (default rtsp://127.0.0.1:8554/cam_g2g)
  URL rtp://IP:P  -> RTP/UDP diretto al viewer (jonny5-quest-viewer in modo udp), senza MediaMTX:
                     stessa cattura e stesso encoder, cambia solo il relay -> A/B pulito sulla latenza.
"""
import signal
import subprocess
import sys
import time

import numpy as np
from picamera2 import Picamera2

BITS = 24
BS = 16
GUARD = 2

_picam = None
_ff = None


def _draw_stamp(img, value):
    for i in range(BITS):
        img[0:BS, i * BS:(i + 1) * BS, :] = 255 if ((value >> i) & 1) else 0
    img[0:BS, BITS * BS:(BITS + 1) * BS, :] = 255
    img[0:BS, (BITS + 1) * BS:(BITS + 2) * BS, :] = 0


def _cleanup(*_a):
    global _picam, _ff
    try:
        if _ff is not None and _ff.stdin:
            _ff.stdin.close()
    except Exception:
        pass
    try:
        if _picam is not None:
            _picam.stop()
    except Exception:
        pass
    try:
        if _ff is not None:
            _ff.terminate()
    except Exception:
        pass
    sys.exit(0)


def main():
    global _picam, _ff
    W = int(sys.argv[1]); H = int(sys.argv[2]); FPS = float(sys.argv[3])
    bitrate = sys.argv[4] if len(sys.argv) > 4 else "3M"
    url = sys.argv[5] if len(sys.argv) > 5 else "rtsp://127.0.0.1:8554/cam_g2g"
    if url.startswith("rtp://"):
        out_args = ["-f", "rtp", "-payload_type", "96", url]           # diretto al viewer, no server
    else:
        out_args = ["-f", "rtsp", "-rtsp_transport", "udp", url]       # publish su MediaMTX (rtspTransports: [udp])

    _picam = Picamera2()
    cfg = _picam.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},  # array in ordine BGR (quirk picamera2)
        controls={"FrameRate": FPS},
    )
    _picam.configure(cfg)
    _picam.start()

    # Encoder knobs for latency sweeps (env): STAMP_GOP (default 30), STAMP_X264 (x264-params string,
    # e.g. "intra-refresh=1:sliced-threads=1"), STAMP_PRESET (default ultrafast).
    import os as _os
    gop = _os.environ.get("STAMP_GOP", "30")
    preset = _os.environ.get("STAMP_PRESET", "ultrafast")
    x264 = _os.environ.get("STAMP_X264", "")
    enc = ["-c:v", "libx264", "-preset", preset, "-tune", "zerolatency",
           "-b:v", bitrate, "-maxrate", bitrate, "-bufsize", "200k", "-pix_fmt", "yuv420p", "-g", gop]
    if x264:
        enc += ["-x264-params", x264]
    _ff = subprocess.Popen(
        ["ffmpeg", "-hide_banner", "-loglevel", "error",
         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-r", str(int(FPS)), "-i", "-",
         *enc, *out_args],
        stdin=subprocess.PIPE,
    )
    signal.signal(signal.SIGTERM, _cleanup)
    signal.signal(signal.SIGINT, _cleanup)

    # STAMP_SENSOR=1: stamp the libcamera SensorTimestamp (start of sensor readout, CLOCK_BOOTTIME)
    # converted to wall time, instead of the capture-callback time. The difference between the two
    # modes is the sensor+ISP+queue delay, identical for rpicam-vid (same libcamera stack).
    use_sensor = _os.environ.get("STAMP_SENSOR", "0") == "1"
    boot_epoch = time.time() - time.clock_gettime(time.CLOCK_BOOTTIME)
    try:
        while True:
            if use_sensor:
                req = _picam.capture_request()
                arr = req.make_array("main")
                sensor_ns = req.get_metadata().get("SensorTimestamp")
                req.release()
                if sensor_ns is None:
                    continue
                ts = round((boot_epoch + sensor_ns / 1e9) * 1000) & 0xFFFFFF
            else:
                arr = _picam.capture_array("main")
                ts = round(time.time() * 1000) & 0xFFFFFF
            if arr is None or arr.ndim != 3 or arr.shape[1] < (BITS + GUARD) * BS:
                continue
            _draw_stamp(arr, ts)
            try:
                _ff.stdin.write(np.ascontiguousarray(arr).tobytes())
            except (BrokenPipeError, OSError):
                break
    finally:
        _cleanup()


if __name__ == "__main__":
    main()
