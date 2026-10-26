#!/bin/bash
# JONNY5 — PRODUCTION camera capture (since 2026-10-14): rpicam-vid x2 -> RTSP publish -> MediaMTX relay.
#
# MediaMTX (profile "relay") no longer opens the cameras. This script owns them and publishes cam0/cam1
# to rtsp://127.0.0.1:8554/{cam0,cam1}; MediaMTX relays to the native app (RTSP/UDP) and to the browser
# viewer / dashboard (WebRTC/WHEP). Why not MediaMTX's own rpiCamera source: its OpenH264 encoder does not
# enforce the bitrate (12.6 Mbit/s in daylight, losses) and has no frame sync.
#
# Cameras: libcamera frame sync (cam0 server, cam1 client, ~0.3 ms), exposure / gain / AWB / focus matched
# from a cam0 probe (MATCH=cam0, default) or fixed (MATCH=manual) or independent (MATCH=auto), optional
# identical ROI; encoder x264 --low-latency, CBR BITRATE, IDR every INTRA frames, SPS/PPS inline.
# Knobs: SYNC, MATCH (cam0|manual|auto), PROBE_S, FOCUS (each|shared), LENS_DELTA, SHUTTER/GAIN/AWBGAINS/LENSPOS, ROI, INTRA.
#
# Usage: scripts/video_capture.sh [WIDTH HEIGHT FPS BITRATE]   (run by jonny5-video-capture.service)
set -u
# FPS default 90 = Quest 2 display refresh: at 120 fps the headset compositor (90 Hz) back-pressures the
# decoder output and the queue fills up (measured 2026-10-17: q 31/31, decode 25-45 ms, purges every ~1 s;
# at 90 fps q 0/1, decode 3.8 ms, 0 purges). Use 120 only for a 120 Hz viewer (phone bench).
W="${1:-800}"; H="${2:-450}"; FPS="${3:-90}"; BITRATE="${4:-3000000}"
INTRA="${INTRA:-15}"   # 6 keyframes/s per camera = stereo collimation rate (was 20)
MTX="${MTX:-rtsp://127.0.0.1:8554}"
SYNC="${SYNC:-1}"
# CAPTURE=picamera2 (default): raspberry/tools/cam_stream.py, same stream + LIVE sensor zoom (ScalerCrop) driven
# by j5mini over UDP (the Quest app's zoom). CAPTURE=rpicam: the previous rpicam-vid capture (no live zoom).
CAPTURE="${CAPTURE:-picamera2}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
# MATCH=master (default since 2026-10-19): no boot probe; cam0 runs auto exposure / white balance and one-shot
# AF on request, cam1 follows cam0 (tools/cam_stream.py --follow 0: same shutter / gains / colour, focus
# cross-checked against cam0). MATCH=cam0: the old 4 s probe, then both cameras frozen (each on its own focus).
MATCH="${MATCH:-master}"
# Sensor mode at boot: wide (66 deg, 56 fps, the operator's choice 2026-10-19) or tele (46.8 deg crop, 90 fps);
# switchable live from the app (Grandangolo button).
MODE="${MODE:-wide}"
PROBE_S="${PROBE_S:-4}"
command -v rpicam-vid >/dev/null || { echo "ERR: rpicam-vid not found" >&2; exit 1; }

# wait for MediaMTX to accept publishers
for i in $(seq 1 30); do
  if (exec 3<>/dev/tcp/127.0.0.1/8554) 2>/dev/null; then exec 3>&-; break; fi
  [ "$i" = 30 ] && { echo "ERR: MediaMTX not listening on 8554" >&2; exit 1; }
  sleep 1
done

# Focus is calibrated PER CAMERA: the two Camera Module 3 actuators differ, the same LensPosition
# (dioptres) does not give the same physical focus on both. Exposure / gain / AWB come from cam0;
# each camera keeps the focus its own AF converged to. The measured offset (cam1 - cam0, dioptres) is
# logged and can be forced with LENS_DELTA (then cam1 = cam0 + LENS_DELTA, lenses move in unison if
# cam0's focus is changed later). FOCUS=shared reproduces the old behaviour (same value on both).
FOCUS="${FOCUS:-each}"
LENS_DELTA="${LENS_DELTA:-}"
MANUAL_ARGS=""
declare -A LENS_ARGS=([0]="" [1]="")
probe_cam() {  # $1 = cam id -> /tmp/probe_cam$1.json
  rm -f "/tmp/probe_cam$1.json"
  rpicam-vid --camera "$1" -t "$((PROBE_S * 1000))" -n --width "$W" --height "$H" --framerate "$FPS"       --codec h264 --libav-format h264 --bitrate 1000000 --denoise off       --autofocus-mode continuous --autofocus-range normal       --metadata "/tmp/probe_cam$1.json" --save-pts "/tmp/probe_cam$1_pts.txt" -o "/tmp/probe_cam$1.h264" 2>/dev/null
}
read_probe() {  # $1 = cam id -> "shutter gain awb_r awb_b lens"
  python3 - "$1" <<'PY'
import json, sys
raw = open("/tmp/probe_cam%s.json" % sys.argv[1]).read().strip()
if not raw.endswith("]"):
    raw = raw.rstrip(",") + "]"
e = json.loads(raw)[-1]
cg = e.get("ColourGains", [1.0, 1.0])
print("%d %.3f %.3f %.3f %.2f" % (int(e["ExposureTime"]), float(e["AnalogueGain"]) * float(e.get("DigitalGain", 1.0)),
                                 cg[0], cg[1], float(e.get("LensPosition", 1.0))))
PY
}
declare -A FOLLOW_ARGS=([0]="" [1]="")
case "$MATCH" in
  master)
    MANUAL_ARGS="--autofocus-mode auto --autofocus-range normal"
    FOLLOW_ARGS[1]="--follow 0"
    echo "cam0 master (auto exposure / AWB, one-shot AF), cam1 follows cam0"
    ;;
  cam0)
    echo "probing both cameras (auto exposure / AWB / AF) for ${PROBE_S}s..."
    probe_cam 0 & P0=$!
    probe_cam 1 & P1=$!
    wait $P0; wait $P1
    read -r SH GN AR AB L0 <<<"$(read_probe 0 2>/dev/null)"
    read -r _ _ _ _ L1 <<<"$(read_probe 1 2>/dev/null)"
    if [ -n "${SH:-}" ]; then
      MANUAL_ARGS="--shutter $SH --gain $GN --awbgains $AR,$AB"
      L1="${L1:-$L0}"
      if [ -n "$LENS_DELTA" ]; then L1="$(python3 -c "print('%.2f' % ($L0 + $LENS_DELTA))")"; fi
      if [ "$FOCUS" = "shared" ]; then L1="$L0"; fi
      LENS_ARGS[0]="--autofocus-mode manual --lens-position $L0"
      LENS_ARGS[1]="--autofocus-mode manual --lens-position $L1"
      echo "cam0: shutter $SH gain $GN awb $AR,$AB lens $L0 | cam1 lens $L1 (delta $(python3 -c "print('%+.2f' % ($L1 - $L0))") dioptres)"
    else
      echo "probe failed, cameras left on auto"
      MANUAL_ARGS="--autofocus-mode continuous --autofocus-range normal"
    fi
    ;;
  manual)
    MANUAL_ARGS="--shutter ${SHUTTER:?} --gain ${GAIN:?} --awbgains ${AWBGAINS:?}"
    L1="$(python3 -c "print('%.2f' % (${LENSPOS:?} + ${LENS_DELTA:-0}))")"
    LENS_ARGS[0]="--autofocus-mode manual --lens-position ${LENSPOS}"
    LENS_ARGS[1]="--autofocus-mode manual --lens-position $L1"
    ;;
  *)
    MANUAL_ARGS="--autofocus-mode continuous --autofocus-range normal"
    ;;
esac
[ -n "${ROI:-}" ] && MANUAL_ARGS="$MANUAL_ARGS --roi $ROI"
declare -A SYNC_ARGS=([0]="" [1]="")
if [ "$SYNC" = "1" ]; then SYNC_ARGS[0]="--sync server"; SYNC_ARGS[1]="--sync client"; fi

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

start_cam() {
  local cam="$1"
  if [ "$CAPTURE" = "picamera2" ]; then
    # same options as rpicam-vid below (LENS_ARGS is not passed there either: continuous AF on both cameras)
    python3 "$HERE/raspberry/tools/cam_stream.py" --camera "$cam"         --width "$W" --height "$H" --fps "$FPS" --bitrate "$BITRATE" --intra "$INTRA" --mode "$MODE"         $MANUAL_ARGS ${SYNC_ARGS[$cam]:-} ${FOLLOW_ARGS[$cam]:-}         -o "$MTX/cam$cam" >"/tmp/video_capture_cam$cam.log" 2>&1 &
    PIDS+=($!)
    echo "cam$cam -> $MTX/cam$cam  (picamera2, ${W}x${H} @ ${FPS} fps, $BITRATE bit/s, IDR every $INTRA, live zoom)"
    return
  fi
  rpicam-vid --camera "$cam" -t 0 -n \
      --width "$W" --height "$H" --framerate "$FPS" \
      --codec h264 --libav-format rtsp --profile baseline --level 4.2 --low-latency \
      --bitrate "$BITRATE" --intra "$INTRA" --inline --flush \
      --denoise off $MANUAL_ARGS ${SYNC_ARGS[$cam]:-} \
      -o "$MTX/cam$cam" 2>"/tmp/video_capture_cam$cam.log" &
  PIDS+=($!)
  echo "cam$cam -> $MTX/cam$cam  (${W}x${H} @ ${FPS} fps, $BITRATE bit/s, IDR every $INTRA)"
}

# with SYNC the client (cam1) must be up before the server (cam0) starts broadcasting
start_cam 1
sleep 1
start_cam 0
echo "video_capture running (logs /tmp/video_capture_cam{0,1}.log)"
# if either camera dies, exit so systemd restarts both (they must be re-synced together)
wait -n
echo "a camera process exited, restarting both" >&2
exit 1
