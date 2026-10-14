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
# Same knobs as video_direct.sh (SYNC, MATCH, PROBE_S, SHUTTER/GAIN/AWBGAINS/LENSPOS, ROI, INTRA).
#
# Usage: scripts/video_capture.sh [WIDTH HEIGHT FPS BITRATE]   (run by jonny5-video-capture.service)
set -u
W="${1:-800}"; H="${2:-450}"; FPS="${3:-120}"; BITRATE="${4:-3000000}"
INTRA="${INTRA:-20}"
MTX="${MTX:-rtsp://127.0.0.1:8554}"
SYNC="${SYNC:-1}"
MATCH="${MATCH:-cam0}"
PROBE_S="${PROBE_S:-4}"
command -v rpicam-vid >/dev/null || { echo "ERR: rpicam-vid not found" >&2; exit 1; }

# wait for MediaMTX to accept publishers
for i in $(seq 1 30); do
  if (exec 3<>/dev/tcp/127.0.0.1/8554) 2>/dev/null; then exec 3>&-; break; fi
  [ "$i" = 30 ] && { echo "ERR: MediaMTX not listening on 8554" >&2; exit 1; }
  sleep 1
done

MANUAL_ARGS=""
case "$MATCH" in
  cam0)
    echo "probing cam0 auto exposure / AWB / AF for ${PROBE_S}s..."
    rm -f /tmp/probe_cam0.json
    rpicam-vid --camera 0 -t "$((PROBE_S * 1000))" -n --width "$W" --height "$H" --framerate "$FPS" \
        --codec h264 --libav-format h264 --bitrate 1000000 --denoise off \
        --autofocus-mode continuous --autofocus-range normal \
        --metadata /tmp/probe_cam0.json --save-pts /tmp/probe_cam0_pts.txt -o /tmp/probe_cam0.h264 2>/dev/null
    MANUAL_ARGS="$(python3 - <<'PY'
import json
raw = open("/tmp/probe_cam0.json").read().strip()
if not raw.endswith("]"):
    raw = raw.rstrip(",") + "]"
e = json.loads(raw)[-1]
cg = e.get("ColourGains", [1.0, 1.0])
print("--shutter %d --gain %.3f --awbgains %.3f,%.3f --autofocus-mode manual --lens-position %.2f" % (
    int(e["ExposureTime"]), float(e["AnalogueGain"]) * float(e.get("DigitalGain", 1.0)), cg[0], cg[1],
    float(e.get("LensPosition", 1.0))))
PY
)" || MANUAL_ARGS=""
    echo "cam0 converged to: ${MANUAL_ARGS:-(probe failed, cameras left on auto)}"
    [ -z "$MANUAL_ARGS" ] && MANUAL_ARGS="--autofocus-mode continuous --autofocus-range normal"
    ;;
  manual)
    MANUAL_ARGS="--shutter ${SHUTTER:?} --gain ${GAIN:?} --awbgains ${AWBGAINS:?} --autofocus-mode manual --lens-position ${LENSPOS:?}"
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
