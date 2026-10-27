#!/bin/bash
# JONNY5 — camera capture (production since 2026-10-14): raspberry/tools/cam_stream.py x2 (Picamera2 -> x264)
# -> RTSP publish to rtsp://127.0.0.1:8554/{cam0,cam1} -> MediaMTX relay (scripts/start_mediamtx.sh). MediaMTX
# relays to the native app (RTSP/UDP) and to the browser viewer / dashboard (WebRTC/WHEP). Why not MediaMTX's own
# rpiCamera source: its OpenH264 encoder does not enforce the bitrate (12.6 Mbit/s in daylight, losses) and has
# no frame sync. The lowest-latency path for the headset is scripts/video_direct.sh (RTP straight to the viewer).
#
# Cameras: libcamera frame sync (cam0 server, cam1 client, ~0.3 ms). MATCH=master (default since 2026-10-19):
# cam0 runs auto exposure / white balance and one-shot AF on request, cam1 follows cam0 (cam_stream.py --follow 0:
# same shutter / gains / colour, focus cross-checked against cam0). MATCH=manual: both fixed (SHUTTER, GAIN,
# AWBGAINS). MATCH=auto: independent continuous AF / AE. Encoder x264 --low-latency, CBR BITRATE, IDR every INTRA
# frames, SPS/PPS inline; live sensor zoom (ScalerCrop) driven by j5mini over UDP (the app's zoom).
# Knobs: SYNC, MATCH (master|manual|auto), MODE (wide|tele), INTRA, SHUTTER/GAIN/AWBGAINS (manual).
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
HERE="$(cd "$(dirname "$0")/.." && pwd)"
MATCH="${MATCH:-master}"
# Sensor mode at boot: wide (66 deg, 56 fps, the operator's choice 2026-10-19) or tele (46.8 deg crop, 90 fps);
# switchable live from the app (Grandangolo button).
MODE="${MODE:-wide}"

# wait for MediaMTX to accept publishers
for i in $(seq 1 30); do
  if (exec 3<>/dev/tcp/127.0.0.1/8554) 2>/dev/null; then exec 3>&-; break; fi
  [ "$i" = 30 ] && { echo "ERR: MediaMTX not listening on 8554" >&2; exit 1; }
  sleep 1
done

MANUAL_ARGS=""
declare -A FOLLOW_ARGS=([0]="" [1]="")
case "$MATCH" in
  master)
    MANUAL_ARGS="--autofocus-mode auto"
    FOLLOW_ARGS[1]="--follow 0"
    echo "cam0 master (auto exposure / AWB, one-shot AF), cam1 follows cam0"
    ;;
  manual)
    MANUAL_ARGS="--shutter ${SHUTTER:?} --gain ${GAIN:?} --awbgains ${AWBGAINS:?}"
    ;;
  *)
    MANUAL_ARGS="--autofocus-mode continuous"
    ;;
esac
declare -A SYNC_ARGS=([0]="" [1]="")
if [ "$SYNC" = "1" ]; then SYNC_ARGS[0]="--sync server"; SYNC_ARGS[1]="--sync client"; fi

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

start_cam() {
  local cam="$1"
  python3 "$HERE/raspberry/tools/cam_stream.py" --camera "$cam" \
      --width "$W" --height "$H" --fps "$FPS" --bitrate "$BITRATE" --intra "$INTRA" --mode "$MODE" \
      $MANUAL_ARGS ${SYNC_ARGS[$cam]:-} ${FOLLOW_ARGS[$cam]:-} \
      -o "$MTX/cam$cam" >"/tmp/video_capture_cam$cam.log" 2>&1 &
  PIDS+=($!)
  echo "cam$cam -> $MTX/cam$cam  (picamera2, ${W}x${H} @ ${FPS} fps, $BITRATE bit/s, IDR every $INTRA, live zoom)"
}

# with SYNC the client (cam1) must be up before the server (cam0) starts broadcasting
start_cam 1
sleep 1
start_cam 0
echo "video_capture running (logs /tmp/video_capture_cam{0,1}.log)"
# if either camera dies, exit so systemd restarts both (they must be re-synced together)
wait -n
echo "a camera process exited, restarting both" >&2
