#!/bin/bash
# JONNY5 — DIRECT video path for the headset: rpicam-vid -> RTP/UDP -> viewer, no MediaMTX.
#
# Same cameras, same software H.264 encoder and same profile as mediamtx_lowlatency.yml
# (800x450 @ 120 fps, 3 Mbit/s, IDR every 20 frames, denoise off, continuous AF); the only
# difference is the relay: MediaMTX is stopped and each camera is packetized by
# raspberry/tools/rtp_push.py straight to the viewer's UDP ports (5000 cam0, 5002 cam1).
# The viewer app must be in "udp" mode (host field = udp) on the given IP.
#
# Usage: scripts/video_direct.sh <viewer-ip> [WIDTH HEIGHT FPS BITRATE]
#   e.g. scripts/video_direct.sh 10.42.0.10            # hotspot client
#        scripts/video_direct.sh 192.168.137.10 1280 720 60 6000000
# Ctrl+C (or SIGTERM) stops both cameras and restarts jonny5-mediamtx.
set -u
VIEWER_IP="${1:?viewer ip required (the headset / phone)}"
W="${2:-800}"; H="${3:-450}"; FPS="${4:-120}"; BITRATE="${5:-3000000}"
INTRA="${INTRA:-20}"
# Optional per-camera extra rpicam-vid args (latency measurement: frame-number overlay + metadata):
#   CAM0_EXTRA="--post-process-file /tmp/annotate.json --metadata /tmp/meta_cam0.json"
declare -A CAM_EXTRA=([0]="${CAM0_EXTRA:-}" [1]="${CAM1_EXTRA:-}")
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PUSH="$REPO_ROOT/raspberry/tools/rtp_push.py"
[ -f "$PUSH" ] || { echo "ERR: $PUSH not found" >&2; exit 1; }
command -v rpicam-vid >/dev/null || { echo "ERR: rpicam-vid not found" >&2; exit 1; }
STDBUF=""; command -v stdbuf >/dev/null && STDBUF="stdbuf -o0"

PIDS=()
cleanup() {
  echo "video_direct: stopping cameras, restoring jonny5-mediamtx"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f "rtp_push.py --host $VIEWER_IP" 2>/dev/null
  sleep 1
  sudo -n systemctl start jonny5-mediamtx 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# MediaMTX owns the CSI cameras while it runs.
sudo -n systemctl stop jonny5-mediamtx 2>/dev/null || true
pkill -f "mediamtx.*mediamtx" 2>/dev/null || true
sleep 1

start_cam() {
  local cam="$1" port="$2"
  # --inline: SPS/PPS before every IDR; --flush: one write per frame (rtp_push relies on it)
  # stdbuf -o0: one write() per encoded frame on the pipe (no stdio 8 KB chunking), so rtp_push
  # gets whole frames without waiting for a quiet gap.
  $STDBUF rpicam-vid --camera "$cam" -t 0 -n \
      --width "$W" --height "$H" --framerate "$FPS" \
      --codec h264 --libav-format h264 --profile baseline --level 4.2 \
      --bitrate "$BITRATE" --intra "$INTRA" --inline --flush \
      --denoise off --autofocus-mode continuous --autofocus-range normal \
      ${CAM_EXTRA[$cam]:-} \
      -o - 2>"/tmp/video_direct_cam$cam.log" \
    | python3 "$PUSH" --host "$VIEWER_IP" --port "$port" &
  PIDS+=($!)
  echo "cam$cam -> udp://$VIEWER_IP:$port  (${W}x${H} @ ${FPS} fps, $BITRATE bit/s, IDR every $INTRA)"
}

start_cam 0 5000
start_cam 1 5002
echo "video_direct running. Logs: /tmp/video_direct_cam0.log, /tmp/video_direct_cam1.log. Ctrl+C to stop."
wait
