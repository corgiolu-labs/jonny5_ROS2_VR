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

# MediaMTX owns the CSI cameras while it runs.
sudo -n systemctl stop jonny5-mediamtx 2>/dev/null || true
pkill -f "mediamtx.*mediamtx" 2>/dev/null || true
sleep 1

# Stereo consistency (note: rpicam-vid writes --metadata entries only together with --save-pts and a file output):
#   SYNC=1 (default)  libcamera frame sync: cam0 = server, cam1 = client (exposures aligned to ~us).
#   MATCH=cam0 (default) probe cam0 in auto mode for a few seconds, then run BOTH cameras with the
#                     converged exposure / gain / AWB gains / lens position as fixed manual values.
#   MATCH=manual      use SHUTTER (us) GAIN AWBGAINS ("r,b") LENSPOS (dioptres) from the environment.
#   MATCH=auto        leave every camera on its own auto loops (old behaviour).
#   ROI="x,y,w,h"     identical sensor crop (zoom) on both cameras, fractions 0..1.
SYNC="${SYNC:-1}"
MATCH="${MATCH:-cam0}"
PROBE_S="${PROBE_S:-4}"
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
entries = json.loads(raw)
e = entries[-1]
cg = e.get("ColourGains", [1.0, 1.0])
print("--shutter %d --gain %.3f --awbgains %.3f,%.3f --autofocus-mode manual --lens-position %.2f" % (
    int(e["ExposureTime"]), float(e["AnalogueGain"]) * float(e.get("DigitalGain", 1.0)), cg[0], cg[1],
    float(e.get("LensPosition", 1.0))))
PY
)"
    echo "cam0 converged to: $MANUAL_ARGS"
    ;;
  manual)
    MANUAL_ARGS="--shutter ${SHUTTER:?} --gain ${GAIN:?} --awbgains ${AWBGAINS:?} --autofocus-mode manual --lens-position ${LENSPOS:?}"
    ;;
  auto)
    MANUAL_ARGS="--autofocus-mode continuous --autofocus-range normal"
    ;;
esac
[ -n "${ROI:-}" ] && MANUAL_ARGS="$MANUAL_ARGS --roi $ROI"
declare -A SYNC_ARGS=([0]="" [1]="")
if [ "$SYNC" = "1" ]; then SYNC_ARGS[0]="--sync server"; SYNC_ARGS[1]="--sync client"; fi
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PUSH="$REPO_ROOT/raspberry/tools/rtp_push.py"
[ -f "$PUSH" ] || { echo "ERR: $PUSH not found" >&2; exit 1; }
# C packetizer (same behaviour, ~10x less CPU): built on demand from rtp_push.c, python fallback.
PUSH_BIN="$REPO_ROOT/raspberry/tools/rtp_push"
if [ -f "$REPO_ROOT/raspberry/tools/rtp_push.c" ] && { [ ! -x "$PUSH_BIN" ] || [ "$REPO_ROOT/raspberry/tools/rtp_push.c" -nt "$PUSH_BIN" ]; }; then
  gcc -O2 -o "$PUSH_BIN" "$REPO_ROOT/raspberry/tools/rtp_push.c" 2>/tmp/rtp_push_build.log || echo "WARN: rtp_push.c build failed (see /tmp/rtp_push_build.log), using python"
fi
if [ -x "$PUSH_BIN" ]; then PUSH_CMD=("$PUSH_BIN"); else PUSH_CMD=(python3 "$PUSH"); fi
command -v rpicam-vid >/dev/null || { echo "ERR: rpicam-vid not found" >&2; exit 1; }
STDBUF=""; command -v stdbuf >/dev/null && STDBUF="stdbuf -o0"

PIDS=()
cleanup() {
  echo "video_direct: stopping cameras, restoring jonny5-mediamtx"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f "rtp_push(.py)? --host $VIEWER_IP" 2>/dev/null
  sleep 1
  sudo -n systemctl start jonny5-mediamtx 2>/dev/null || true
}
trap cleanup EXIT INT TERM


start_cam() {
  local cam="$1" port="$2"
  # --inline: SPS/PPS before every IDR; --flush: one write per frame (rtp_push relies on it)
  # stdbuf -o0: one write() per encoded frame on the pipe (no stdio 8 KB chunking), so rtp_push
  # gets whole frames without waiting for a quiet gap.
  $STDBUF rpicam-vid --camera "$cam" -t 0 -n \
      --width "$W" --height "$H" --framerate "$FPS" \
      --codec h264 --libav-format h264 --profile baseline --level 4.2 \
      --bitrate "$BITRATE" --intra "$INTRA" --inline --flush \
      --denoise off $MANUAL_ARGS ${SYNC_ARGS[$cam]:-} \
      ${CAM_EXTRA[$cam]:-} \
      -o - 2>"/tmp/video_direct_cam$cam.log" \
    | "${PUSH_CMD[@]}" --host "$VIEWER_IP" --port "$port" &
  PIDS+=($!)
  echo "cam$cam -> udp://$VIEWER_IP:$port  (${W}x${H} @ ${FPS} fps, $BITRATE bit/s, IDR every $INTRA)"
}

# with SYNC the client (cam1) must be up before the server (cam0) starts broadcasting
start_cam 1 5002
sleep 1
start_cam 0 5000
echo "video_direct running. Logs: /tmp/video_direct_cam0.log, /tmp/video_direct_cam1.log. Ctrl+C to stop."
wait
