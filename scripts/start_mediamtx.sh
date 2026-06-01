#!/bin/bash
# JONNY5 — Avvia MediaMTX con profilo video selezionato in config_runtime/video/.
# Usato da jonny5-mediamtx.service. Richiede video_pipeline: webrtc.
#
# Selezione profilo:
#   video_pipeline.yaml --> "video_profile: <name>" (default: lowlatency)
#   Profili disponibili (file mediamtx_<name>.yml accanto a video_pipeline.yaml):
#     lowlatency      800x450  @ 120 FPS  ~3 Mbps   (baseline VR teleop)
#     zoomfriendly   1280x720  @  60 FPS  ~6 Mbps   (compromesso latenza/zoom)
#     inspection     1920x1080 @  30 FPS  ~8 Mbps   (ispezione, no VR teleop)
#     maxres         4608x2592 @  14 FPS ~25 Mbps   (max sensore, no VR, vr-live)
#
#   Fallback: se video_profile non e' specificato o il file del profilo manca,
#   carica il legacy mediamtx.yml (presente accanto ai mediamtx_*.yml).

set -e
RASPBERY5_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME_VIDEO_DIR="$RASPBERY5_ROOT/config_runtime/video"
CONFIG="$RUNTIME_VIDEO_DIR/video_pipeline.yaml"

if [ ! -f "$CONFIG" ]; then
  echo "ERR: video_pipeline config non trovata (required): $CONFIG" >&2
  exit 1
fi
if grep -q "video_pipeline: mjpeg" "$CONFIG"; then
  echo "ERR: video_pipeline è mjpeg; stack operativo è solo low-latency (webrtc). Imposta video_pipeline: webrtc in $CONFIG" >&2
  exit 1
fi
if ! grep -q "video_pipeline: webrtc" "$CONFIG"; then
  echo "ERR: richiesto video_pipeline: webrtc in $CONFIG" >&2
  exit 1
fi

# Profilo: estraggo "video_profile: <name>" da video_pipeline.yaml. Whitelist.
VIDEO_PROFILE="$(grep -E '^[[:space:]]*video_profile:' "$CONFIG" \
                 | sed -E 's/^[[:space:]]*video_profile:[[:space:]]*//; s/[[:space:]]+$//' \
                 | head -1)"
case "$VIDEO_PROFILE" in
  lowlatency|zoomfriendly|inspection|maxres|initial) ;;  # ok
  "") VIDEO_PROFILE="lowlatency" ;;               # default
  *)
    echo "WARN: video_profile='$VIDEO_PROFILE' non riconosciuto, fallback a lowlatency." >&2
    VIDEO_PROFILE="lowlatency"
    ;;
esac

PROFILE_YML="$RUNTIME_VIDEO_DIR/mediamtx_${VIDEO_PROFILE}.yml"
if [ -f "$PROFILE_YML" ]; then
  MEDIAMTX_YML="$PROFILE_YML"
elif [ -f "$RUNTIME_VIDEO_DIR/mediamtx.yml" ]; then
  echo "WARN: profilo '$VIDEO_PROFILE' non trovato ($PROFILE_YML), uso legacy mediamtx.yml." >&2
  MEDIAMTX_YML="$RUNTIME_VIDEO_DIR/mediamtx.yml"
else
  echo "ERR: nessun MediaMTX config disponibile (cercato $PROFILE_YML e mediamtx.yml)." >&2
  exit 1
fi
echo "Using MediaMTX profile: $VIDEO_PROFILE ($MEDIAMTX_YML)"

MEDIAMTX_BIN=""
if [ -x "/home/jonny5/mediamtx" ]; then
  MEDIAMTX_BIN="/home/jonny5/mediamtx"
elif [ -x "/home/jonny5/mediamtx/mediamtx" ]; then
  MEDIAMTX_BIN="/home/jonny5/mediamtx/mediamtx"
elif command -v mediamtx >/dev/null 2>&1; then
  MEDIAMTX_BIN="mediamtx"
else
  echo "ERR: MediaMTX non trovato." >&2
  exit 1
fi

exec "$MEDIAMTX_BIN" "$MEDIAMTX_YML"
