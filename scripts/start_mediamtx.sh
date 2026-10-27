#!/bin/bash
# JONNY5 — starts MediaMTX with raspberry/config_runtime/video/mediamtx_relay.yml (jonny5-mediamtx.service).
# The cameras are published by scripts/video_capture.sh; MediaMTX only relays cam0/cam1 (RTSP/UDP to the native
# app, WebRTC/WHEP to the dashboard). Until 27/10/2026 this script chose among six "profiles" and a legacy
# mediamtx.yml that no longer exist: video_pipeline.yaml stays as the switch (video_pipeline: webrtc).
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_VIDEO_DIR=""
for _cand in "$SCRIPT_DIR/../raspberry/config_runtime/video" "$SCRIPT_DIR/../config_runtime/video"; do
  if [ -f "$_cand/mediamtx_relay.yml" ]; then RUNTIME_VIDEO_DIR="$(cd "$_cand" && pwd)"; break; fi
done
if [ -z "$RUNTIME_VIDEO_DIR" ]; then
  echo "ERR: config_runtime/video/mediamtx_relay.yml non trovato (cercato accanto a $SCRIPT_DIR)" >&2
  exit 1
fi
CONFIG="$RUNTIME_VIDEO_DIR/video_pipeline.yaml"
if [ -f "$CONFIG" ] && ! grep -q "video_pipeline: webrtc" "$CONFIG"; then
  echo "ERR: richiesto video_pipeline: webrtc in $CONFIG" >&2
  exit 1
fi
MEDIAMTX_YML="$RUNTIME_VIDEO_DIR/mediamtx_relay.yml"
echo "Using MediaMTX config: $MEDIAMTX_YML"

MEDIAMTX_BIN=""
for _b in "$HOME/mediamtx" "$HOME/mediamtx/mediamtx"; do
  if [ -x "$_b" ]; then MEDIAMTX_BIN="$_b"; break; fi
done
if [ -z "$MEDIAMTX_BIN" ] && command -v mediamtx >/dev/null 2>&1; then MEDIAMTX_BIN="mediamtx"; fi
if [ -z "$MEDIAMTX_BIN" ]; then
  echo "ERR: MediaMTX non trovato (cercato \$HOME/mediamtx e il PATH)." >&2
  exit 1
fi
exec "$MEDIAMTX_BIN" "$MEDIAMTX_YML"
