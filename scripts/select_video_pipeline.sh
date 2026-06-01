#!/bin/bash
# JONNY5 — Boot video low-latency only: imposta video_pipeline: webrtc e avvia MediaMTX (jonny5-mediamtx).
# Non esiste più selezione MJPEG / dual-mode nel repo.
set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME_CONFIG="$REPO_ROOT/config/runtime/video/video_pipeline.yaml"
if [ ! -f "$RUNTIME_CONFIG" ]; then
  echo "ERR: video_pipeline config non trovata (required): $RUNTIME_CONFIG" >&2
  exit 1
fi

echo "Pipeline: webrtc (MediaMTX low-latency, unico path supportato)"

# Spegni stack legacy MJPEG se ancora presente sul device (no-op se unit assente)
sudo systemctl stop jonny5-webrtc 2>/dev/null || true
sudo systemctl stop jonny5-mediamtx 2>/dev/null || true
pkill -f "mediamtx.*mediamtx\\.yml" 2>/dev/null || true
sleep 2

if [ -d "$(dirname "$RUNTIME_CONFIG")" ]; then
  echo "video_pipeline: webrtc" > "$RUNTIME_CONFIG"
fi

sudo systemctl start jonny5-mediamtx
