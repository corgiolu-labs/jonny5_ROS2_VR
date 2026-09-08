#!/bin/bash
# g2g_supervisor.sh — supervisor a vita limitata per la misura glass-to-glass WebRTC
# (Idea 1, Stadio 3c). Lanciato detached da https_server (/api/webrtc-g2g-start).
#
# Ferma prod mediamtx (libera la camera CSI), avvia un mediamtx di TEST (mediamtx_g2g.yml,
# no rpiCamera) + il publisher H264 timbrato (stamped_h264_source.py → RTSP cam_g2g).
# Dopo WINDOW secondi (o se ucciso) ripristina prod mediamtx via trap EXIT.
# L'auto-scadenza è la sicurezza: anche se il browser non chiude mai, il video di
# produzione torna su da solo.
#
# Uso: g2g_supervisor.sh [W] [H] [FPS] [WINDOW_S]
W=${1:-800}; H=${2:-450}; F=${3:-30}; WINDOW=${4:-45}
MTX=/home/jonny5ros2/mediamtx
CFG=/home/jonny5ros2/JONNY5_ROS2/raspberry/config_runtime/video/mediamtx_g2g.yml
PUB=/home/jonny5ros2/JONNY5_ROS2/raspberry/controller/web_services/stamped_h264_source.py

# Uccidi eventuali istanze stale di una run precedente.
pkill -f stamped_h264_source.py 2>/dev/null
pkill -f mediamtx_g2g.yml 2>/dev/null
sleep 1

cleanup() {
  [ -n "$PUB_PID" ] && kill "$PUB_PID" 2>/dev/null
  [ -n "$MTX_PID" ] && kill "$MTX_PID" 2>/dev/null
  sleep 1
  sudo -n systemctl start jonny5-mediamtx
}
trap cleanup EXIT

sudo -n systemctl stop jonny5-mediamtx
sleep 2
"$MTX" "$CFG" >/tmp/g2g_mtx.log 2>&1 &
MTX_PID=$!
sleep 2
python3 "$PUB" "$W" "$H" "$F" 3M >/tmp/g2g_pub.log 2>&1 &
PUB_PID=$!
sleep "$WINDOW"
