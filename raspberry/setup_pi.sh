#!/usr/bin/env bash
# JONNY5 minimal stack on the Raspberry Pi 5 (Raspberry Pi OS / Debian, 64-bit). Run from the repo checkout
# as the user that will own the services; idempotent, re-run after a git pull.
#
#   raspberry/setup_pi.sh            # everything below
#
# 1. hardware: SPI, 2x IMX708 cameras, UART0 for the STM32 (config.txt, reboot needed the first time)
# 2. eth0: .221 static on the PC (ICS) / DHCP on the home router, switched by j5-net-autoswitch
# 3. wlan0: 5 GHz hotspot for the headset (10.42.0.1) + firewall "video out, commands in"
# 4. j5mini: Python deps, TLS pair, j5mini.json (from the example if missing)
# 5. services: j5mini, MediaMTX, camera capture, sync route; optional: stereo collimator (on by default) and
#    vision (on when raspberry/.venv-vision and raspberry/models/yolo11n.onnx exist); the old stack (ws-teleop,
#    SPI bridge, ROS 2, dashboards, captive portal) stopped and disabled
#
# Env: J5_HOTSPOT_SSID, J5_HOTSPOT_PSK (8..63 chars), J5_HOTSPOT_CHANNEL; J5_SKIP_NET=1 skips 2 and 3;
#      J5_COLLIMATOR=0 / J5_VISION=0 leave the optional services off.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
RUN_USER="$(id -un)"
NET_DIR="$REPO/raspberry/network"

echo "[1/5] Hardware interfaces (SPI + cameras + UART)..."
sudo raspi-config nonint do_spi 0 2>/dev/null || true
sudo raspi-config nonint do_serial_cons 1 2>/dev/null || true       # no login console on the STM32 UART
CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
if [ ! -f "$CFG" ]; then
  echo "    WARN: config.txt not found, configure cameras/UART manually."
elif grep -q "JONNY5 cameras + UART" "$CFG"; then
  echo "    cameras + UART already in $CFG"
else
  sudo tee -a "$CFG" >/dev/null <<'CFG_BLOCK'
# === JONNY5 cameras + UART (added by raspberry/setup_pi.sh) ===
[all]
dtparam=spi=on
camera_auto_detect=1
dtoverlay=imx708,cam0
dtoverlay=imx708,cam1
dtparam=uart0=on
enable_uart=1
dtoverlay=uart0-pi5
CFG_BLOCK
  echo "    added to $CFG (reboot required)"
fi

if [ "${J5_SKIP_NET:-0}" != "1" ]; then
  echo "[2/5] eth0: .221 on the PC (ICS) / DHCP on the router..."
  sudo nmcli con delete "Wired connection 1" >/dev/null 2>&1 || true
  sudo nmcli con delete "eth0-ics-static" >/dev/null 2>&1 || true
  sudo nmcli con add type ethernet ifname eth0 con-name "eth0-ics-static" \
    ipv4.method manual ipv4.addresses "192.168.137.221/24" ipv4.gateway "192.168.137.1" \
    ipv4.dns "192.168.137.1 8.8.8.8" ipv4.route-metric 100 connection.autoconnect no >/dev/null
  sudo nmcli con delete "eth0-dhcp" >/dev/null 2>&1 || true
  sudo nmcli con add type ethernet ifname eth0 con-name "eth0-dhcp" \
    ipv4.method auto ipv4.route-metric 50 connection.autoconnect yes >/dev/null
  sudo install -m 755 "$NET_DIR/j5_net_autoswitch.sh" /usr/local/sbin/j5_net_autoswitch.sh
  sudo install -m 644 "$NET_DIR/j5-net-autoswitch.service" /etc/systemd/system/j5-net-autoswitch.service

  echo "[3/5] wlan0: headset hotspot (5 GHz, 10.42.0.1) + firewall..."
  # One radio hop from the headset to the robot. 5 GHz non-DFS channel 36, 80 MHz, power save off, WPA2.
  # An older profile (jonny5-ap) keeps its SSID and password so known headsets reconnect.
  if nmcli -t -f NAME con show 2>/dev/null | grep -qx "jonny5-ap"; then
    : "${J5_HOTSPOT_SSID:=$(nmcli -g 802-11-wireless.ssid con show jonny5-ap 2>/dev/null || true)}"
    : "${J5_HOTSPOT_PSK:=$(sudo nmcli -s -g 802-11-wireless-security.psk con show jonny5-ap 2>/dev/null || true)}"
    sudo nmcli con delete jonny5-ap >/dev/null 2>&1 || true
  fi
  SSID="${J5_HOTSPOT_SSID:-JONNY5}"
  PSK="${J5_HOTSPOT_PSK:-jonny5robot}"
  sudo raspi-config nonint do_wifi_country IT 2>/dev/null || sudo iw reg set IT || true
  sudo rfkill unblock wlan 2>/dev/null || true
  sudo nmcli con delete "jonny5-hotspot" >/dev/null 2>&1 || true
  sudo nmcli con add type wifi ifname wlan0 con-name "jonny5-hotspot" ssid "$SSID" \
    wifi.mode ap wifi.band a wifi.channel "${J5_HOTSPOT_CHANNEL:-36}" wifi.channel-width 80mhz wifi.powersave 2 \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.psk "$PSK" \
    ipv4.method shared ipv4.addresses "10.42.0.1/24" ipv6.method disabled \
    connection.autoconnect yes connection.autoconnect-priority 100 >/dev/null
  sudo rm -f /etc/NetworkManager/dnsmasq-shared.d/jonny5-captive-portal.conf     # captive portal removed
  command -v nft >/dev/null 2>&1 || sudo apt-get install -y nftables
  sudo install -m 644 "$NET_DIR/j5-wlan0.nft" /etc/j5-wlan0.nft
  sudo install -m 644 "$NET_DIR/j5-wlan0-firewall.service" /etc/systemd/system/j5-wlan0-firewall.service
  sudo install -d /etc/systemd/system/jonny5-mediamtx.service.d
  sudo install -m 644 "$NET_DIR/jonny5-mediamtx-network-online.conf" \
    /etc/systemd/system/jonny5-mediamtx.service.d/10-network-online.conf
fi

echo "[4/5] j5mini: dependencies, TLS, config..."
sudo apt-get install -y python3-websockets python3-spidev python3-serial python3-av python3-numpy >/dev/null
TLS="$REPO/raspberry/config_runtime/tls"
if [ ! -f "$TLS/webrtc.crt" ]; then
  mkdir -p "$TLS"
  openssl req -x509 -newkey rsa:2048 -nodes -days 3650 -subj "/CN=jonny5" \
    -addext "subjectAltName=IP:10.42.0.1,IP:192.168.137.221" \
    -keyout "$TLS/webrtc.key" -out "$TLS/webrtc.crt" 2>/dev/null
  echo "    self-signed TLS pair created in $TLS"
fi
J5CFG="$REPO/minimal/j5mini/j5mini.json"
if [ ! -f "$J5CFG" ]; then
  cp "$REPO/minimal/j5mini/j5mini.example.json" "$J5CFG"
  echo "    WARN: $J5CFG created from the example: offsets/limits are the repo defaults."
  echo "          With the old stack still installed, export the calibrated ones instead (minimal/j5mini/README.md)."
fi

echo "[5/5] Services..."
OLD="jonny5-ws-teleop jonny5-spi-j5vr jonny5-spi-shm-bridge jonny5-ros2 jonny5-https jonny5-https-443-proxy
     jonny5-captive-portal jonny5-refocus-cams jonny5-refocus-cams.timer jonny5-video-init jonny5-video-fallback.timer"
for u in $OLD; do
  sudo systemctl disable --now "$u" >/dev/null 2>&1 || true
done
for f in "$REPO"/raspberry/systemd/*.service; do
  sed -e "s|@REPO@|$REPO|g" -e "s|@USER@|$RUN_USER|g" "$f" | sudo tee "/etc/systemd/system/$(basename "$f")" >/dev/null
done
sudo systemctl daemon-reload
EN="jonny5-sync-route jonny5-mediamtx jonny5-video-capture jonny5-j5mini"
[ "${J5_SKIP_NET:-0}" = "1" ] || EN="$EN j5-net-autoswitch j5-wlan0-firewall"
# optional services (j5mini relays their messages to the app)
if [ "${J5_COLLIMATOR:-1}" = "1" ]; then
  EN="$EN jonny5-stereo-collimator"
else
  sudo systemctl disable --now jonny5-stereo-collimator >/dev/null 2>&1 || true
fi
if [ "${J5_VISION:-1}" = "1" ] && [ -x "$REPO/raspberry/.venv-vision/bin/python" ]    && [ -f "$REPO/raspberry/models/yolo11n.onnx" ]; then
  EN="$EN jonny5-vision"
else
  sudo systemctl disable --now jonny5-vision >/dev/null 2>&1 || true
  if [ "${J5_VISION:-1}" = "1" ]; then
    echo "    vision off: raspberry/.venv-vision or raspberry/models/yolo11n.onnx missing"
  fi
fi
sudo systemctl enable $EN >/dev/null
sudo systemctl restart $EN || true
[ "${J5_SKIP_NET:-0}" = "1" ] || sudo nmcli con up jonny5-hotspot >/dev/null 2>&1 || true

echo "Done. Check:"
echo "    systemctl --no-pager status jonny5-j5mini jonny5-video-capture"
echo "    journalctl -u jonny5-j5mini -f        # BOOT_READY, config lines OK"
echo "    ls -l /dev/spidev0.0 /dev/serial0; rpicam-hello --list-cameras"
echo "    Headset: join the hotspot, app host 10.42.0.1:8554"
