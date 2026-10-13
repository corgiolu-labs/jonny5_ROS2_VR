#!/usr/bin/env bash
# One-time host setup on a fresh Raspberry Pi OS Trixie / Debian 13 (64-bit).
# (Docker is installed via get.docker.com, which auto-detects the 'trixie' codename.)
# Enables SPI, cameras (2x IMX708) + STM32 UART, the eth0 .221/router autoswitch,
# the wlan0 5 GHz hotspot for the headset (10.42.0.1), and installs Docker.
# The MediaMTX/video pipeline stays native (legacy setup), unchanged here.
set -euo pipefail

echo "[1/4] Enabling hardware interfaces (SPI + cameras + UART)..."
if command -v raspi-config >/dev/null 2>&1; then
  sudo raspi-config nonint do_spi 0
else
  # Fallback for images without raspi-config.
  CFG=/boot/firmware/config.txt
  [ -f "$CFG" ] || CFG=/boot/config.txt
  if ! grep -q "^dtparam=spi=on" "$CFG"; then
    echo "dtparam=spi=on" | sudo tee -a "$CFG" >/dev/null
  fi
fi

echo "    Configuring cameras (2x IMX708) + STM32 UART in config.txt..."
# Proven hardware config, NOT handled by raspi-config: two Camera Module 3 (IMX708)
# on cam0/cam1 + UART0 on the 40-pin header for the serial link to the STM32
# (/dev/serial0 -> ttyAMA0 @115200; see raspberry/controller/uart/uart_manager.py).
CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
if [ ! -f "$CFG" ]; then
  echo "    WARN: config.txt not found — configure cameras/UART manually."
elif grep -q "JONNY5 cameras + UART" "$CFG"; then
  echo "    Cameras + UART already present in $CFG — skipping."
else
  sudo tee -a "$CFG" >/dev/null <<'CFG_BLOCK'

# === JONNY5 cameras + UART (added by host_setup_pi.sh) ===
# [all] applies to every board revision (avoids [cm4]/[pi5] scoping).
# Two Camera Module 3 (IMX708): cam0 = i2c@88000, cam1 = i2c@80000.
[all]
camera_auto_detect=1
dtoverlay=imx708,cam0
dtoverlay=imx708,cam1
# UART0 on the header for the STM32 link (serial login console stays OFF).
dtparam=uart0=on
enable_uart=1
dtoverlay=uart0-pi5
CFG_BLOCK
  echo "    Cameras (IMX708 cam0/cam1) + UART added to $CFG (reboot required)."
fi
# Ensure no serial login console grabs the STM32 UART (idempotent; console only).
sudo raspi-config nonint do_serial_cons 1 2>/dev/null || true

echo "[2/4] Configuring eth0 network (.221 ICS / router autoswitch)..."
NET_DIR="$(cd "$(dirname "$0")" && pwd)"
# eth0 NM profiles (the autoswitch daemon flips between them by subnet)
sudo nmcli con delete "Wired connection 1" 2>/dev/null || true
sudo nmcli con delete "eth0-ics-static" 2>/dev/null || true
sudo nmcli con add type ethernet ifname eth0 con-name "eth0-ics-static" \
  ipv4.method manual ipv4.addresses "192.168.137.221/24" \
  ipv4.gateway "192.168.137.1" ipv4.dns "192.168.137.1 8.8.8.8" \
  ipv4.route-metric 100 connection.autoconnect no
sudo nmcli con delete "eth0-dhcp" 2>/dev/null || true
sudo nmcli con add type ethernet ifname eth0 con-name "eth0-dhcp" \
  ipv4.method auto ipv4.route-metric 50 connection.autoconnect yes
# Autoswitch daemon: 192.168.137.x -> .221 (ICS), 192.168.10.x -> DHCP (home router)
if [ -f "$NET_DIR/j5_net_autoswitch.sh" ] && [ -f "$NET_DIR/j5-net-autoswitch.service" ]; then
  sudo install -m 755 "$NET_DIR/j5_net_autoswitch.sh" /usr/local/sbin/j5_net_autoswitch.sh
  sudo install -m 644 "$NET_DIR/j5-net-autoswitch.service" /etc/systemd/system/j5-net-autoswitch.service
  sudo systemctl daemon-reload
  sudo systemctl enable j5-net-autoswitch.service
  echo "    eth0 profiles + j5-net-autoswitch enabled (starts on next boot)."
else
  echo "    WARN: j5_net_autoswitch.sh not found next to this script — daemon not installed."
fi

echo "[3/5] Configuring wlan0 as the headset hotspot (5 GHz, 10.42.0.1)..."
# The headset (Quest / Android viewer app) connects straight to the robot: one radio hop,
# no home router in the path. eth0 stays the PC/dashboard/SSH side.
# Latency-relevant choices: 5 GHz non-DFS channel 36 (legal in IT with the country code set),
# Wi-Fi power save OFF on the Pi (wifi.powersave 2), WPA2 only, IPv6 off.
# NetworkManager "shared" mode runs dnsmasq on 10.42.0.1 (DHCP + DNS for the client) and NATs
# towards eth0; the captive-portal dnsmasq snippet makes Quest/Android connectivity probes land
# on the robot's own portal (raspberry/controller/web_services/captive_portal_server.py).
# Override with env: J5_HOTSPOT_SSID, J5_HOTSPOT_PSK (8..63 chars), J5_HOTSPOT_CHANNEL.
HOTSPOT_SSID="${J5_HOTSPOT_SSID:-JONNY5}"
HOTSPOT_PSK="${J5_HOTSPOT_PSK:-jonny5robot}"
HOTSPOT_CHANNEL="${J5_HOTSPOT_CHANNEL:-36}"
REPO_ROOT="$(cd "$NET_DIR/../.." && pwd)"
sudo raspi-config nonint do_wifi_country IT 2>/dev/null || sudo iw reg set IT || true
sudo rfkill unblock wlan 2>/dev/null || true
sudo nmcli con delete "jonny5-hotspot" 2>/dev/null || true
sudo nmcli con add type wifi ifname wlan0 con-name "jonny5-hotspot" ssid "$HOTSPOT_SSID" \
  wifi.mode ap wifi.band a wifi.channel "$HOTSPOT_CHANNEL" wifi.powersave 2 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.psk "$HOTSPOT_PSK" \
  ipv4.method shared ipv4.addresses "10.42.0.1/24" ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 100
CAPTIVE_CONF="$REPO_ROOT/raspberry/networkmanager/dnsmasq-shared.d/jonny5-captive-portal.conf"
if [ -f "$CAPTIVE_CONF" ]; then
  sudo install -d /etc/NetworkManager/dnsmasq-shared.d
  sudo install -m 644 "$CAPTIVE_CONF" /etc/NetworkManager/dnsmasq-shared.d/jonny5-captive-portal.conf
  echo "    Captive-portal dnsmasq snippet installed."
else
  echo "    WARN: $CAPTIVE_CONF not found — hotspot works, captive portal probes won't redirect."
fi
# Firewall on wlan0: the radio link carries only video (Pi -> headset) and commands
# (headset -> Pi). No SSH/DDS/Docker on the hotspot, no forwarding to eth0 (no internet,
# no PC LAN through the robot). J5_WLAN_ALLOW_WEB=0 also drops the legacy WebXR viewer
# ports (80/443/8189) once only the native app is used.
if [ -f "$NET_DIR/j5-wlan0.nft" ] && [ -f "$NET_DIR/j5-wlan0-firewall.service" ]; then
  if ! command -v nft >/dev/null 2>&1; then
    sudo apt-get install -y nftables
  fi
  if [ "${J5_WLAN_ALLOW_WEB:-1}" = "0" ]; then
    grep -v '# web$' "$NET_DIR/j5-wlan0.nft" | sudo tee /etc/j5-wlan0.nft >/dev/null
    echo "    wlan0 firewall: native app only (RTSP 8554 + ws-teleop 8557)."
  else
    sudo install -m 644 "$NET_DIR/j5-wlan0.nft" /etc/j5-wlan0.nft
    echo "    wlan0 firewall: native app + legacy WebXR viewer (80/443/8189)."
  fi
  sudo install -m 644 "$NET_DIR/j5-wlan0-firewall.service" /etc/systemd/system/j5-wlan0-firewall.service
  sudo systemctl daemon-reload
  sudo systemctl enable --now j5-wlan0-firewall.service \
    && echo "    j5-wlan0-firewall active (nft list table inet j5wlan to inspect)." \
    || echo "    WARN: j5-wlan0-firewall failed to start — check: sudo nft -c -f /etc/j5-wlan0.nft"
else
  echo "    WARN: j5-wlan0.nft / j5-wlan0-firewall.service not found next to this script — hotspot left OPEN."
fi
# Bring it up now if the radio is free; otherwise it starts at boot (autoconnect).
sudo nmcli con up "jonny5-hotspot" >/dev/null 2>&1 \
  && echo "    Hotspot '$HOTSPOT_SSID' up on 10.42.0.1 (5 GHz ch $HOTSPOT_CHANNEL)." \
  || echo "    Hotspot profile saved; it comes up at boot (nmcli con up jonny5-hotspot to force)."

echo "[4/5] Installing Docker (if missing)..."
if ! command -v docker >/dev/null 2>&1; then
  curl -fsSL https://get.docker.com | sh
  sudo usermod -aG docker "$USER"
  echo "    Added $USER to the 'docker' group (re-login or reboot required)."
fi

echo "[5/5] Done. REBOOT now, then verify:"
echo "    ls -l /dev/spidev0.0          # SPI device present"
echo "    rpicam-hello --list-cameras   # 2x imx708"
echo "    ls -l /dev/serial0            # STM32 UART -> ttyAMA0"
echo "    groups | grep docker          # docker group active"
echo "    ip -4 addr show eth0          # 192.168.137.221 on the ICS bench (~10s after boot)"
echo "    nmcli -f NAME,DEVICE,STATE con show --active   # jonny5-hotspot on wlan0"
echo "    iw dev wlan0 info             # type AP, channel 36 (5180 MHz)"
echo "    Headset/phone: join '$HOTSPOT_SSID', then rtsp://10.42.0.1:8554/cam0 (app) or https://10.42.0.1 (browser)"
