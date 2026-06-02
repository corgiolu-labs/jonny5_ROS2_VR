#!/usr/bin/env bash
# One-time host setup on a fresh Raspberry Pi OS Trixie / Debian 13 (64-bit).
# (Docker is installed via get.docker.com, which auto-detects the 'trixie' codename.)
# Enables SPI, cameras (2x IMX708) + STM32 UART, the eth0 .221/router autoswitch,
# and installs Docker.
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

echo "[3/4] Installing Docker (if missing)..."
if ! command -v docker >/dev/null 2>&1; then
  curl -fsSL https://get.docker.com | sh
  sudo usermod -aG docker "$USER"
  echo "    Added $USER to the 'docker' group (re-login or reboot required)."
fi

echo "[4/4] Done. REBOOT now, then verify:"
echo "    ls -l /dev/spidev0.0          # SPI device present"
echo "    rpicam-hello --list-cameras   # 2x imx708"
echo "    ls -l /dev/serial0            # STM32 UART -> ttyAMA0"
echo "    groups | grep docker          # docker group active"
echo "    ip -4 addr show eth0          # 192.168.137.221 on the ICS bench (~10s after boot)"
