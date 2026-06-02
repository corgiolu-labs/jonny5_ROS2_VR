#!/usr/bin/env bash
# One-time host setup on a fresh Raspberry Pi OS Trixie / Debian 13 (64-bit).
# (Docker is installed via get.docker.com, which auto-detects the 'trixie' codename.)
# Enables SPI and installs Docker. Cameras/MediaMTX are configured separately
# (legacy native setup) and are intentionally left untouched here.
set -euo pipefail

echo "[1/3] Enabling SPI..."
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

echo "[2/3] Installing Docker (if missing)..."
if ! command -v docker >/dev/null 2>&1; then
  curl -fsSL https://get.docker.com | sh
  sudo usermod -aG docker "$USER"
  echo "    Added $USER to the 'docker' group (re-login or reboot required)."
fi

echo "[3/3] Done. REBOOT now, then verify:"
echo "    ls -l /dev/spidev0.0   # SPI device present"
echo "    groups | grep docker   # docker group active"
