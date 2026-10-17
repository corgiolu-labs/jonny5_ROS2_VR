#!/usr/bin/env bash
# JONNY5 — wlan0 as the headset hotspot (5 GHz, 10.42.0.1) + firewall "video out, commands in".
# Standalone: can be re-run any time without touching eth0 / SPI / Docker (host_setup_pi.sh calls it).
# Env overrides: J5_HOTSPOT_SSID, J5_HOTSPOT_PSK (8..63 chars), J5_HOTSPOT_CHANNEL, J5_WLAN_ALLOW_WEB=0.
# If an older hotspot profile exists (jonny5-ap, SSID JONNY5_HOTSPOT) its SSID and password are reused by
# default, so headsets/phones that already know the network reconnect without reconfiguration.
set -euo pipefail
NET_DIR="$(cd "$(dirname "$0")" && pwd)"
OLD_PROFILE="jonny5-ap"
if nmcli -t -f NAME con show 2>/dev/null | grep -qx "$OLD_PROFILE"; then
  OLD_SSID="$(nmcli -g 802-11-wireless.ssid con show "$OLD_PROFILE" 2>/dev/null || true)"
  OLD_PSK="$(sudo nmcli -s -g 802-11-wireless-security.psk con show "$OLD_PROFILE" 2>/dev/null || true)"
  : "${J5_HOTSPOT_SSID:=${OLD_SSID:-JONNY5}}"
  : "${J5_HOTSPOT_PSK:=${OLD_PSK:-jonny5robot}}"
  echo "    Reusing SSID '$J5_HOTSPOT_SSID' and password from old profile '$OLD_PROFILE' (profile will be removed)."
  sudo nmcli con delete "$OLD_PROFILE" >/dev/null 2>&1 || true
fi
echo "Configuring wlan0 as the headset hotspot (5 GHz, 10.42.0.1)..."
# The headset (Quest / Android viewer app) connects straight to the robot: one radio hop,
# no home router in the path. eth0 stays the PC/dashboard/SSH side.
# Latency-relevant choices: 5 GHz non-DFS channel 36 (legal in IT with the country code set), 80 MHz width,
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
  wifi.mode ap wifi.band a wifi.channel "$HOTSPOT_CHANNEL" wifi.channel-width 80mhz wifi.powersave 2 \
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
# MediaMTX must start after the network is online (ICE candidates are gathered once at startup).
if [ -f "$NET_DIR/jonny5-mediamtx-network-online.conf" ]; then
  sudo install -d /etc/systemd/system/jonny5-mediamtx.service.d
  sudo install -m 644 "$NET_DIR/jonny5-mediamtx-network-online.conf" /etc/systemd/system/jonny5-mediamtx.service.d/10-network-online.conf
  sudo systemctl daemon-reload
  echo "    jonny5-mediamtx: network-online drop-in installed."
fi
# Production capture: rpicam-vid (sync + matched exposure) publishing into MediaMTX (profile "relay").
if [ -f "$REPO_ROOT/raspberry/systemd/jonny5-video-capture.service" ]; then
  sudo install -m 644 "$REPO_ROOT/raspberry/systemd/jonny5-video-capture.service" /etc/systemd/system/jonny5-video-capture.service
  sudo systemctl daemon-reload
  sudo systemctl enable jonny5-video-capture.service >/dev/null 2>&1
  echo "    jonny5-video-capture enabled (needs video_profile: relay in config_runtime/video/video_pipeline.yaml)."
fi
# On-demand camera refocus (POST /api/refocus-cameras, right stick click in the VR viewers): restarts MediaMTX,
# which restarts jonny5-video-capture (PartOf) and re-probes AF on both cameras. Only the service is installed:
# the periodic timer would black out the video for ~6 s every 10 minutes.
if [ -f "$REPO_ROOT/raspberry/systemd/jonny5-refocus-cams.service" ]; then
  sudo install -m 644 "$REPO_ROOT/raspberry/systemd/jonny5-refocus-cams.service" /etc/systemd/system/jonny5-refocus-cams.service
  sudo systemctl daemon-reload
  echo "    jonny5-refocus-cams installed (on demand, no timer)."
fi
# Automatic crosshair collimation for the VR viewers (stereo disparity at the crosshair + distance).
if [ -f "$REPO_ROOT/raspberry/systemd/jonny5-stereo-collimator.service" ]; then
  sudo install -m 644 "$REPO_ROOT/raspberry/systemd/jonny5-stereo-collimator.service" /etc/systemd/system/jonny5-stereo-collimator.service
  sudo systemctl daemon-reload
  sudo systemctl enable jonny5-stereo-collimator.service >/dev/null 2>&1
  echo "    jonny5-stereo-collimator enabled."
fi
# Bring it up now if the radio is free; otherwise it starts at boot (autoconnect).
sudo nmcli con up "jonny5-hotspot" >/dev/null 2>&1 \
  && echo "    Hotspot '$HOTSPOT_SSID' up on 10.42.0.1 (5 GHz ch $HOTSPOT_CHANNEL)." \
  || echo "    Hotspot profile saved; it comes up at boot (nmcli con up jonny5-hotspot to force)."

