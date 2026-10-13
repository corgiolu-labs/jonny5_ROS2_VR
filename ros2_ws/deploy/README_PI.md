# JONNY5 ROS2 — Raspberry Pi deployment (Pi OS + Docker)

Target architecture (see [ADR-001](../docs/ADR-001-migration-strategy.md)):

```
Raspberry Pi OS Trixie / Debian 13 (64-bit) — HOST
├── cameras / libcamera / MediaMTX   ← NATIVE, unchanged (the 37-38 ms video path)
└── Docker
    └── ros:jazzy container          ← ROS2 control plane
        ├── jonny5_spi_driver   (SPI data plane → STM32, reuses the legacy codec)
        ├── jonny5_vr_bridge    (WebXR/WebSocket :8567 → TeleopIntent)
        └── robot_state_publisher
```

The container talks to the STM32 through `/dev/spidev0.0` (passed through) and to the LAN
through host networking (DDS + the VR WebSocket on 8567). The STM32 firmware and the
64-byte J5VR SPI protocol are unchanged.

## Prerequisites

- Raspberry Pi OS **Trixie (Debian 13) 64-bit** freshly imaged. *(Trixie is the proven host on this Pi — the native camera/MediaMTX 37–38 ms path already runs on it; ROS2 stays in the `ros:jazzy` container regardless of host OS.)*
- The STM32 wired to the Pi SPI1 bus as in the legacy setup.
- (Separately) only the native MediaMTX/video pipeline, as in the legacy deployment —
  the camera overlays (2× IMX708) and the STM32 UART are configured by `host_setup_pi.sh`.

## 1. Host setup (once)

```bash
git clone <repo> ~/JONNY5_ROS2          # must contain ros2_ws/ and raspberry/
cd ~/JONNY5_ROS2/ros2_ws/deploy
J5_HOTSPOT_PSK='choose-a-password' bash host_setup_pi.sh   # SPI + cameras + UART, eth0 profiles, wlan0 hotspot, Docker
sudo reboot
```

After reboot, verify:

```bash
ls -l /dev/spidev0.0          # SPI device present
rpicam-hello --list-cameras   # 2x imx708
ls -l /dev/serial0            # STM32 UART -> ttyAMA0
groups | grep docker          # docker group active
nmcli -f NAME,DEVICE,STATE con show --active   # jonny5-hotspot on wlan0
iw dev wlan0 info             # type AP, 5 GHz channel 36
```

### Network layout

| Interface | Address | Peer | Traffic |
|-----------|---------|------|---------|
| `wlan0` hotspot `JONNY5` (5 GHz, power save off) | `10.42.0.1/24` | headset / phone viewer app, only client | RTSP 8554 (native app), HTTPS 443 + WHEP (web viewer), ws-teleop 8557, captive portal 80 |
| `eth0` | `192.168.137.221` on the ICS bench, DHCP at home (autoswitch) | PC | dashboard, SSH, deploy, RViz / DDS |

The headset talks to the robot over a single radio hop; the home router is never in the video path.
The link is firewalled (`j5-wlan0.nft`, loaded by `j5-wlan0-firewall.service`) so that it carries
**only video Pi → headset and commands headset → Pi**:

| Allowed on wlan0 | Purpose |
|------------------|---------|
| TCP 8554, UDP 8000-8001 | RTSP session (UDP transport only, MediaMTX `rtspTransports: [udp]`) + RTCP; RTP video flows Pi → headset |
| TCP 8557 | ws-teleop WebSocket: commands in, telemetry back on the same socket |
| TCP 80/443, UDP 53/8189 | legacy WebXR viewer + captive portal; WebRTC media UDP only (dropped with `J5_WLAN_ALLOW_WEB=0`) |
| UDP 67, ICMP | DHCP lease and ping |

Everything else is dropped (SSH, ROS 2 DDS, Docker, MediaMTX on other ports) and **no traffic is
forwarded** between wlan0 and eth0: the headset gets no internet through the robot and cannot reach the
PC LAN. The headset will report "no internet" on this network: accept "stay connected" once.
Hotspot SSID / password / channel can be overridden with `J5_HOTSPOT_SSID`, `J5_HOTSPOT_PSK`,
`J5_HOTSPOT_CHANNEL` when running the setup script. Inspect the live rules with
`sudo nft list table inet j5wlan` (the `counter drop` lines show what was blocked).

## 2. Build the image and the workspace

```bash
cd ~/JONNY5_ROS2/ros2_ws/deploy
docker compose build
# Build the colcon workspace inside the container (writes ros2_ws/install via the mount):
docker compose run --rm jonny5 colcon build --symlink-install
```

> Always build **inside the container** — the host has no ROS2.

## 3. Hardware-free smoke test (recommended first)

No SPI device, synthetic telemetry + simulated intent:

```bash
docker compose -f docker-compose.yml -f docker-compose.mock.yml up
```

In another shell, inspect the graph:

```bash
docker exec -it jonny5_ros2 bash -lc \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list && ros2 topic echo --once /jonny5/spi/telemetry"
```

Expect `/joint_states`, `/imu/data`, `/jonny5/status`, `/jonny5/spi/telemetry` with live values.

## 4. Real hardware

```bash
docker compose up        # use_mock_spi:=false, opens /dev/spidev0.0
```

Bring-up checklist:
- `/jonny5/status` → `spi_online: true`, `imu_online: true`.
- `/jonny5/spi/telemetry` → IMU quaternion + `servo_deg` change when the arm moves.
- Publish a `TeleopIntent` (or connect the headset to `ws://<pi-ip>:8567`) and confirm the arm responds.

## 5. Autostart (optional)

```bash
sudo cp jonny5-ros2.service /etc/systemd/system/
# edit User= / WorkingDirectory= if your clone path differs
sudo systemctl enable --now jonny5-ros2
```

## Notes

- **Video is not in ROS2.** Keep running the native camera/MediaMTX stack as before; it is
  independent of this container.
- **DDS across machines:** host networking means ROS2 topics are visible on the LAN. Set a
  unique `ROS_DOMAIN_ID` in `docker-compose.yml` if multiple robots share the network.
- **`legacy_root`:** the driver imports the reused codec from `/opt/jonny5/raspberry`
  (`JONNY5_LEGACY_ROOT`, set in the compose file). No `/dev/shm` IPC is used.
- FSM state fields (`deadman`, mode echo, `diag_mask`) come from STATUS (0x03) frames and
  are wired in a follow-up (telemetry task); 0x01 telemetry (IMU + servo) is complete.

### Direct video path (A/B against MediaMTX)

`scripts/video_direct.sh <viewer-ip>` stops `jonny5-mediamtx` and runs the two cameras through
`rpicam-vid --inline --flush -o - | raspberry/tools/rtp_push.py`, i.e. RTP/UDP pushed straight to the
viewer (ports 5000 / 5002), same lowlatency profile, no server in the loop. The viewer app must be in
`udp` mode (host field = `udp`). Ctrl+C restores MediaMTX. On the hotspot the viewer is the only DHCP
client, so its address is stable (check with `nmcli -f IP4 dev show wlan0` / dnsmasq leases).

Clean latency A/B with the stamped source (same capture + encoder, only the relay changes):

```bash
# A: via MediaMTX (publish cam_g2g, app reads rtsp://<pi>:8554/cam_g2g)
sudo systemctl stop jonny5-mediamtx; ~/mediamtx raspberry/config_runtime/video/mediamtx_g2g.yml &
python3 raspberry/controller/web_services/stamped_h264_source.py 800 450 30 3M rtsp://127.0.0.1:8554/cam_g2g
# B: direct (app in udp mode, viewer at 10.42.0.10)
python3 raspberry/controller/web_services/stamped_h264_source.py 800 450 30 3M rtp://10.42.0.10:5000
```

Read the result from a phone screenshot with `jonny5-quest-viewer/tools/read_stamp.py` (block stamp = Pi
epoch ms, phone clock drawn by the app; pass the Pi-minus-phone clock offset).

Measured 2026-10-13 (Pi 5 + Galaxy A16 on the 5 GHz hotspot, 800x450, 4 samples each):

| Path | capture -> display |
|------|--------------------|
| A: stamped source -> MediaMTX -> RTSP/UDP -> app | 59-70 ms |
| B: stamped source -> rtp:// direct -> app | 62-75 ms |
| C: `video_direct.sh` (rpicam-vid + rtp_push), production profile | 2x 120 fps, 3.1 Mbit/s, 0 packet loss (no stamp) |

The relay is not where the latency is: A and B are equal within the +-5 ms clock-offset error. Open point
for C: the app reports ~10 dropped frames/s with zero packet loss (bursty delivery from rtp_push's
coalescing hitting the decoder backlog policy); the MediaMTX path shows ~4 drops in total.

Phone notes: Android treats the hotspot as "no internet" and routes app sockets to mobile data, so the
app binds itself to the Wi-Fi network (`CHANGE_NETWORK_STATE`). Join from adb:
`adb shell cmd wifi connect-network JONNY5_HOTSPOT wpa2 <psk>`.

**Production path measured (2026-10-13, rpicam-vid + rtp_push, 800x450 @ 120 fps, one eye, A16):**
`--metadata` and `--save-pts` do not work with the libav (software) encoder (empty file / "incompatible
options"), but `-v 2` prints `Video start timestamp : <epoch us>` for the first encoded frame and the
sensor period is exactly FrameDuration (8324 us at 120 fps), so with the frame number burned in by
annotate_cv (`%frame`) the capture time of frame F is start + F * 8.324 ms. Six screenshots:
53 / 50 / 50 / 51 / 49 / 58 ms (median 50.5) from first-frame-at-encoder to display, the same as the
stamped picamera2 pipeline (43-53 ms over runs). Add exposure + readout + ISP (~10 ms; exposure is
7.9 ms at 120 fps) for true sensor-to-display: about 60 ms on the A16. Recipe:
`rpicam-vid ... -v 2 --post-process-file /tmp/annotate.json -o /tmp/cam0.fifo` with
`python3 raspberry/tools/rtp_push.py ... < /tmp/cam0.fifo`, phone clock via `read_stamp.py --phone-only`.

### Browser vs native app, same phone, same source (2026-10-13)

Stamped source 800x450 @ 120 fps on the g2g MediaMTX; Galaxy A16 on the 5 GHz / 80 MHz hotspot;
same decoding method (`read_stamp.py`, phone clock drawn as a block row by both the app and the test
page `jonny5-quest-viewer/tools/g2g_browser.html`). Capture -> display, 6 samples each:

| Receiver | samples (ms) | median |
|----------|--------------|--------|
| Chrome, WebRTC/WHEP like the original viewer (default jitter buffer, Chrome reported jb 11 ms) | 93 78 74 77 77 72 | 77 |
| Chrome, WebRTC with playoutDelayHint = 0 | 78 67 68 77 77 80 | 77 |
| Native app, RTSP/RTP UDP, MediaCodec low latency | 64 52 52 47 50 47 | 51 |

The native app is ~25 ms faster than the browser on the same device; the browser's jitter buffer is
only part of it (11 ms), the rest is Chrome's video/compositor path. Note: Chrome needs mobile data
off on the A16 to reach the hotspot (no-internet network); the app binds itself to Wi-Fi.

### Stereo sync + matched exposure (video_direct.sh, 2026-10-13)

`scripts/video_direct.sh` now runs cam0 as libcamera sync server and cam1 as sync client (`SYNC=1`,
default): rpicam-vid logs `Sync achieved! Difference -322us`, i.e. the two sensors expose within a
fraction of a frame. Start order matters (client first), the script does it. With `MATCH=cam0`
(default) it first probes cam0 in auto for `PROBE_S` seconds (needs `--metadata` + `--save-pts` + a
file output, otherwise rpicam-vid writes no metadata), then runs BOTH cameras with the converged
shutter / gain / AWB gains / lens position as fixed values, so the two eyes have identical exposure,
colour and focus (`MATCH=manual` with SHUTTER GAIN AWBGAINS LENSPOS, `MATCH=auto` for the old
independent loops, `ROI=x,y,w,h` for an identical crop). Verified on the A16: before, the right eye was
visibly brighter and warmer; after, both eyes match. Example of a converged set indoors at night:
`--shutter 7882 --gain 16.000 --awbgains 1.426,2.724 --lens-position 3.39` (shutter and gain at the
120 fps maximum: low light costs noise, not latency).

### Encoder options and CPU (rpicam-vid, 800x450 @ 120 fps, measured 2026-10-13)

| rpicam-vid encoder options | x264 CPU (one core) | capture -> display (A16, frame-counter method) |
|----------------------------|---------------------|------------------------------------------------|
| defaults (what video_direct.sh uses) | 70 % | 50 ms |
| `--low-latency` (rpicam-apps libx264 low-latency preset, multi-slice: 590 pkt/s) | 63 % | 61 ms |
| `--libav-video-codec-opts preset=ultrafast;tune=zerolatency` (replaces the internal defaults) | 56 % | 105 ms |

Do not touch the encoder options on this receiver: every CPU saving measured cost latency (the MediaTek
decoder of the A16 handles the multi-slice / custom-option streams worse). Re-check on the Quest 2 only
if its decoder behaves differently. Packetizer: `rtp_push.c` (1 % CPU) replaced the Python one (20-25 %).
Untested: `performance` governor + SCHED_FIFO for encoder/packetizer (the sudo/chrt test run aborted).
