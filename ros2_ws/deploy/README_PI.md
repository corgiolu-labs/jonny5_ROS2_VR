# JONNY5 ROS2 — Raspberry Pi deployment (Pi OS + Docker)

Target architecture (see [ADR-001](../docs/ADR-001-migration-strategy.md)):

```
Raspberry Pi OS Bookworm (64-bit) — HOST
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

- Raspberry Pi OS **Bookworm 64-bit** freshly imaged.
- The STM32 wired to the Pi SPI1 bus as in the legacy setup.
- (Separately) the native camera + MediaMTX setup, as in the legacy deployment.

## 1. Host setup (once)

```bash
git clone <repo> ~/JONNY5_ROS2          # must contain ros2_ws/ and raspberry/
cd ~/JONNY5_ROS2/ros2_ws/deploy
bash host_setup_pi.sh                    # enables SPI, installs Docker
sudo reboot
```

After reboot, verify:

```bash
ls -l /dev/spidev0.0      # SPI device present
groups | grep docker      # docker group active
```

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
