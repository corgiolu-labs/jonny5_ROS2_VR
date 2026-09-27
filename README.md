# JONNY5 ROS 2 VR

Experimental ROS 2 integration layer for [JONNY5](https://github.com/corgiolu-labs/jonny5), a physical VR-teleoperated 6-DoF robot arm.

This repository places typed ROS 2 control and observation interfaces around the proven STM32/Zephyr, Raspberry Pi and WebXR/WebRTC core. It is deliberately incremental: the original low-latency firmware and video path remain intact while ROS 2 standardizes robot description, telemetry, teleoperation and bring-up.

## Engineering status

| Capability | Status |
|---|---|
| Package build and launch | Implemented |
| Custom messages and typed topics | Implemented |
| URDF/Xacro, joint limits and RViz configuration | Implemented |
| Mock SPI telemetry and simulated VR intent | Implemented and smoke-tested |
| WebSocket-to-ROS 2 teleoperation bridge | Implemented and smoke-tested |
| Docker/Compose deployment for Raspberry Pi | Implemented |
| Native JONNY5 SPI codec integration | Implemented; hardware verification required after changes |
| ROS 2 actuation on the physical robot | Experimental; not presented as production-ready |
| rosbag2 dataset capture | Reproducible recording workflow provided; dataset quality remains experiment-dependent |

This is hands-on ROS 2 integration work, but it is **not a claim of deep production ROS 2 or fleet experience**. AI tools assisted parts of the migration; architecture, review and hardware sign-off remain human-owned.

## Firmware hardware targets

The firmware under `firmware/stm32/` builds for two boards from the same source tree; the platform is selected from the devicetree (`src/core/j5_board.h`).

| PlatformIO env | Board | Status |
|---|---|---|
| `nucleo_f446re` | STM32 Nucleo-F446RE — the original thesis build | Build unchanged by the port |
| `weact_g474_shield` | STM32G474 on the **JONNY5 SHIELD rev3** (custom 4-layer carrier: Raspberry Pi 5 + STM32G474) | Verified on hardware: boot, UART and SPI bridges (SPI ≈ 96 Hz), IMU quaternions, servos armed-idle |

**Hardware E-STOP (G474 shield only).** A normally-closed mushroom button on PC6 is polled and debounced inside the 1 kHz real-time tick. The input is read fail-safe: button pressed, cut wire, read error or init failure all mean STOP. On engage the firmware disables all servos, aborts any in-flight trajectory and latches `STOPPED`; motion commands answer `ERR ESTOP_ACTIVE`. Releasing the button only clears the latch — there is no auto-resume, the operator has to re-enable explicitly.

**SPI frame watchdog (both boards).** Checked every 100 ms; if no valid frame has arrived from the Raspberry Pi for more than 500 ms (`SPI_FRAME_TIMEOUT_MS`), the state machine is forced to `SAFE` and the servos are disabled. Any HOME/PARK/SETPOSE trajectory in flight is aborted. Leaving `SAFE` automatically requires fresh SPI frames *and* a VR heartbeat that has advanced since `SAFE` was entered, so a stale last intent cannot re-arm the arm.

**SPI protocol v2 (both boards).** Same 64-byte frame plus a CRC-16 on every frame. Every reply is a complete state report: FSM state, E-STOP, joint positions, RT-loop health, sequence gaps and CRC counters. It also adds J5IK joint streaming for ROS 2, rate-limited on the MCU and held if frames stop. v1 stays the default and works unchanged; select v2 with `protocol_version:=2` after flashing. See [SPI_PROTOCOL_V2.md](ros2_ws/docs/SPI_PROTOCOL_V2.md).

**ros2_control.** `ros2 launch jonny5_bringup control.launch.py` runs `jonny5_control/Jonny5System` with `joint_trajectory_controller` (or `forward_position_controller`), `joint_state_broadcaster` and `imu_sensor_broadcaster`. It uses mock firmware by default; pass `mock_hardware:=false` on the robot. It is exclusive with `bringup.launch.py`, because only one process may own SPI. See [ROS2_CONTROL.md](ros2_ws/docs/ROS2_CONTROL.md).

**Stale-intent watchdog (ROS 2 driver).** `jonny5_spi_driver` streams IDLE frames (no buttons, so the firmware deadman drops) when no `TeleopIntent` has arrived for `intent_timeout_s` (default 0.25 s).

## ROS graph

```text
WebXR / WebSocket
        |
        v
jonny5_vr_bridge ---> /jonny5/teleop/intent
                              |
                              v
                      jonny5_spi_driver <--> STM32 / mock SPI
                         |       |       |
                         v       v       v
                  /joint_states /imu/data /jonny5/status
                                      \
                                       -> /jonny5/spi/telemetry

robot_state_publisher <--- URDF/Xacro + /joint_states
```

## Packages

```text
ros2_ws/src/
  jonny5_msgs/          Custom TeleopIntent, SpiTelemetry and RobotStatus messages
  jonny5_description/   URDF/Xacro, camera/IMU frames, joint limits and RViz config
  jonny5_hardware/      Native/mock SPI driver and ROS 2 telemetry publishers
  jonny5_control/       ros2_control hardware interface (C++, SPI protocol v2) + controllers
  jonny5_teleop_vr/     WebXR/WebSocket to TeleopIntent bridge
  jonny5_sim/           Hardware-free teleoperation intent simulator
  jonny5_bringup/       Launch files and runtime parameter composition
```

## Hardware-free verification

Target environment: ROS 2 Jazzy on Ubuntu/WSL or the supplied Jazzy container.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
bash tools/smoke_ros2_dryrun.sh
```

The smoke test builds with `colcon`, launches mock bring-up, verifies nodes and topics, receives joint/status/intent messages, and injects a WebSocket command. Expected result:

```text
[PASS] JONNY5 ROS2 dry-run smoke test completed
```

See [SMOKE_TEST.md](ros2_ws/SMOKE_TEST.md) for manual inspection commands.

## Containerized Raspberry Pi deployment

The ROS 2 control plane runs in `ros:jazzy`; cameras, libcamera and MediaMTX remain native so the validated video path is not disturbed.

```bash
cd ros2_ws/deploy
docker compose build
docker compose -f docker-compose.yml -f docker-compose.mock.yml up
```

Real-hardware bring-up passes `/dev/spidev0.0` into the container and must follow the checklist in [README_PI.md](ros2_ws/deploy/README_PI.md).

## Dataset recording

After bring-up, record the synchronized ROS-facing telemetry with:

```bash
cd ros2_ws
bash tools/record_dataset.sh
```

The script records robot status, SPI telemetry, joint states, IMU and teleoperation intent into a timestamped rosbag2 directory. See [DATA_CAPTURE.md](ros2_ws/docs/DATA_CAPTURE.md) for topic semantics, metadata and limitations.

## Repository layout

- `firmware/` — unchanged STM32/Zephyr real-time core
- `raspberry/` — existing controller and SPI codec reused by the bridge
- `web/` — existing dashboard and WebXR/WebRTC frontend
- `ros2_ws/src/` — ROS 2 packages
- `ros2_ws/deploy/` — Docker, Compose and Raspberry Pi services
- `ros2_ws/tools/` — smoke testing, bridging and recording utilities
- `ros2_ws/docs/` — architecture decisions and data-capture documentation

## Relationship to the main project

Use [corgiolu-labs/jonny5](https://github.com/corgiolu-labs/jonny5) for the physical build, measured performance, media and proven real-time architecture. This repository documents the ROS 2 migration and keeps experimental claims separate from hardware-validated results.
