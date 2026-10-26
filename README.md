# JONNY5 — minimal stack (branch `fw/minimal`)

VR-teleoperated 6-DoF robot arm: STM32G474 firmware on the JONNY5 SHIELD rev3, a Raspberry Pi 5 with two
Camera Module 3, and the native Quest app ([jonny5-quest-viewer](https://github.com/corgiolu-labs/jonny5-quest-viewer)).

This branch keeps only what teleoperation needs. The ROS 2 layer (ros2_control, MoveIt, URDF, rosbag),
the web dashboards and WebXR viewer, the old Pi controller (ws-teleop, SPI bridge, IK/FK, ASSIST gimbal) and
the captive portal were removed on 26/10/2026; they remain in the git history (`main`,
`feature/video-direct-rtp`) and ROS 2 can be added back on top of j5mini when needed. The stereo collimator
(convergence and distance in the headset) and the vision service (objects, distance, text) are optional
services, WebSocket clients of j5mini like the app; the vision "point" action (arm towards the object) needs
the ASSIST gimbal and is not available.

```
Quest app ──wss 8557 / https 443──► j5mini (Pi) ──SPI 64 B @100 Hz + UART──► STM32 (1 kHz RT loop, servos, E-STOP)
          ◄──RTSP 8554 (RTP/UDP)── MediaMTX ◄── cam_stream.py ×2 (frame sync, matched exposure, sensor zoom)
```

## Layout

| Path | What |
|---|---|
| `firmware/stm32/` | Zephyr firmware for the G474 shield. `MINIMAL.md`: what was pruned, how equivalence with the flashed firmware was proven, the 64-byte SPI transfer, 13 known quirks |
| `firmware/stm32/tools/fw_sim/` | The real firmware sources compiled for the PC (Zephyr stubs, byte-wise SPI DMA): behaviour tests and a differential test against the original firmware |
| `minimal/j5mini/` | The Pi controller: one Python process (SPI, UART, WebSocket for the app, camera controls, HTTPS API). `--sim` runs it on a PC against the simulated firmware |
| `minimal/dashboard/` | Web dashboard served by j5mini: diagnostics, commands, joints with POE kinematics and 3D view, settings, tests; also a teaching bench for a 6R arm (README in Italian) |
| `raspberry/tools/cam_stream.py`, `scripts/` | Video: capture to MediaMTX (`video_capture.sh`, `start_mediamtx.sh`) or direct RTP to the app (`video_direct.sh`) |
| `raspberry/tools/stereo_collimator.py`, `vision_service.py` | Optional: crosshair collimation (on by default) and vision (on when `raspberry/.venv-vision` and `raspberry/models/yolo11n.onnx` are on the Pi) |
| `raspberry/network/`, `raspberry/systemd/` | Headset hotspot + firewall, eth0 autoswitch, service units |
| `raspberry/setup_pi.sh` | One-shot install on the Pi (hardware config, network, services; disables the old stack) |

## Firmware

```bash
cd firmware/stm32 && pio run -e weact_g474_shield            # build (from a path without spaces, see MINIMAL.md)
pio run -e weact_g474_shield -t upload                        # flash over ST-Link
```

**Hardware E-STOP.** A normally-closed mushroom button on PC6 is polled and debounced inside the 1 kHz
real-time tick, read fail-safe (pressed, cut wire, read error or init failure all mean STOP). On engage the
firmware disables all servos, aborts any trajectory and latches `STOPPED`; motion commands answer
`ERR ESTOP_ACTIVE`. Releasing the button only clears the latch: the operator has to re-enable.

**SPI frame watchdog.** If no valid frame arrives for more than 500 ms the state machine goes to `SAFE`
and the servos are disabled. Leaving `SAFE` automatically requires fresh frames *and* an advanced VR
heartbeat, so a stale last intent cannot re-arm the arm.

**SPI link.** 64-byte J5 frames (protocol v1), one full-duplex transfer every 10 ms. The reply type follows
the request (J5VR → TELEMETRY, STATUS → STATUS with diagnostics) and arrives one transfer later.
Master and firmware must use the same transfer length (`CONFIG_J5_CANONICAL_PADDED_128_MODE`).

## Raspberry Pi

```bash
git clone -b fw/minimal https://github.com/corgiolu-labs/jonny5_ROS2_VR.git ~/JONNY5_ROS2
~/JONNY5_ROS2/raspberry/setup_pi.sh          # then reboot the first time
```

Calibrated offsets and limits: export them from the old install before removing it
(`minimal/j5mini/README.md`). MediaMTX is expected as a binary in `~/mediamtx` or on the `PATH`. The
collimator needs `websockets` >= 11 (sync client: Debian 13 / Raspberry Pi OS Trixie). Vision models and
venv are not versioned: on the robot they are already in the checkout; `J5_VISION=0` / `J5_COLLIMATOR=0`
keep the optional services off.

## Dashboard

On the robot: `https://10.42.0.1/` (hotspot) or `https://192.168.137.221/` (cable). Without the robot:
`python3 minimal/j5mini/j5mini.py --sim` and `http://localhost:8558/`: the real firmware runs simulated on the PC.

## Tests without hardware

```bash
cd firmware/stm32
python -m pytest tools/fw_sim ../../minimal/j5mini -q        # firmware simulator + j5mini (35 tests)
cd ../.. && python -m pytest minimal/dashboard -q             # dashboard kinematics vs numpy (needs node)
```

The differential test needs the original firmware sources (`tools/fw_sim/README.md`); CI runs it.

## Relationship to the main project

[JONNY5](https://github.com/corgiolu-labs/jonny5) is the original thesis project. The stereo head on a
Luckfox Aura ([jonny5-stereo-head](https://github.com/corgiolu-labs/jonny5-stereo-head)) talks to the same
firmware over the same SPI link.
