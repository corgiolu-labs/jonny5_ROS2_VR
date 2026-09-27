# JONNY5 with ros2_control

`jonny5_control` is a C++ `hardware_interface::SystemInterface`
(`jonny5_control/Jonny5System`). It drives the arm over **SPI protocol v2**
(see [SPI_PROTOCOL_V2.md](SPI_PROTOCOL_V2.md)):
- each controller cycle sends one J5IK joint-stream frame;
- each cycle reads back one TELEMETRY_V2 frame.

It needs the v2 firmware.

```text
JointTrajectoryController / ForwardCommandController   (position, rad)
                    |
          controller_manager @ 100 Hz
                    |
      Jonny5System  write(): J5IK (mode 6, STREAM_ENABLE)  ──SPI──>  STM32
                    read():  TELEMETRY_V2  <──SPI──  (FSM, E-STOP, joints, IMU)
                    |
  joint_state_broadcaster -> /joint_states      imu_sensor_broadcaster -> /imu_sensor_broadcaster/imu
```

## Two exclusive runtime modes

| Launch | SPI owner | Use |
|---|---|---|
| `bringup.launch.py` | `jonny5_spi_driver` (Python) | VR teleoperation (`TeleopIntent`), protocol v1 or v2 |
| `control.launch.py` | `ros2_control_node` + `Jonny5System` | Trajectories / MoveIt / any `ros2_controllers` controller |

Only one process may drive `/dev/spidev0.0`: never run both launch files on the robot.

## Interfaces

- **Joints** `base_joint … wrist_roll_joint`:
  - command: `position`;
  - state: `position` and `velocity`.
  - Units are rad, and 0 rad = mechanical HOME.
  - Conversion: `rad = radians((physical_deg − offset) · dir)`, with the offsets/dirs from `j5_settings.json`, set in the xacro.
  - Commands are clamped to the URDF limits; the firmware clamps again to its runtime joint limits.
  - `velocity` is estimated from consecutive telemetry samples, using the STM32 timestamps.
- **Sensor** `imu_sensor`: orientation, angular_velocity, linear_acceleration. The BNO085 path currently fills only the orientation.
- **GPIO** `jonny5_status` (state only):
  - FSM state and E-STOP: `fsm_state`, `estop_active`, `movement_allowed`, `joint_stream_live`;
  - link: `link_ok`, `rx_seq_gaps`, `crc_errors_stm`, `crc_errors_pi`;
  - RT loop: `rt_loop_period_us`, `rt_overruns`.

## Lifecycle and safety

| Hardware state | Behaviour |
|---|---|
| configure | Opens SPI and reads the current pose. Fails if no TELEMETRY_V2 arrives (for example v1 firmware). |
| activate | Commands are initialised to the measured pose (no jump), then streaming starts with STREAM_ENABLE set. |
| deactivate | Sends J5IK with STREAM_ENABLE clear: the firmware disables the servos. |
| `read()` error | No valid telemetry for `link_timeout_ms` (500 ms). The controller_manager then deactivates the hardware. |

Firmware-side guarantees are independent of the Pi:
- motion only in FSM IDLE;
- E-STOP / STOPPED always win;
- the arm holds if J5IK frames stop for 100 ms;
- after 500 ms without SPI frames: SAFE, servos off;
- per-joint velocity limits.

If the STM32 is in SAFE, the first streamed frames re-arm it automatically, because the heartbeat advances. After an E-STOP the operator must release it and send SAFE → ENABLE.

## Usage

```bash
# Mock (simulated v2 firmware, no hardware)
ros2 launch jonny5_bringup control.launch.py
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint, wrist_yaw_joint, wrist_pitch_joint, wrist_roll_joint],
    points: [{positions: [0.3, -0.2, 0.2, 0.1, -0.1, 0.0], time_from_start: {sec: 2}}]}}"

# Streaming controller instead of trajectories
ros2 launch jonny5_bringup control.launch.py controller:=forward_position_controller
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1, 0, 0, 0, 0, 0]}"

# Real robot: v2 firmware flashed, arm clear, E-STOP in hand
ros2 launch jonny5_bringup control.launch.py mock_hardware:=false
```

`transfer_len` (default 128) must match the firmware build: 128 with `CONFIG_J5_CANONICAL_PADDED_128_MODE=y`, which is the default `prj.conf`, otherwise 64.

## Notes and limits

- The SPI exchange runs synchronously in `write()`: about 1 ms at 1 MHz × 128 B, which is fine at 100 Hz. For higher rates, move it to a dedicated thread.
- Joint positions are the firmware's **commanded** angles (no encoders), with 1° resolution today.
- `ros2_control_node` on the Pi needs RT privileges for its FIFO thread (`cap_add: SYS_NICE` / `ulimits: rtprio` in Docker).
- Next steps: MoveIt 2 config (SRDF, kinematics) and MoveIt Servo for VR teleoperation on top of `forward_position_controller`.

## Tests

- `test_j5_protocol_v2`:
  - CRC check value;
  - parsing of a frame produced by the firmware C code;
  - J5IK layout identical to the Python codec;
  - handling of corrupted frames and of the 128-byte padded transport;
  - mock firmware tracking.
- `test_jonny5_system`:
  - rad ↔ physical conversions;
  - full mock lifecycle (configure, activate, a command is tracked, clamping to limits, deactivate).
