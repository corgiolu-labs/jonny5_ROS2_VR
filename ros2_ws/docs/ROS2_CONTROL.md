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
| activate | Commands are initialised to the measured pose (no jump). J5IK frames start, with STREAM_ENABLE governed by the consent gate below. |
| deactivate | Sends J5IK with STREAM_ENABLE clear: the firmware disables the servos. |
| `read()` error | No valid telemetry for `link_timeout_ms` (500 ms). The controller_manager then deactivates the hardware. |

**Consent gate.** STREAM_ENABLE is sent only while all of these hold:
- the STM32 is in IDLE;
- the E-STOP is not active;
- when consent is (re)granted, the controller command is within `arm_gate_rad` (0.05 rad) of the current pose.

If the STM32 leaves IDLE (SAFE, STOPPED or E-STOP), consent is withdrawn at once. After the operator's UART ENABLE it comes back only if the command still matches the pose. A trajectory that kept running while the firmware was in SAFE therefore does not make the arm jump when it is re-enabled.

Firmware-side guarantees are independent of the Pi:
- motion only in FSM IDLE;
- E-STOP / STOPPED always win;
- the arm holds if J5IK frames stop for 100 ms;
- after 500 ms without SPI frames: SAFE, servos off;
- per-joint velocity limits.

The firmware never leaves SAFE on its own in JOINT_STREAM, so arming is always an operator action:

1. Launch `control.launch.py`. The hardware activates while the STM32 may still be in SAFE, and the J5IK frames keep the SPI watchdog alive.
2. Send `python3 raspberry/tools/j5_uart.py ENABLE`.
3. The stream arms and the controllers take over from the current pose.

After an E-STOP: release it, send `j5_uart.py SAFE ENABLE`, and streaming resumes through the consent gate.

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
