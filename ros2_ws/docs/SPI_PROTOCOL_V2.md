# J5 SPI protocol v2

Status: implemented in firmware (`firmware/stm32/src/spi/`), in the Pi codec
(`raspberry/controller/spi_dataplane/j5_protocol_v2.py`) and in
`jonny5_spi_driver` (`protocol_version:=2`). **Needs validation on the robot
before use.**

## Goals

- Integrity: every frame carries a CRC-16. Frames torn by the SPI slave's
  double-buffered DMA, or corrupted on the wire, are rejected.
- Loss detection: the Pi's sequence number is tracked, and the STM32 adds its
  own reply counter so repeated or stale replies are visible.
- A complete state report in every reply: FSM state, E-STOP, arming flags,
  joint positions, IMU and real-time loop health. No 0x03 polling.
- Joint streaming (J5IK) for ROS 2 / `ros2_control`: on-MCU rate limiting and
  timeouts.

Protocol v1 is still supported unchanged. The firmware answers each request in
the version of that request, so v1 and v2 hosts work with the same firmware
image. The transport envelope (64 B, or 64 B padded to 128 B with
`CONFIG_J5_CANONICAL_PADDED_128_MODE`) does not change.

## Frame (64 bytes, both directions)

| Bytes | Field | v1 | v2 |
|---|---|---|---|
| 0-1 | header | `'J' '5'` | `'J' '5'` |
| 2 | protocol_version | 1 | **2** |
| 3 | frame_type | | |
| 4-5 | sequence (BE) | Pi counter; reply echoes it | same |
| 6 | payload_len | 64 | 64 |
| 7 | flags | 0 | **0x01 = CRC16** |
| 8-61 | payload (54 B) | | |
| 62-63 | reserved | 0, or `rt_loop_period_us` in TELEMETRY | **CRC-16 (BE)** |

**CRC-16/CCITT-FALSE**:
- Polynomial 0x1021, init 0xFFFF, no reflection, xorout 0.
- Computed over bytes 0..61.
- Check value: `"123456789"` → `0x29B1`.

A v2 frame whose CRC is wrong is never parsed. The firmware counts it (`crc_errors`), answers STATUS with `payload[0] = 0xEC` and does not refresh the SPI watchdog.

The SPI watchdog now only counts **valid** frames: header, version, type and, for v2, the CRC. Noise on the bus no longer keeps it alive.

## Frame types

| Type | Direction | v2 reply |
|---|---|---|
| 0x01 TELEMETRY | Pi → STM (poll) | TELEMETRY_V2 |
| 0x02 TEST_ECHO | Pi → STM | TEST_ECHO (v2, CRC) |
| 0x03 STATUS | Pi → STM | STATUS (same payload as v1, v2 header + CRC) |
| 0x04 J5VR | Pi → STM | TELEMETRY_V2 |
| 0x05 J5IK | Pi → STM | TELEMETRY_V2 |
| 0x08 TELEMETRY_V2 | STM → Pi only | — |

The SPI slave answers one transaction late: the reply to request *N* is clocked out during transfer *N+1*. Its header sequence is the sequence of request *N*.

## TELEMETRY_V2 payload (0x08, big-endian)

| Offset | Type | Field |
|---|---|---|
| 0-3 | u32 | `stm_time_ms` — STM32 uptime when the reply was built |
| 4-5 | u16 | `stm_tx_seq` — +1 per TELEMETRY_V2; unchanged = repeated reply |
| 6-7 | u16 | `rx_seq_gaps` — Pi frames lost (saturating) |
| 8 | u8 | `fsm_state`: 0 SAFE, 1 IDLE, 2 STOPPED |
| 9 | u8 | `status_flags` (see below) |
| 10 | u8 | active `mode` (0-5 VR, 6 JOINT_STREAM) |
| 11 | u8 | `diag_flags` (see below) |
| 12-23 | 6 × i16 | commanded physical joint position, centi-degrees, B S G Y P R |
| 24-31 | 4 × i16 | IMU quaternion W X Y Z, Q15 (÷32767) |
| 32-37 | 3 × i16 | gyro X Y Z, mrad/s |
| 38-43 | 3 × i16 | accel X Y Z, cm/s² |
| 44-45 | u16 | IMU sample counter (low 16 bits) |
| 46-47 | u16 | RT loop period, µs (EWMA) |
| 48-49 | u16 | RT step time, µs (EWMA) |
| 50-51 | u16 | RT tick overruns (saturating) |
| 52-53 | u16 | RX CRC errors seen by the STM32 (saturating) |

`status_flags`:

| Bit | Flag |
|---|---|
| 0 | E-STOP active |
| 1 | movement allowed (FSM IDLE) |
| 2 | deadman (both grips) |
| 3 | VR input active |
| 4 | armed |
| 5 | freeze |
| 6 | SETPOSE / HOME / PARK running |
| 7 | IMU orientation valid |

`diag_flags`:

| Bit | Flag |
|---|---|
| 0 | guard seen |
| 1 | IMU present |
| 2 | IMU reads enabled |
| 3 | joint stream live |

Joint positions are the firmware's commanded servo angles, not encoder readings. The servos have no feedback. They currently have 1° resolution because `servo_get_angle` is integer; the field is in centi-degrees so a finer source can be added later without changing the wire format. The BNO085 path does not yet populate gyro/accel, so those fields read 0.

## J5IK joint streaming (0x05, mode 6 = JOINT_STREAM)

| Offset | Field |
|---|---|
| 0 | valid |
| 1 | control_flags — bit7 = **STREAM_ENABLE** (bit0 grip, bit1 hold: legacy) |
| 2-3 | target_id |
| 4-5 | heartbeat — must advance (also used for SAFE → IDLE auto re-arm) |
| 6 | mode = 6 |
| 8-19 | 6 × i16 target, physical centi-degrees, B S G Y P R |

Firmware behaviour (RT loop, 1 kHz):
- The frame acts only in FSM **IDLE**. SAFE, STOPPED and E-STOP always win.
- If `valid` is 0, the mode is not 6, or STREAM_ENABLE is clear, the servos are disabled, the same as a VR disarm.
- If the last J5IK frame is older than **100 ms**, the arm **holds** where it is and no new target is applied.
- Otherwise:
  - targets are clamped to the runtime joint limits;
  - the servos move with the per-joint velocity limits of `j5vr_actuation_apply_desired`.
- If SPI frames stop for more than 500 ms, the SPI watchdog forces SAFE and the servos turn off.

## ROS 2 (`jonny5_spi_driver`)

- `protocol_version` parameter or launch argument (default 1). Set it to 2 only after flashing the v2 firmware.
  - A v1 firmware rejects v2 frames. The node then gets no telemetry and reports `stm32_online: false`; nothing moves.
- `jonny5/joint_commands` (`std_msgs/Float64MultiArray`): 6 positions in rad, `joint_names` order, 0 rad = HOME. Same shape as the `forward_position_controller` command topic.
- `jonny5/joint_stream/enable` (`std_srvs/SetBool`):
  - While enabled, the driver streams J5IK frames with the latest command, which is held if no new command arrives. `TeleopIntent` is ignored.
  - When disabled, the driver goes back to the J5VR / `TeleopIntent` path.
  - Enabling needs protocol 2 and at least one command.
- With protocol 2:
  - `jonny5/spi/telemetry` fills the new fields: `stm_time_ms`, `fsm_state`, `estop_active`, `rx_seq_gaps`, CRC counters, `rt_overruns`, and so on.
  - `jonny5/status` reports the real FSM state and the E-STOP.
  - 0x03 polling is disabled.

Example (mock, no hardware):

```bash
ros2 launch jonny5_bringup bringup.launch.py protocol_version:=2
ros2 topic pub -r 50 /jonny5/joint_commands std_msgs/msg/Float64MultiArray "{data: [0.2, 0, 0, 0, 0, 0]}"
ros2 service call /jonny5/joint_stream/enable std_srvs/srv/SetBool "{data: true}"
```

## Hardware validation checklist

1. Flash the v2 firmware. With `protocol_version:=1` everything must behave exactly as before.
2. Switch to `protocol_version:=2`, then check:
   - `crc_errors_*` and `rx_seq_gaps` stay near 0 at 100 Hz;
   - `repeated_replies` is low;
   - `fsm_state` and `estop_active` follow the mushroom button.
3. Joint streaming with the arm clear of obstacles and E-STOP in hand:
   - ENABLE;
   - small steps on one joint;
   - kill the driver: the arm holds, then goes to SAFE after 500 ms.
