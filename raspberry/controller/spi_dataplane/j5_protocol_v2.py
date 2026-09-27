"""
j5_protocol_v2.py -- J5 SPI protocol v2 codec (pure Python, no I/O).

Same 64-byte frame as v1 (see j5vr_frame.py / firmware src/spi/j5_protocol.h):

    [0-1]   'J' '5'
    [2]     protocol_version = 2
    [3]     frame_type
    [4-5]   sequence_counter (BE)
    [6]     payload_len = 64
    [7]     flags = FLAG_CRC16
    [8-61]  payload (54 B)
    [62-63] CRC-16/CCITT-FALSE (BE) over bytes 0..61

The firmware answers a v2 request with a v2 reply. J5VR / J5IK / TELEMETRY
requests are answered with TELEMETRY_V2 (0x08). Full specification:
ros2_ws/docs/SPI_PROTOCOL_V2.md.
"""

from __future__ import annotations

import struct
from typing import Any, Dict, List, Optional, Sequence

FRAME_SIZE = 64
PAYLOAD_OFFSET = 8
PAYLOAD_LEN = 54

PROTOCOL_VERSION_V1 = 1
PROTOCOL_VERSION_V2 = 2
FLAG_CRC16 = 0x01

FRAME_TYPE_TELEMETRY = 0x01
FRAME_TYPE_TEST_ECHO = 0x02
FRAME_TYPE_STATUS = 0x03
FRAME_TYPE_J5VR = 0x04
FRAME_TYPE_J5IK = 0x05
FRAME_TYPE_TELEMETRY_V2 = 0x08

# J5IK streaming (mode JOINT_STREAM)
MODE_JOINT_STREAM = 6
J5IK_FLAG_STREAM_ENABLE = 1 << 7

# STATUS error codes (payload[0])
STATUS_ERR_BAD_FRAME = 0xEE
STATUS_ERR_BAD_CRC = 0xEC

FSM_STATE_NAMES = {0: "SAFE", 1: "IDLE", 2: "STOPPED"}

# TELEMETRY_V2 status_flags (payload[9])
ST_ESTOP = 1 << 0
ST_MOVE_ALLOWED = 1 << 1
ST_DEADMAN = 1 << 2
ST_INPUT = 1 << 3
ST_ARMED = 1 << 4
ST_FREEZE = 1 << 5
ST_SETPOSE = 1 << 6
ST_IMU_VALID = 1 << 7

# TELEMETRY_V2 diag_flags (payload[11])
DG_GUARD_SEEN = 1 << 0
DG_IMU_PRESENT = 1 << 1
DG_IMU_ENABLED = 1 << 2
DG_STREAM_LIVE = 1 << 3

SERVO_LABELS = ("B", "S", "G", "Y", "P", "R")

_TLM2 = struct.Struct(">IHHBBBB6h4h3h3hHHHHH")
assert _TLM2.size == PAYLOAD_LEN


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def seal_v2(frame: bytes) -> bytes:
    """Return ``frame`` (64 B) as v2: version=2, flags=CRC16, CRC in bytes 62-63."""
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"frame must be {FRAME_SIZE} bytes, got {len(frame)}")
    out = bytearray(frame)
    out[2] = PROTOCOL_VERSION_V2
    out[7] = FLAG_CRC16
    struct.pack_into(">H", out, 62, crc16_ccitt(bytes(out[:62])))
    return bytes(out)


def is_v2(frame: Optional[bytes]) -> bool:
    return bool(frame) and len(frame) >= FRAME_SIZE and frame[0:2] == b"J5" and frame[2] == PROTOCOL_VERSION_V2


def crc_ok(frame: bytes) -> bool:
    """True if ``frame`` is a v2 frame with a valid CRC."""
    if not is_v2(frame) or frame[7] != FLAG_CRC16 or frame[6] != FRAME_SIZE:
        return False
    return crc16_ccitt(frame[:62]) == struct.unpack_from(">H", frame, 62)[0]


def _header(frame_type: int, sequence: int) -> bytearray:
    frame = bytearray(FRAME_SIZE)
    frame[0:2] = b"J5"
    frame[2] = PROTOCOL_VERSION_V2
    frame[3] = frame_type & 0xFF
    struct.pack_into(">H", frame, 4, sequence & 0xFFFF)
    frame[6] = FRAME_SIZE
    frame[7] = FLAG_CRC16
    return frame


def build_request_v2(frame_type: int, sequence: int) -> bytes:
    """Empty-payload v2 request (e.g. STATUS 0x03 or TELEMETRY 0x01 poll)."""
    return seal_v2(bytes(_header(frame_type, sequence)))


def build_j5ik_frame(
    targets_cdeg: Sequence[int],
    *,
    sequence: int,
    heartbeat: int,
    enable: bool,
    target_id: int = 0,
    valid: bool = True,
    control_flags: int = 0,
) -> bytes:
    """J5IK (0x05) joint-stream frame, v2.

    Payload: [0] valid, [1] control_flags (bit7 = STREAM_ENABLE), [2-3] target_id,
    [4-5] heartbeat, [6] mode = JOINT_STREAM, [7] 0, [8-19] 6 x int16 physical
    centi-degrees (B S G Y P R).
    """
    if len(targets_cdeg) != 6:
        raise ValueError("targets_cdeg needs 6 values (B S G Y P R)")
    frame = _header(FRAME_TYPE_J5IK, sequence)
    flags = int(control_flags) & 0x7F
    if enable:
        flags |= J5IK_FLAG_STREAM_ENABLE
    p = PAYLOAD_OFFSET
    frame[p + 0] = 1 if valid else 0
    frame[p + 1] = flags
    struct.pack_into(">H", frame, p + 2, int(target_id) & 0xFFFF)
    struct.pack_into(">H", frame, p + 4, int(heartbeat) & 0xFFFF)
    frame[p + 6] = MODE_JOINT_STREAM
    for i, v in enumerate(targets_cdeg):
        struct.pack_into(">h", frame, p + 8 + 2 * i, max(-32768, min(32767, int(v))))
    return seal_v2(bytes(frame))


def _q15(v: float) -> int:
    return max(-32768, min(32767, int(round(float(v) * 32767.0))))


def _scaled(v: float, scale: float) -> int:
    return max(-32768, min(32767, int(round(float(v) * scale))))


def build_telemetry_v2(
    *,
    sequence: int,
    stm_time_ms: int,
    stm_tx_seq: int,
    fsm_state: int,
    status_flags: int,
    mode: int,
    joint_cdeg: Sequence[int],
    quat: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    gyro: Sequence[float] = (0.0, 0.0, 0.0),
    accel: Sequence[float] = (0.0, 0.0, 0.0),
    imu_sample_counter: int = 0,
    rt_loop_period_us: int = 1000,
    rt_step_us: int = 0,
    rt_overruns: int = 0,
    crc_errors: int = 0,
    rx_seq_gaps: int = 0,
    diag_flags: int = 0,
) -> bytes:
    """Encode a TELEMETRY_V2 frame (firmware-side layout; used by mocks/tests)."""
    payload = _TLM2.pack(
        int(stm_time_ms) & 0xFFFFFFFF,
        int(stm_tx_seq) & 0xFFFF,
        min(int(rx_seq_gaps), 0xFFFF),
        int(fsm_state) & 0xFF,
        int(status_flags) & 0xFF,
        int(mode) & 0xFF,
        int(diag_flags) & 0xFF,
        *[max(-32768, min(32767, int(v))) for v in joint_cdeg],
        *[_q15(v) for v in quat],
        *[_scaled(v, 1000.0) for v in gyro],
        *[_scaled(v, 100.0) for v in accel],
        int(imu_sample_counter) & 0xFFFF,
        int(rt_loop_period_us) & 0xFFFF,
        int(rt_step_us) & 0xFFFF,
        min(int(rt_overruns), 0xFFFF),
        min(int(crc_errors), 0xFFFF),
    )
    frame = _header(FRAME_TYPE_TELEMETRY_V2, sequence)
    frame[PAYLOAD_OFFSET:PAYLOAD_OFFSET + PAYLOAD_LEN] = payload
    return seal_v2(bytes(frame))


def parse_telemetry_v2(frame: bytes) -> Optional[Dict[str, Any]]:
    """Decode a TELEMETRY_V2 frame. Returns None if not a CRC-valid 0x08 frame.

    Keys shared with the v1 telemetry dict (``imu_*``, ``servo_deg_*``,
    ``imu_sample_counter``, ``rt_loop_period_us``) keep their meaning; servo
    angles are physical degrees with 0.01 deg resolution.
    """
    if not crc_ok(frame) or frame[3] != FRAME_TYPE_TELEMETRY_V2:
        return None
    (
        stm_time_ms, stm_tx_seq, rx_seq_gaps, fsm_state, st, mode, dg,
        j0, j1, j2, j3, j4, j5,
        qw, qx, qy, qz,
        gx, gy, gz,
        ax, ay, az,
        imu_sc, period_us, step_us, overruns, crc_errors,
    ) = _TLM2.unpack_from(frame, PAYLOAD_OFFSET)
    joints_cdeg: List[int] = [j0, j1, j2, j3, j4, j5]
    out: Dict[str, Any] = {
        "protocol_version": PROTOCOL_VERSION_V2,
        "frame_type": FRAME_TYPE_TELEMETRY_V2,
        "sequence": struct.unpack_from(">H", frame, 4)[0],
        "stm_time_ms": stm_time_ms,
        "stm_tx_seq": stm_tx_seq,
        "rx_seq_gaps": rx_seq_gaps,
        "fsm_state": fsm_state,
        "fsm_state_name": FSM_STATE_NAMES.get(fsm_state, f"UNKNOWN_{fsm_state}"),
        "status_flags": st,
        "diag_flags": dg,
        "mode": mode,
        "estop_active": bool(st & ST_ESTOP),
        "movement_allowed": bool(st & ST_MOVE_ALLOWED),
        "deadman_active": bool(st & ST_DEADMAN),
        "input_active": bool(st & ST_INPUT),
        "armed": bool(st & ST_ARMED),
        "freeze": bool(st & ST_FREEZE),
        "setpose_active": bool(st & ST_SETPOSE),
        "imu_valid": bool(st & ST_IMU_VALID),
        "guard_seen": bool(dg & DG_GUARD_SEEN),
        "imu_present": bool(dg & DG_IMU_PRESENT),
        "imu_enabled": bool(dg & DG_IMU_ENABLED),
        "joint_stream_active": bool(dg & DG_STREAM_LIVE),
        "joint_cdeg": joints_cdeg,
        "imu_q_w": qw / 32767.0,
        "imu_q_x": qx / 32767.0,
        "imu_q_y": qy / 32767.0,
        "imu_q_z": qz / 32767.0,
        "imu_gyro_x": gx / 1000.0,
        "imu_gyro_y": gy / 1000.0,
        "imu_gyro_z": gz / 1000.0,
        "imu_accel_x": ax / 100.0,
        "imu_accel_y": ay / 100.0,
        "imu_accel_z": az / 100.0,
        "imu_sample_counter": imu_sc,
        "rt_loop_period_us": period_us,
        "rt_step_us": step_us,
        "rt_overruns": overruns,
        "crc_errors_stm": crc_errors,
    }
    for label, cdeg in zip(SERVO_LABELS, joints_cdeg):
        out[f"servo_deg_{label}"] = cdeg / 100.0
    out["imu_stale"] = not out["imu_valid"]
    if 100 <= period_us <= 60000:
        out["rt_loop_hz_est"] = 1_000_000.0 / period_us
    return out
