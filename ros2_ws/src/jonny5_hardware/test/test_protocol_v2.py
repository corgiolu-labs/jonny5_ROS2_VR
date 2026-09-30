"""SPI protocol v2: codec, CRC and the bridge/mock round trip (no hardware)."""

import math
import struct

import pytest

v2 = pytest.importorskip("controller.spi_dataplane.j5_protocol_v2")
from controller.spi_dataplane.j5vr_frame import build_setpoint_frame  # noqa: E402
from controller.spi_dataplane.j5vr_spi_bridge import J5VRSPIBridge  # noqa: E402
from controller.spi_dataplane.spi_transport_mode import (  # noqa: E402
    extract_canonical_frame64_from_transport_rx,
)

from jonny5_hardware.mock_spi import MockSpiWorker  # noqa: E402
from jonny5_hardware.spi_driver_node import (  # noqa: E402
    DEFAULT_SERVO_DIRS,
    DEFAULT_SERVO_OFFSETS_DEG,
    joint_rad_to_physical_cdeg,
    physical_deg_to_joint_rad,
    v2_diag_mask,
)


def test_crc16_ccitt_false_check_value():
    assert v2.crc16_ccitt(b"123456789") == 0x29B1


def test_j5vr_v1_frame_is_unchanged_and_v2_is_sealed():
    state = {"mode": 2, "joy_x": 0.5, "buttons_left": 2, "buttons_right": 2, "heartbeat": 7}
    f1 = build_setpoint_frame(state, sequence_counter=5).to_bytes()
    f2 = build_setpoint_frame(state, sequence_counter=5, protocol_version=2).to_bytes()
    assert f1[2] == 1 and f1[7] == 0 and f1[62:64] == b"\x00\x00"
    assert f2[2] == 2 and f2[7] == v2.FLAG_CRC16 and v2.crc_ok(f2)
    # Same header fields and payload: only version, flags and CRC differ.
    assert f1[3:7] == f2[3:7] and f1[8:62] == f2[8:62]


def test_crc_detects_single_bit_flip():
    frame = bytearray(v2.build_request_v2(v2.FRAME_TYPE_STATUS, 42))
    assert v2.crc_ok(bytes(frame))
    frame[20] ^= 0x01
    assert not v2.crc_ok(bytes(frame))


def test_j5ik_frame_layout():
    targets = [10000, 8800, 9300, 9500, 9000, 9500]
    f = v2.build_j5ik_frame(targets, sequence=3, heartbeat=99, enable=True, target_id=7)
    assert v2.crc_ok(f) and f[3] == v2.FRAME_TYPE_J5IK
    p = f[8:62]
    assert p[0] == 1 and p[1] & v2.J5IK_FLAG_STREAM_ENABLE and p[6] == v2.MODE_JOINT_STREAM
    assert struct.unpack_from(">HH", p, 2) == (7, 99)
    assert list(struct.unpack_from(">6h", p, 8)) == targets
    off = v2.build_j5ik_frame(targets, sequence=3, heartbeat=99, enable=False)
    assert not off[8 + 1] & v2.J5IK_FLAG_STREAM_ENABLE


def test_telemetry_v2_round_trip():
    joints = [9012, 8801, 9300, 9500, 9000, 9555]
    f = v2.build_telemetry_v2(
        sequence=11, stm_time_ms=123456, stm_tx_seq=77, fsm_state=1,
        status_flags=v2.ST_ESTOP | v2.ST_MOVE_ALLOWED | v2.ST_IMU_VALID,
        mode=6, joint_cdeg=joints, quat=(0.7071, 0.0, 0.0, 0.7071),
        gyro=(0.1, -0.2, 0.3), accel=(0.0, 0.0, 9.81), imu_sample_counter=65537,
        rt_loop_period_us=1000, rt_step_us=43, rt_overruns=2, crc_errors=1,
        rx_seq_gaps=5, diag_flags=v2.DG_STREAM_LIVE,
    )
    t = v2.parse_telemetry_v2(f)
    assert t["stm_time_ms"] == 123456 and t["stm_tx_seq"] == 77 and t["rx_seq_gaps"] == 5
    assert t["fsm_state_name"] == "IDLE" and t["estop_active"] and t["movement_allowed"]
    assert t["joint_cdeg"] == joints and t["servo_deg_R"] == pytest.approx(95.55)
    assert t["imu_q_w"] == pytest.approx(0.7071, abs=1e-4)
    assert t["imu_accel_z"] == pytest.approx(9.81) and t["imu_gyro_y"] == pytest.approx(-0.2)
    assert t["imu_sample_counter"] == 1  # LSB16
    assert t["rt_step_us"] == 43 and t["rt_overruns"] == 2 and t["crc_errors_stm"] == 1
    assert t["joint_stream_active"] and t["mode"] == 6
    assert not t["pose_known"]  # no SETPOSE since boot: angles are init defaults
    corrupted = bytearray(f)
    corrupted[30] ^= 0xFF
    assert v2.parse_telemetry_v2(bytes(corrupted)) is None


def test_telemetry_v2_pose_known_flag():
    f = v2.build_telemetry_v2(
        sequence=1, stm_time_ms=0, stm_tx_seq=1, fsm_state=1, status_flags=0, mode=0,
        joint_cdeg=[10000, 8800, 9300, 9500, 9000, 9500], diag_flags=v2.DG_POSE_KNOWN,
    )
    t = v2.parse_telemetry_v2(f)
    assert t["pose_known"] and not t["joint_stream_active"]


def test_transport_extracts_v2_frame_from_padded_128():
    f = v2.build_request_v2(v2.FRAME_TYPE_STATUS, 1)
    assert extract_canonical_frame64_from_transport_rx(f + bytes(64)) == f


class _Provider:
    def __init__(self):
        self.telemetry = []
        self.intent = None

    def read_intent_from_file(self):
        return self.intent

    def write_telemetry_to_file(self, t):
        self.telemetry.append(t)


def _bridge(protocol_version):
    spi = MockSpiWorker()
    spi.open()
    provider = _Provider()
    return J5VRSPIBridge(spi_worker=spi, state_provider=provider, protocol_version=protocol_version), provider


def test_bridge_v2_setpoint_yields_telemetry_v2():
    bridge, provider = _bridge(2)
    assert bridge.send_setpoint_once()
    t = provider.telemetry[-1]
    assert t["protocol_version"] == 2 and t["wire_source"] == "v2_0x08"
    assert t["fsm_state_name"] == "IDLE" and "servo_deg_B" in t


def test_bridge_v1_default_is_legacy_telemetry():
    bridge, provider = _bridge(1)
    assert bridge.send_setpoint_once()
    assert provider.telemetry[-1]["wire_source"] == "legacy_0x01"


def test_bridge_joint_stream_moves_mock_joints():
    bridge, provider = _bridge(2)
    target = [10000, 8800, 9300, 9500, 9000, 9500]
    for hb in range(1, 80):
        bridge.send_joint_targets_once(target, enable=True, heartbeat=hb)
    t = provider.telemetry[-1]
    assert t["joint_cdeg"] == target and t["joint_stream_active"]


def test_bridge_counts_corrupted_replies():
    bridge, provider = _bridge(2)
    real_transfer = bridge.spi_worker.transfer

    def corrupt(tx):
        rx = bytearray(real_transfer(tx))
        rx[40] ^= 0x55
        return bytes(rx)

    bridge.spi_worker.transfer = corrupt
    bridge.send_setpoint_once()
    assert bridge.v2_rx_crc_errors == 1 and provider.telemetry == []


def test_joint_stream_requires_v2():
    bridge, _ = _bridge(1)
    with pytest.raises(RuntimeError):
        bridge.send_joint_targets_once([9000] * 6, enable=True, heartbeat=1)


def test_joint_rad_physical_round_trip():
    rad = [0.1, -0.2, 0.3, 0.0, -0.4, 0.25]
    cdeg = joint_rad_to_physical_cdeg(rad, DEFAULT_SERVO_OFFSETS_DEG, DEFAULT_SERVO_DIRS)
    back = physical_deg_to_joint_rad([c / 100.0 for c in cdeg], DEFAULT_SERVO_OFFSETS_DEG, DEFAULT_SERVO_DIRS)
    for a, b in zip(rad, back):
        assert math.isclose(a, b, abs_tol=math.radians(0.01))


def test_v2_diag_mask_matches_legacy_bits():
    assert v2_diag_mask({"deadman_active": True, "armed": True, "guard_seen": True}) == 0b10101
