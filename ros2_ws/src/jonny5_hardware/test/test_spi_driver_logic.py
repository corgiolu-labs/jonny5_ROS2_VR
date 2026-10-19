"""Unit tests for the pure logic of spi_driver_node (joint conversion, stale intent)."""

import math

from jonny5_hardware.spi_driver_node import (
    DEFAULT_SERVO_DIRS,
    DEFAULT_SERVO_OFFSETS_DEG,
    Ros2StateProvider,
    physical_deg_to_joint_rad,
)


class _FakeLogger:
    def warning(self, _msg):
        pass


class _FakeNode:
    def get_logger(self):
        return _FakeLogger()


def test_home_pose_is_zero_rad():
    joints = physical_deg_to_joint_rad(
        DEFAULT_SERVO_OFFSETS_DEG, DEFAULT_SERVO_OFFSETS_DEG, DEFAULT_SERVO_DIRS
    )
    assert joints == [0.0] * 6


def test_direction_and_offset_are_applied():
    offsets = [100.0, 88.0, 93.0, 95.0, 90.0, 95.0]
    dirs = [1, -1, -1, 1, -1, 1]
    physical = [110.0, 98.0, 83.0, 95.0, 90.0, 85.0]
    joints = physical_deg_to_joint_rad(physical, offsets, dirs)
    expected_deg = [10.0, -10.0, 10.0, 0.0, 0.0, -10.0]
    for got, exp in zip(joints, expected_deg):
        assert math.isclose(got, math.radians(exp), abs_tol=1e-9)


def test_stale_intent_is_dropped(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(
        "jonny5_hardware.spi_driver_node.time.monotonic", lambda: clock[0]
    )
    provider = Ros2StateProvider(_FakeNode(), intent_timeout_s=0.25)
    assert provider.read_intent_from_file() is None

    intent = {"mode": 1, "buttons_left": 2, "buttons_right": 2}
    provider.set_intent(intent)
    clock[0] += 0.2
    assert provider.read_intent_from_file() is intent

    clock[0] += 0.1  # 0.3 s since the last intent
    assert provider.read_intent_from_file() is None

    provider.set_intent(intent)
    assert provider.read_intent_from_file() is intent


def test_zero_timeout_disables_watchdog(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(
        "jonny5_hardware.spi_driver_node.time.monotonic", lambda: clock[0]
    )
    provider = Ros2StateProvider(_FakeNode(), intent_timeout_s=0.0)
    intent = {"mode": 0}
    provider.set_intent(intent)
    clock[0] += 1000.0
    assert provider.read_intent_from_file() is intent
