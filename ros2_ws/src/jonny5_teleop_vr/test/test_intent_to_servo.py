import pytest

from jonny5_msgs.msg import TeleopIntent

from jonny5_teleop_vr.intent_to_servo_node import (
    DEFAULT_JOINT_SIGNS, TwistLimits, intent_to_joint_velocities, intent_to_twist)

LIM = TwistLimits(max_linear=0.05, max_angular=0.5, deadzone=0.08)


def _intent(**kw):
    msg = TeleopIntent()
    msg.mode = TeleopIntent.MODE_MANUAL
    msg.buttons_left = 2
    msg.buttons_right = 2
    for k, v in kw.items():
        setattr(msg, k, v)
    return msg


def test_no_deadman_no_motion():
    assert intent_to_twist(_intent(buttons_right=0, joy_y=32767), LIM) is None


def test_idle_mode_no_motion():
    assert intent_to_twist(_intent(mode=TeleopIntent.MODE_IDLE, joy_y=32767), LIM) is None


def test_full_stick_maps_to_limits():
    vx, vy, vz, _, _, wz = intent_to_twist(
        _intent(joy_y=32767, joy_x=32767, pitch=-32767, yaw=32767), LIM)
    assert vx == pytest.approx(0.05)
    assert vy == pytest.approx(-0.05)
    assert vz == pytest.approx(-0.05)
    assert wz == pytest.approx(-0.5)


def test_deadzone():
    assert intent_to_twist(_intent(joy_y=2000), LIM) == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_joint_mode_needs_deadman():
    assert intent_to_joint_velocities(
        _intent(buttons_left=0, joy_y=32767), DEFAULT_JOINT_SIGNS, 0.35, 0.08) is None
    assert intent_to_joint_velocities(
        _intent(mode=TeleopIntent.MODE_IDLE, joy_y=32767), DEFAULT_JOINT_SIGNS, 0.35, 0.08) is None


def test_joint_mode_axes_and_signs():
    # joy_x right -> base right (-), joy_y forward -> shoulder forward (+),
    # stick up -> elbow up (-), yaw right -> wrist yaw right (-).
    base, shoulder, elbow, wrist_yaw = intent_to_joint_velocities(
        _intent(joy_x=32767, joy_y=32767, pitch=32767, yaw=32767), DEFAULT_JOINT_SIGNS, 0.35, 0.08)
    assert base == pytest.approx(-0.35) and shoulder == pytest.approx(0.35)
    assert elbow == pytest.approx(-0.35) and wrist_yaw == pytest.approx(-0.35)


def test_joint_mode_deadzone_and_half_stick():
    v = intent_to_joint_velocities(_intent(joy_y=2000), DEFAULT_JOINT_SIGNS, 0.35, 0.08)
    assert v == (0.0, 0.0, 0.0, 0.0)
    half = intent_to_joint_velocities(_intent(joy_y=16384), DEFAULT_JOINT_SIGNS, 0.35, 0.08)[1]
    assert 0.15 < half < 0.17
