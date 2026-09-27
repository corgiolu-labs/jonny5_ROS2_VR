import pytest

from jonny5_msgs.msg import TeleopIntent

from jonny5_teleop_vr.intent_to_servo_node import TwistLimits, intent_to_twist

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
