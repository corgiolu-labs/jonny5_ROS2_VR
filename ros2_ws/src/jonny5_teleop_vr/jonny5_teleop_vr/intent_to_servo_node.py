"""VR TeleopIntent -> MoveIt Servo twist (VR teleoperation on ros2_control).

Sticks drive the tool in Cartesian space:
  left stick  (joy_y, joy_x) -> linear x (forward) / linear y (left)
  right stick (pitch, yaw)   -> linear z (up)      / angular z (turn left)

Motion requires the deadman (both grips, buttons bit1), a non-IDLE mode and an
intent younger than ``intent_timeout_s``. Otherwise nothing is published and
Servo halts after its own ``incoming_command_timeout``.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from jonny5_msgs.msg import TeleopIntent
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

GRIP_BIT = 1 << 1


@dataclass
class TwistLimits:
    max_linear: float = 0.05    # m/s at full stick
    max_angular: float = 0.5    # rad/s at full stick
    deadzone: float = 0.08      # normalised stick deadzone


def _axis(raw: int, deadzone: float) -> float:
    v = max(-1.0, min(1.0, raw / 32767.0))
    if abs(v) < deadzone:
        return 0.0
    # Re-scale so the output starts from 0 at the edge of the deadzone.
    return (abs(v) - deadzone) / (1.0 - deadzone) * (1.0 if v > 0 else -1.0)


def deadman_active(msg: TeleopIntent) -> bool:
    return bool(msg.buttons_left & GRIP_BIT) and bool(msg.buttons_right & GRIP_BIT)


def intent_to_twist(msg: TeleopIntent, limits: TwistLimits) -> Optional[Tuple[float, ...]]:
    """Return (vx, vy, vz, wx, wy, wz) or None when the intent must not move the arm."""
    if msg.mode == TeleopIntent.MODE_IDLE or not deadman_active(msg):
        return None
    dz = limits.deadzone
    vx = _axis(msg.joy_y, dz) * limits.max_linear
    vy = -_axis(msg.joy_x, dz) * limits.max_linear
    vz = _axis(msg.pitch, dz) * limits.max_linear
    wz = -_axis(msg.yaw, dz) * limits.max_angular
    return (vx, vy, vz, 0.0, 0.0, wz)


class IntentToServoNode(Node):
    def __init__(self) -> None:
        super().__init__("jonny5_intent_to_servo")
        self.declare_parameter("max_linear", 0.05)
        self.declare_parameter("max_angular", 0.5)
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("intent_timeout_s", 0.2)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("servo_node", "/servo_node")

        self.limits = TwistLimits(
            float(self.get_parameter("max_linear").value),
            float(self.get_parameter("max_angular").value),
            float(self.get_parameter("deadzone").value),
        )
        self.timeout = float(self.get_parameter("intent_timeout_s").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        servo = str(self.get_parameter("servo_node").value).rstrip("/")

        self._intent: Optional[TeleopIntent] = None
        self._intent_mono = 0.0
        self._twist_mode_set = False

        self.pub = self.create_publisher(TwistStamped, f"{servo}/delta_twist_cmds", 10)
        self.create_subscription(TeleopIntent, "jonny5/teleop/intent", self._on_intent, 10)
        self._switch_cli = self.create_client(ServoCommandType, f"{servo}/switch_command_type")
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._ensure_twist_mode)

    def _on_intent(self, msg: TeleopIntent) -> None:
        self._intent = msg
        self._intent_mono = time.monotonic()

    def _ensure_twist_mode(self) -> None:
        if self._twist_mode_set or not self._switch_cli.service_is_ready():
            return
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        self._switch_cli.call_async(req).add_done_callback(self._on_switched)

    def _on_switched(self, future) -> None:
        try:
            ok = bool(future.result().success)
        except Exception:  # service went away: retry on the next timer tick
            ok = False
        if ok and not self._twist_mode_set:
            self.get_logger().info("MoveIt Servo switched to TWIST commands")
        self._twist_mode_set = ok

    def _tick(self) -> None:
        if self._intent is None or (time.monotonic() - self._intent_mono) > self.timeout:
            return
        twist = intent_to_twist(self._intent, self.limits)
        if twist is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
         msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z) = twist
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntentToServoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
