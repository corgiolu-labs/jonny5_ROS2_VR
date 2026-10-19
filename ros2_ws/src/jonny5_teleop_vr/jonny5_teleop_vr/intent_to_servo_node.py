"""VR TeleopIntent -> MoveIt Servo (VR teleoperation on ros2_control).

command_mode "joint" (default): each stick axis drives one joint (Servo JOINT_JOG):
  left stick  X (joy_x) -> base_joint       (stick right: base turns right)
  left stick  Y (joy_y) -> shoulder_joint   (stick forward: shoulder forward)
  right stick Y (pitch) -> elbow_joint      (stick up: forearm up, tool rises)
  right stick X (yaw)   -> wrist_yaw_joint  (stick right: wrist turns right)
  Mapping and signs are parameters (joint_axes, joint_signs).

command_mode "twist" (experimental): sticks drive the tool in Cartesian space:
  left stick  (joy_y, joy_x) -> linear x (forward) / linear y (left)
  right stick (pitch, yaw)   -> linear z (up)      / angular z (turn left)
  On JONNY5 this is ill-conditioned almost everywhere (60 mm between the
  shoulder and elbow axes): Servo scales and bends the commanded direction and
  joints move a lot for small tool motions. Hardware test, phase 8.

Motion requires the deadman (both grips, buttons bit1), a non-IDLE mode and an
intent younger than ``intent_timeout_s``. Otherwise nothing is published and
Servo halts after its own ``incoming_command_timeout``.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from jonny5_msgs.msg import TeleopIntent
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

GRIP_BIT = 1 << 1
# Stick axes in TeleopIntent order (joy_x, joy_y, pitch, yaw) -> joint, sign.
DEFAULT_JOINT_AXES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_yaw_joint"]
DEFAULT_JOINT_SIGNS = [-1.0, 1.0, -1.0, -1.0]


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


def intent_to_joint_velocities(msg: TeleopIntent, signs: Sequence[float], max_vel: float,
                               deadzone: float) -> Optional[Tuple[float, ...]]:
    """Joint velocities (rad/s) for (joy_x, joy_y, pitch, yaw), or None when the
    intent must not move the arm."""
    if msg.mode == TeleopIntent.MODE_IDLE or not deadman_active(msg):
        return None
    axes = (msg.joy_x, msg.joy_y, msg.pitch, msg.yaw)
    return tuple(float(s) * _axis(a, deadzone) * max_vel for a, s in zip(axes, signs))


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
        self.declare_parameter("command_mode", "joint")
        self.declare_parameter("joint_axes", DEFAULT_JOINT_AXES)
        self.declare_parameter("joint_signs", DEFAULT_JOINT_SIGNS)
        self.declare_parameter("max_joint_vel", 0.35)   # rad/s at full stick

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
        self._was_moving = False
        self.mode = str(self.get_parameter("command_mode").value).lower()
        if self.mode not in ("joint", "twist"):
            raise ValueError(f"command_mode must be 'joint' or 'twist', got {self.mode!r}")
        self.joint_axes = list(self.get_parameter("joint_axes").value)
        self.joint_signs = [float(v) for v in self.get_parameter("joint_signs").value]
        if len(self.joint_axes) != 4 or len(self.joint_signs) != 4:
            raise ValueError("joint_axes and joint_signs need 4 entries (joy_x, joy_y, pitch, yaw)")
        self.max_joint_vel = float(self.get_parameter("max_joint_vel").value)
        self._mode_set = False
        self.pub = self.create_publisher(TwistStamped, f"{servo}/delta_twist_cmds", 10)
        self.jog_pub = self.create_publisher(JointJog, f"{servo}/delta_joint_cmds", 10)
        self.create_subscription(TeleopIntent, "jonny5/teleop/intent", self._on_intent, 10)
        self._switch_cli = self.create_client(ServoCommandType, f"{servo}/switch_command_type")
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._ensure_twist_mode)

    def _on_intent(self, msg: TeleopIntent) -> None:
        self._intent = msg
        self._intent_mono = time.monotonic()

    def _ensure_twist_mode(self) -> None:
        if self._mode_set or not self._switch_cli.service_is_ready():
            return
        req = ServoCommandType.Request()
        req.command_type = (ServoCommandType.Request.JOINT_JOG if self.mode == "joint"
                            else ServoCommandType.Request.TWIST)
        self._switch_cli.call_async(req).add_done_callback(self._on_switched)

    def _on_switched(self, future) -> None:
        try:
            ok = bool(future.result().success)
        except Exception:  # service went away: retry on the next timer tick
            ok = False
        if ok and not self._mode_set:
            self.get_logger().info("MoveIt Servo switched to %s commands"
                                   % ("JOINT_JOG" if self.mode == "joint" else "TWIST"))
        self._mode_set = ok

    def _tick(self) -> None:
        cmd = None
        if self._intent is not None and (time.monotonic() - self._intent_mono) <= self.timeout:
            if self.mode == "joint":
                cmd = intent_to_joint_velocities(self._intent, self.joint_signs,
                                                 self.max_joint_vel, self.limits.deadzone)
            else:
                cmd = intent_to_twist(self._intent, self.limits)
        if cmd is None:
            if self._was_moving:
                # Deadman released / stream lost: one explicit zero command so Servo
                # starts decelerating now instead of after incoming_command_timeout.
                self._send((0.0,) * (4 if self.mode == "joint" else 6))
                self._was_moving = False
            return
        self._was_moving = True
        self._send(cmd)

    def _send(self, cmd: Tuple[float, ...]) -> None:
        if self.mode == "joint":
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = self.joint_axes
            msg.velocities = list(cmd)
            self.jog_pub.publish(msg)
        else:
            self._publish(cmd)

    def _publish(self, twist: Tuple[float, ...]) -> None:
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
