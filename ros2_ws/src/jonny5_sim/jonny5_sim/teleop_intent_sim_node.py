import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from jonny5_msgs.msg import TeleopIntent
from rclpy.node import Node


class TeleopIntentSimNode(Node):
    """Publish deterministic VR-like TeleopIntent messages for dry-run tests."""

    def __init__(self) -> None:
        super().__init__("jonny5_teleop_intent_sim")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.publisher = self.create_publisher(TeleopIntent, "jonny5/teleop/intent", 10)
        self.sequence = 0
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info("Teleop intent simulator publishing /jonny5/teleop/intent")

    def _tick(self) -> None:
        t = self.sequence / 20.0
        yaw = 0.35 * math.sin(t * 0.8)
        half = yaw / 2.0

        msg = TeleopIntent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = TeleopIntent.MODE_MANUAL
        msg.joy_x = int(12000 * math.sin(t * 0.7))
        msg.joy_y = int(9000 * math.cos(t * 0.5))
        msg.pitch = int(6000 * math.sin(t * 0.9))
        msg.yaw = int(6000 * math.cos(t * 0.9))
        msg.intensity = 128
        msg.grip = False
        msg.heartbeat = self.sequence & 0xFFFF
        msg.priority = 0
        msg.safe_mask = 0
        msg.headset_orientation = Quaternion(
            w=math.cos(half),
            x=0.0,
            y=0.0,
            z=math.sin(half),
        )
        msg.buttons_left = 0
        msg.buttons_right = 0
        self.publisher.publish(msg)
        self.sequence += 1


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeleopIntentSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()