import asyncio
import json
import threading
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from jonny5_msgs.msg import TeleopIntent
from rclpy.node import Node

try:
    import websockets
except ImportError:
    websockets = None


class WebSocketTeleopBridgeNode(Node):
    """Accept current WebXR intent JSON and publish typed ROS2 TeleopIntent."""

    def __init__(self) -> None:
        super().__init__("jonny5_vr_bridge")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8567)

        self.host = str(self.get_parameter("bind_host").value)
        self.port = int(self.get_parameter("bind_port").value)
        self.publisher = self.create_publisher(TeleopIntent, "jonny5/teleop/intent", 10)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "JONNY5 VR WebSocket ROS2 bridge listening on ws://%s:%d",
            self.host,
            self.port,
        )

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())
        self._loop.run_forever()

    async def _serve(self) -> None:
        if websockets is None:
            self.get_logger().error("Python package 'websockets' is required")
            return
        await websockets.serve(self._handle_client, self.host, self.port, ping_interval=20, ping_timeout=10)

    async def _handle_client(self, websocket, path=None) -> None:
        self.get_logger().info("VR bridge client connected: %s", getattr(websocket, "remote_address", "?"))
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            msg = self._intent_from_json(data)
            if msg is not None:
                self.publisher.publish(msg)

    def _intent_from_json(self, data: Dict[str, Any]) -> Optional[TeleopIntent]:
        if not isinstance(data, dict):
            return None
        mode = data.get("mode")
        if isinstance(mode, str):
            mode = {"IDLE": 0, "RELATIVE_MOVE": 1, "ABSOLUTE_POSE": 2}.get(mode)
        if not isinstance(mode, int) or mode < 0 or mode > 5:
            return None

        msg = TeleopIntent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = int(mode)
        msg.joy_x = self._axis_i16(data.get("joy_x", 0))
        msg.joy_y = self._axis_i16(data.get("joy_y", 0))
        msg.pitch = self._axis_i16(data.get("pitch", 0))
        msg.yaw = self._axis_i16(data.get("yaw", 0))
        msg.intensity = self._u8(data.get("intensity", 0))
        msg.grip = bool(data.get("grip", 0))
        msg.heartbeat = int(data.get("heartbeat", data.get("vr_heartbeat", 0)) or 0) & 0xFFFF
        msg.priority = int(data.get("priority", 0) or 0) & 0xFF
        msg.safe_mask = int(data.get("safe_mask", 0) or 0) & 0xFFFF
        msg.headset_orientation = Quaternion(
            w=float(data.get("quat_w", 1.0) or 1.0),
            x=float(data.get("quat_x", 0.0) or 0.0),
            y=float(data.get("quat_y", 0.0) or 0.0),
            z=float(data.get("quat_z", 0.0) or 0.0),
        )
        msg.buttons_left = int(data.get("buttons_left", 0) or 0) & 0xFFFF
        msg.buttons_right = int(data.get("buttons_right", 0) or 0) & 0xFFFF

        mode5 = data.get("mode5_arm") if isinstance(data.get("mode5_arm"), dict) else {}
        msg.mode5_arm_valid = bool(mode5.get("valid", False))
        msg.mode5_grip_active = bool(mode5.get("grip_active", False))
        msg.mode5_hold_active = bool(mode5.get("hold_active", False))
        msg.mode5_target_id = int(mode5.get("target_id", 0) or 0) & 0xFFFF
        physical = mode5.get("physical_deg") if isinstance(mode5.get("physical_deg"), list) else []
        if len(physical) >= 3:
            msg.mode5_base_deg = float(physical[0])
            msg.mode5_shoulder_deg = float(physical[1])
            msg.mode5_elbow_deg = float(physical[2])
        return msg

    @staticmethod
    def _axis_i16(value: Any) -> int:
        try:
            if isinstance(value, int):
                return max(-32768, min(32767, value))
            value_f = max(-1.0, min(1.0, float(value)))
            return int(round(value_f * 32767.0))
        except Exception:
            return 0

    @staticmethod
    def _u8(value: Any) -> int:
        try:
            if isinstance(value, int):
                return max(0, min(255, value))
            value_f = max(0.0, min(1.0, float(value)))
            return int(round(value_f * 255.0))
        except Exception:
            return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebSocketTeleopBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
