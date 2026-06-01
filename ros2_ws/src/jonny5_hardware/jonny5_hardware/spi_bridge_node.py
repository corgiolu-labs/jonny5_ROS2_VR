import json
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from jonny5_msgs.msg import RobotStatus, SpiTelemetry, TeleopIntent
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


SERVO_KEYS = [
    "servo_deg_B",
    "servo_deg_S",
    "servo_deg_G",
    "servo_deg_Y",
    "servo_deg_P",
    "servo_deg_R",
]


class SpiBridgeNode(Node):
    """Bridge the current JONNY5 SPI/telemetry path into ROS2 topics.

    This first migration phase defaults to dry-run mode. It reads the existing
    telemetry JSON file and publishes ROS2 state. When hardware is enabled, it
    also writes incoming TeleopIntent messages to the legacy intent file so the
    existing SPI service can consume them.
    """

    def __init__(self) -> None:
        super().__init__("jonny5_spi_bridge")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("telemetry_file", "/dev/shm/j5vr_telemetry.json")
        self.declare_parameter("intent_file", "/dev/shm/j5vr_latest_intent.json")
        self.declare_parameter("joint_names", [
            "base_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_yaw_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
        ])

        self.hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self.telemetry_file = Path(str(self.get_parameter("telemetry_file").value))
        self.intent_file = Path(str(self.get_parameter("intent_file").value))
        self.joint_names = [str(x) for x in self.get_parameter("joint_names").value]
        self._last_mtime: Optional[float] = None
        self._cached_telemetry: Dict[str, Any] = {}

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.telemetry_pub = self.create_publisher(SpiTelemetry, "jonny5/spi/telemetry", 10)
        self.status_pub = self.create_publisher(RobotStatus, "jonny5/status", 10)
        self.intent_sub = self.create_subscription(
            TeleopIntent,
            "jonny5/teleop/intent",
            self._on_intent,
            10,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish_state)
        self.get_logger().info(
            "JONNY5 SPI bridge started (hardware_enabled=%s, telemetry_file=%s)",
            self.hardware_enabled,
            self.telemetry_file,
        )

    def _on_intent(self, msg: TeleopIntent) -> None:
        if not self.hardware_enabled:
            return
        payload = self._intent_to_legacy_dict(msg)
        self.intent_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.intent_file.with_suffix(self.intent_file.suffix + ".tmp")
        tmp.write_text(json.dumps(payload), encoding="utf-8")
        os.replace(tmp, self.intent_file)

    def _publish_state(self) -> None:
        telemetry, fresh = self._read_telemetry()
        now = self.get_clock().now().to_msg()

        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = self.joint_names
        joint_msg.position = [
            math.radians(float(telemetry.get(key, 90.0)) - 90.0)
            for key in SERVO_KEYS
        ]
        self.joint_pub.publish(joint_msg)

        q = self._quaternion_from_telemetry(telemetry)
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation = q
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

        spi_msg = SpiTelemetry()
        spi_msg.stamp = now
        spi_msg.header_ok = bool(telemetry.get("header_ok", fresh))
        spi_msg.telemetry_fresh = fresh
        spi_msg.imu_valid = bool(telemetry.get("imu_valid", False))
        spi_msg.imu_sample_counter = int(telemetry.get("imu_sample_counter", 0) or 0)
        spi_msg.imu_orientation = q
        spi_msg.servo_deg = [float(telemetry.get(key, 90.0)) for key in SERVO_KEYS]
        spi_msg.raw_mode = int(telemetry.get("intent_mode", telemetry.get("mode", 0)) or 0)
        spi_msg.raw_heartbeat = int(telemetry.get("vr_heartbeat", 0) or 0) & 0xFFFF
        spi_msg.diag_mask = int(telemetry.get("diag_mask", 0) or 0) & 0xFFFF
        spi_msg.rt_loop_period_us = int(telemetry.get("rt_loop_period_us", 0) or 0) & 0xFFFF
        spi_msg.rt_step_us = int(telemetry.get("rt_step_us", 0) or 0) & 0xFFFF
        self.telemetry_pub.publish(spi_msg)

        status_msg = RobotStatus()
        status_msg.stamp = now
        status_msg.state = str(telemetry.get("robot_state", "UNKNOWN" if not fresh else "IDLE"))
        status_msg.spi_online = fresh
        status_msg.stm32_online = fresh and any(key in telemetry for key in SERVO_KEYS)
        status_msg.imu_online = bool(telemetry.get("imu_valid", False))
        status_msg.deadman_active = bool(telemetry.get("deadman", False))
        status_msg.input_active = bool(telemetry.get("input_active", False))
        status_msg.movement_allowed = status_msg.state not in ("SAFE", "STOPPED", "UNKNOWN")
        status_msg.detail = "legacy telemetry bridge"
        self.status_pub.publish(status_msg)

    def _read_telemetry(self) -> tuple[Dict[str, Any], bool]:
        try:
            stat = self.telemetry_file.stat()
        except OSError:
            return self._cached_telemetry, False
        fresh = True
        if self._last_mtime == stat.st_mtime:
            return self._cached_telemetry, fresh
        self._last_mtime = stat.st_mtime
        try:
            self._cached_telemetry = json.loads(self.telemetry_file.read_text(encoding="utf-8"))
        except Exception as exc:
            self.get_logger().warning("Failed to read telemetry JSON: %s", exc)
        return self._cached_telemetry, fresh

    @staticmethod
    def _quaternion_from_telemetry(telemetry: Dict[str, Any]) -> Quaternion:
        q = Quaternion()
        q.w = float(telemetry.get("imu_q_w", telemetry.get("imu_w", 1.0)) or 1.0)
        q.x = float(telemetry.get("imu_q_x", telemetry.get("imu_x", 0.0)) or 0.0)
        q.y = float(telemetry.get("imu_q_y", telemetry.get("imu_y", 0.0)) or 0.0)
        q.z = float(telemetry.get("imu_q_z", telemetry.get("imu_z", 0.0)) or 0.0)
        return q

    @staticmethod
    def _intent_to_legacy_dict(msg: TeleopIntent) -> Dict[str, Any]:
        return {
            "mode": int(msg.mode),
            "joy_x": int(msg.joy_x),
            "joy_y": int(msg.joy_y),
            "pitch": int(msg.pitch),
            "yaw": int(msg.yaw),
            "intensity": int(msg.intensity),
            "grip": 1 if msg.grip else 0,
            "heartbeat": int(msg.heartbeat),
            "priority": int(msg.priority),
            "safe_mask": int(msg.safe_mask),
            "quat_w": float(msg.headset_orientation.w),
            "quat_x": float(msg.headset_orientation.x),
            "quat_y": float(msg.headset_orientation.y),
            "quat_z": float(msg.headset_orientation.z),
            "buttons_left": int(msg.buttons_left),
            "buttons_right": int(msg.buttons_right),
            "mode5_arm": {
                "valid": bool(msg.mode5_arm_valid),
                "grip_active": bool(msg.mode5_grip_active),
                "hold_active": bool(msg.mode5_hold_active),
                "target_id": int(msg.mode5_target_id),
                "physical_deg": [
                    float(msg.mode5_base_deg),
                    float(msg.mode5_shoulder_deg),
                    float(msg.mode5_elbow_deg),
                ],
            },
        }


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SpiBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
