import json
import math
import os
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node


class LegacyTelemetrySimNode(Node):
    """Write fake legacy telemetry JSON for the ROS2 SPI bridge.

    This simulator intentionally targets the existing `/dev/shm` file contract
    so the first migration layer can be tested without Raspberry Pi hardware.
    """

    def __init__(self) -> None:
        super().__init__("jonny5_legacy_telemetry_sim")
        self.declare_parameter("telemetry_file", "/dev/shm/j5vr_telemetry.json")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.telemetry_file = Path(str(self.get_parameter("telemetry_file").value))
        self.sample_counter = 0
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(f"Legacy telemetry simulator writing {self.telemetry_file}")

    def _tick(self) -> None:
        t = self.sample_counter / 50.0
        servo = [
            90.0 + 20.0 * math.sin(t * 0.6),
            90.0 + 15.0 * math.sin(t * 0.7 + 0.5),
            90.0 + 12.0 * math.sin(t * 0.8 + 1.0),
            90.0 + 10.0 * math.sin(t * 1.2),
            90.0 + 8.0 * math.sin(t * 1.1 + 0.3),
            90.0 + 7.0 * math.sin(t * 1.4 + 0.8),
        ]
        yaw = 0.25 * math.sin(t * 0.5)
        half = yaw / 2.0
        payload = {
            "header_ok": True,
            "robot_state": "IDLE",
            "imu_valid": True,
            "imu_sample_counter": self.sample_counter,
            "imu_q_w": math.cos(half),
            "imu_q_x": 0.0,
            "imu_q_y": 0.0,
            "imu_q_z": math.sin(half),
            "servo_deg_B": servo[0],
            "servo_deg_S": servo[1],
            "servo_deg_G": servo[2],
            "servo_deg_Y": servo[3],
            "servo_deg_P": servo[4],
            "servo_deg_R": servo[5],
            "intent_mode": 0,
            "vr_heartbeat": self.sample_counter & 0xFFFF,
            "diag_mask": 0,
            "rt_loop_period_us": 1000,
            "rt_step_us": 45,
            "deadman": False,
            "input_active": False,
        }
        self.telemetry_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.telemetry_file.with_suffix(self.telemetry_file.suffix + ".tmp")
        tmp.write_text(json.dumps(payload), encoding="utf-8")
        os.replace(tmp, self.telemetry_file)
        self.sample_counter += 1


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LegacyTelemetrySimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()