"""Native ROS2 SPI driver for JONNY5.

This node owns the SPI data plane to the STM32 directly, *reusing the proven legacy
codec unchanged* (``controller.spi_dataplane.j5vr_spi_bridge.J5VRSPIBridge`` +
``SPIWorker`` + ``j5vr_frame``). It does so by injecting a ROS2-backed
``state_provider`` that is a drop-in for the legacy ``shared_state`` module:

- ``read_intent_from_file()``  -> latest ``TeleopIntent`` (instead of /dev/shm JSON)
- ``write_telemetry_to_file()`` -> publishes ROS2 topics (instead of /dev/shm JSON)

Because telemetry is published straight from the parsed RX dict, no field is lost in a
JSON round-trip. See ADR-001.

Dry-run: with ``use_mock_spi:=true`` a synthetic SPI worker fabricates protocol-valid
telemetry frames, so the full ROS2 graph runs without a Raspberry Pi.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from jonny5_msgs.msg import RobotStatus, SpiTelemetry, TeleopIntent
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from jonny5_hardware.mock_spi import MockSpiWorker

SERVO_KEYS = [
    "servo_deg_B",
    "servo_deg_S",
    "servo_deg_G",
    "servo_deg_Y",
    "servo_deg_P",
    "servo_deg_R",
]


def resolve_legacy_root(explicit: str = "") -> Optional[Path]:
    """Find the directory that contains the legacy ``controller`` package.

    Order: explicit param -> ``JONNY5_LEGACY_ROOT`` env -> walk up from this file ->
    common deploy locations. Returns the path to add to ``sys.path`` (the parent of
    ``controller/``), or None if not found.
    """
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env = os.environ.get("JONNY5_LEGACY_ROOT", "").strip()
    if env:
        candidates.append(Path(env))
    # Walk up from this source file looking for <root>/controller/spi_dataplane.
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / "raspberry")
        candidates.append(parent)
    candidates.append(Path.home() / "JONNY5_ROS2" / "raspberry")
    candidates.append(Path("/home/jonny5/raspberry5"))

    for cand in candidates:
        if (cand / "controller" / "spi_dataplane" / "j5vr_spi_bridge.py").is_file():
            return cand
    return None


class Ros2StateProvider:
    """Drop-in for the legacy ``shared_state`` module, backed by ROS2.

    ``J5VRSPIBridge`` consumes a ``state_provider`` object via duck typing:
    ``read_intent_from_file`` for the setpoint and ``write_telemetry_to_file`` /
    feedback hooks for the RX path. We satisfy that surface and translate to/from
    ROS2 messages.
    """

    def __init__(self, node: "SpiDriverNode") -> None:
        self._node = node
        self._latest_intent: Optional[Dict[str, Any]] = None
        self._feedback: Optional[Dict[str, Any]] = None

    # --- intent (setpoint) path ------------------------------------------
    def set_intent(self, intent: Optional[Dict[str, Any]]) -> None:
        self._latest_intent = intent

    def read_intent_from_file(self) -> Optional[Dict[str, Any]]:
        return self._latest_intent

    # --- telemetry (RX) path ---------------------------------------------
    def write_telemetry_to_file(self, telemetry: Dict[str, Any]) -> None:
        self._node.publish_telemetry(telemetry)

    # --- feedback (TELEOPPOSE ACK) path ----------------------------------
    def read_feedback_from_file(self) -> Optional[Dict[str, Any]]:
        return self._feedback

    def write_feedback_to_file(self, feedback: Dict[str, Any]) -> None:
        self._feedback = feedback


class SpiDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("jonny5_spi_driver")
        self.declare_parameter("use_mock_spi", True)
        self.declare_parameter("spi_device", "/dev/spidev0.0")
        self.declare_parameter("spi_speed_hz", 1_000_000)
        self.declare_parameter("tx_rate_hz", 100.0)
        self.declare_parameter("legacy_root", "")
        self.declare_parameter("joint_names", [
            "base_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_yaw_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
        ])

        self.use_mock = bool(self.get_parameter("use_mock_spi").value)
        self.joint_names = [str(x) for x in self.get_parameter("joint_names").value]

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.telemetry_pub = self.create_publisher(SpiTelemetry, "jonny5/spi/telemetry", 10)
        self.status_pub = self.create_publisher(RobotStatus, "jonny5/status", 10)
        self.intent_sub = self.create_subscription(
            TeleopIntent, "jonny5/teleop/intent", self._on_intent, 10
        )

        self.provider = Ros2StateProvider(self)
        self.bridge = self._build_bridge()

        rate = float(self.get_parameter("tx_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"JONNY5 native SPI driver started (mock={self.use_mock}, tx_rate={rate} Hz)"
        )

    def _build_bridge(self):
        legacy_root = resolve_legacy_root(str(self.get_parameter("legacy_root").value))
        if legacy_root is None:
            raise RuntimeError(
                "Legacy controller package not found. Set the 'legacy_root' parameter "
                "or JONNY5_LEGACY_ROOT to the directory that contains 'controller/'."
            )
        if str(legacy_root) not in sys.path:
            sys.path.insert(0, str(legacy_root))
        self.get_logger().info(f"Legacy data-plane root: {legacy_root}")

        from controller.spi_dataplane.j5vr_spi_bridge import J5VRSPIBridge

        if self.use_mock:
            spi = MockSpiWorker()
        else:
            from controller.spi_dataplane.spi_worker import SPIWorker

            spi = SPIWorker(
                device=str(self.get_parameter("spi_device").value),
                mode=0,
                max_speed_hz=int(self.get_parameter("spi_speed_hz").value),
            )
        spi.open()
        return J5VRSPIBridge(spi_worker=spi, state_provider=self.provider)

    def _on_intent(self, msg: TeleopIntent) -> None:
        self.provider.set_intent(self._intent_to_legacy_dict(msg))

    def _tick(self) -> None:
        try:
            self.bridge.send_setpoint_once()
        except Exception as exc:  # keep the node alive on transient SPI errors
            self.get_logger().warning(f"SPI tick failed: {exc}")

    # --- telemetry publishing (called by the provider per RX frame) -------
    def publish_telemetry(self, t: Dict[str, Any]) -> None:
        now = self.get_clock().now().to_msg()
        q = Quaternion(
            w=float(t.get("imu_q_w", 1.0) or 1.0),
            x=float(t.get("imu_q_x", 0.0) or 0.0),
            y=float(t.get("imu_q_y", 0.0) or 0.0),
            z=float(t.get("imu_q_z", 0.0) or 0.0),
        )
        servo = [float(t.get(k, 90.0)) for k in SERVO_KEYS]
        imu_valid = bool(t.get("imu_valid", False))

        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = self.joint_names
        joint_msg.position = [math.radians(deg - 90.0) for deg in servo]
        self.joint_pub.publish(joint_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation = q
        imu_msg.orientation_covariance[0] = 0.0 if imu_valid else -1.0
        self.imu_pub.publish(imu_msg)

        spi_msg = SpiTelemetry()
        spi_msg.stamp = now
        spi_msg.packet_index = int(t.get("packet_index", 0) or 0) & 0xFFFFFFFF
        spi_msg.frame_type = int(t.get("frame_type", 0) or 0) & 0xFF
        spi_msg.header_ok = True
        spi_msg.telemetry_fresh = True
        spi_msg.imu_valid = imu_valid
        spi_msg.imu_sample_counter = int(t.get("imu_sample_counter", 0) or 0) & 0xFFFFFFFF
        spi_msg.imu_orientation = q
        spi_msg.servo_deg = servo
        spi_msg.rt_loop_period_us = int(t.get("rt_loop_period_us", 0) or 0) & 0xFFFF
        spi_msg.rt_step_us = 0  # not carried in 0x01 telemetry frames
        spi_msg.raw_mode = 0  # FSM/mode echo lives in 0x03 STATUS frames (see task #3)
        spi_msg.raw_heartbeat = 0
        spi_msg.diag_mask = 0
        self.telemetry_pub.publish(spi_msg)

        status_msg = RobotStatus()
        status_msg.stamp = now
        status_msg.state = "TELEMETRY_OK" if imu_valid else "NO_IMU"
        status_msg.spi_online = True
        status_msg.stm32_online = True
        status_msg.imu_online = imu_valid
        status_msg.deadman_active = False  # only known from 0x03 STATUS frames
        status_msg.input_active = self.provider.read_intent_from_file() is not None
        status_msg.movement_allowed = True
        status_msg.detail = "native spi_driver (0x01 telemetry; FSM state via 0x03 pending)"
        self.status_pub.publish(status_msg)

    @staticmethod
    def _intent_to_legacy_dict(msg: TeleopIntent) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "mode": int(msg.mode),
            "joy_x": int(msg.joy_x),
            "joy_y": int(msg.joy_y),
            "pitch": int(msg.pitch),
            "yaw": int(msg.yaw),
            "intensity": int(msg.intensity),
            "grip": 1 if msg.grip else 0,
            "heartbeat": int(msg.heartbeat),
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
        cam_name = {1: "focus", 2: "zoom", 3: "conv"}.get(int(msg.camctrl_cmd))
        if cam_name and int(msg.camctrl_delta) != 0:
            out["camctrl"] = {"cmd": cam_name, "delta": int(msg.camctrl_delta)}
        return out


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SpiDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
