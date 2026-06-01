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
        self.declare_parameter("status_request_hz", 5.0)
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
        self._tick_count = 0
        self._fw_diag: Optional[Dict[str, Any]] = None
        self._imu_ok = False

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
        status_hz = float(self.get_parameter("status_request_hz").value)
        # Every Nth tick sends a 0x03 STATUS request instead of a setpoint, to read
        # the firmware diag (deadman/armed/freeze/guard). 0 disables.
        self._status_every = int(round(rate / status_hz)) if status_hz > 0 else 0
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"JONNY5 native SPI driver started (mock={self.use_mock}, tx_rate={rate} Hz, "
            f"status_every={self._status_every})"
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
        from controller.spi_dataplane.j5vr_frame import J5VRFrame
        from controller.spi_dataplane.spi_transport_mode import (
            extract_canonical_frame64_from_transport_rx,
        )

        self._make_frame = J5VRFrame
        self._extract_rx = extract_canonical_frame64_from_transport_rx

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
            self._tick_count += 1
            if self._status_every and (self._tick_count % self._status_every == 0):
                self._request_status()
            else:
                self.bridge.send_setpoint_once()
        except Exception as exc:  # keep the node alive on transient SPI errors
            self.get_logger().warning(f"SPI tick failed: {exc}")

    def _request_status(self) -> None:
        """Poll the STM32 with a 0x03 STATUS frame and parse the firmware diag.

        Firmware responds to 0x04/0x05 with 0x01 telemetry, but the diag bits
        (deadman/input/armed/freeze/guard, mode echo) ride only in the 0x03
        STATUS reply (see firmware j5vr_fill_tx_telemetry). A 0x03 request does
        not update the setpoint; skipping a few setpoints/sec is negligible.
        """
        seq = int(self.bridge.sequence_counter) & 0xFFFF
        tx = self._make_frame(sequence_counter=seq, frame_type=0x03).to_bytes()
        fl = int(getattr(self.bridge.spi_worker, "_frame_len", 64))
        if len(tx) == 64 and fl != 64:
            tx = tx + b"\x00" * (fl - 64)
        rx = self.bridge.spi_worker.transfer(tx)
        self.bridge.sequence_counter = (seq + 1) & 0xFFFF
        rxc = self._extract_rx(rx) if rx and len(rx) >= 64 else rx
        if not rxc or len(rxc) < 64 or rxc[0:2] != b"J5" or rxc[3] != 0x03:
            return
        pl = rxc[8:62]
        diag = (pl[50] << 8) | pl[51]
        self._fw_diag = {
            "mask": diag,
            "mode": pl[48],
            "hb": (pl[46] << 8) | pl[47],
            "deadman": bool(diag & (1 << 0)),
            "input": bool(diag & (1 << 1)),
            "armed": bool(diag & (1 << 2)),
            "freeze": bool(diag & (1 << 3)),
            "guard": bool(diag & (1 << 4)),
        }
        self._publish_status()

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
        cmd = self.provider.read_intent_from_file() or {}
        spi_msg.raw_mode = int(cmd.get("mode", 0) or 0) & 0xFF
        spi_msg.raw_heartbeat = int(cmd.get("heartbeat", 0) or 0) & 0xFFFF
        spi_msg.diag_mask = (int(self._fw_diag["mask"]) & 0xFFFF) if self._fw_diag else 0
        self.telemetry_pub.publish(spi_msg)

        self._imu_ok = imu_valid
        self._publish_status()

    def _publish_status(self) -> None:
        """Publish RobotStatus from the firmware 0x03 diag when available, else
        from the commanded intent (deadman uses the same grip logic as the
        firmware: both grips pressed)."""
        now = self.get_clock().now().to_msg()
        cmd = self.provider.read_intent_from_file() or {}
        d = self._fw_diag
        msg = RobotStatus()
        msg.stamp = now
        msg.spi_online = True
        msg.stm32_online = True
        msg.imu_online = self._imu_ok
        if d is not None:
            msg.deadman_active = bool(d["deadman"])
            msg.input_active = bool(d["input"])
            msg.movement_allowed = bool(d["armed"] and not d["freeze"])
            msg.state = "IDLE" if d["armed"] else "SAFE"
            flags = [n for n, on in (("freeze", d["freeze"]), ("guard", d["guard"])) if on]
            suffix = (" [" + ",".join(flags) + "]") if flags else ""
            msg.detail = ("diag from 0x03 STATUS; SAFE/IDLE/STOPPED FSM not on wire "
                          "(state approx from armed bit)" + suffix)
        else:
            gl = bool(int(cmd.get("buttons_left", 0) or 0) & (1 << 1))
            gr = bool(int(cmd.get("buttons_right", 0) or 0) & (1 << 1))
            msg.deadman_active = gl and gr
            msg.input_active = bool(cmd)
            msg.movement_allowed = True
            msg.state = "TELEMETRY_OK" if self._imu_ok else "NO_IMU"
            msg.detail = "0x01 telemetry; awaiting first 0x03 STATUS for firmware diag"
        self.status_pub.publish(msg)

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
