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
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from jonny5_msgs.msg import RobotStatus, SpiTelemetry, TeleopIntent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from jonny5_hardware.mock_spi import MockSpiWorker

SERVO_KEYS = [
    "servo_deg_B",
    "servo_deg_S",
    "servo_deg_G",
    "servo_deg_Y",
    "servo_deg_P",
    "servo_deg_R",
]

# Runtime calibration of the robot (raspberry/config_runtime/robot/j5_settings.json).
# Telemetry carries *physical* servo degrees; the joint angle is
# (physical - offset) * dir, i.e. settings_manager.physical_to_virtual() - 90.
DEFAULT_SERVO_OFFSETS_DEG = [100.0, 88.0, 93.0, 95.0, 90.0, 95.0]
DEFAULT_SERVO_DIRS = [1, -1, -1, 1, -1, 1]

# A telemetry / SPI reply older than this marks the link offline in RobotStatus.
LINK_TIMEOUT_S = 0.5


def physical_deg_to_joint_rad(
    physical_deg: List[float], offsets_deg: List[float], dirs: List[int]
) -> List[float]:
    """Physical servo degrees -> URDF joint radians (0 rad = mechanical HOME)."""
    out: List[float] = []
    for i, deg in enumerate(physical_deg):
        offset = float(offsets_deg[i]) if i < len(offsets_deg) else 90.0
        direction = -1.0 if (i < len(dirs) and int(dirs[i]) < 0) else 1.0
        out.append(math.radians((float(deg) - offset) * direction))
    return out


def joint_rad_to_physical_cdeg(
    joint_rad: List[float], offsets_deg: List[float], dirs: List[int]
) -> List[int]:
    """URDF joint radians -> physical servo centi-degrees (inverse of the above),
    clamped to the servo range 0..180 deg. The firmware applies its own joint limits."""
    out: List[int] = []
    for i, rad in enumerate(joint_rad):
        offset = float(offsets_deg[i]) if i < len(offsets_deg) else 90.0
        direction = -1.0 if (i < len(dirs) and int(dirs[i]) < 0) else 1.0
        deg = offset + direction * math.degrees(float(rad))
        deg = max(0.0, min(180.0, deg))
        out.append(int(round(deg * 100.0)))
    return out


def v2_diag_mask(t: Dict[str, Any]) -> int:
    """TELEMETRY_V2 flags -> legacy 0x03 diag_mask bit layout."""
    mask = 0
    for bit, key in enumerate(("deadman_active", "input_active", "armed", "freeze", "guard_seen")):
        if t.get(key):
            mask |= 1 << bit
    return mask


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

    def __init__(self, node: "SpiDriverNode", intent_timeout_s: float) -> None:
        self._node = node
        self._intent_timeout_s = float(intent_timeout_s)
        self._latest_intent: Optional[Dict[str, Any]] = None
        self._intent_rx_mono: float = 0.0
        self._intent_stale_reported = False
        self._feedback: Optional[Dict[str, Any]] = None

    # --- intent (setpoint) path ------------------------------------------
    def set_intent(self, intent: Optional[Dict[str, Any]]) -> None:
        self._latest_intent = intent
        self._intent_rx_mono = time.monotonic()
        self._intent_stale_reported = False

    def intent_is_fresh(self) -> bool:
        if self._latest_intent is None:
            return False
        if self._intent_timeout_s <= 0.0:
            return True
        return (time.monotonic() - self._intent_rx_mono) <= self._intent_timeout_s

    def read_intent_from_file(self) -> Optional[Dict[str, Any]]:
        """Latest intent, or None once it is older than ``intent_timeout_s``.

        None makes the bridge send an empty (IDLE, no buttons -> no deadman)
        frame, so a dead headset / WebSocket / bridge node can never keep the
        last joystick or grip command streaming to the STM32.
        """
        if self._latest_intent is not None and not self.intent_is_fresh():
            if not self._intent_stale_reported:
                self._intent_stale_reported = True
                self._node.get_logger().warning(
                    f"TeleopIntent older than {self._intent_timeout_s:.3f}s: "
                    "sending IDLE until a new intent arrives"
                )
            return None
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
        # Stale-intent watchdog: after this many seconds without a TeleopIntent the
        # driver streams IDLE frames. 0 disables (not recommended on hardware).
        self.declare_parameter("intent_timeout_s", 0.25)
        self.declare_parameter("servo_offsets_deg", DEFAULT_SERVO_OFFSETS_DEG)
        self.declare_parameter("servo_dirs", DEFAULT_SERVO_DIRS)
        # SPI protocol: 1 = legacy (works with any firmware), 2 = CRC-16 +
        # TELEMETRY_V2 + J5IK joint streaming (needs firmware with v2 support).
        self.declare_parameter("protocol_version", 1)
        # Joint streaming consent: the command must be this recent and this close
        # to the current pose when streaming is enabled (no jumps to old targets).
        self.declare_parameter("joint_cmd_timeout_s", 0.5)
        self.declare_parameter("stream_arm_gate_rad", 0.05)
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
        self.servo_offsets_deg = [float(x) for x in self.get_parameter("servo_offsets_deg").value]
        self.servo_dirs = [int(x) for x in self.get_parameter("servo_dirs").value]
        if len(self.servo_offsets_deg) != 6 or len(self.servo_dirs) != 6:
            raise ValueError("servo_offsets_deg and servo_dirs must have 6 entries (B S G Y P R)")
        self._tick_count = 0
        self._fw_diag: Optional[Dict[str, Any]] = None
        self._imu_ok = False
        self._last_spi_rx_mono = 0.0
        self._last_telemetry_mono = 0.0
        self.protocol_version = int(self.get_parameter("protocol_version").value)
        if self.protocol_version not in (1, 2):
            raise ValueError("protocol_version must be 1 or 2")
        self._v2_state: Optional[Dict[str, Any]] = None
        # J5IK joint streaming (protocol v2 only)
        self._joint_cmd_cdeg: Optional[List[int]] = None
        self._joint_cmd_rad: Optional[List[float]] = None
        self._joint_cmd_mono = 0.0
        self._joint_pos_rad: Optional[List[float]] = None
        self._stream_enabled = False
        self._stream_heartbeat = 0
        self._joint_cmd_timeout_s = float(self.get_parameter("joint_cmd_timeout_s").value)
        self._stream_gate_rad = float(self.get_parameter("stream_arm_gate_rad").value)

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.telemetry_pub = self.create_publisher(SpiTelemetry, "jonny5/spi/telemetry", 10)
        self.status_pub = self.create_publisher(RobotStatus, "jonny5/status", 10)
        self.intent_sub = self.create_subscription(
            TeleopIntent, "jonny5/teleop/intent", self._on_intent, 10
        )
        # Joint streaming: 6 positions in rad (joint_names order), like the
        # ros2_controllers forward_position_controller command topic.
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, "jonny5/joint_commands", self._on_joint_command, 10
        )
        self.create_service(SetBool, "jonny5/joint_stream/enable", self._on_stream_enable)

        self.provider = Ros2StateProvider(
            self, float(self.get_parameter("intent_timeout_s").value)
        )
        self.bridge = self._build_bridge()

        rate = float(self.get_parameter("tx_rate_hz").value)
        status_hz = float(self.get_parameter("status_request_hz").value)
        if self.protocol_version == 2:
            status_hz = 0.0  # TELEMETRY_V2 already carries FSM state and diag flags
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
        return J5VRSPIBridge(
            spi_worker=spi, state_provider=self.provider, protocol_version=self.protocol_version
        )

    def _on_intent(self, msg: TeleopIntent) -> None:
        self.provider.set_intent(self._intent_to_legacy_dict(msg))

    def _on_joint_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 6 or not all(math.isfinite(v) for v in msg.data):
            self.get_logger().warning("jonny5/joint_commands needs 6 finite values (rad)")
            return
        self._joint_cmd_rad = [float(v) for v in msg.data]
        self._joint_cmd_cdeg = joint_rad_to_physical_cdeg(
            self._joint_cmd_rad, self.servo_offsets_deg, self.servo_dirs
        )
        self._joint_cmd_mono = time.monotonic()

    def _on_stream_enable(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self._stream_enabled = False
            response.success = True
            response.message = "joint streaming disabled"
            return response
        refusal = self._stream_enable_refusal()
        if refusal:
            response.success = False
            response.message = refusal
        else:
            self._stream_enabled = True
            response.success = True
            response.message = "joint streaming enabled"
        return response

    def _stream_enable_refusal(self) -> str:
        """Why joint streaming cannot be enabled now ('' = it can)."""
        if self.protocol_version != 2:
            return "joint streaming needs protocol_version=2"
        if self._joint_cmd_rad is None:
            return "publish a command on jonny5/joint_commands first"
        age = time.monotonic() - self._joint_cmd_mono
        if age > self._joint_cmd_timeout_s:
            return f"last joint command is {age:.1f}s old: publish the current target first"
        v2s = self._v2_state
        if v2s is None or not self._stm32_link_fresh():
            return "no TELEMETRY_V2 from the STM32"
        if v2s.get("estop_active") or v2s.get("fsm_state_name") != "IDLE":
            return f"STM32 is {v2s.get('fsm_state_name')} (E-STOP {v2s.get('estop_active')}): send UART ENABLE first"
        if self._joint_pos_rad is None:
            return "no joint state yet"
        jump = max(abs(a - b) for a, b in zip(self._joint_cmd_rad, self._joint_pos_rad))
        if jump > self._stream_gate_rad:
            return (f"command is {jump:.3f} rad from the current pose "
                    f"(gate {self._stream_gate_rad}): command the current pose first")
        return ""

    def _stm32_link_fresh(self) -> bool:
        return (time.monotonic() - self._last_telemetry_mono) <= LINK_TIMEOUT_S

    def _send_joint_stream(self) -> Optional[bytes]:
        """One J5IK frame with the latest command. The last command is held
        (forward_position_controller semantics); if this node dies the SPI
        frames stop and the firmware holds, then its 500 ms watchdog -> SAFE."""
        self._stream_heartbeat = ((self._stream_heartbeat + 1) & 0xFFFF) or 1
        return self.bridge.send_joint_targets_once(
            self._joint_cmd_cdeg, enable=True, heartbeat=self._stream_heartbeat
        )

    def _tick(self) -> None:
        try:
            self._tick_count += 1
            if self._status_every and (self._tick_count % self._status_every == 0):
                self._request_status()
            else:
                # send_*_once() swallow SPI errors and return None.
                if self._stream_enabled:
                    v2s = self._v2_state or {}
                    if (v2s.get("estop_active") or v2s.get("fsm_state_name") != "IDLE"
                            or not self._stm32_link_fresh()):
                        # Firmware left IDLE (SAFE/STOPPED/E-STOP/link): withdraw consent.
                        # The operator re-enables after UART ENABLE (never automatic).
                        self._stream_enabled = False
                        self.get_logger().warning(
                            "STM32 left IDLE or E-STOP/link loss: joint streaming disabled; "
                            "call jonny5/joint_stream/enable again after UART ENABLE"
                        )
                if self._stream_enabled and self._joint_cmd_cdeg is not None:
                    rx = self._send_joint_stream()
                else:
                    rx = self.bridge.send_setpoint_once()
                if rx:
                    self._last_spi_rx_mono = time.monotonic()
        except Exception as exc:  # keep the node alive on transient SPI errors
            self.get_logger().warning(f"SPI tick failed: {exc}")

    def safe_stop(self) -> None:
        """Send a few IDLE frames (no buttons -> no deadman) and close SPI."""
        try:
            self._stream_enabled = False
            self.provider.set_intent(None)
            for _ in range(3):
                self.bridge.send_setpoint_once()
        except Exception as exc:
            self.get_logger().warning(f"IDLE on shutdown failed: {exc}")
        try:
            self.bridge.spi_worker.close()
        except Exception:
            pass

    def _request_status(self) -> None:
        """Poll the STM32 with a 0x03 STATUS frame and parse the firmware diag.

        Firmware responds to 0x04/0x05 with 0x01 telemetry, but the diag bits
        (deadman/input/armed/freeze/guard, mode echo) ride only in the 0x03
        STATUS reply (see firmware j5vr_fill_tx_telemetry). A 0x03 request does
        not update the setpoint; skipping a few setpoints/sec is negligible.
        """
        seq = int(self.bridge.sequence_counter) & 0xFFFF
        tx = self._make_frame(
            sequence_counter=seq, frame_type=0x03, protocol_version=self.protocol_version
        ).to_bytes()
        fl = int(getattr(self.bridge.spi_worker, "_frame_len", 64))
        if len(tx) == 64 and fl != 64:
            tx = tx + b"\x00" * (fl - 64)
        rx = self.bridge.spi_worker.transfer(tx)
        self.bridge.sequence_counter = (seq + 1) & 0xFFFF
        rxc = self._extract_rx(rx) if rx and len(rx) >= 64 else rx
        if not rxc or len(rxc) < 64 or rxc[0:2] != b"J5":
            return
        self._last_spi_rx_mono = time.monotonic()
        if rxc[3] != 0x03:
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
        # The legacy bridge re-emits the last telemetry inside a 0.5 s grace window
        # when the STM32 answers with a non-telemetry frame: that data is not new.
        fresh = not bool(t.get("telemetry_heartbeat", False))
        now_mono = time.monotonic()
        self._last_spi_rx_mono = now_mono
        is_v2 = int(t.get("protocol_version", 1) or 1) == 2
        if fresh:
            self._last_telemetry_mono = now_mono
            if is_v2:
                self._v2_state = t

            joint_msg = JointState()
            joint_msg.header.stamp = now
            joint_msg.name = self.joint_names
            joint_msg.position = physical_deg_to_joint_rad(
                servo, self.servo_offsets_deg, self.servo_dirs
            )
            self._joint_pos_rad = list(joint_msg.position)
            self.joint_pub.publish(joint_msg)

            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = "imu_link"
            imu_msg.orientation = q
            imu_msg.orientation_covariance[0] = 0.0 if imu_valid else -1.0
            # Only orientation is published: mark the other fields as unknown.
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu_msg)

        spi_msg = SpiTelemetry()
        spi_msg.stamp = now
        spi_msg.packet_index = int(t.get("packet_index", 0) or 0) & 0xFFFFFFFF
        spi_msg.frame_type = int(t.get("frame_type", 0) or 0) & 0xFF
        spi_msg.header_ok = True
        spi_msg.telemetry_fresh = fresh
        spi_msg.imu_valid = imu_valid
        spi_msg.imu_sample_counter = int(t.get("imu_sample_counter", 0) or 0) & 0xFFFFFFFF
        spi_msg.imu_orientation = q
        spi_msg.servo_deg = servo
        spi_msg.rt_loop_period_us = int(t.get("rt_loop_period_us", 0) or 0) & 0xFFFF
        spi_msg.rt_step_us = int(t.get("rt_step_us", 0) or 0) & 0xFFFF  # v2 only
        cmd = self.provider.read_intent_from_file() or {}
        spi_msg.raw_heartbeat = int(cmd.get("heartbeat", 0) or 0) & 0xFFFF
        if is_v2:
            spi_msg.raw_mode = int(t.get("mode", 0) or 0) & 0xFF
            spi_msg.diag_mask = v2_diag_mask(t)
            spi_msg.protocol_version = 2
            spi_msg.stm_time_ms = int(t.get("stm_time_ms", 0)) & 0xFFFFFFFF
            spi_msg.stm_tx_seq = int(t.get("stm_tx_seq", 0)) & 0xFFFF
            spi_msg.fsm_state = int(t.get("fsm_state", 0)) & 0xFF
            spi_msg.estop_active = bool(t.get("estop_active"))
            spi_msg.setpose_active = bool(t.get("setpose_active"))
            spi_msg.joint_stream_active = bool(t.get("joint_stream_active"))
            spi_msg.rt_overruns = int(t.get("rt_overruns", 0)) & 0xFFFF
            spi_msg.rx_seq_gaps = int(t.get("rx_seq_gaps", 0)) & 0xFFFF
            spi_msg.crc_errors_stm = int(t.get("crc_errors_stm", 0)) & 0xFFFF
            spi_msg.crc_errors_pi = int(getattr(self.bridge, "v2_rx_crc_errors", 0)) & 0xFFFFFFFF
            spi_msg.repeated_replies = int(getattr(self.bridge, "v2_rx_repeated", 0)) & 0xFFFFFFFF
        else:
            spi_msg.raw_mode = int(cmd.get("mode", 0) or 0) & 0xFF
            spi_msg.diag_mask = (int(self._fw_diag["mask"]) & 0xFFFF) if self._fw_diag else 0
            spi_msg.protocol_version = 1
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
        now_mono = time.monotonic()
        msg.spi_online = (now_mono - self._last_spi_rx_mono) <= LINK_TIMEOUT_S
        msg.stm32_online = (now_mono - self._last_telemetry_mono) <= LINK_TIMEOUT_S
        msg.imu_online = self._imu_ok and msg.stm32_online
        v2s = self._v2_state
        if v2s is not None:
            msg.estop_active = bool(v2s.get("estop_active"))
            msg.deadman_active = bool(v2s.get("deadman_active"))
            msg.input_active = bool(v2s.get("input_active"))
            msg.movement_allowed = (
                bool(v2s.get("movement_allowed")) and not msg.estop_active and msg.stm32_online
            )
            msg.state = str(v2s.get("fsm_state_name", "UNKNOWN"))
            flags = [
                name
                for name, key in (
                    ("estop", "estop_active"), ("armed", "armed"), ("freeze", "freeze"),
                    ("guard", "guard_seen"), ("setpose", "setpose_active"),
                    ("joint_stream", "joint_stream_active"),
                )
                if v2s.get(key)
            ]
            msg.detail = f"TELEMETRY_V2 mode={v2s.get('mode', 0)}" + (
                " [" + ",".join(flags) + "]" if flags else ""
            )
        elif d is not None:
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
            # Unknown until the firmware diag arrives: never report it as allowed.
            msg.movement_allowed = False
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
