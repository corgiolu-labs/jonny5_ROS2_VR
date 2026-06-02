#!/usr/bin/env python3
"""
spi_shm_bridge.py — Shim SPI bidirezionale ROS2 <-> ws_server legacy (via /dev/shm).

La ws_server legacy (host) usa shared_state (file /dev/shm) per il data-plane SPI:
  - LEGGE  telemetria: /dev/shm/j5vr_telemetry.json     (formato legacy)
  - SCRIVE intent:     /dev/shm/j5vr_latest_intent.json
Nello stack ROS2 quei dati vivono nei topic. Questo nodo (nel container ros:jazzy,
rclpy + jonny5_msgs) li collega in entrambe le direzioni:
  - sub  /jonny5/spi/telemetry + /jonny5/status  ->  scrive j5vr_telemetry.json
  - watch j5vr_latest_intent.json                ->  pub /jonny5/teleop/intent (TeleopIntent)

Cosi' la ws_server completa (telemetria, comandi giunti, UART, settings, IK, video)
gira sull'host invariata, col data-plane SPI servito da ROS2. Sostituisce il
mini telemetry_web_bridge (la ws_server ora serve lei la WS su 8557).

Richiede /dev/shm condiviso host<->container -> docker compose: `ipc: host`.

NB: il path intent->topico MUOVE il braccio (via driver SPI->STM32). Il firmware
richiede deadman (entrambe le grip) per muoversi: senza deadman resta IDLE/SAFE.
"""
import json
import os

import rclpy
from rclpy.node import Node
from jonny5_msgs.msg import SpiTelemetry, RobotStatus, TeleopIntent

TEL_FILE = os.environ.get("J5VR_TELEMETRY_FILE", "/dev/shm/j5vr_telemetry.json")
INT_FILE = os.environ.get("J5VR_INTENT_FILE", "/dev/shm/j5vr_latest_intent.json")
_SERVO = ("servo_deg_B", "servo_deg_S", "servo_deg_G",
          "servo_deg_Y", "servo_deg_P", "servo_deg_R")


def _clamp(v, lo, hi):
    try:
        return max(lo, min(hi, int(v)))
    except Exception:
        return 0


def _write_atomic(path, data):
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, path)
    except Exception:
        pass


class SpiShmBridge(Node):
    def __init__(self):
        super().__init__("jonny5_spi_shm_bridge")
        self._status = {}
        self.create_subscription(SpiTelemetry, "/jonny5/spi/telemetry", self._on_tel, 10)
        self.create_subscription(RobotStatus, "/jonny5/status", self._on_status, 10)
        self.intent_pub = self.create_publisher(TeleopIntent, "/jonny5/teleop/intent", 10)
        self._intent_mtime = 0.0
        self.create_timer(0.01, self._poll_intent)  # 100 Hz
        self.get_logger().info(f"spi_shm_bridge: tel->{TEL_FILE}  intent<-{INT_FILE}")

    # --- telemetry ROS2 -> /dev/shm (formato legacy letto da shared_state) ---
    def _on_status(self, m: RobotStatus):
        self._status = {
            "robot_state": m.state,
            "deadman": bool(m.deadman_active),
            "input_active": bool(m.input_active),
            "movement_allowed": bool(m.movement_allowed),
            "spi_online": bool(m.spi_online),
            "stm32_online": bool(m.stm32_online),
            "imu_online": bool(m.imu_online),
        }

    def _on_tel(self, m: SpiTelemetry):
        q = m.imu_orientation
        d = {
            "imu_valid": bool(m.imu_valid),
            "imu_q_w": float(q.w), "imu_q_x": float(q.x),
            "imu_q_y": float(q.y), "imu_q_z": float(q.z),
            "packet_index": int(m.packet_index),
            "imu_sample_counter": int(m.imu_sample_counter),
            "rt_loop_period_us": int(m.rt_loop_period_us),
            "frame_type": int(m.frame_type),
            "diag_mask": int(m.diag_mask),
        }
        servos = list(m.servo_deg)
        for i, k in enumerate(_SERVO):
            if i < len(servos):
                d[k] = float(servos[i])
        d.update(self._status)
        _write_atomic(TEL_FILE, d)

    # --- intent /dev/shm (scritto da ws_server) -> topic TeleopIntent ---
    def _poll_intent(self):
        try:
            if not os.path.isfile(INT_FILE):
                return
            mt = os.path.getmtime(INT_FILE)
            if mt == self._intent_mtime:
                return
            self._intent_mtime = mt
            with open(INT_FILE) as f:
                d = json.load(f)
            if isinstance(d, dict):
                self.intent_pub.publish(self._dict_to_intent(d))
        except Exception:
            pass

    def _dict_to_intent(self, d: dict) -> TeleopIntent:
        msg = TeleopIntent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = _clamp(d.get("mode", 0), 0, 255)
        msg.joy_x = _clamp(d.get("joy_x", 0), -32768, 32767)
        msg.joy_y = _clamp(d.get("joy_y", 0), -32768, 32767)
        msg.pitch = _clamp(d.get("pitch", 0), -32768, 32767)
        msg.yaw = _clamp(d.get("yaw", 0), -32768, 32767)
        msg.intensity = _clamp(d.get("intensity", 0), 0, 255)
        msg.grip = bool(d.get("grip", 0))
        msg.heartbeat = _clamp(d.get("heartbeat", 0), 0, 65535)
        msg.buttons_left = _clamp(d.get("buttons_left", 0), 0, 65535)
        msg.buttons_right = _clamp(d.get("buttons_right", 0), 0, 65535)
        q = msg.headset_orientation
        q.w = float(d.get("quat_w", 1.0) or 1.0)
        q.x = float(d.get("quat_x", 0.0) or 0.0)
        q.y = float(d.get("quat_y", 0.0) or 0.0)
        q.z = float(d.get("quat_z", 0.0) or 0.0)
        cc = d.get("camctrl")
        if isinstance(cc, dict):
            cmd = cc.get("cmd")
            cmap = {"focus": 1, "zoom": 2, "conv": 3, "convergence": 3}
            msg.camctrl_cmd = (int(cmap.get(cmd, 0)) if isinstance(cmd, str)
                               else _clamp(cmd or 0, 0, 255))
            msg.camctrl_delta = _clamp(cc.get("delta", 0), -32768, 32767)
        m5 = d.get("mode5_arm")
        if isinstance(m5, dict):
            msg.mode5_arm_valid = bool(m5.get("valid", False))
            msg.mode5_grip_active = bool(m5.get("grip_active", False))
            msg.mode5_hold_active = bool(m5.get("hold_active", False))
            msg.mode5_target_id = _clamp(m5.get("target_id", 0), 0, 65535)
            msg.mode5_base_deg = float(m5.get("base_deg", 0.0) or 0.0)
            msg.mode5_shoulder_deg = float(m5.get("shoulder_deg", 0.0) or 0.0)
            msg.mode5_elbow_deg = float(m5.get("elbow_deg", 0.0) or 0.0)
        return msg


def main():
    rclpy.init()
    node = SpiShmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
