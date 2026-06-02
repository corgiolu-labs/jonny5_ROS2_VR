#!/usr/bin/env python3
"""
telemetry_web_bridge.py — Ponte ROS2 -> WebSocket dashboard legacy (:8557).

La dashboard (web/shared/js/j5_common.js) si aspetta messaggi WS
``{"type":"telemetry", ...}`` con i campi prodotti dal vecchio
``ws_handlers_imu._build_imu_servo_payload`` (servo_deg_B/S/G/Y/P/R,
imu_q_w/x/y/z, diagnostica SPI, robot_state). Nello stack ROS2 quei dati
vivono nei topic ``/jonny5/spi/telemetry`` (SpiTelemetry) e ``/jonny5/status``
(RobotStatus). Questo nodo si abbona ai topic e ribalta l'ultimo stato sui
client WS connessi a ~25 Hz, ricostruendo il formato atteso dal frontend.

Esecuzione (dentro il container, ROS env + workspace già sourced):
    python3 telemetry_web_bridge.py

NB: il quaternione è inviato grezzo (niente correzione world-bias come nel
legacy) -> lo yaw non è azzerato in HOME, ma la telemetria scorre live.
Solo telemetria (robot->browser); i comandi del browser sono drenati e ignorati.
"""
import asyncio
import json
import os
import ssl
import threading

import rclpy
from rclpy.node import Node
from jonny5_msgs.msg import SpiTelemetry, RobotStatus
import websockets

_SERVO_KEYS = ("servo_deg_B", "servo_deg_S", "servo_deg_G",
               "servo_deg_Y", "servo_deg_P", "servo_deg_R")

_latest = {"type": "telemetry"}
_lock = threading.Lock()
# stato per stima rate SPI lato bridge
_rate_state = {"idx": None, "t": None}


class TelemetryWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("jonny5_telemetry_web_bridge")
        self.create_subscription(SpiTelemetry, "/jonny5/spi/telemetry", self._on_tel, 10)
        self.create_subscription(RobotStatus, "/jonny5/status", self._on_status, 10)
        self.get_logger().info("telemetry_web_bridge: sub topics, WS server on :8557")

    def _on_tel(self, m: SpiTelemetry) -> None:
        d = {"type": "telemetry", "imu_valid": bool(m.imu_valid)}
        q = m.imu_orientation
        for a in ("w", "x", "y", "z"):
            v = float(getattr(q, a))
            d[f"imu_q_{a}"] = v
            d[f"imu_q_raw_{a}"] = v
        servos = list(m.servo_deg)
        for i, k in enumerate(_SERVO_KEYS):
            if i < len(servos):
                d[k] = float(servos[i])
        d["spi_packet_index"] = int(m.packet_index)
        d["imu_sample_counter"] = int(m.imu_sample_counter)
        d["rt_loop_period_us"] = int(m.rt_loop_period_us)
        d["telemetry_fresh"] = bool(m.telemetry_fresh)
        # stima rate SPI dai delta di packet_index
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            pidx = int(m.packet_index)
            if _rate_state["idx"] is not None and _rate_state["t"] is not None:
                di = pidx - _rate_state["idx"]
                dt = now - _rate_state["t"]
                if dt > 1e-6 and di >= 0:
                    d["ws_spi_rate_hz_est"] = float(di / dt)
            _rate_state["idx"] = pidx
            _rate_state["t"] = now
        except Exception:
            pass
        with _lock:
            _latest.update(d)

    def _on_status(self, m: RobotStatus) -> None:
        with _lock:
            _latest["robot_state"] = m.state
            _latest["movement_allowed"] = bool(m.movement_allowed)
            _latest["deadman_active"] = bool(m.deadman_active)
            _latest["input_active"] = bool(m.input_active)
            _latest["spi_online"] = bool(m.spi_online)
            _latest["stm32_online"] = bool(m.stm32_online)
            _latest["imu_online"] = bool(m.imu_online)


async def _serve(ws, *_args):
    """Un client: drena i comandi in ingresso (ignorati) e spinge telemetria a ~25 Hz."""
    async def _drain():
        try:
            async for _ in ws:
                pass
        except Exception:
            pass
    drain_task = asyncio.create_task(_drain())
    try:
        while True:
            with _lock:
                payload = dict(_latest)
            await ws.send(json.dumps(payload))
            await asyncio.sleep(0.04)
    except Exception:
        pass
    finally:
        drain_task.cancel()


def _make_ssl_ctx():
    """https_server proxya verso :8557 in TLS (wss) -> il bridge DEVE servire wss.
    Riusa i cert self-signed di https_server; il proxy salta la verifica (CERT_NONE)."""
    cert = os.environ.get("J5_WS_CERT", "/opt/jonny5/raspberry/config_runtime/tls/webrtc.crt")
    key = os.environ.get("J5_WS_KEY", "/opt/jonny5/raspberry/config_runtime/tls/webrtc.key")
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=cert, keyfile=key)
    return ctx


async def _main_async():
    async with websockets.serve(_serve, "0.0.0.0", 8557, ssl=_make_ssl_ctx(), ping_interval=None):
        await asyncio.Future()


def main() -> None:
    rclpy.init()
    node = TelemetryWebBridge()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    try:
        asyncio.run(_main_async())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
