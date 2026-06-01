"""
ws_handlers_kinematics.py — Messaggi WS per cinematica (FK POE), senza UART.

- type=compute_fk_poe → type=fk_poe_result { ok, x_mm, y_mm, z_mm, roll_deg, ... }
"""

from __future__ import annotations

import json
import logging

from controller.web_services import ik_solver
from controller.web_services.ws_core import _ws_safe_send

logger = logging.getLogger("ws_teleop")


async def handle_compute_fk_poe(websocket, data: dict) -> None:
    angles = data.get("angles_deg")
    result = ik_solver.compute_fk_poe_virtual_deg(angles)
    payload = {"type": "fk_poe_result", **result}
    await _ws_safe_send(websocket, json.dumps(payload))
