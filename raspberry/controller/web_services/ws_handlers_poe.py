"""
ws_handlers_poe.py — Handler WS per parametri POE (S + M).

- type=get_poe_params  → type=poe_params { S, M, persisted }
- type=set_poe_params → body { S, M } → type=poe_params_saved { ok }
"""

from __future__ import annotations

import json
import logging

from controller.web_services import poe_params_manager
from controller.web_services.ws_core import _ws_safe_send

logger = logging.getLogger("ws_teleop")


async def handle_get_poe_params(websocket) -> None:
    cfg = poe_params_manager.load()
    payload = {
        "type": "poe_params",
        "S": cfg["S"],
        "M": cfg["M"],
        "persisted": bool(cfg.get("persisted", False)),
    }
    await _ws_safe_send(websocket, json.dumps(payload))


async def handle_set_poe_params(websocket, data: dict) -> None:
    if not isinstance(data.get("S"), list) or not isinstance(data.get("M"), list):
        await _ws_safe_send(websocket, json.dumps({"type": "poe_params_saved", "ok": False}))
        return
    ok = poe_params_manager.save({"S": data["S"], "M": data["M"]})
    if ok:
        logger.info("[WS] POE salvato")
    else:
        logger.warning("[WS] salvataggio POE fallito")
    await _ws_safe_send(websocket, json.dumps({"type": "poe_params_saved", "ok": ok}))
