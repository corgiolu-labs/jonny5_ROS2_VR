"""
ws_handlers_settings.py — Gestione impostazioni e calibrazione VR.

Contiene:
  - handle_get_settings   (type=get_settings)
  - handle_save_settings  (type=save_settings)
  - handle_apply_offsets  (type=apply_offsets) — SET_OFFSETS UART + HOME post-calibrazione
  - handle_vr_calib       (type=vr_calib)      — broadcast stereo a tutti i client
  - _build_setpose_cmd    — helper condiviso virtuale→fisico SETPOSE

[RPi-0.5] Creato da ws_server.py. Nessuna modifica comportamentale.
[RPi-0.6] Import ordinati PEP8; docstring modulo aggiornata.
"""

# stdlib
import asyncio
import json
import logging

# local
from controller.uart import uart_manager
from controller.web_services import settings_manager
from controller.web_services import ws_handlers_imu as _imu_h
from controller.web_services.ws_core import _ws_safe_send, clients

logger = logging.getLogger("ws_teleop")

# ---------------------------------------------------------------------------
# Helper interno
# ---------------------------------------------------------------------------

def _build_setpose_cmd(virtual_angles: list, vel: int, profile: str, offsets: list, dirs: list) -> str:
    """
    Converte 6 angoli virtuali (90=HOME) in fisici e costruisce la stringa
    SETPOSE per il firmware STM32.
    [RPi-0.5] Spostato da ws_server.py. Formula identica.
    """
    physical = settings_manager.virtual_to_physical(virtual_angles, offsets, dirs)
    return "SETPOSE " + " ".join(str(v) for v in physical) + f" {vel} {profile}"


# ---------------------------------------------------------------------------
# Handler get_settings
# ---------------------------------------------------------------------------

async def handle_get_settings(websocket) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Risponde al messaggio type=get_settings con le impostazioni correnti.
    """
    settings = settings_manager.load()
    await _ws_safe_send(websocket, json.dumps({"type": "settings", **settings}))


# ---------------------------------------------------------------------------
# Handler save_settings
# ---------------------------------------------------------------------------

async def handle_save_settings(websocket, data: dict) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Salva le impostazioni ricevute e risponde con l'esito.
    """
    payload = {k: v for k, v in data.items() if k != "type"}
    ok = settings_manager.save(payload)
    if ok:
        _imu_h.invalidate_settings_cache()
    await _ws_safe_send(websocket, json.dumps({"type": "settings_saved", "ok": ok}))


# ---------------------------------------------------------------------------
# Handler apply_offsets
# ---------------------------------------------------------------------------

async def handle_apply_offsets(websocket, data: dict) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Applica offset meccanici via SET_OFFSETS UART, aggiorna settings.json
    e sposta il robot in HOME.
    """
    raw     = data.get("offsets")
    err_msg = None
    vals    = None

    if not isinstance(raw, list) or len(raw) != 6:
        err_msg = "offsets deve essere una lista di 6 interi [0,180]"
    else:
        try:
            vals = [int(v) for v in raw]
        except (TypeError, ValueError):
            err_msg = "offsets contiene valori non interi"
        else:
            if not all(0 <= v <= 180 for v in vals):
                err_msg = "offsets fuori range [0, 180]"

    if err_msg:
        logger.warning("[WS] apply_offsets: %s", err_msg)
        await _ws_safe_send(websocket, json.dumps({"type": "offsets_applied", "ok": False, "error": err_msg}))
        return

    uart_cmd = "SET_OFFSETS " + " ".join(str(v) for v in vals)
    try:
        ok, response = await uart_manager.send_uart_command(uart_cmd, timeout_s=1.5)
        if ok:
            # Aggiorna settings.json affinché la dashboard mostri i valori corretti al riavvio
            current = settings_manager.load()
            current["offsets"] = vals
            settings_manager.save(current)
            _imu_h.invalidate_settings_cache()
            logger.info("[UART] SET_OFFSETS ok: %s", vals)
            # Sposta il robot alla nuova posizione HOME (conferma visiva della calibrazione).
            # HOME virtuale = [90,90,90,90,90,90] → fisico = i nuovi offset appena impostati.
            # Usa vel=40 e profilo RTR5 — morbido e sicuro per una mossa di calibrazione.
            vel         = min(int(current.get("vel_max", 40)), 40)
            profile     = current.get("profile", "RTR5")
            dirs        = current.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
            home_virtual = [90, 90, 90, 90, 90, 90]
            setpose_cmd  = _build_setpose_cmd(home_virtual, vel, profile, vals, dirs)
            ok_sp, resp_sp = await uart_manager.send_uart_command(setpose_cmd, timeout_s=1.5)
            logger.info("[UART] SETPOSE post-offset ok=%s resp=%s cmd=%s", ok_sp, resp_sp, setpose_cmd)
        else:
            logger.warning("[UART] SET_OFFSETS fallito: %s", response)
        await _ws_safe_send(websocket, json.dumps({"type": "offsets_applied", "ok": ok,
                                                   "offsets": vals, "response": response or ""}))
    except Exception as e:
        logger.warning("[UART] apply_offsets errore: %s", e)
        await _ws_safe_send(websocket, json.dumps({"type": "offsets_applied", "ok": False, "error": str(e)}))


# ---------------------------------------------------------------------------
# Handler vr_calib
# ---------------------------------------------------------------------------

async def handle_vr_calib(websocket, data: dict) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Calibrazione stereo: broadcast a tutti i client connessi (visore + dashboard).
    """
    _VR_CALIB_KEYS = {
        "convPx", "vertPx", "vertPx0", "vertPx1",
        "rollDeg0", "rollDeg1",
        "zoom0", "zoom1",
        "focusPos0", "focusPos1",
    }
    filtered = {k: v for k, v in data.items() if k in _VR_CALIB_KEYS and isinstance(v, (int, float))}
    if filtered:
        msg = json.dumps({"type": "vr_calib", **filtered})
        await asyncio.gather(
            *[c.send(msg) for c in list(clients) if c is not websocket],
            return_exceptions=True,
        )
        logger.info("[VR_CALIB] broadcast a %d client: %s", len(clients) - 1, filtered)


# ---------------------------------------------------------------------------
# Handler controller_mappings_updated
# Relay live della nuova mappatura controller a tutti gli altri client.
# Tipico flow: dashboard /controllers POSTa /api/controller-mappings (HTTP),
# poi invia questo messaggio via WS per notificare il viewer XR (e altre
# dashboard aperte) di aggiornare in-memory senza reload.
# ---------------------------------------------------------------------------

async def handle_cameras_refocus_triggered(websocket, data: dict) -> None:
    """Relay del trigger refocus camere a tutti gli altri client (es. dashboard
    vr-live) per consentire WHEP reconnect coordinato dopo restart MediaMTX."""
    msg = json.dumps({"type": "cameras_refocus_triggered"})
    await asyncio.gather(
        *[c.send(msg) for c in list(clients) if c is not websocket],
        return_exceptions=True,
    )
    logger.info("[CAM_REFOCUS] broadcast a %d client", len(clients) - 1)


async def handle_controller_mappings_updated(websocket, data: dict) -> None:
    cfg = data.get("config")
    if not isinstance(cfg, dict):
        return
    msg = json.dumps({"type": "controller_mappings_updated", "config": cfg})
    await asyncio.gather(
        *[c.send(msg) for c in list(clients) if c is not websocket],
        return_exceptions=True,
    )
    try:
        n_modes = len(cfg.get("modes", {}))
        logger.info("[CTRL_MAPPINGS] broadcast a %d client (modes=%d)", len(clients) - 1, n_modes)
    except Exception:
        pass
