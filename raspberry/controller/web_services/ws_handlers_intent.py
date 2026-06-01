"""
ws_handlers_intent.py — Validazione intent VR, HEAD mode, comandi teleop.

Contiene:
  - validate_and_build_intent
  - parser assi (_parse_axis_i16, _parse_intensity_u8)
  - supporto modalità legacy (_handle_legacy_mode_if_present)
  - gestione HEAD mode dashboard (handle_set_vr_mode, heartbeat loop)
  - stato HEAD mode (modulo-level)
  - get_dashboard_head_active() — getter pubblico per ws_server

[RPi-0.5] Creato da ws_server.py. Nessuna modifica comportamentale.
[RPi-0.6] Import ordinati PEP8; rimossi _ws_safe_send e _TELEOPPOSE_CLEAR_S
          (importati ma non usati in questo modulo).
[RPi-0.7] Aggiunto get_dashboard_head_active(); aggiunti commenti avviso
          su punti sensibili (Event a modulo load, stato condiviso asyncio).
[RPi-1.0] _head_mode_stop_event spostato da modulo load a init_events();
          aggiunta init_events() chiamata da ws_server.main().
"""

# stdlib
import asyncio
import json
import logging
import time

# local
from controller.teleop import shared_state
from controller.web_services.ws_core import (
    _is_finite,
    _clamp_float,
    _FEEDBACK_LOOP_SLEEP_S,
    clients,
)

logger = logging.getLogger("ws_teleop")

# ---------------------------------------------------------------------------
# NOTE [RPi-0.3]:
# Queste modalità legacy potrebbero essere rimosse in futuro,
# ma per ora vengono mantenute per massima compatibilità.
# ---------------------------------------------------------------------------
MODE_VALUES_LEGACY = ("IDLE", "RELATIVE_MOVE", "ABSOLUTE_POSE")
MODE_TO_J5VR_LEGACY = {"IDLE": 0, "RELATIVE_MOVE": 1, "ABSOLUTE_POSE": 2}
# Set locale dei mode supportati dal parser intent (v1.1.5+).
# 5 = prototipo sperimentale: DX wrist-center translation (server-side, isolato).
MODE_VALUES_V115 = (0, 1, 2, 3, 4, 5)

# ---------------------------------------------------------------------------
# Stato condiviso del modulo intent
#
# NOTA [RPi-0.7]: tutte queste variabili sono modificate esclusivamente
# dall'event loop asyncio (single-thread), quindi non servono lock.
# ws_server accede a _dashboard_head_active tramite get_dashboard_head_active().
# ---------------------------------------------------------------------------
# HEAD mode dashboard
# NOTA [RPi-1.0]: _head_mode_stop_event viene inizializzato da init_events()
# all'interno del loop asyncio attivo, evitando DeprecationWarning in Python ≥ 3.12.
# Prima di init_events() il valore è None; init_events() è idempotente (no double-init).
_head_mode_task: "asyncio.Task | None" = None
_head_mode_stop_event: "asyncio.Event | None" = None
_head_mode_hb: int = 1
_dashboard_head_active: bool = False


def init_events() -> None:
    """
    [RPi-1.0] Inizializza gli asyncio.Event del modulo dentro il loop attivo.
    Chiamare da ws_server.main() subito dopo asyncio.get_running_loop().
    Idempotente: se già inizializzati, non li ricrea.
    """
    global _head_mode_stop_event
    if _head_mode_stop_event is None:
        _head_mode_stop_event = asyncio.Event()


def get_dashboard_head_active() -> bool:
    """
    [RPi-0.7] Getter pubblico per _dashboard_head_active.
    Usare questo invece dell'accesso diretto al membro privato del modulo
    (es. _intent._dashboard_head_active in ws_server).
    """
    return _dashboard_head_active


def stop_head_mode(reason: str = "") -> None:
    """
    Richiede lo stop del loop HEAD mode, se attivo.
    Safe da chiamare più volte.
    """
    global _head_mode_task, _head_mode_stop_event
    if _head_mode_task and not _head_mode_task.done() and _head_mode_stop_event is not None:
        _head_mode_stop_event.set()
        if reason:
            logger.info("[WS] stop HEAD richiesto: %s", reason)


# ---------------------------------------------------------------------------
# Parsing / normalizzazione
# ---------------------------------------------------------------------------

def _parse_axis_i16(v):
    """
    Accetta:
    - int già in scala int16 [-32768..32767]
    - float normalizzato [-1..1] (compatibilità)
    """
    if isinstance(v, int):
        return max(-32768, min(32767, v))
    if _is_finite(v):
        x = float(v)
        if x < -1.0:
            x = -1.0
        if x > 1.0:
            x = 1.0
        return max(-32768, min(32767, int(round(x * 32767))))
    return None


def _parse_intensity_u8(v):
    """
    Accetta:
    - int [0..255] (target)
    - float [0..1] (compatibilità)
    """
    if isinstance(v, int):
        return max(0, min(255, v))
    if _is_finite(v):
        x = float(v)
        if x < 0.0:
            x = 0.0
        if x > 1.0:
            x = 1.0
        return max(0, min(255, int(round(x * 255))))
    return None


def _parse_optional_quat(data: dict, prefix: str):
    """
    Legge un quaternione opzionale con chiavi tipo:
      {prefix}_w, {prefix}_x, {prefix}_y, {prefix}_z

    Se tutti i componenti sono assenti -> None.
    Se il quaternione è parziale o contiene valori invalidi -> None.
    """
    keys = [f"{prefix}_w", f"{prefix}_x", f"{prefix}_y", f"{prefix}_z"]
    raw = [data.get(k) for k in keys]
    if all(v is None for v in raw):
        return None
    vals = [_clamp_float(v, -1.0, 1.0) for v in raw]
    if any(v is None for v in vals):
        return None
    return {
        f"{prefix}_w": float(vals[0]),
        f"{prefix}_x": float(vals[1]),
        f"{prefix}_y": float(vals[2]),
        f"{prefix}_z": float(vals[3]),
    }


def _handle_legacy_mode_if_present(mode_str: str):
    """
    Interpreta una mode stringa legacy (es. "IDLE", "RELATIVE_MOVE", "ABSOLUTE_POSE")
    e la converte nel codice int corrispondente, oppure ritorna None se non riconosciuta.
    Identico alla logica legacy attuale in validate_and_build_intent.
    Nessuna modifica comportamentale: solo estratto per pulizia del dispatcher.
    """
    return MODE_TO_J5VR_LEGACY.get(mode_str)


def validate_and_build_intent(data):
    """
    Valida il JSON ricevuto e costruisce l'intent interno.
    Ritorna (intent_dict, error_msg). Se error_msg non è None, intent è None.
    """
    global _teleoppose_latched_until

    if not isinstance(data, dict):
        return None, "body non è un oggetto JSON"

    # mode (v1.1.5+ target: int 0|1|2|3|4; legacy: stringa)
    mode_raw = data.get("mode")
    mode_code = None
    if isinstance(mode_raw, int):
        if mode_raw in MODE_VALUES_V115:
            mode_code = int(mode_raw)
    elif isinstance(mode_raw, str):
        mode_code = _handle_legacy_mode_if_present(mode_raw)
    if mode_code is None:
        return None, f"mode deve essere int in {MODE_VALUES_V115} (target v1.1.5+) oppure legacy in {MODE_VALUES_LEGACY}"

    # NOTE [HYBRID-H3]: gating HYBRID lato RPi (control-plane).
    # Se HYBRID non è abilitato lato RPi, forziamo fallback sicuro a MANUAL (mode=2).
    if mode_code == 4:
        from . import ws_handlers_uart

        if not ws_handlers_uart.is_hybrid_enabled_rpi():
            mode_code = 2

    # Invariante: mode_code deve rimanere all'interno di MODE_VALUES_V115.
    # Questa assert non cambia il comportamento per input validi, ma
    # documenta l'aspettativa ed evidenzia eventuali bug futuri in validazione.
    assert mode_code in MODE_VALUES_V115, "[INTENT][ASSERT] mode_code fuori range MODE_VALUES_V115"

    joy_x_i16    = _parse_axis_i16(data.get("joy_x"))
    joy_y_i16    = _parse_axis_i16(data.get("joy_y"))
    pitch_i16    = _parse_axis_i16(data.get("pitch"))
    yaw_i16      = _parse_axis_i16(data.get("yaw"))
    intensity_u8 = _parse_intensity_u8(data.get("intensity"))

    if joy_x_i16 is None:
        return None, "joy_x mancante o non numerico/valido (int16 o float normalizzato)"
    if joy_y_i16 is None:
        return None, "joy_y mancante o non numerico/valido (int16 o float normalizzato)"
    if pitch_i16 is None:
        return None, "pitch mancante o non numerico/valido (int16 o float normalizzato)"
    if yaw_i16 is None:
        return None, "yaw mancante o non numerico/valido (int16 o float normalizzato)"
    if intensity_u8 is None:
        return None, "intensity mancante o non numerico/valido (uint8 o float normalizzato)"

    # grip (opzionale: default 0)
    grip = data.get("grip", 0)
    if grip not in (0, 1):
        grip = 0

    # heartbeat (opzionale: default 0)
    heartbeat = data.get("heartbeat", 0)
    if not isinstance(heartbeat, int) or not _is_finite(heartbeat):
        heartbeat = 0

    # Quaternioni orientamento visore (opzionali, default identità)
    quat_w = _clamp_float(data.get("quat_w"), -1.0, 1.0)
    quat_x = _clamp_float(data.get("quat_x"), -1.0, 1.0)
    quat_y = _clamp_float(data.get("quat_y"), -1.0, 1.0)
    quat_z = _clamp_float(data.get("quat_z"), -1.0, 1.0)
    if quat_w is None:
        quat_w = 1.0
    if quat_x is None:
        quat_x = 0.0
    if quat_y is None:
        quat_y = 0.0
    if quat_z is None:
        quat_z = 0.0

    ctrl_left_quat = _parse_optional_quat(data, "ctrl_left_quat")
    ctrl_right_quat = _parse_optional_quat(data, "ctrl_right_quat")

    # Pulsanti joystick (opzionali, default 0)
    buttons_left  = data.get("buttons_left", 0)
    buttons_right = data.get("buttons_right", 0)
    if not isinstance(buttons_left, int) or buttons_left < 0 or buttons_left > 65535:
        buttons_left = 0
    if not isinstance(buttons_right, int) or buttons_right < 0 or buttons_right > 65535:
        buttons_right = 0

    # camctrl (opzionale): { cmd: "focus"|"zoom"|"conv", delta: int }
    camctrl         = data.get("camctrl")
    camctrl_payload = None
    camctrl_obj     = None
    if isinstance(camctrl, dict):
        camctrl_cmd = camctrl.get("cmd")
        delta       = camctrl.get("delta")
        if camctrl_cmd in ("focus", "zoom", "conv") and isinstance(delta, int) and delta != 0:
            # Mini-payload dedicato (layer dataplane): formato testuale richiesto
            camctrl_payload = f"CAMCTRL,{camctrl_cmd},{int(delta)}"
            camctrl_obj     = {"cmd": camctrl_cmd, "delta": int(delta)}

    intent = {
        "mode":         int(mode_code),   # 0..4
        "joy_x":        int(joy_x_i16),
        "joy_y":        int(joy_y_i16),
        "pitch":        int(pitch_i16),
        "yaw":          int(yaw_i16),
        "intensity":    int(intensity_u8),
        "grip":         int(grip),
        "heartbeat":    int(heartbeat),
        "quat_w":       float(quat_w),
        "quat_x":       float(quat_x),
        "quat_y":       float(quat_y),
        "quat_z":       float(quat_z),
        "buttons_left":  int(buttons_left)  & 0xFFFF,
        "buttons_right": int(buttons_right) & 0xFFFF,
        "timestamp":    time.monotonic(),
    }
    if ctrl_left_quat is not None:
        intent.update(ctrl_left_quat)
    if ctrl_right_quat is not None:
        intent.update(ctrl_right_quat)
    if camctrl_obj is not None:
        intent["camctrl"]         = camctrl_obj
        intent["camctrl_payload"] = camctrl_payload
    return intent, None


# ---------------------------------------------------------------------------
# Handler set_vr_mode (HEAD mode dashboard)
# ---------------------------------------------------------------------------

async def handle_set_vr_mode(websocket, data: dict) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Gestisce il messaggio type=set_vr_mode (HEAD mode con heartbeat e grip).
    Per mode 2 (HEAD) e 3 (HYBRID): applica al firmware la config persistita
    in routing_config.json prima
    di attivare la modalità (HEADZERO è solo calib quaternion, non sostituisce questo apply).
    """
    global _head_mode_task, _head_mode_stop_event, _head_mode_hb, _dashboard_head_active

    from controller.web_services.ws_handlers_uart import apply_persisted_vr_config_to_firmware

    async def _stop_head_mode_clean() -> None:
        """Arresta il loop HEAD e rilascia subito ogni forzatura sticky."""
        global _head_mode_task, _dashboard_head_active
        _dashboard_head_active = False
        task = _head_mode_task
        if task and not task.done():
            try:
                if _head_mode_stop_event is not None:
                    _head_mode_stop_event.set()
                await asyncio.wait_for(task, timeout=max(0.20, _FEEDBACK_LOOP_SLEEP_S * 6.0))
            except asyncio.TimeoutError:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
            except Exception:
                pass
        cur = dict(shared_state.latest_intent or {})
        cur["mode"] = 1
        cur["grip"] = 0
        cur["buttons_left"] = 0
        cur["buttons_right"] = 0
        cur["timestamp"] = time.monotonic()
        shared_state.latest_intent = cur
        shared_state.write_intent_to_file(cur)
        _head_mode_task = None

    new_mode = int(data.get("mode", 2))
    try:
        if new_mode == 3:
            # Ferma eventuale task precedente
            await _stop_head_mode_clean()
            _head_mode_stop_event = asyncio.Event()
            _head_mode_hb = 1

            vr_ok = await apply_persisted_vr_config_to_firmware()
            logger.info("[WS] set_vr_mode HEAD: persisted->firmware ok=%s (prima heartbeat)", vr_ok)

            async def _head_heartbeat_loop():
                global _head_mode_hb, _dashboard_head_active, _head_mode_task
                GRIP_BTN = 0x0002  # bit1 = grip
                try:
                    _dashboard_head_active = True
                    logger.info("[WS] HEAD mode heartbeat loop avviato")
                    while not _head_mode_stop_event.is_set():
                        cur = dict(shared_state.latest_intent or {})
                        cur["mode"]          = 3
                        cur["grip"]          = 1
                        cur["buttons_left"]  = GRIP_BTN
                        cur["buttons_right"] = GRIP_BTN
                        cur["heartbeat"]     = _head_mode_hb
                        cur["timestamp"]     = time.monotonic()
                        _head_mode_hb = (_head_mode_hb + 1) & 0xFFFF
                        shared_state.latest_intent = cur
                        shared_state.write_intent_to_file(cur)
                        await asyncio.sleep(0.02)  # 50 Hz — più veloce del visore (100Hz) per vincere la gara
                finally:
                    # Rilascio sicuro in ogni caso (stop/cancel/errore).
                    _dashboard_head_active = False
                    cur = dict(shared_state.latest_intent or {})
                    cur["mode"]          = 2
                    cur["grip"]          = 0
                    cur["buttons_left"]  = 0
                    cur["buttons_right"] = 0
                    shared_state.latest_intent = cur
                    shared_state.write_intent_to_file(cur)
                    _head_mode_task = None
                    logger.info("[WS] HEAD mode heartbeat loop terminato")

            _head_mode_task = asyncio.create_task(_head_heartbeat_loop())
            logger.info("[WS] set_vr_mode → HEAD (mode=3) con deadman+heartbeat attivi")

        else:
            # Ferma HEAD mode
            await _stop_head_mode_clean()
            logger.info("[WS] set_vr_mode → stop HEAD mode (clean)")
            if new_mode in (4, 5):
                vr_ok = await apply_persisted_vr_config_to_firmware()
                label = "HYBRID" if new_mode == 4 else "HEAD ASSIST"
                logger.info("[WS] set_vr_mode %s (mode=%d): persisted->firmware ok=%s", label, new_mode, vr_ok)

        await websocket.send(json.dumps({"type": "ack", "set_vr_mode": new_mode}))
    except Exception as e:
        logger.warning("[WS] set_vr_mode error: %s", e)


