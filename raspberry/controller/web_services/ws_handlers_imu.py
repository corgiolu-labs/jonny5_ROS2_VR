"""
ws_handlers_imu.py — Loop IMU debug, conversione quaternione→RPY,
                      debounce validità IMU, feedback_loop ACK TELEOPPOSE.

Contiene:
  - imu_debug_loop    — pubblica telemetria IMU + VR ai client ogni 10 ms
  - feedback_loop     — invia ACK teleop_pose_ack ai client ogni 50 ms
  - _quat_to_rpy_deg  — conversione quaternione (w,x,y,z) → (roll,pitch,yaw)°
  - stato debounce IMU (imu_valid_stable, contatori, last-state)
  - DEBUG_IMU_UI / IMU_DEBOUNCE_N — costanti IMU locali

Contratti importanti:
  - `imu_q_*` descrive un orientamento stimato del polso/end-effector, non una
    misura cinematica completa del robot;
  - `servo_deg_*` Ã¨ telemetria di command state / stato interno comandato
    esportata dal firmware per UI e diagnostica, non feedback encoder reale;
  - questo modulo inoltra telemetria/diagnostica/high-level status verso WS,
    ma non partecipa al servo loop hard real-time STM32.

- sintesi operativa ASCII: imu_q_* = estimated orientation, servo_deg_* = commanded/internal servo state.

[RPi-0.5] Creato da ws_server.py. Nessuna modifica comportamentale.
[RPi-0.6] Import ordinati PEP8; docstring modulo aggiornata.
[RPi-0.7] Aggiunti commenti avviso su stato debounce condiviso e
          iterazione su `clients` senza lock (safe asyncio single-thread).
"""

# stdlib
import asyncio
import json
import logging
import math
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation

# local
from controller.teleop import shared_state
from controller.uart import uart_manager
from controller.web_services import ik_solver, runtime_config_paths as rcfg, settings_manager
from controller.web_services.ws_core import (
    _IMU_DEBUG_SLEEP_S,
    _FEEDBACK_LOOP_SLEEP_S,
    clients,
    _ws_safe_send,
)
from controller.web_services import ws_handlers_uart as _ws_handlers_uart

logger = logging.getLogger("ws_teleop")

# ---------------------------------------------------------------------------
# Costanti debounce IMU
# ---------------------------------------------------------------------------
IMU_DEBOUNCE_N = 2

# Debug IMU in UI VR: quando True, il server invia imu_valid e imu_q_* ai client
DEBUG_IMU_UI = True

# ---------------------------------------------------------------------------
# Stato debounce IMU (modulo-level, aggiornato da imu_debug_loop)
#
# NOTA [RPi-0.7]: queste variabili sono scritte e lette esclusivamente
# dall'event loop asyncio (single-thread), quindi non richiedono lock.
# imu_valid_stable è letta anche da imu_debug_loop per decidere il payload
# da inviare ai client. Safe per il modello asyncio cooperativo.
# ---------------------------------------------------------------------------
imu_valid_stable:      bool  = False
imu_consecutive_true:  int   = 0
imu_consecutive_false: int   = 0
_last_telemetry_fresh: "bool | None" = None
_last_imu_valid_raw:   "bool | None" = None
_last_imu_valid_stable:"bool | None" = None

# ---------------------------------------------------------------------------
# Stato feedback loop
# ---------------------------------------------------------------------------
_last_feedback_id: int = -1
_last_spi_packet_index: "int | None" = None
_last_spi_packet_ts: "float | None" = None
_last_fk_live_key: "tuple | None" = None
_last_fk_live_payload: "dict | None" = None

_WRIST_CENTER_TO_TOOL_TOOL_M = np.array([0.06, 0.0, 0.0], dtype=float)


# ---------------------------------------------------------------------------
# Settings cache — avoid re-reading JSON every 10 ms tick
# ---------------------------------------------------------------------------
_cached_settings: "dict | None" = None
_cached_settings_ts: float = 0.0
_SETTINGS_CACHE_TTL_S = 2.0

# Stato zoom VR (ricevuto dal viewer XR via vr_zoom_state, incluso in telemetry).
_vr_zoom: dict = {"zoom0": None, "zoom1": None, "ts": 0.0}


def set_vr_zoom_state(zoom0, zoom1) -> None:
    """Aggiorna lo stato zoom VR (chiamato da ws_server al ricevere vr_zoom_state)."""
    if isinstance(zoom0, (int, float)):
        _vr_zoom["zoom0"] = float(zoom0)
    if isinstance(zoom1, (int, float)):
        _vr_zoom["zoom1"] = float(zoom1)
    _vr_zoom["ts"] = time.monotonic()


def _get_cached_settings() -> dict:
    global _cached_settings, _cached_settings_ts
    now = time.monotonic()
    if _cached_settings is None or (now - _cached_settings_ts) > _SETTINGS_CACHE_TTL_S:
        _cached_settings = settings_manager.load()
        _cached_settings_ts = now
    return _cached_settings


# ---------------------------------------------------------------------------
# Cache imu_world_bias.json — applicata al quaternione IMU prima del display.
#
# Lo storico `imu_q_*` nel payload telemetria conteneva il quaternione raw del
# BNO085, il cui yaw e' allineato al nord magnetico (non al HOME meccanico).
# Per dare all'operatore yaw=0 quando il robot e' in HOME, applichiamo qui
# `R_world_bias^-1 * R_imu_raw`: world_bias e' il quaternione catturato in HOME
# tramite il tool calibrate_world_bias.py (sola componente di yaw assoluto).
# Il quaternione raw resta esposto come `imu_q_raw_w/x/y/z` per debug.
# ---------------------------------------------------------------------------
_cached_world_bias_inv: "tuple[float,float,float,float] | None" = None
_cached_world_bias_ts: float = 0.0
_WORLD_BIAS_CACHE_TTL_S = 60.0


def _load_world_bias_inv() -> "tuple[float,float,float,float] | None":
    """Ritorna (w,x,y,z) del quaternione inverso di world_bias, o None se assente."""
    global _cached_world_bias_inv, _cached_world_bias_ts
    now = time.monotonic()
    if (now - _cached_world_bias_ts) < _WORLD_BIAS_CACHE_TTL_S:
        return _cached_world_bias_inv
    _cached_world_bias_ts = now
    try:
        cfg = rcfg.load_runtime_json("imu_world_bias", default=None)
        if not cfg or not isinstance(cfg, dict):
            _cached_world_bias_inv = None
            return None
        q = cfg.get("quat_wxyz")
        if not q or len(q) != 4:
            _cached_world_bias_inv = None
            return None
        w, x, y, z = (float(v) for v in q)
        n2 = w * w + x * x + y * y + z * z
        if n2 <= 0.0:
            _cached_world_bias_inv = None
            return None
        # Inverso di un quaternione unitario = coniugato. Normalizziamo per
        # robustezza nel caso il file non sia perfettamente unitario.
        inv_norm = 1.0 / math.sqrt(n2)
        # Coniugato: (w, -x, -y, -z), poi normalizzato.
        _cached_world_bias_inv = (
            w * inv_norm,
            -x * inv_norm,
            -y * inv_norm,
            -z * inv_norm,
        )
        return _cached_world_bias_inv
    except Exception:
        _cached_world_bias_inv = None
        return None


def invalidate_world_bias_cache() -> None:
    """Forza il reload di imu_world_bias.json al prossimo tick.

    Da chiamare dopo aver scritto il file (es. handler set_imu_world_bias del
    ws_server) per garantire che la correzione applicata al payload tracking
    sia sempre coerente con il file persistito.
    """
    global _cached_world_bias_ts
    _cached_world_bias_ts = 0.0


def _quat_mul_wxyz(a, b):
    """Moltiplicazione quaternionica (w,x,y,z) * (w,x,y,z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _apply_world_bias_correction(qw: float, qx: float, qy: float, qz: float):
    """Applica R_world_bias^-1 * R_imu_raw. Ritorna (qw, qx, qy, qz) corretti.

    Se `imu_world_bias.json` e' assente o invalido, restituisce il quaternione
    invariato (graceful degradation: comportamento pre-modifica).
    """
    inv = _load_world_bias_inv()
    if inv is None:
        return qw, qx, qy, qz
    return _quat_mul_wxyz(inv, (qw, qx, qy, qz))


def invalidate_settings_cache() -> None:
    """Forza il reload settings al prossimo tick. Da chiamare dopo settings_manager.save()."""
    global _cached_settings_ts
    _cached_settings_ts = 0.0


# ---------------------------------------------------------------------------
# Conversione quaternione → RPY
# ---------------------------------------------------------------------------

def _quat_to_rpy_deg(w: float, x: float, y: float, z: float) -> tuple:
    """
    Converte un quaternione (w, x, y, z) in (roll, pitch, yaw) in gradi.
    """
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw  = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def _append_pose_payload(payload: dict, intent: dict, in_prefix: str, out_prefix: str) -> bool:
    """
    Copia un quaternione opzionale dall'intent al payload WS e aggiunge anche gli Euler in gradi.
    Ritorna True se il quaternione sorgente era completo e valido.
    """
    keys = {
        "w": f"{in_prefix}_w",
        "x": f"{in_prefix}_x",
        "y": f"{in_prefix}_y",
        "z": f"{in_prefix}_z",
    }
    try:
        qw = intent.get(keys["w"])
        qx = intent.get(keys["x"])
        qy = intent.get(keys["y"])
        qz = intent.get(keys["z"])
        if qw is None or qx is None or qy is None or qz is None:
            return False
        qw = float(qw)
        qx = float(qx)
        qy = float(qy)
        qz = float(qz)
        roll, pitch, yaw = _quat_to_rpy_deg(qw, qx, qy, qz)
        payload[f"{out_prefix}_roll"] = roll
        payload[f"{out_prefix}_pitch"] = pitch
        payload[f"{out_prefix}_yaw"] = yaw
        payload[f"{out_prefix}_quat_w"] = round(qw, 4)
        payload[f"{out_prefix}_quat_x"] = round(qx, 4)
        payload[f"{out_prefix}_quat_y"] = round(qy, 4)
        payload[f"{out_prefix}_quat_z"] = round(qz, 4)
        return True
    except Exception:
        return False


def _compute_live_fk_payload(telemetry: dict) -> dict:
    """
    Costruisce una posa FK live coerente con la telemetria servo fisica.

    Espone:
      - posa tool (x/y/z + YPR + quat)
      - posa wrist-center (x/y/z), utile per ancorare la stima IMU short-term
    """
    global _last_fk_live_key, _last_fk_live_payload

    try:
        physical = [
            float(telemetry["servo_deg_B"]),
            float(telemetry["servo_deg_S"]),
            float(telemetry["servo_deg_G"]),
            float(telemetry["servo_deg_Y"]),
            float(telemetry["servo_deg_P"]),
            float(telemetry["servo_deg_R"]),
        ]
        cfg = _get_cached_settings()
        offsets = [float(v) for v in cfg.get("offsets", settings_manager.DEFAULTS.get("offsets", [104, 107, 77, 88, 95, 104]))]
        dirs = [int(v) for v in cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))]
        cache_key = (
            tuple(round(v, 4) for v in physical),
            tuple(round(v, 4) for v in offsets),
            tuple(int(v) for v in dirs),
        )
        if cache_key == _last_fk_live_key and isinstance(_last_fk_live_payload, dict):
            return dict(_last_fk_live_payload)

        virt = settings_manager.physical_to_virtual(list(physical), list(offsets), list(dirs))
        fk = ik_solver.compute_fk_poe_virtual_deg(list(virt))
        if not bool(fk.get("ok")):
            return {}
        quat_xyzw = fk.get("quat_xyzw")
        if not isinstance(quat_xyzw, list) or len(quat_xyzw) != 4:
            return {}

        p_tool_m = np.array(
            [
                float(fk.get("x_mm", 0.0)) / 1000.0,
                float(fk.get("y_mm", 0.0)) / 1000.0,
                float(fk.get("z_mm", 0.0)) / 1000.0,
            ],
            dtype=float,
        )
        r_tool = Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
        p_wc_m = p_tool_m - (r_tool @ _WRIST_CENTER_TO_TOOL_TOOL_M)

        out = {
            "fk_live_valid": True,
            "fk_live_x_mm": round(float(fk["x_mm"]), 3),
            "fk_live_y_mm": round(float(fk["y_mm"]), 3),
            "fk_live_z_mm": round(float(fk["z_mm"]), 3),
            "fk_live_yaw": round(float(fk["yaw_deg"]), 3),
            "fk_live_pitch": round(float(fk["pitch_deg"]), 3),
            "fk_live_roll": round(float(fk["roll_deg"]), 3),
            "fk_live_quat_x": round(float(quat_xyzw[0]), 6),
            "fk_live_quat_y": round(float(quat_xyzw[1]), 6),
            "fk_live_quat_z": round(float(quat_xyzw[2]), 6),
            "fk_live_quat_w": round(float(quat_xyzw[3]), 6),
            "fk_live_wc_x_mm": round(float(p_wc_m[0] * 1000.0), 3),
            "fk_live_wc_y_mm": round(float(p_wc_m[1] * 1000.0), 3),
            "fk_live_wc_z_mm": round(float(p_wc_m[2] * 1000.0), 3),
        }
        _last_fk_live_key = cache_key
        _last_fk_live_payload = dict(out)
        return dict(out)
    except Exception:
        return {}


# ---------------------------------------------------------------------------
# imu_debug_loop
#
# NOTA [RPi-0.7]: `clients` è il set condiviso da ws_core, acceduto senza lock.
# Safe perché sia imu_debug_loop che feedback_loop girano nello stesso event loop
# asyncio (single-thread cooperative). list(clients) crea uno snapshot per
# evitare "Set changed size during iteration" se un client si disconnette
# durante l'iterazione. Non modificare questo pattern senza test.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# imu_debug_loop helpers — each builds one section of the telemetry payload
# ---------------------------------------------------------------------------

_IMU_KEYS = ("imu_accel_x", "imu_accel_y", "imu_accel_z",
             "imu_gyro_x", "imu_gyro_y", "imu_gyro_z")
_SERVO_KEYS = ("servo_deg_B", "servo_deg_S", "servo_deg_G",
               "servo_deg_Y", "servo_deg_P", "servo_deg_R")
_SPI_DIAG_KEYS = ("imu_sample_counter", "imu_sample_delta",
                   "imu_sample_repeated", "imu_sample_jump", "imu_rate_hz_est",
                   "rt_loop_period_us", "rt_loop_hz_est")


def _debounce_imu_valid(telemetry_fresh: bool, telemetry: dict) -> None:
    """Update imu_valid_stable with 2-sample debounce."""
    global imu_valid_stable, imu_consecutive_true, imu_consecutive_false
    global _last_telemetry_fresh, _last_imu_valid_raw, _last_imu_valid_stable
    imu_valid_raw = telemetry_fresh and telemetry.get("imu_valid", False)
    if imu_valid_raw:
        imu_consecutive_true += 1; imu_consecutive_false = 0
        if imu_consecutive_true >= IMU_DEBOUNCE_N: imu_valid_stable = True
    else:
        imu_consecutive_false += 1; imu_consecutive_true = 0
        if imu_consecutive_false >= IMU_DEBOUNCE_N: imu_valid_stable = False
    if (telemetry_fresh != _last_telemetry_fresh or
            imu_valid_raw != _last_imu_valid_raw or
            imu_valid_stable != _last_imu_valid_stable):
        logger.info("[IMU] fresh=%s raw=%s stable=%s",
                    telemetry_fresh, imu_valid_raw, imu_valid_stable)
        _last_telemetry_fresh = telemetry_fresh
        _last_imu_valid_raw = imu_valid_raw
        _last_imu_valid_stable = imu_valid_stable


def _build_imu_servo_payload(telemetry: dict) -> dict:
    """Extract IMU + servo fields from telemetry.

    `imu_q_*` riporta il quaternione *corretto* da world_bias (yaw assoluto in
    HOME cancellato), in modo che il display dashboard mostri yaw=0 quando il
    robot e' in HOME. Il quaternione raw del BNO085 e' comunque esposto come
    `imu_q_raw_*` per debug e per i tool di analytics.
    """
    payload: dict = {"imu_valid": imu_valid_stable}
    for k in _IMU_KEYS:
        if k in telemetry: payload[k] = telemetry[k]
    qw_raw = float(telemetry.get("imu_q_w", 1.0))
    qx_raw = float(telemetry.get("imu_q_x", 0.0))
    qy_raw = float(telemetry.get("imu_q_y", 0.0))
    qz_raw = float(telemetry.get("imu_q_z", 0.0))
    qw, qx, qy, qz = _apply_world_bias_correction(qw_raw, qx_raw, qy_raw, qz_raw)
    payload["imu_q_w"] = qw
    payload["imu_q_x"] = qx
    payload["imu_q_y"] = qy
    payload["imu_q_z"] = qz
    payload["imu_q_raw_w"] = qw_raw
    payload["imu_q_raw_x"] = qx_raw
    payload["imu_q_raw_y"] = qy_raw
    payload["imu_q_raw_z"] = qz_raw
    for k in _SERVO_KEYS:
        if k in telemetry: payload[k] = telemetry[k]
    if "imu_temp" in telemetry: payload["imu_temp"] = telemetry["imu_temp"]
    return payload


def _append_intent_to_payload(payload: dict, intent, intent_fresh: bool) -> None:
    """Append VR headset + controller poses and teleop state."""
    global _vr_active_ts
    vr_active = False
    if intent:
        try:
            if _append_pose_payload(payload, intent, "quat", "vr"):
                vr_active = intent_fresh
                if vr_active:
                    _vr_active_ts = time.monotonic()
        except Exception: pass
    payload["vr_active"] = vr_active
    ctrl_left_ok = ctrl_right_ok = False
    if intent:
        try:
            ctrl_left_ok = _append_pose_payload(payload, intent, "ctrl_left_quat", "ctrl_left")
            ctrl_right_ok = _append_pose_payload(payload, intent, "ctrl_right_quat", "ctrl_right")
        except Exception: pass
    payload["ctrl_left_active"] = bool(intent_fresh and intent and ctrl_left_ok)
    payload["ctrl_right_active"] = bool(intent_fresh and intent and ctrl_right_ok)
    if intent and isinstance(intent, dict):
        payload["intent_mode"] = intent.get("mode")
        payload["intent_heartbeat"] = intent.get("heartbeat")
        intent_mtime = shared_state.get_intent_cache_mtime()
        if intent_mtime > 0:
            payload["intent_age_ms"] = int((time.time() - intent_mtime) * 1000)
        payload["teleop_mode"] = int(intent.get("mode", -1))
        payload["joy_x"] = intent.get("joy_x", 0)
        payload["joy_y"] = intent.get("joy_y", 0)
        payload["intensity"] = intent.get("intensity", 0)
    else:
        payload["teleop_mode"] = -1



def _append_spi_rate(payload: dict, telemetry: dict) -> None:
    """Compute SPI packet rate and append diagnostics."""
    global _last_spi_packet_index, _last_spi_packet_ts
    if "packet_index" in telemetry:
        payload["spi_packet_index"] = telemetry["packet_index"]
        try:
            cur_idx = int(telemetry["packet_index"])
            now_mono = time.monotonic()
            if _last_spi_packet_index is not None and _last_spi_packet_ts is not None:
                d_idx = cur_idx - _last_spi_packet_index
                dt_s = now_mono - _last_spi_packet_ts
                if dt_s > 1e-6 and d_idx >= 0:
                    payload["ws_spi_rate_hz_est"] = float(d_idx / dt_s)
            _last_spi_packet_index = cur_idx
            _last_spi_packet_ts = now_mono
        except Exception: pass
    for k in _SPI_DIAG_KEYS:
        if k in telemetry: payload[k] = telemetry[k]


# FK throttle state
_fk_counter: int = 0
_fk_cache: dict = {}

# Per-client task pendente: evita accumulo di future a 100 Hz se un client è lento.
# Se il task precedente non è ancora completato, il frame viene droppato per quel client.
_pending_sends: dict = {}  # ws → asyncio.Task

# Telemetria adattiva: timestamp dell'ultimo intent VR con quaternione valido.
# Se il visore non è attivo da più di _VR_ACTIVE_WINDOW_S secondi,
# il loop rallenta a 20 Hz per ridurre il carico CPU sul Pi.
_vr_active_ts: float = 0.0
_VR_ACTIVE_WINDOW_S: float = 2.0
_IMU_DEBUG_SLEEP_IDLE_S: float = 0.050  # 20 Hz senza visore



# ---------------------------------------------------------------------------
# Main telemetry broadcast loop (100 Hz)
# ---------------------------------------------------------------------------

async def imu_debug_loop(get_robot_state) -> None:
    """Broadcast telemetry to WS clients.
    Rate adattivo: 100 Hz con visore attivo, 20 Hz senza (solo dashboard).
    FK throttled a 10 Hz; fire-and-forget broadcast."""
    global _fk_counter, _fk_cache

    while True:
        try:
            if DEBUG_IMU_UI and clients:
                telemetry = shared_state.read_telemetry_from_file()
                if isinstance(telemetry, dict) and (
                    "imu_valid" in telemetry or "imu_q_w" in telemetry or "servo_deg_B" in telemetry
                ):
                    telemetry_fresh = shared_state.is_telemetry_fresh(1.5)
                    _debounce_imu_valid(telemetry_fresh, telemetry)

                    payload = _build_imu_servo_payload(telemetry)

                    _fk_counter += 1
                    if _fk_counter % 10 == 0:
                        _fk_cache = _compute_live_fk_payload(telemetry)
                    payload.update(_fk_cache)

                    intent = shared_state.read_intent_from_file()
                    intent_fresh = shared_state.is_intent_fresh(1.5)
                    _append_intent_to_payload(payload, intent, intent_fresh)

                    payload["uart_active"] = uart_manager.is_available()
                    try:
                        payload["hybrid_allowed"] = bool(_ws_handlers_uart.is_hybrid_enabled_rpi())
                    except Exception:
                        payload["hybrid_allowed"] = False

                    _append_spi_rate(payload, telemetry)

                    payload["robot_state"] = get_robot_state()
                    # Zoom VR (cam0/cam1) inviato dal viewer XR via vr_zoom_state.
                    if _vr_zoom["zoom0"] is not None:
                        payload["vr_zoom0"] = _vr_zoom["zoom0"]
                    if _vr_zoom["zoom1"] is not None:
                        payload["vr_zoom1"] = _vr_zoom["zoom1"]
                    payload["type"] = "telemetry"
                    msg = json.dumps(payload)
                    active = set(clients)
                    # Rimuovi entry di client disconnessi
                    for stale in [k for k in _pending_sends if k not in active]:
                        del _pending_sends[stale]
                    for c in active:
                        task = _pending_sends.get(c)
                        if task is not None and not task.done():
                            continue  # client occupato: drop frame per questo tick
                        _pending_sends[c] = asyncio.create_task(_ws_safe_send(c, msg))
        except Exception:
            pass
        vr_live = (time.monotonic() - _vr_active_ts) < _VR_ACTIVE_WINDOW_S
        await asyncio.sleep(_IMU_DEBUG_SLEEP_S if vr_live else _IMU_DEBUG_SLEEP_IDLE_S)


# ---------------------------------------------------------------------------
# feedback_loop
# ---------------------------------------------------------------------------

async def feedback_loop() -> None:
    """
    Poll feedback file e inoltra ACK al WebViewer via WS.
    [RPi-0.4] Funzione standalone (non inline in handle_client): nessuna modifica strutturale necessaria.
    [RPi-0.5] Estratto da ws_server.py.
    """
    global _last_feedback_id
    while True:
        try:
            fb = shared_state.read_feedback_from_file()
            if isinstance(fb, dict) and fb.get("teleop_pose_ack") is True:
                fid = int(fb.get("id", 0))
                if fid != _last_feedback_id:
                    _last_feedback_id = fid
                    msg = json.dumps({"teleop_pose_ack": True})
                    if clients:
                        logger.info("[WS] TELEOPPOSE dispatched (id=%d, clients=%d)", fid, len(clients))
                        await asyncio.gather(*[c.send(msg) for c in list(clients)], return_exceptions=True)
        except Exception:
            pass
        await asyncio.sleep(_FEEDBACK_LOOP_SLEEP_S)
