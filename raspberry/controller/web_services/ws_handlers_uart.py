"""
ws_handlers_uart.py — Comandi UART e loop di sistema.

Contiene:
  - handle_set_imu        (type=set_imu)  — IMUON / IMUOFF
  - handle_uart_cmd       (type=uart)     — ENABLE, STOP, STATUS?, SAFE, RESET,
                                            HOME, PARK, TELEOPPOSE, SETPOSE,
                                            SET_JOINT_LIMITS, SET_VR_PARAMS,
                                            DEMO, IK_SOLVE
  - _run_demo_sequence / _wait_setpose_done_async — demo orchestrata
  - robot_state_poll_loop — polling STATUS? ogni 2 s
  - startup_imuon         — IMUON al boot con retry
  - on_uart_unsolicited / _broadcast_setpose_done — handler SETPOSE_DONE
  - get_robot_state / set_main_loop — accessori di stato

[RPi-0.5] Creato da ws_server.py. Nessuna modifica comportamentale.
[RPi-0.6] Import ordinati PEP8; rimossa _build_setpose_cmd_uart (duplicata
          di ws_handlers_settings._build_setpose_cmd, mai chiamata in questo modulo).
[RPi-0.7] Aggiunti commenti avviso su punti sensibili (Event a modulo load,
          on_uart_unsolicited cross-thread, robot_state write concurrency).
[RPi-1.0] _demo_stop_event spostato da modulo load a init_events();
          aggiunta init_events() chiamata da ws_server.main().

[Cleanup-PreFeature] Modulo volutamente "grande ma centrale":
- mantenuto monolitico per minimizzare il rischio prima di nuove feature;
- consentite solo micro-pulizie locali (dead code/commenti), no refactor strutturale.
"""

# stdlib
import asyncio
import json
import logging
import os
import time

# local
from controller.teleop import shared_state
from controller.uart import uart_manager
from controller.web_services import settings_manager
from controller.web_services import runtime_config_paths as rcfg
from controller.web_services.vr_config_defaults import (
    LEGACY_VR_HEAD_LOOP_DEFAULTS,
    VR_TUNE_DEFAULTS,
    merge_vr_config_with_defaults,
)
from controller.web_services.ws_core import (
    _ws_safe_send,
    clients,
    _STATUS_BOOT_DELAY_S,
    _STATUS_POLL_PERIOD_S,
)
from controller.web_services import self_test_imu
from controller.web_services import ik_solver as _ik_solver

logger = logging.getLogger("ws_teleop")

# Percorso routing_config.json (stesso file usato dalla dashboard IMU-VR e da https_server).
# NOTA: questo file rappresenta la configurazione persistita sul Raspberry Pi.
# L'applicazione al firmware è un passo esplicito e separato.
_CONTROLLER_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_RPI5_DIR = os.path.dirname(_CONTROLLER_DIR)
_JOINT_LIMITS_DEFAULT = {
    "base": {"min": 10, "max": 170},
    "spalla": {"min": 10, "max": 170},
    "gomito": {"min": 10, "max": 170},
    "yaw": {"min": 10, "max": 170},
    "pitch": {"min": 45, "max": 135},
    "roll": {"min": 45, "max": 135},
}
_JOINT_LIMITS_ORDER = ["base", "spalla", "gomito", "yaw", "pitch", "roll"]

# Throttle: intent visore a 50–100 Hz — evita spam SET_VR_PARAMS su ogni frame.
_INTENT_VR_CFG_THROTTLE_S = 2.5
_last_intent_vr_cfg_apply_mono: float = 0.0

# ---------------------------------------------------------------------------
# Stato condiviso del modulo UART
#
# NOTA [RPi-0.7]: _robot_state è scritto sia da robot_state_poll_loop (asyncio)
# sia da on_uart_unsolicited (chiamato dal thread UART reader via call_soon_threadsafe).
# L'assegnazione di str in CPython è atomica quindi è safe, ma in caso di
# refactor futuro (es. struttura dati complessa) sarà necessario un lock.
#
# NOTA [RPi-0.7]: (risolto in RPi-1.0) _demo_stop_event era creato a modulo load;
# ora è inizializzato da init_events() dentro il loop asyncio attivo.
# ---------------------------------------------------------------------------

# Stato robot STM32: aggiornato dal polling STATUS? ogni 2 s
# Valori: "IDLE" | "STOPPED" | "SAFE" | "UNKNOWN"
_robot_state: str = "UNKNOWN"

# NOTE [HYBRID-H3]: stato HYBRID lato RPi (control-plane).
_hybrid_enabled_rpi: bool = False

# Demo orchestrata lato Raspberry
# NOTA [RPi-1.0]: _demo_stop_event viene inizializzato da init_events()
# all'interno del loop asyncio attivo (non a modulo load). Idempotente.
_demo_task: "asyncio.Task | None" = None
_demo_stop_event: "asyncio.Event | None" = None

# Event settato dal callback unsolicited quando arriva SETPOSE_DONE.
# RESTA None a modulo load: creato dinamicamente in _wait_setpose_done_async().
_setpose_done_event: "asyncio.Event | None" = None

# Riferimento al loop principale (impostato da ws_server.main())
_main_loop: "asyncio.AbstractEventLoop | None" = None
_last_unknown_recovery_mono: float = 0.0
_unknown_recovery_cooldown_s: float = 3.0
_unknown_recovery_in_progress: bool = False
_unknown_since_mono: float = 0.0
_unknown_hard_timeout_s: float = 5.0
_hard_recovery_min_interval_s: float = 90.0
_HARD_RECOVERY_STAMP_PATH = "/tmp/j5_unknown_hard_recovery.ts"

# Richiesta "zero riferimento" dopo TELEOPPOSE completato.
# Viene usata per HEAD/HYBRID: dopo SETPOSE_DONE inviamo HEADZERO al firmware
# senza cambiare la modalità corrente.
_pose_zero_pending: bool = False
_pose_zero_mode: int = 0

# HOME → relax PITCH/ROLL: ogni volta che HOME viene accettato dal firmware
# (ack OK SETPOSE dopo traduzione lato Pi), al successivo SETPOSE_DONE inviamo
# RELAX_DIGITAL per rilasciare i due servo più stressati e ridurne il
# surriscaldamento. Altri servo restano ingaggiati. Flag consumata e azzerata
# una sola volta per HOME.
_home_relax_pending: bool = False
_self_test_task: "asyncio.Task | None" = None


def init_events() -> None:
    """
    [RPi-1.0] Inizializza gli asyncio.Event fissi del modulo dentro il loop attivo.
    Chiamare da ws_server.main() subito dopo asyncio.get_running_loop().
    Idempotente: se già inizializzato, non ricrea.
    Nota: _setpose_done_event NON viene creato qui — è gestito dinamicamente
    da _wait_setpose_done_async() ad ogni passo della demo.
    """
    global _demo_stop_event
    if _demo_stop_event is None:
        _demo_stop_event = asyncio.Event()


def set_main_loop(loop: "asyncio.AbstractEventLoop") -> None:
    """Registra il loop asyncio principale (chiamato da ws_server.main())."""
    global _main_loop
    _main_loop = loop


def get_robot_state() -> str:
    """Ritorna lo stato corrente del robot STM32."""
    return _robot_state


async def _send_json(websocket, payload: dict[str, object]) -> None:
    await _ws_safe_send(websocket, json.dumps(payload))


def _self_test_status_payload(message: str, *, state: str = "running", run_dir: str = "") -> dict[str, object]:
    return {
        "type": "self_test_status",
        "state": state,
        "message": message,
        "running": state == "running",
        "run_dir": run_dir,
    }


def _uart_response_payload(cmd: str, ok: bool, response: str, *, warning: str = "") -> dict[str, object]:
    return {
        "type": "uart_response",
        "cmd": cmd,
        "ok": ok,
        "response": response,
        "warning": warning,
    }


async def _send_uart_response(websocket, cmd: str, ok: bool, response: str, *, warning: str = "") -> None:
    await _send_json(websocket, _uart_response_payload(cmd, ok, response, warning=warning))


async def _broadcast_json(payload: dict) -> None:
    if not clients:
        return
    raw = json.dumps(payload)
    await asyncio.gather(*[_ws_safe_send(c, raw) for c in list(clients)], return_exceptions=True)


async def _emit_self_test_status(message: str, *, state: str = "running", run_dir: str = "") -> None:
    await _broadcast_json(_self_test_status_payload(message, state=state, run_dir=run_dir))


async def _emit_self_test_result(payload: dict[str, object]) -> None:
    await _broadcast_json({"type": "self_test_result", **payload})


async def _run_self_test_sequence() -> None:
    global _self_test_task
    stamp = time.strftime("self_test_%Y%m%d_%H%M%S")
    run_dir = os.path.join(_RPI5_DIR, "logs", "imu_validation", stamp)
    os.makedirs(run_dir, exist_ok=True)
    try:
        if _demo_task and not _demo_task.done():
            raise RuntimeError("DEMO is active; stop DEMO before running SELF TEST.")
        await _emit_self_test_status("Starting self-test...", run_dir=run_dir)

        async def _status(message: str) -> None:
            await _emit_self_test_status(message, run_dir=run_dir)

        payload = await self_test_imu.run_self_test_imu(run_dir, emit_status=_status)
        await _emit_self_test_status("Completed", state="completed", run_dir=run_dir)
        await _emit_self_test_result(payload)
    except Exception as e:
        logger.exception("[SELF TEST] failure: %s", e)
        payload = self_test_imu.build_failed_self_test_payload(run_dir, str(e))
        await _emit_self_test_status(str(e), state="failed", run_dir=run_dir)
        await _emit_self_test_result(payload)
    finally:
        _self_test_task = None


async def handle_self_test(websocket, data: dict) -> None:
    global _self_test_task
    action = str(data.get("action", "run")).strip().lower()
    if action != "run":
        await _send_json(websocket, {"type": "error", "message": f"Unsupported self_test action: {action}"})
        return
    if _self_test_task and not _self_test_task.done():
        await _send_json(websocket, _self_test_status_payload("Self-test already running"))
        return
    _self_test_task = asyncio.create_task(_run_self_test_sequence())
    await _send_json(websocket, _self_test_status_payload("Self-test started"))


def _parse_status_payload(response: str) -> str | None:
    """Converte una risposta STATUS:* nel valore stato interno."""
    if not isinstance(response, str):
        return None
    r = response.strip().upper()
    if r == "STATUS:IDLE":
        return "IDLE"
    if r == "STATUS:STOPPED":
        return "STOPPED"
    if r == "STATUS:SAFE":
        return "SAFE"
    if r == "STATUS:UNKNOWN":
        return "UNKNOWN"
    return None


async def _uart_cmd_with_retries(
    cmd: str,
    timeout_s: float = 1.0,
    attempts: int = 2,
    pause_s: float = 0.12,
) -> tuple[bool, str]:
    """Invio UART con retry breve per assorbire timeout transitori."""
    last_resp = "TIMEOUT"
    for i in range(max(1, int(attempts))):
        ok, resp = await uart_manager.send_uart_command(cmd, timeout_s=timeout_s)
        last_resp = resp
        if ok:
            return True, resp
        if i < attempts - 1:
            await asyncio.sleep(max(0.0, float(pause_s)))
    return False, last_resp


async def _read_robot_status_with_retries(
    attempts: int = 2,
    timeout_s: float = 0.9,
    pause_s: float = 0.12,
) -> str | None:
    """Legge STATUS? con retry e ritorna stato normalizzato o None."""
    ok, resp = await _uart_cmd_with_retries("STATUS?", timeout_s=timeout_s, attempts=attempts, pause_s=pause_s)
    if not ok:
        return None
    return _parse_status_payload(resp)


def _build_setpose_t_from_virtual(virtual_pose: list[int], duration_ms: int = 3000, planner: str = "RTR3") -> str:
    """Costruisce comando SETPOSE_T da una posa virtuale completa a 6 giunti."""
    current_settings = settings_manager.load()
    offsets = current_settings.get("offsets", settings_manager.DEFAULTS["offsets"])
    dirs = current_settings.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
    physical = settings_manager.virtual_to_physical(virtual_pose, offsets, dirs)
    return "SETPOSE_T " + " ".join(str(v) for v in physical) + f" {int(duration_ms)} {planner}"


def _build_home_setpose_t_cmd(duration_ms: int = 3000, planner: str = "RTR3") -> str:
    """Costruisce comando HOME dolce in spazio fisico."""
    current_settings = settings_manager.load()
    home_virtual = current_settings.get("home", settings_manager.DEFAULTS["home"])
    return _build_setpose_t_from_virtual(home_virtual, duration_ms=duration_ms, planner=planner)


def _read_last_hard_recovery_ts() -> float:
    try:
        with open(_HARD_RECOVERY_STAMP_PATH, "r", encoding="utf-8") as f:
            return float((f.read() or "0").strip() or "0")
    except Exception:
        return 0.0


def _write_last_hard_recovery_ts(ts: float) -> None:
    try:
        with open(_HARD_RECOVERY_STAMP_PATH, "w", encoding="utf-8") as f:
            f.write(f"{float(ts):.3f}")
    except Exception as e:
        logger.warning("[STATUS RECOVERY] impossibile scrivere stamp hard-recovery: %s", e)


async def _trigger_hard_unknown_recovery_if_allowed(reason: str) -> bool:
    """
    Ultima ratio: forza restart del processo ws-teleop (systemd Restart=always).
    Rate-limited con timestamp persistente su /tmp per evitare loop aggressivi.
    """
    now = time.time()
    last = _read_last_hard_recovery_ts()
    if (now - last) < _hard_recovery_min_interval_s:
        logger.warning(
            "[STATUS RECOVERY] hard-recovery rate-limited (delta=%.1fs < %.1fs) reason=%s",
            (now - last), _hard_recovery_min_interval_s, reason,
        )
        return False
    _write_last_hard_recovery_ts(now)
    logger.error("[STATUS RECOVERY] HARD restart ws-teleop trigger reason=%s", reason)
    await asyncio.sleep(0.05)
    os._exit(42)


def _load_global_joint_limits() -> dict:
    """Legge limiti giunto globali da routing_config runtime-only con guardrail esplicito."""
    out = {k: dict(v) for k, v in _JOINT_LIMITS_DEFAULT.items()}
    try:
        cfg = rcfg.load_routing_config_strict()
        limits = cfg.get("limits")
        if not isinstance(limits, dict):
            return out
        for j in _JOINT_LIMITS_ORDER:
            row = limits.get(j)
            if not isinstance(row, dict):
                continue
            mn = int(row.get("min", out[j]["min"]))
            mx = int(row.get("max", out[j]["max"]))
            if 0 <= mn < mx <= 180:
                out[j] = {"min": mn, "max": mx}
    except Exception as e:
        logger.warning("[JOINT_LIMITS] guardrail routing_config: %s", e)
        return out
    return out


def _clamp_setpose_cmd_if_needed(uart_cmd: str) -> tuple[str, str]:
    """
    Clamp globale limiti giunto su SETPOSE/SETPOSE_T (angoli fisici) e ritorna:
      (comando_clampato, warning_testuale)
    warning vuoto se nessun clamp.
    """
    if not isinstance(uart_cmd, str):
        return uart_cmd, ""
    parts = uart_cmd.strip().split()
    if len(parts) < 8:
        return uart_cmd, ""
    prefix = parts[0].upper()
    if prefix not in ("SETPOSE", "SETPOSE_T"):
        return uart_cmd, ""
    if len(parts) < 9:
        return uart_cmd, ""
    try:
        joints = [int(parts[i]) for i in range(1, 7)]
    except Exception:
        return uart_cmd, ""
    limits = _load_global_joint_limits()
    exceeded = []
    clamped = []
    for i, name in enumerate(_JOINT_LIMITS_ORDER):
        mn = int(limits[name]["min"])
        mx = int(limits[name]["max"])
        raw = int(joints[i])
        val = max(mn, min(mx, raw))
        if val != raw:
            exceeded.append(f"{name.upper()}:{raw}->{val} [{mn},{mx}]")
        clamped.append(val)
    if not exceeded:
        return uart_cmd, ""
    tail = " ".join(parts[7:])
    new_cmd = f"{parts[0]} {' '.join(str(v) for v in clamped)} {tail}"
    warn = "Joint limits exceeded: " + ", ".join(exceeded)
    return new_cmd, warn


def is_hybrid_enabled_rpi() -> bool:
    """Ritorna lo stato HYBRID lato RPi (non RT).
    True  → ultimo comando HYBRID ENABLE inviato.
    False → ultimo comando HYBRID DISABLE o default.
    """
    return _hybrid_enabled_rpi


# ---------------------------------------------------------------------------
# Handler set_imu (IMUON / IMUOFF)
# ---------------------------------------------------------------------------

async def handle_set_imu(websocket, data: dict) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Invia IMUON o IMUOFF via UART e risponde con l'esito.
    """
    enabled = bool(data.get("enabled", False))
    logger.info("[WS] set_imu msg= enabled=%s", enabled)
    try:
        ok, response = await uart_manager.send_uart_command("IMUON" if enabled else "IMUOFF", timeout_s=0.5)
        logger.info("[UART] set_imu enabled=%s ok=%s response=%s", enabled, ok, response or "")
        if not ok:
            logger.warning("UART IMU %s fallito", "IMUON" if enabled else "IMUOFF")
    except Exception as e:
        logger.warning("Errore invio comando IMU: %s", e)
    await _send_json(websocket, {"imu_enabled": enabled})


# ---------------------------------------------------------------------------
# Handler comandi UART generici (type=uart)
# ---------------------------------------------------------------------------

async def handle_uart_cmd(websocket, data: dict) -> None:
    """
    [RPi-0.5] Estratto 1:1 dal corpo di handle_client().
    Gestisce tutti i messaggi type=uart: ENABLE, STOP, STATUS?, SAFE, RESET,
    HOME, PARK, TELEOPPOSE, SETPOSE, SET_JOINT_LIMITS, SET_VR_PARAMS, DEMO, IK_SOLVE.
    """
    global _demo_task, _demo_stop_event, _pose_zero_pending, _pose_zero_mode

    cmd       = (data.get("cmd") or "").strip()
    cmd_upper = cmd.upper()

    # IK_SOLVE: risolto lato Raspberry (non inviato al firmware UART)
    # Formato: IK_SOLVE x y z roll pitch yaw [payload_b64]
    if cmd_upper.startswith("IK_SOLVE "):
        import base64
        t0 = time.monotonic()
        try:
            parts = cmd.split()
            x_mm     = float(parts[1])
            y_mm     = float(parts[2])
            z_mm     = float(parts[3])
            roll_deg = float(parts[4])
            pitch_deg= float(parts[5])
            yaw_deg  = float(parts[6])
            solver_kw = {}
            if len(parts) > 7:
                try:
                    raw = base64.b64decode(parts[7]).decode("utf-8")
                    opts = json.loads(raw)
                    if "reset_solver" in opts:
                        solver_kw["reset_solver"] = bool(opts["reset_solver"])
                    if "max_pos_error_mm" in opts:
                        solver_kw["max_pos_error_mm"] = float(opts["max_pos_error_mm"])
                    if "max_ori_error_deg" in opts:
                        solver_kw["max_ori_error_deg"] = float(opts["max_ori_error_deg"])
                except Exception:
                    pass
            result = _ik_solver.solve(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, **solver_kw)
            elapsed_ms = (time.monotonic() - t0) * 1000.0
            payload = {
                "type": "ik_result",
                "reachable": bool(result.get("reachable", False)),
                "angles_deg": result.get("angles_deg", []),
                "error_pos": round(float(result.get("error_pos", 0)), 4),
                "error_ori": round(float(result.get("error_ori", 0)), 4),
                "iterations": int(result.get("iterations", 0)),
                "message": result.get("message", ""),
                "solver_used": "POE",
                "elapsed_ms": round(elapsed_ms, 2),
            }
            logger.info("[IK_SOLVE] (%.1f,%.1f,%.1f) reachable=%s err_pos=%.3f err_ori=%.3f ms=%.1f",
                        x_mm, y_mm, z_mm, payload["reachable"], payload["error_pos"], payload["error_ori"], elapsed_ms)
        except Exception as e:
            elapsed_ms = (time.monotonic() - t0) * 1000.0
            payload = {
                "type": "ik_result",
                "reachable": False,
                "angles_deg": [],
                "error_pos": 0,
                "error_ori": 0,
                "iterations": 0,
                "message": f"IK_SOLVE errore: {e}",
                "solver_used": "POE",
                "elapsed_ms": round(elapsed_ms, 2),
            }
            logger.warning("[IK_SOLVE] errore: %s", e)
        try:
            await websocket.send(json.dumps(payload))
        except Exception:
            pass
        return

    # DEMO: orchestrata lato Raspberry (sequenza SETPOSE da settings.json)
    if cmd_upper == "DEMO":
        if _demo_task and not _demo_task.done():
            logger.info("[DEMO] toggle stop richiesto")
            _demo_stop_event.set()
            hold_cmd = _build_demo_hold_current_pose_cmd(duration_ms=120, planner="RTR3")
            hold_ok = True
            hold_resp = "OK DEMO STOPPED"
            if hold_cmd:
                hold_ok, hold_resp = await _uart_cmd_with_retries(hold_cmd, timeout_s=1.0, attempts=2, pause_s=0.08)
                if not hold_ok:
                    logger.warning("[DEMO] stop pulito fallito: %s", hold_resp)
            await _send_uart_response(
                websocket,
                "DEMO",
                hold_ok,
                "OK DEMO STOPPED" if hold_ok else f"DEMO STOP FAIL: {hold_resp}",
            )
        else:
            _demo_stop_event = asyncio.Event()
            _demo_task = asyncio.create_task(_run_demo_sequence())
            logger.info("[DEMO] task avviato")
            await _send_uart_response(websocket, "DEMO", True, "OK DEMO STARTED")
        return

    # Whitelist: comandi semplici (exact match, case-insensitive)
    _simple_cmds = {
        "ENABLE",
        "STOP",
        "STATUS?",
        "SAFE",
        "RESET",
        "IMUON",
        "IMUOFF",
        "HOME",
        "PARK",
        "TELEOPPOSE",
        "HYBRID ENABLE",
        "HYBRID DISABLE",
        "PP?",                # Pick&Place: lettura stato duty cycle
    }
    # SETPOSE accetta un payload variabile: "SETPOSE B S G Y P R vel PLANNER"
    _is_setpose = (cmd_upper.startswith("SETPOSE ") and len(cmd.strip()) > 8) or \
                  (cmd_upper.startswith("SETPOSE_T ") and len(cmd.strip()) > 10)
    # SETPOSE_T_HR (high resolution): angoli x10 = 50..1750 (5.0..175.0°)
    _is_setpose_hr = cmd_upper.startswith("SETPOSE_T_HR ") and len(cmd.strip()) > 13
    # SET_JOINT_LIMITS: "SET_JOINT_LIMITS Bmin Bmax Smin Smax Gmin Gmax Ymin Ymax Pmin Pmax Rmin Rmax"
    _is_joint_limits = cmd_upper.startswith("SET_JOINT_LIMITS ") and len(cmd.strip()) > 17
    # SET_VR_PARAMS: "SET_VR_PARAMS yaw_g pitch_g roll_g a_small a_large dz maxstep velmax veldig lpf_p lpf_r joy_dz"
    _is_vr_params = cmd_upper.startswith("SET_VR_PARAMS ") and len(cmd.strip()) > 14
    # Pick&Place test: "PP1 <0..100>" / "PP2 <0..100>" — duty cycle MOSFET gate.
    _is_pickplace = (cmd_upper.startswith("PP1 ") or cmd_upper.startswith("PP2 ")) \
                    and len(cmd.strip()) > 4

    if cmd_upper in _simple_cmds or _is_setpose or _is_setpose_hr or _is_joint_limits or _is_vr_params or _is_pickplace:
        # Recovery sicuro post-STOP/UNKNOWN:
        # se arriva ENABLE con stato non consistente, forza SAFE->ENABLE
        # e un AUTO_HOME dolce e deterministico a 6 giunti.
        if cmd_upper == "ENABLE":
            state_before = get_robot_state()
            state_probe = state_before
            try:
                st = await _read_robot_status_with_retries(attempts=2, timeout_s=0.8, pause_s=0.1)
                if st is not None:
                    state_probe = st
                    global _robot_state
                    _robot_state = state_probe
            except Exception:
                pass
            if state_probe in ("STOPPED", "UNKNOWN"):
                logger.warning(
                    "[SAFE RECOVERY] ENABLE richiesto con stato=%s: eseguo SAFE->ENABLE->AUTO_HOME",
                    state_probe,
                )
                ok_safe, resp_safe = await _uart_cmd_with_retries("SAFE", timeout_s=1.2, attempts=2, pause_s=0.15)
                if not ok_safe:
                    await _send_uart_response(websocket, "ENABLE", False, f"RECOVERY SAFE FAIL: {resp_safe}")
                    return

                ok_enable, resp_enable = await _uart_cmd_with_retries("ENABLE", timeout_s=1.2, attempts=2, pause_s=0.15)
                if not ok_enable:
                    await _send_uart_response(websocket, "ENABLE", False, f"RECOVERY ENABLE FAIL: {resp_enable}")
                    return

                # AUTO_HOME completa a 6 giunti.
                try:
                    home_cmd = _build_home_setpose_t_cmd(duration_ms=3000, planner="RTR3")
                    ok_home, resp_home = await _uart_cmd_with_retries(home_cmd, timeout_s=2.0, attempts=2, pause_s=0.2)
                    if not ok_home:
                        await _send_uart_response(websocket, "ENABLE", False, f"RECOVERY HOME FAIL: {resp_home}")
                        return
                    _robot_state = "IDLE"
                    await _send_uart_response(websocket, "ENABLE", True, "OK ENABLE + AUTO_HOME")
                    return
                except Exception as ex:
                    await _send_uart_response(websocket, "ENABLE", False, f"RECOVERY HOME EXCEPTION: {ex}")
                    return

        if _is_setpose_hr:
            # SETPOSE_T_HR: angoli virtuali x10 (50..1750). Convertiamo virtual→physical
            # mantenendo precisione decimale, poi rimoltiplichiamo per 10 prima di
            # inoltrare al firmware. Il firmware accetta interi 50..1750.
            try:
                parts = cmd.split()
                # parts: ["SETPOSE_T_HR", B*10, S*10, G*10, Y*10, P*10, R*10, time_ms, PLANNER]
                virtual_x10 = [int(parts[i]) for i in range(1, 7)]
                tail = parts[7:]   # [time_ms, PLANNER]
                current_settings = settings_manager.load()
                offsets = current_settings.get("offsets", settings_manager.DEFAULTS["offsets"])
                dirs    = current_settings.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
                # Conversione virtual_x10 → physical_x10 con float intermedio (0.1° preciso).
                physical_x10 = []
                for i, vx10 in enumerate(virtual_x10):
                    v_real = vx10 / 10.0
                    p_real = (v_real - 90.0) * dirs[i] + offsets[i]
                    # clamp 5.0..175.0 (firmware refuserebbe altrimenti)
                    if   p_real < 5.0:   p_real = 5.0
                    elif p_real > 175.0: p_real = 175.0
                    physical_x10.append(int(round(p_real * 10)))
                uart_cmd = "SETPOSE_T_HR " + " ".join(str(v) for v in physical_x10) + " " + " ".join(tail)
                logger.info(
                    "[WS→UART][HR] virtual_x10=%s -> physical_x10=%s (offs=%s dirs=%s)",
                    virtual_x10, physical_x10, offsets, dirs,
                )
            except Exception as e:
                logger.warning("[WS→UART] conversione SETPOSE_T_HR fallita: %s — cmd=%r", e, cmd)
                uart_cmd = cmd  # fallback
        elif _is_setpose:
            # Converti angoli virtuali → fisici prima di inviare al firmware.
            # Per SETPOSE: il client manda vel in °/s, il firmware vuole percentuale (1-100).
            # Conversione: pct = round((vel_deg_s - VEL_MIN) / (VEL_MAX - VEL_MIN) * 100)
            # con VEL_MIN=1, VEL_MAX=120. SETPOSE_T usa time_ms: nessuna conversione.
            try:
                parts    = cmd.split()
                # SETPOSE: parts = ["SETPOSE", B, S, G, Y, P, R, vel_deg_s, PLANNER]
                # SETPOSE_T: parts = ["SETPOSE_T", B, S, G, Y, P, R, time_ms, PLANNER]
                prefix   = parts[0]                          # "SETPOSE" o "SETPOSE_T"
                # Spazi:
                # - virtual: 0..180 con HOME=90 (UI / output IK)
                # - physical: 0..180 servo reali (UART firmware)
                virtual  = [int(parts[i]) for i in range(1, 7)]
                tail     = parts[7:]                         # [vel_deg_s, PLANNER] oppure [time_ms, PLANNER]
                current_settings = settings_manager.load()
                offsets  = current_settings.get("offsets", settings_manager.DEFAULTS["offsets"])
                dirs     = current_settings.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
                physical = settings_manager.virtual_to_physical(virtual, offsets, dirs)
                if prefix.upper() == "SETPOSE":
                    # Converti vel °/s → percentuale per il firmware
                    vel_deg_s = float(tail[0])
                    vel_pct   = max(1, min(100, round((vel_deg_s - 1.0) / 119.0 * 100)))
                    tail      = [str(vel_pct)] + tail[1:]
                    logger.info("[WS→UART] vel %.1f °/s → %d%%", vel_deg_s, vel_pct)
                uart_cmd = prefix + " " + " ".join(str(v) for v in physical) + " " + " ".join(tail)
                logger.info(
                    "[WS→UART][MAP] %s virtual=%s offsets=%s dirs=%s -> physical=%s",
                    prefix, virtual, offsets, dirs, physical,
                )
            except Exception as e:
                logger.warning("[WS→UART] conversione SETPOSE fallita: %s — cmd=%r", e, cmd)
                uart_cmd = cmd  # fallback: manda as-is
        elif _is_joint_limits:
            # SET_JOINT_LIMITS <idx> <min> <max> — 3 argomenti, passa direttamente
            # Il firmware valida i range; nessun controllo rigido qui.
            parts = cmd.split()
            logger.info("[WS→UART] SET_JOINT_LIMITS: %s", parts[1:])
            uart_cmd = cmd
        elif _is_vr_params:
            # SET_VR_PARAMS — 15 float + 6 int opzionali, passa direttamente
            # Il firmware valida tutti i parametri; nessun controllo rigido qui.
            parts = cmd.split()
            if len(parts) < 2:
                logger.warning("[UART][VALIDATION] SET_VR_PARAMS senza argomenti: %r", cmd)
            logger.info("[WS→UART] SET_VR_PARAMS (%d args): %s", len(parts) - 1, parts[1:])
            uart_cmd = cmd
        elif cmd_upper in ("HOME", "PARK", "TELEOPPOSE"):
            # HOME / PARK / TELEOPPOSE usano pose da settings.json (spazio virtuale).
            # Le traduciamo in SETPOSE con angoli fisici per rispettare gli offset calibrati.
            try:
                current_settings = settings_manager.load()
                offsets  = current_settings.get("offsets", settings_manager.DEFAULTS["offsets"])
                dirs     = current_settings.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
                profile  = current_settings.get("profile", "RTR5")
                if cmd_upper == "HOME":
                    virtual = current_settings.get("home", settings_manager.DEFAULTS["home"])
                elif cmd_upper in ("PARK", "TELEOPPOSE"):
                    key     = "park" if cmd_upper == "PARK" else "vr"
                    virtual = current_settings.get(key, settings_manager.DEFAULTS[key])
                physical  = settings_manager.virtual_to_physical(virtual, offsets, dirs)
                vel_deg_s = int(current_settings.get("vel_max", settings_manager.DEFAULTS["vel_max"]))
                vel_pct   = max(1, min(100, round((vel_deg_s - 1.0) / 119.0 * 100)))
                uart_cmd  = "SETPOSE " + " ".join(str(v) for v in physical) + f" {vel_pct} {profile}"
                logger.info("[WS→UART] %s → SETPOSE virtuale=%s fisico=%s", cmd_upper, virtual, physical)
            except Exception as e:
                logger.warning("[WS→UART] conversione %s fallita: %s — uso comando raw", cmd_upper, e)
                uart_cmd = cmd_upper  # fallback al comando firmware diretto
        else:
            uart_cmd = cmd_upper

        if cmd_upper == "STOP":
            # Ferma la demo orchestrata lato Raspberry se attiva
            if _demo_task and not _demo_task.done():
                _demo_stop_event.set()
                logger.info("[UART CMD] STOP — demo sequence interrotta")
            logger.info("[UART CMD] STOP inviato — interrompe DEMO/SETPOSE")

        # [DIAG] Log del comando prima di entrare in uart_manager
        logger.info("[WS→UART DIAG] cmd_original=%r cmd_upper=%r uart_cmd=%r is_setpose=%s",
                    cmd, cmd_upper, uart_cmd, _is_setpose)
        limits_warning = ""
        pre_clamp_uart_cmd = uart_cmd
        uart_cmd, limits_warning = _clamp_setpose_cmd_if_needed(uart_cmd)
        if pre_clamp_uart_cmd != uart_cmd:
            logger.info("[WS→UART][CLAMP] pre=%r post=%r", pre_clamp_uart_cmd, uart_cmd)
        if limits_warning:
            logger.warning("[JOINT_LIMITS] %s", limits_warning)
        uart_ok = False
        uart_resp = ""
        try:
            uart_ok, uart_resp = await uart_manager.send_uart_command(uart_cmd, timeout_s=1.5)
            ok, response = uart_ok, uart_resp
            # NOTE [HYBRID-H3]: aggiorna stato HYBRID lato RPi SOLO dopo ack robusto firmware.
            global _hybrid_enabled_rpi
            if ok and uart_cmd == "HYBRID ENABLE":
                resp_u = (response or "").upper()
                if resp_u.startswith("OK ") and "HYBRID ENABLED" in resp_u:
                    _hybrid_enabled_rpi = True
            elif ok and uart_cmd == "HYBRID DISABLE":
                resp_u = (response or "").upper()
                if resp_u.startswith("OK ") and "HYBRID DISABLED" in resp_u:
                    _hybrid_enabled_rpi = False
            # [DIAG] Log del risultato da uart_manager
            logger.info("[WS←UART DIAG] cmd=%r ok=%s response=%r", uart_cmd, ok, response)
            if not ok and cmd_upper != "STATUS?":
                logger.warning("UART %s fallito: %s", uart_cmd, response)
            if ok and uart_cmd == "HYBRID ENABLE":
                try:
                    cfg_ok = await apply_persisted_vr_config_to_firmware()
                    logger.info(
                        "[VR_CONFIG][APPLY] dopo HYBRID ENABLE: persisted->firmware ok=%s "
                        "(HEADZERO resta comando separato per sola calib orientamento)",
                        cfg_ok,
                    )
                except Exception as ex:
                    logger.warning("[VR_CONFIG][APPLY] apply dopo HYBRID ENABLE fallita: %s", ex)
            if ok and cmd_upper in ("TELEOPPOSE", "PARK"):
                shared_state.write_feedback_to_file({
                    "teleop_pose_ack": True,
                    "id": int(time.monotonic() * 1000)
                })
                logger.info("[UART] %s eseguito → teleop_pose_ack inviato al browser", cmd_upper)
            if cmd_upper == "TELEOPPOSE":
                # Se siamo in HEAD/HYBRID, dopo il completamento posa (SETPOSE_DONE)
                # azzeriamo il riferimento del loop testa con HEADZERO.
                mode_now = -1
                try:
                    cur = dict(shared_state.latest_intent or {})
                    mode_now = int(cur.get("mode", -1))
                except Exception:
                    mode_now = -1
                if ok and mode_now in (3, 4):
                    _pose_zero_pending = True
                    _pose_zero_mode = mode_now
                    logger.info("[POSE] TELEOPPOSE ok in mode=%d: HEADZERO pianificato su SETPOSE_DONE", mode_now)
                elif not ok:
                    _pose_zero_pending = False
            if cmd_upper == "HOME":
                # HOME → dopo SETPOSE_DONE, rilascia automaticamente PITCH e ROLL
                # (i due servo che scaldano di più) inviando RELAX_DIGITAL al
                # firmware. Nessuna modifica alle altre pose o agli altri giunti.
                global _home_relax_pending
                _home_relax_pending = bool(ok)
                if ok:
                    logger.info("[POSE] HOME ok: RELAX_DIGITAL pianificato su SETPOSE_DONE (PITCH+ROLL)")
            if ok and _is_setpose:
                logger.info("[UART] SETPOSE eseguito: %s", uart_cmd)
        except Exception as e:
            logger.warning("Errore invio comando UART %s: %s", uart_cmd, e)
            uart_ok = False
            uart_resp = str(e)

        resp_cmd = cmd_upper
        if _is_vr_params:
            resp_cmd = "SET_VR_PARAMS"
        elif _is_joint_limits:
            resp_cmd = "SET_JOINT_LIMITS"
        elif _is_setpose:
            resp_cmd = "SETPOSE_T" if uart_cmd.upper().startswith("SETPOSE_T") else "SETPOSE"
        elif cmd_upper in ("HOME", "PARK", "TELEOPPOSE"):
            resp_cmd = cmd_upper
        try:
            await _send_uart_response(
                websocket,
                resp_cmd,
                uart_ok,
                (uart_resp or "")[:500],
                warning=limits_warning,
            )
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Demo sequence
# ---------------------------------------------------------------------------

async def _run_demo_sequence() -> None:
    """
    Esegue la sequenza demo da settings.json in loop infinito.
    Invia SETPOSE uno alla volta e aspetta SETPOSE_DONE prima del passo successivo.
    Si ferma quando _demo_stop_event è settato (STOP ricevuto) o su errore UART.
    [RPi-0.5] Estratto 1:1 da ws_server.py.
    """
    global _demo_task
    logger.info("[DEMO] sequenza avviata lato Raspberry")
    try:
        while not _demo_stop_event.is_set():
            current_settings = settings_manager.load()
            steps   = current_settings.get("demo_steps", settings_manager.DEFAULTS["demo_steps"])
            offsets = current_settings.get("offsets",    settings_manager.DEFAULTS["offsets"])

            for step in steps:
                if _demo_stop_event.is_set():
                    break
                virtual   = step["angles"]
                vel_deg_s = step["vel"]
                profile   = step["profile"]
                physical  = settings_manager.virtual_to_physical(virtual, offsets)
                vel_pct   = max(1, min(100, round((vel_deg_s - 1.0) / 119.0 * 100)))
                uart_cmd  = "SETPOSE " + " ".join(str(v) for v in physical) + f" {vel_pct} {profile}"
                logger.info("[DEMO] step virtuale=%s fisico=%s vel=%s°/s (%d%%) profile=%s",
                            virtual, physical, vel_deg_s, vel_pct, profile)

                ok, response = await uart_manager.send_uart_command(uart_cmd, timeout_s=2.0)
                if not ok or _demo_stop_event.is_set():
                    logger.info("[DEMO] interrotta (ok=%s stop=%s)", ok, _demo_stop_event.is_set())
                    return

                # Attendi SETPOSE_DONE con timeout (durata massima posa = 10 s)
                done = await _wait_setpose_done_async(timeout_s=10.0)
                if not done or _demo_stop_event.is_set():
                    logger.info("[DEMO] step timeout o stop — interrompo")
                    return

    except asyncio.CancelledError:
        logger.info("[DEMO] task cancellato")
    except Exception as e:
        logger.warning("[DEMO] errore inaspettato: %s", e)
    finally:
        _demo_task = None
        logger.info("[DEMO] sequenza terminata")


def _build_demo_hold_current_pose_cmd(duration_ms: int = 120, planner: str = "RTR3") -> str | None:
    """
    Costruisce un SETPOSE_T che congela il robot sulla posa fisica corrente.
    Usato per fermare la DEMO senza entrare in STOPPED.
    """
    try:
        telemetry = shared_state.read_telemetry_from_file() or {}
        keys = ("servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R")
        pose = []
        for key in keys:
            raw = telemetry.get(key)
            if raw is None:
                return None
            pose.append(int(max(5, min(175, round(float(raw))))))
        safe_time_ms = max(20, int(duration_ms))
        safe_planner = planner if planner in ("RTR3", "RTR5", "BB", "BCB") else "RTR3"
        return f"SETPOSE_T {' '.join(str(v) for v in pose)} {safe_time_ms} {safe_planner}"
    except Exception as e:
        logger.warning("[DEMO] impossibile costruire hold current pose: %s", e)
        return None


async def _wait_setpose_done_async(timeout_s: float) -> bool:
    """
    Attende l'evento SETPOSE_DONE dal firmware (max timeout_s secondi).
    Ritorna True se ricevuto, False se timeout.
    """
    global _setpose_done_event
    if _main_loop is None:
        return False
    _setpose_done_event = asyncio.Event()
    try:
        await asyncio.wait_for(_setpose_done_event.wait(), timeout=timeout_s)
        return True
    except asyncio.TimeoutError:
        return False
    finally:
        _setpose_done_event = None


# ---------------------------------------------------------------------------
# Robot state poll loop
#
# NOTA [RPi-0.7]: robot_state_poll_loop scrive _robot_state da asyncio;
# on_uart_unsolicited lo legge (non lo scrive) dal thread UART. L'accesso
# è one-writer / multi-reader su str CPython, quindi safe. Se in futuro
# _robot_state diventa struttura mutabile, aggiungere asyncio.Lock.
# ---------------------------------------------------------------------------

async def robot_state_poll_loop() -> None:
    """
    Interroga lo stato STM32 ogni 2 s via STATUS? e aggiorna _robot_state.
    Il valore viene poi iniettato nel payload di telemetria per i client WS.
    [RPi-0.5] Estratto 1:1 da ws_server.py.
    [RPi-0.7] Vedi nota su concorrenza _robot_state sopra.
    """
    global _robot_state, _last_unknown_recovery_mono, _unknown_recovery_in_progress, _unknown_since_mono
    consecutive_failures = 0
    # Bootstrap sync: alza rapidamente uno stato valido dopo startup servizio,
    # riducendo la finestra in cui la UI vede UNKNOWN.
    await asyncio.sleep(_STATUS_BOOT_DELAY_S)  # attendi boot STM32 prima del primo poll
    for _ in range(4):
        try:
            st = await _read_robot_status_with_retries(attempts=3, timeout_s=0.8, pause_s=0.12)
            if st is not None:
                _robot_state = st
                consecutive_failures = 0
                break
            consecutive_failures += 1
        except Exception:
            consecutive_failures += 1
        await asyncio.sleep(0.35)
    while True:
        try:
            st = await _read_robot_status_with_retries(attempts=2, timeout_s=0.9, pause_s=0.12)
            if st is not None:
                _robot_state = st
                consecutive_failures = 0
                if st != "UNKNOWN":
                    _unknown_since_mono = 0.0
                elif _unknown_since_mono <= 0.0:
                    _unknown_since_mono = time.monotonic()
            else:
                consecutive_failures += 1
        except Exception as e:
            consecutive_failures += 1
            logger.warning("[STATUS] polling exception: %s (keep=%s)", e, _robot_state)

        # Dopo troppi fallimenti consecutivi, dichiara stato sconosciuto.
        if consecutive_failures >= 5:
            _robot_state = "UNKNOWN"
            if _unknown_since_mono <= 0.0:
                _unknown_since_mono = time.monotonic()
            now = time.monotonic()
            can_recover = (now - _last_unknown_recovery_mono) >= _unknown_recovery_cooldown_s
            if can_recover and not _unknown_recovery_in_progress:
                _unknown_recovery_in_progress = True
                _last_unknown_recovery_mono = now
                try:
                    await _attempt_unknown_recovery()
                finally:
                    _unknown_recovery_in_progress = False
            # Ultima ratio: UNKNOWN persistente oltre timeout -> restart processo (rate-limited).
            if (
                _robot_state == "UNKNOWN"
                and _unknown_since_mono > 0.0
                and (time.monotonic() - _unknown_since_mono) >= _unknown_hard_timeout_s
                and not _unknown_recovery_in_progress
            ):
                await _trigger_hard_unknown_recovery_if_allowed("unknown_persist_gt_5s")
        await asyncio.sleep(_STATUS_POLL_PERIOD_S)


async def _attempt_unknown_recovery() -> None:
    """
    Recovery minimo quando il backend entra in UNKNOWN per timeout UART ripetuti.
    Obiettivo: riallineare STM32 in SAFE->IDLE senza restart servizi.
    """
    global _robot_state, _unknown_since_mono
    logger.warning("[STATUS RECOVERY] avvio recovery automatico da UNKNOWN")
    # 1) probe STATUS? con retry: se torna valido, riallinea subito.
    st = await _read_robot_status_with_retries(attempts=3, timeout_s=1.0, pause_s=0.15)
    if st in ("IDLE", "SAFE"):
        _robot_state = st
        _unknown_since_mono = 0.0
        logger.warning("[STATUS RECOVERY] stato riallineato senza azioni motorie: %s", st)
        return

    # 2) Se il link torna disponibile ma siamo in STOPPED/UNKNOWN, porta ASAP in SAFE.
    # Safety-first: niente movimenti automatici in recovery UNKNOWN.
    ok_safe, resp_safe = await _uart_cmd_with_retries("SAFE", timeout_s=1.2, attempts=2, pause_s=0.15)
    if not ok_safe:
        logger.warning("[STATUS RECOVERY] SAFE fallito: %s", resp_safe)
        return
    # 3) verifica finale stato
    st_fin = await _read_robot_status_with_retries(attempts=3, timeout_s=1.0, pause_s=0.15)
    if st_fin is not None:
        _robot_state = st_fin
        if st_fin != "UNKNOWN":
            _unknown_since_mono = 0.0
        logger.warning("[STATUS RECOVERY] completato in safety mode: stato=%s", st_fin)
    else:
        logger.warning("[STATUS RECOVERY] STATUS? finale fallito")


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Configurazione PWM servo runtime (SET_PWM_CONFIG)
# ---------------------------------------------------------------------------

PWM_CONFIG_DEFAULTS: dict = {
    "tim8_hz": 50, "tim8_min_us": 500, "tim8_max_us": 2500, "tim8_max_deg": 180,
    "tim1_hz": 50, "tim1_min_us": 500, "tim1_max_us": 2500, "tim1_max_deg": 180,
}


def load_persisted_pwm_config() -> dict:
    """Legge pwm_config.json; ritorna i default se assente o malformato."""
    try:
        cfg = rcfg.load_runtime_json("pwm_config", default=None)
        if isinstance(cfg, dict):
            merged = dict(PWM_CONFIG_DEFAULTS)
            merged.update({k: int(v) for k, v in cfg.items() if k in PWM_CONFIG_DEFAULTS})
            return merged
    except Exception:
        pass
    return dict(PWM_CONFIG_DEFAULTS)


def save_pwm_config(cfg: dict) -> bool:
    """Persiste pwm_config.json in config_runtime/robot/."""
    try:
        cleaned = {k: int(cfg.get(k, v)) for k, v in PWM_CONFIG_DEFAULTS.items()}
        return rcfg.save_runtime_json("pwm_config", cleaned)
    except Exception as e:
        logger.warning("[PWM_CONFIG] save fallito: %s", e)
        return False


def _build_set_pwm_config_cmd(cfg: dict) -> str:
    """Costruisce la stringa SET_PWM_CONFIG <8 uint> per STM32."""
    def v(key: str) -> int:
        return int(cfg.get(key, PWM_CONFIG_DEFAULTS[key]))
    return (
        f"SET_PWM_CONFIG "
        f"{v('tim8_hz')} {v('tim8_min_us')} {v('tim8_max_us')} {v('tim8_max_deg')} "
        f"{v('tim1_hz')} {v('tim1_min_us')} {v('tim1_max_us')} {v('tim1_max_deg')}"
    )


async def apply_persisted_offsets_to_firmware() -> bool:
    """Invia SET_OFFSETS al firmware con gli offset meccanici da j5_settings.

    Necessario perché il firmware nasce con offset di fabbrica hard-coded
    (servo_control.c: {104,107,77,88,95,104}) che possono differire dai
    valori calibrati persistiti in j5_settings.offsets. Sincronizzare questa
    array fa sì che il pulsante fisico HOME (rt_loop.c → j5vr_center_all_servos
    → usa _servo_offset_deg come target) produca ESATTAMENTE la stessa posa
    raggiunta da WS HOME (che invece traduce virtuale→fisico usando j5_settings).

    Unica fonte di verità: j5_settings.offsets. Entrambe le pipeline HOME la usano.
    """
    try:
        current = settings_manager.load()
        offsets = current.get("offsets", settings_manager.DEFAULTS["offsets"])
        vals = [int(v) for v in offsets]
        if len(vals) != 6:
            logger.warning("[OFFSETS][APPLY] offsets malformati (len=%d): %s", len(vals), vals)
            return False
        cmd = "SET_OFFSETS " + " ".join(str(v) for v in vals)
        logger.info("[OFFSETS][APPLY] %s", cmd)
        ok, response = await uart_manager.send_uart_command(cmd, timeout_s=1.5)
        if ok:
            logger.info("[OFFSETS][APPLY] OK (synced firmware _servo_offset_deg to %s)", vals)
        else:
            logger.warning("[OFFSETS][APPLY] firmware ERR: %s", response)
        return bool(ok)
    except Exception as e:
        logger.warning("[OFFSETS][APPLY] exception: %s", e)
        return False


async def apply_pwm_config_now(cfg: dict | None = None) -> bool:
    """Invia SET_PWM_CONFIG al firmware. Usa config persistita se cfg=None."""
    if cfg is None:
        cfg = load_persisted_pwm_config()
    cmd = _build_set_pwm_config_cmd(cfg)
    logger.info("[PWM_CONFIG][APPLY] %s", cmd)
    try:
        ok, response = await uart_manager.send_uart_command(cmd, timeout_s=1.5)
        if ok:
            logger.info("[PWM_CONFIG][APPLY] OK")
        else:
            logger.warning("[PWM_CONFIG][APPLY] firmware ERR: %s", response)
        return ok
    except Exception as e:
        logger.warning("[PWM_CONFIG][APPLY] exception: %s", e)
        return False


# ---------------------------------------------------------------------------
# Caricamento config IMU-VR da file all'avvio
# ---------------------------------------------------------------------------

def load_persisted_vr_config() -> dict | None:
    """
    Legge routing_config.json senza effetti collaterali UART.

    Questo è il passo di sola lettura/persistenza. L'eventuale apply al firmware
    viene gestito separatamente.
    """
    try:
        cfg = rcfg.load_routing_config_strict()
    except Exception as e:
        logger.warning("[VR_CONFIG][LOAD] guardrail routing_config: %s", e)
        return None
    if not isinstance(cfg, dict):
        logger.warning("[VR_CONFIG][LOAD] routing_config.json non contiene un oggetto JSON")
        return None
    if not cfg.get("pbState") and not cfg.get("tuning"):
        logger.info("[VR_CONFIG][LOAD] routing_config.json vuoto - skip")
        return None
    return cfg


def _build_runtime_vr_config_for_apply(raw_cfg: dict) -> dict:
    """
    Risolve una config persistita parziale in una config runtime completa.

    I default restano centralizzati lato backend; la decisione di applicare
    davvero la config al firmware resta separata.
    """
    return merge_vr_config_with_defaults(raw_cfg)


def _sign_for_src(cfg: dict, src_idx: int) -> int:
    """Per ogni asse visore (0=YAW, 1=PITCH, 2=ROLL) restituisce il segno del servo che lo usa. Default +1."""
    pb = cfg.get("pbState") or {}
    for axis in ("yaw", "pitch", "roll"):
        srv = pb.get(axis, {})
        if srv.get("src") == src_idx:
            return 1 if srv.get("sign", 1) == 1 else -1
    return 1


def _build_set_vr_params_from_config(cfg: dict) -> str:
    """
    Costruisce la stringa SET_VR_PARAMS per STM32.

    La pagina Settings VR salva solo i parametri operativi correnti; il blocco
    legacy del vecchio head-loop resta fissato a default backend per
    compatibilita' con il protocollo UART esistente.
    """
    tuning = cfg.get("tuning") or {}
    pb = cfg.get("pbState") or {}
    pb_en = cfg.get("pbEn") or {}

    def tune(key: str, default: float = 0.0) -> str:
        v = tuning.get(key, VR_TUNE_DEFAULTS.get(key, default))
        if isinstance(v, bool):
            return "1" if v else "0"
        v = float(v) if v is not None else default
        if v == int(v):
            return str(int(v))
        return f"{v:.3f}".rstrip("0").rstrip(".")

    # 1-13: blocco legacy richiesto dallo STM32, ora fissato a default backend.
    legacy_param_ids = [
        "tg-yaw", "tg-pitch", "tg-roll", "tg-alpha-small", "tg-alpha-large", "tg-deadzone",
        "tg-maxstep", "tg-velmax", "tg-veldigital", "tg-lpf-pitch", "tg-lpf-roll", "tg-joy-dz",
        "tg-sensitivity",
    ]
    params = [tune(k, LEGACY_VR_HEAD_LOOP_DEFAULTS[k]) for k in legacy_param_ids]
    # LPF legacy del polso dismesso: firmware usa path uniforme senza filtro speciale P/R.
    # Manteniamo i token in posizione per compatibilità protocollo UART esistente.
    params[9] = "1"
    params[10] = "1"
    # 14-16: sign_yaw, sign_pitch, sign_roll (per asse visore 0,1,2)
    params.append(str(_sign_for_src(cfg, 0)))
    params.append(str(_sign_for_src(cfg, 1)))
    params.append(str(_sign_for_src(cfg, 2)))
    # 17-19: src_roll, src_pitch, src_yaw (quale asse visore guida ogni servo robot)
    params.append(str(pb.get("roll", {}).get("src", 0)))
    params.append(str(pb.get("pitch", {}).get("src", 2)))
    params.append(str(pb.get("yaw", {}).get("src", 1)))
    # 20-22: en_roll, en_pitch, en_yaw
    params.append("1" if pb_en.get("roll", True) else "0")
    params.append("1" if pb_en.get("pitch", True) else "0")
    params.append("1" if pb_en.get("yaw", True) else "0")
    # 23-25: velBase, velSpalla, velGomito
    params.append(tune("tg-vel-base"))
    params.append(tune("tg-vel-spalla"))
    params.append(tune("tg-vel-gomito"))
    # 26-28: velYaw, velPitch, velRoll
    params.append(tune("tg-vel-yaw"))
    params.append(tune("tg-vel-pitch"))
    params.append(tune("tg-vel-roll"))
    # 32-34: velYawHead, velPitchHead, velRollHead
    params.append(tune("tg-vel-yaw-head"))
    params.append(tune("tg-vel-pitch-head"))
    params.append(tune("tg-vel-roll-head"))
    # 35-37: velBaseHead, velSpallaHead, velGomitoHead
    params.append(tune("tg-vel-base-head"))
    params.append(tune("tg-vel-spalla-head"))
    params.append(tune("tg-vel-gomito-head"))

    return "SET_VR_PARAMS " + " ".join(params)


async def apply_vr_config_now() -> bool:
    """
    Carica la configurazione persistita da routing_config.json e la applica
    al firmware via SET_VR_PARAMS e SET_JOINT_LIMITS.

    Ritorna True solo se SET_VR_PARAMS è stato inviato e accettato dallo STM32 (OK in risposta).
    Su fallimento di SET_VR_PARAMS non invia limiti e ritorna False (niente falsi positivi).
    """
    cfg = load_persisted_vr_config()
    if not cfg:
        return False
    cfg = _build_runtime_vr_config_for_apply(cfg)

    cmd_vr = _build_set_vr_params_from_config(cfg)
    parts = cmd_vr.split()
    if len(parts) >= 20:
        logger.info(
            "[VR_CONFIG][APPLY] SET_VR_PARAMS -> STM32 gain_ypr=%s %s %s sign_ypr=%s %s %s src_rpy=%s %s %s",
            parts[1], parts[2], parts[3], parts[14], parts[15], parts[16], parts[17], parts[18], parts[19],
        )
    else:
        logger.info("[VR_CONFIG][APPLY] SET_VR_PARAMS -> STM32 (%d token)", len(parts))

    # G1: retry SET_VR_PARAMS — max 3 tentativi con backoff 1 s.
    # Necessario perché lo STM32 può essere occupato su SETPOSE al boot.
    ok, response = False, "NOT_TRIED"
    for attempt in range(3):
        try:
            ok, response = await uart_manager.send_uart_command(cmd_vr, timeout_s=1.5)
            if ok:
                break
            logger.warning("[VR_CONFIG][APPLY] SET_VR_PARAMS tentativo %d/%d fallito: %s",
                           attempt + 1, 3, response)
        except Exception as e:
            logger.warning("[VR_CONFIG][APPLY] SET_VR_PARAMS tentativo %d/%d eccezione: %s",
                           attempt + 1, 3, e)
        if attempt < 2:
            await asyncio.sleep(1.0)

    if not ok:
        logger.warning("[VR_CONFIG][APPLY] SET_VR_PARAMS fallito dopo 3 tentativi: %s", response)
        return False

    logger.info("[VR_CONFIG][APPLY] SET_VR_PARAMS accettato da STM32 - applico limiti giunto")

    # G2: retry SET_JOINT_LIMITS — 2 tentativi per giunto, warning aggregato finale.
    limits = cfg.get("limits") or {}
    joint_names = ["base", "spalla", "gomito", "yaw", "pitch", "roll"]
    failed_joints = []
    for idx, j in enumerate(joint_names):
        lim = limits.get(j)
        if not lim or "min" not in lim or "max" not in lim:
            continue
        cmd_lim = f"SET_JOINT_LIMITS {idx} {int(lim['min'])} {int(lim['max'])}"
        lok = False
        for attempt in range(2):
            try:
                lok, lr = await uart_manager.send_uart_command(cmd_lim, timeout_s=0.8)
                if lok:
                    logger.debug("[VR_CONFIG][APPLY] SET_JOINT_LIMITS %s ok", j)
                    break
            except Exception as e:
                lr = str(e)
            if attempt == 0:
                await asyncio.sleep(0.3)
        if not lok:
            failed_joints.append(j)
    if failed_joints:
        logger.warning("[VR_CONFIG][APPLY] SET_JOINT_LIMITS falliti (limiti a default firmware): %s",
                       failed_joints)

    return True


async def apply_persisted_vr_config_to_firmware() -> bool:
    """
    Path esplicito "config persistita -> apply runtime firmware".

    Mantiene separati il concetto di file/config salvata sul Raspberry Pi e
    l'effettivo push della configurazione al firmware STM32.
    """
    return await apply_vr_config_now()


async def apply_vr_config_on_intent_head_hybrid_entry(intent_mode: int) -> None:
    """
    Chiamato quando il visore entra in una modalita' XR assistita che usa i
    parametri persistiti Settings VR.

    Oggi copre:
      - mode 2 (HEAD)
      - mode 3 (HYBRID)
      - mode 5 (HEAD ASSIST / overflow polso)

    Throttle per evitare decine di SET_VR_PARAMS al secondo.

    Nota: HEADZERO (UART) resetta solo la calibrazione quaternion / q_offset testa,
    non sostituisce questi parametri di routing/tuning.
    """
    global _last_intent_vr_cfg_apply_mono
    now = time.monotonic()
    if now - _last_intent_vr_cfg_apply_mono < _INTENT_VR_CFG_THROTTLE_S:
        return
    _last_intent_vr_cfg_apply_mono = now
    ok = await apply_persisted_vr_config_to_firmware()
    logger.info(
        "[VR_CONFIG][APPLY] intent assisted XR (mode=%d): persisted->firmware ok=%s "
        "(throttle=%.1fs; HEADZERO e' comando separato per sola calib orientamento)",
        intent_mode,
        ok,
        _INTENT_VR_CFG_THROTTLE_S,
    )


# True dopo il primo apply VR config (via BOOT_READY o fallback).
# Evita il double-apply quando BOOT_READY arriva nei primi 1.5 s.
_boot_config_applied: bool = False


async def apply_vr_config_at_startup() -> None:
    """
    Fallback all'avvio: applica la config VR persistita se il segnale
    BOOT_READY dallo STM32 non è stato ricevuto entro il timeout.

    Il percorso normale è: STM32 invia BOOT_READY via UART →
    on_uart_unsolicited() chiama apply_persisted_vr_config_to_firmware()
    immediatamente e setta _boot_config_applied.
    Questo task copre il caso in cui la porta UART non fosse ancora
    aperta al momento del BOOT_READY (boot simultaneo Pi+STM32).
    """
    global _boot_config_applied
    await asyncio.sleep(1.5)
    if _boot_config_applied:
        logger.debug("[STARTUP] config VR già applicata via BOOT_READY — fallback saltato")
        return
    await apply_persisted_offsets_to_firmware()
    await apply_persisted_vr_config_to_firmware()
    await apply_pwm_config_now()
    _boot_config_applied = True


# ---------------------------------------------------------------------------
# Startup IMUON
# ---------------------------------------------------------------------------

async def startup_imuon() -> None:
    """
    Invia IMUON al boot, indipendente da connessioni WebSocket.
    Retry fino a 10 volte con backoff per tollerare STM32 non ancora pronto.
    [RPi-0.5] Estratto 1:1 da ws_server.py.
    """
    await asyncio.sleep(1.0)  # attendi che UART manager sia inizializzato
    for attempt in range(10):
        try:
            ok, response = await uart_manager.send_uart_command("IMUON", timeout_s=1.0)
            if ok:
                logger.info("[STARTUP] IMUON inviato ok (attempt=%d response=%s)", attempt + 1, response)
                return
            else:
                logger.warning("[STARTUP] IMUON tentativo %d fallito: %s", attempt + 1, response)
        except Exception as e:
            logger.warning("[STARTUP] IMUON errore tentativo %d: %s", attempt + 1, e)
        await asyncio.sleep(2.0)
    logger.error("[STARTUP] IMUON fallito dopo 10 tentativi — IMU potrebbe restare disabilitata")


# ---------------------------------------------------------------------------
# Handler righe UART non-solicitate (es. SETPOSE_DONE)
#
# NOTA [RPi-0.7]: on_uart_unsolicited è chiamato dal thread UART reader
# (non dall'event loop asyncio). Usa _main_loop.call_soon_threadsafe() per
# rientrare nel loop asyncio in modo sicuro. _broadcast_setpose_done non è
# thread-safe di per sé: viene schedulata correttamente via call_soon_threadsafe.
# Non modificare questo pattern senza aggiungere lock o asyncio.run_coroutine_threadsafe.
# -------------------------------------------------------------------------

def on_uart_unsolicited(line: str) -> None:
    """
    Riceve righe UART non-solicitate dal worker thread e le invia ai client WS.
    Chiamato dal worker thread di uart_manager via set_unsolicited_callback.
    [RPi-0.5] Estratto 1:1 da ws_server.py.
    [RPi-0.7] Vedi nota cross-thread sopra.
    """
    if line.startswith("BOOT_READY"):
        logger.info("[UART UNSOLICITED] BOOT_READY ricevuto — applico offsets + config VR + PWM persistita")
        global _boot_config_applied
        _boot_config_applied = True
        if _main_loop is not None:
            async def _apply_all():
                # Offsets first: the firmware physical-button HOME path reads
                # _servo_offset_deg as its target, so this sync makes that path
                # identical to WS HOME. Must run before any user HOME attempt.
                await apply_persisted_offsets_to_firmware()
                await apply_persisted_vr_config_to_firmware()
                await apply_pwm_config_now()
            asyncio.run_coroutine_threadsafe(_apply_all(), _main_loop)
        return

    if not line.startswith("SETPOSE_DONE"):
        logger.info("[UART UNSOLICITED] %r", line[:80])
        return

    # Segnala alla demo sequence che il passo corrente è completato
    if _setpose_done_event is not None and _main_loop is not None:
        _main_loop.call_soon_threadsafe(_setpose_done_event.set)

    # Formato: SETPOSE_DONE time_ms=XXXX vel_max=XX.X acc_max=XXXXX.X
    logger.info("[SETPOSE_DONE] ricevuto: %s", line)
    try:
        parts = {}
        for token in line.split():
            if "=" in token:
                k, v = token.split("=", 1)
                parts[k] = v

        time_ms = int(parts.get("time_ms",  0))
        vel_max = float(parts.get("vel_max", 0.0))
        acc_max = float(parts.get("acc_max", 0.0))

        payload = json.dumps({
            "type":    "setpose_done",
            "time_ms": time_ms,
            "vel_max": round(vel_max, 1),
            "acc_max": round(acc_max, 1),
        })
    except Exception as e:
        logger.warning("[SETPOSE_DONE] parse error: %s — line=%r", e, line)
        return

    if _main_loop is not None and clients:
        asyncio.run_coroutine_threadsafe(_broadcast_setpose_done(payload), _main_loop)

    # Se TELEOPPOSE aveva richiesto l'azzeramento riferimento in HEAD/HYBRID,
    # invia HEADZERO subito dopo il completamento movimento.
    global _pose_zero_pending, _pose_zero_mode
    if _pose_zero_pending and _main_loop is not None:
        mode = _pose_zero_mode
        _pose_zero_pending = False
        asyncio.run_coroutine_threadsafe(_send_headzero_after_pose(mode), _main_loop)

    # Se HOME aveva richiesto il rilascio di PITCH/ROLL, invialo ora.
    # Flag consumata una sola volta, esattamente a fine traiettoria HOME.
    global _home_relax_pending
    if _home_relax_pending and _main_loop is not None:
        _home_relax_pending = False
        asyncio.run_coroutine_threadsafe(_send_relax_digital_after_home(), _main_loop)


async def _broadcast_setpose_done(payload: str) -> None:
    """Invia il messaggio setpose_done a tutti i client WS connessi."""
    if clients:
        await asyncio.gather(*[c.send(payload) for c in list(clients)], return_exceptions=True)
        logger.info("[SETPOSE_DONE] inviato a %d client", len(clients))


async def _send_headzero_after_pose(mode: int) -> None:
    """Invia HEADZERO dopo TELEOPPOSE completato (solo HEAD/HYBRID)."""
    try:
        ok, response = await uart_manager.send_uart_command("HEADZERO", timeout_s=1.0)
        if ok:
            logger.info("[POSE] HEADZERO eseguito (mode=%d): riferimento azzerato", mode)
        else:
            logger.warning("[POSE] HEADZERO fallito (mode=%d): %s", mode, response)
    except Exception as e:
        logger.warning("[POSE] errore invio HEADZERO (mode=%d): %s", mode, e)


async def _send_relax_digital_after_home() -> None:
    """Invia RELAX_DIGITAL dopo SETPOSE_DONE di HOME per rilasciare PITCH/ROLL.

    Evita il surriscaldamento dei due servo più stressati quando il braccio
    viene lasciato a HOME. Gli altri servo restano ingaggiati. La richiesta
    viene emessa una sola volta per ciclo HOME."""
    try:
        ok, response = await uart_manager.send_uart_command("RELAX_DIGITAL", timeout_s=1.0)
        if ok:
            logger.info("[POSE] HOME → RELAX_DIGITAL eseguito: PITCH+ROLL rilasciati")
        else:
            logger.warning("[POSE] HOME → RELAX_DIGITAL fallito: %s", response)
    except Exception as e:
        logger.warning("[POSE] errore invio RELAX_DIGITAL dopo HOME: %s", e)
