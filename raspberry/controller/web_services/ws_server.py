#!/usr/bin/env python3
"""
ws_server.py — WebSocket Teleoperation Server (WS-TELEOP), punto di ingresso.

Porta 8557. Riceve intent JSON dal browser WebXR, valida, converte in
struttura J5VR (senza inviare SPI).

Questo file contiene SOLO:
  - startup del server (main, SSL, task background)
  - handle_client: dispatcher messaggi → moduli ws_handlers_*
  - contatori di log locali (LOG_INTENT_EVERY_N, _intent_log_counter)

Tutta la logica è nei moduli:
  ws_core.py               — costanti, utility
  ws_handlers_intent.py    — intent VR, HEAD mode
  ws_handlers_settings.py  — impostazioni, calibrazione
  ws_handlers_uart.py      — comandi UART, demo, polling stato
  ws_handlers_imu.py       — telemetria IMU, feedback ACK

Nota: eseguire da root (raspberry5 sul Pi) con PYTHONPATH:
  PYTHONPATH=/home/jonny5/raspberry5 python3 controller/web_services/ws_server.py

[RPi-0.5] Ridotto a puro startup + dispatcher.
[RPi-0.6] Import ordinati PEP8 (std → third-party → local); docstring aggiornata.
[RPi-0.7] Accesso a _dashboard_head_active via getter pubblico
          _intent.get_dashboard_head_active() anziché accesso diretto.
[RPi-1.0] Chiamate a init_events() in main() per inizializzare asyncio.Event
          nei moduli handler dentro il loop attivo (no modulo load).
"""

# stdlib
import asyncio
import json
import logging
import os
import ssl
import subprocess
import time

# third-party (per la gestione profilo video)
try:
    import yaml as _yaml
except ImportError:
    _yaml = None

# third-party
try:
    import websockets
except ImportError:
    raise SystemExit("Richiesto: pip install websockets")

# local
from controller.teleop import shared_state
from controller.uart import uart_manager
from controller.web_services import ws_handlers_intent as _intent
from controller.web_services import ws_handlers_imu as _imu
from controller.web_services import ws_handlers_poe as _poe
from controller.web_services import ws_handlers_kinematics as _kinematics
from controller.web_services import ws_handlers_settings as _settings
from controller.web_services import ws_handlers_uart as _uart
from controller.web_services.head_assist import (
    HeadAssistState,
    _quat_to_rpy_deg,
    parse_head_assist_cfg,
    step_mode5_head_assist,
)
from controller.web_services.head_assist_dls import (
    parse_assist_dls_cfg,
    step_dls_head_assist,
)
from controller.web_services import runtime_config_paths as rcfg
from controller.web_services.vr_config_defaults import merge_vr_config_with_defaults
from controller.web_services.ws_core import (
    clients,
)

# ---------------------------------------------------------------------------
# Configurazione
# ---------------------------------------------------------------------------
WS_PORT = 8557

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("ws_teleop")


def _resolve_tls_pair() -> tuple[str, str, str, str]:
    requested_cert = os.path.abspath(
        os.environ.get("WEBRTC_CERT_FILE", rcfg.get_runtime_config_path("tls_cert"))
    )
    requested_key = os.path.abspath(
        os.environ.get("WEBRTC_KEY_FILE", rcfg.get_runtime_config_path("tls_key"))
    )
    resolved_cert = rcfg.resolve_existing_config_path("tls_cert", env_var="WEBRTC_CERT_FILE")
    resolved_key = rcfg.resolve_existing_config_path("tls_key", env_var="WEBRTC_KEY_FILE")
    return requested_cert, requested_key, resolved_cert, resolved_key

# ---------------------------------------------------------------------------
# Stato locale del dispatcher
# ---------------------------------------------------------------------------
# Throttle log intent
LOG_INTENT_EVERY_N  = 20
_intent_log_counter = 0

# Contatore per log diagnostico 4 livelli (quaternioni visore → SPI)
_vr_input_log_count = 0


# ---------------------------------------------------------------------------
# MODE=5 — HEAD OVERFLOW ASSIST (ex IK MODE): B/S/G solo in overflow polso.
# Parametri: routing_config.json → merge_vr_config_with_defaults (headAssist).
# ---------------------------------------------------------------------------
_head_assist_state = HeadAssistState()
_dx_runtime_cfg_cache = {
    "path": rcfg.get_runtime_config_read_path("routing_config"),
    "mtime": None,
    "cfg": merge_vr_config_with_defaults({}),
}
_last_assist_mode_flag: str | None = None


def _extract_servo_physical_deg_from_telemetry() -> list[float] | None:
    telem = shared_state.read_telemetry_from_file()
    if not isinstance(telem, dict):
        return None
    keys = ("servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R")
    if any(k not in telem for k in keys):
        return None
    try:
        return [float(telem[k]) for k in keys]
    except Exception:
        return None


def _attach_mode5_arm_target(intent: dict, physical_angles: list[int] | None, grip_active: bool, hold_active: bool, target_id: int) -> None:
    if physical_angles is None or len(physical_angles) != 3:
        intent["mode5_arm"] = {
            "valid": False,
            "grip_active": bool(grip_active),
            "hold_active": bool(hold_active),
            "target_id": int(target_id) & 0xFFFF,
            "physical_deg": None,
        }
        return
    intent["mode5_arm"] = {
        "valid": True,
        "grip_active": bool(grip_active),
        "hold_active": bool(hold_active),
        "target_id": int(target_id) & 0xFFFF,
        "physical_deg": [int(v) for v in physical_angles[:3]],
    }


def _load_dx_runtime_cfg() -> dict:
    path = rcfg.get_runtime_config_read_path("routing_config")
    cached_path = _dx_runtime_cfg_cache.get("path")
    cached_mtime = _dx_runtime_cfg_cache.get("mtime")
    if path != cached_path:
        _dx_runtime_cfg_cache["path"] = path
        _dx_runtime_cfg_cache["mtime"] = None
        cached_mtime = None
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        mtime = None
    if mtime == cached_mtime and isinstance(_dx_runtime_cfg_cache.get("cfg"), dict):
        return _dx_runtime_cfg_cache["cfg"]
    raw_cfg = rcfg.load_runtime_json("routing_config", default={}) or {}
    merged = merge_vr_config_with_defaults(raw_cfg if isinstance(raw_cfg, dict) else {})
    _dx_runtime_cfg_cache["mtime"] = mtime
    _dx_runtime_cfg_cache["cfg"] = merged
    return merged


def _process_head_assist_mode(intent: dict) -> bool:
    """
    Mode=5 — HEAD OVERFLOW ASSIST:
      deadman classico a doppio grip;
      B/S/G da telemetria + correzione morbida se yaw/pitch/roll polso in warning/critico;
      polso continua sulla pipeline HEAD del firmware.
    """
    if int(intent.get("mode", -1)) != 5:
        return False

    st = _head_assist_state
    cfg = _load_dx_runtime_cfg()
    ha = parse_head_assist_cfg(cfg.get("headAssist") or {})

    if not ha["enabled"]:
        _attach_mode5_arm_target(intent, None, grip_active=False, hold_active=False, target_id=st.target_id)
        st.filt_b = st.filt_s = st.filt_g = None
        st.last_arm_physical = None
        st.last_ts = 0.0
        return True

    now = float(intent.get("timestamp", time.monotonic()))
    grip = int(intent.get("grip", 0)) == 1
    head_quat = None
    head_rpy = None
    try:
        qw = float(intent.get("quat_w", 1.0))
        qx = float(intent.get("quat_x", 0.0))
        qy = float(intent.get("quat_y", 0.0))
        qz = float(intent.get("quat_z", 0.0))
        head_quat = (qw, qx, qy, qz)
        head_rpy = _quat_to_rpy_deg(qw, qx, qy, qz)
    except Exception:
        head_quat = None
        head_rpy = None

    servo_physical = _extract_servo_physical_deg_from_telemetry()
    phys_list: list[float] | None = (
        [float(x) for x in servo_physical] if servo_physical else None
    )

    assist_mode_flag = str(cfg.get("assistMode", "rate")).strip().lower()
    global _last_assist_mode_flag
    if assist_mode_flag != _last_assist_mode_flag:
        logger.info("[HEAD-ASSIST] assistMode transition: %s -> %s",
                    _last_assist_mode_flag, assist_mode_flag)
        _last_assist_mode_flag = assist_mode_flag
    if assist_mode_flag == "dls":
        ha_dls = parse_assist_dls_cfg(cfg.get("assistDls") or {})
        arm, g_active, hold, tid = step_dls_head_assist(
            raw_grip_active=grip,
            physical_six=phys_list,
            head_quat_wxyz=head_quat,
            limits_src=cfg.get("limits") or {},
            ha=ha,
            ha_dls=ha_dls,
            state=st,
            now=now,
        )
    else:
        arm, g_active, hold, tid = step_mode5_head_assist(
            raw_grip_active=grip,
            physical_six=phys_list,
            head_rpy_deg=head_rpy,
            limits_src=cfg.get("limits") or {},
            ha=ha,
            state=st,
            now=now,
        )

    if grip and phys_list is None:
        logger.warning("[HEAD-ASSIST] grip attivo ma telemetria servo assente")
        _attach_mode5_arm_target(intent, arm, grip_active=False, hold_active=hold, target_id=tid)
        return True

    _attach_mode5_arm_target(intent, arm, grip_active=g_active, hold_active=hold, target_id=tid)
    return True


# ---------------------------------------------------------------------------
# Profilo video MediaMTX: switch runtime su richiesta dashboard.
# ---------------------------------------------------------------------------

_VIDEO_PROFILES_ALLOWED = ("lowlatency", "zoomfriendly", "inspection", "maxres", "initial")
_VIDEO_PROFILE_RESTART_GRACE_S = 4.0  # tempo prima del broadcast cameras_refocus


async def _handle_set_video_profile(requester_ws, data: dict) -> None:
    """Cambia il profilo MediaMTX corrente, riavvia il servizio e notifica i client."""
    raw_profile = str(data.get("profile", "")).strip().lower()
    if raw_profile not in _VIDEO_PROFILES_ALLOWED:
        try:
            await requester_ws.send(json.dumps({
                "type": "video_profile_error",
                "profile": raw_profile,
                "reason": "unknown_profile",
                "allowed": list(_VIDEO_PROFILES_ALLOWED),
            }))
        except Exception:
            pass
        return

    # Aggiorno video_pipeline.yaml mantenendo video_pipeline: webrtc.
    try:
        if _yaml is None:
            raise RuntimeError("modulo yaml non disponibile")
        cfg_path = rcfg.get_runtime_config_write_path("video_pipeline")
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                current = _yaml.safe_load(f) or {}
        except FileNotFoundError:
            current = {}
        if not isinstance(current, dict):
            current = {}
        current.setdefault("video_pipeline", "webrtc")
        current["video_profile"] = raw_profile
        rcfg.save_runtime_yaml("video_pipeline", current, mirror_legacy=True)
        logger.info("video_pipeline.yaml aggiornato: video_profile=%s", raw_profile)
    except Exception as e:
        logger.warning("impossibile aggiornare video_pipeline.yaml: %s", e)
        try:
            await requester_ws.send(json.dumps({
                "type": "video_profile_error",
                "profile": raw_profile,
                "reason": "config_write_failed",
                "detail": str(e),
            }))
        except Exception:
            pass
        return

    # Restart jonny5-mediamtx (sudo NOPASSWD configurato per l'utente jonny5).
    try:
        proc = await asyncio.create_subprocess_exec(
            "sudo", "-n", "/bin/systemctl", "restart", "jonny5-mediamtx",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
        )
        await proc.wait()
        if proc.returncode != 0:
            err_b = await proc.stderr.read() if proc.stderr else b""
            logger.warning("systemctl restart jonny5-mediamtx rc=%d err=%s",
                           proc.returncode, err_b.decode("utf-8", "ignore"))
    except Exception as e:
        logger.warning("restart jonny5-mediamtx exception: %s", e)

    # Attendo stabilizzazione MediaMTX (riapertura porte WebRTC/RTSP + encoder).
    await asyncio.sleep(_VIDEO_PROFILE_RESTART_GRACE_S)

    # Conferma al richiedente + broadcast a tutti i client perché riconnettano WHEP.
    confirm = json.dumps({"type": "video_profile_changed", "profile": raw_profile})
    refocus = json.dumps({"type": "cameras_refocus_triggered"})
    for client in list(clients):
        try:
            await client.send(confirm)
            await client.send(refocus)
        except Exception:
            pass
    logger.info("video profile attivato e propagato: %s", raw_profile)


# ---------------------------------------------------------------------------
# Calibrazione IMU world_bias: salva il quaternione di yaw assoluto in HOME.
# ---------------------------------------------------------------------------

async def _handle_set_imu_world_bias(requester_ws, data: dict) -> None:
    """Salva imu_world_bias.json dopo calibrazione dalla dashboard.

    Schema atteso da client:
      type=set_imu_world_bias
      quat_wxyz=[w,x,y,z]  (richiesto, 4 float)
      rpy_deg=[r,p,y]      (opzionale, per metadata)
      samples=N            (opzionale)
      duration_s=...       (opzionale)
      rate_hz_target=...   (opzionale)
      source="..."         (opzionale, libero)

    Effetti:
      - Backup del file precedente con suffisso _bak_<timestamp>.json
      - Scrittura atomica del nuovo file
      - Invalidazione cache in ws_handlers_imu
      - Broadcast `imu_world_bias_updated` a tutti i client
    """
    try:
        quat = data.get("quat_wxyz")
        if not isinstance(quat, (list, tuple)) or len(quat) != 4:
            await requester_ws.send(json.dumps({
                "type": "imu_world_bias_error",
                "reason": "invalid_quat_shape",
            }))
            return
        try:
            quat_f = [float(v) for v in quat]
        except (TypeError, ValueError):
            await requester_ws.send(json.dumps({
                "type": "imu_world_bias_error",
                "reason": "quat_not_numeric",
            }))
            return
        norm2 = sum(v * v for v in quat_f)
        if norm2 <= 1e-9:
            await requester_ws.send(json.dumps({
                "type": "imu_world_bias_error",
                "reason": "quat_zero_norm",
            }))
            return

        # Normalizza
        inv_n = norm2 ** -0.5
        quat_f = [v * inv_n for v in quat_f]

        rpy = data.get("rpy_deg") or [0.0, 0.0, 0.0]
        try:
            rpy_f = [float(v) for v in rpy][:3]
            while len(rpy_f) < 3:
                rpy_f.append(0.0)
        except (TypeError, ValueError):
            rpy_f = [0.0, 0.0, 0.0]

        samples = int(data.get("samples") or 0)
        duration_s = float(data.get("duration_s") or 0.0)
        rate_hz_target = float(data.get("rate_hz_target") or 30.0)
        source = str(data.get("source") or "")

        from datetime import datetime, timezone
        new_cfg = {
            "description": (
                "BNO085 Rotation-Vector world-frame yaw bias (magnetometer-dependent). "
                "Refresh when BNO085 yaw reference drifts. "
                "Validators only; not consumed by operational path."
            ),
            "quat_wxyz": quat_f,
            "rpy_deg": rpy_f,
            "calibrated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S"),
            "samples": samples,
            "duration_s": duration_s,
            "rate_hz_target": rate_hz_target,
        }
        if source:
            new_cfg["source"] = source

        # Backup del precedente con timestamp
        try:
            target_path = rcfg.get_runtime_config_write_path("imu_world_bias")
            if os.path.exists(target_path):
                ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                stem, ext = os.path.splitext(target_path)
                bak = f"{stem}.bak_{ts_str}{ext}"
                import shutil
                shutil.copy2(target_path, bak)
                logger.info("backup imu_world_bias: %s", bak)
        except Exception as e:
            logger.warning("backup imu_world_bias fallito: %s", e)

        ok = rcfg.save_runtime_json("imu_world_bias", new_cfg)
        if not ok:
            await requester_ws.send(json.dumps({
                "type": "imu_world_bias_error",
                "reason": "save_failed",
            }))
            return

        # Invalida la cache nel modulo IMU così la prossima telemetria usa il nuovo bias
        try:
            _imu.invalidate_world_bias_cache()
        except Exception as e:
            logger.warning("invalidate_world_bias_cache fallito: %s", e)

        confirm_msg = json.dumps({
            "type": "imu_world_bias_updated",
            "quat_wxyz": quat_f,
            "rpy_deg": rpy_f,
            "calibrated_at": new_cfg["calibrated_at"],
            "samples": samples,
        })
        for client in list(clients):
            try:
                await client.send(confirm_msg)
            except Exception:
                pass
        logger.info(
            "imu_world_bias aggiornato: yaw=%.3f° samples=%d source=%s",
            rpy_f[2], samples, source or "n/a",
        )
    except Exception as e:
        logger.exception("set_imu_world_bias errore: %s", e)
        try:
            await requester_ws.send(json.dumps({
                "type": "imu_world_bias_error",
                "reason": "exception",
                "detail": str(e),
            }))
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Test MJPEG baseline: misura sperimentale live della pipeline storica (Cap.10).
# ---------------------------------------------------------------------------

_MJPEG_BASELINE_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "tools", "measure_mjpeg_baseline.py",
)
_MJPEG_BASELINE_TIMEOUT_S = 60.0  # supporta anche profili lenti (es. MAX-RES @ 14 FPS / 600 frame)


async def _handle_start_mjpeg_baseline_test(requester_ws, data: dict) -> None:
    """Esegue lo script measure_mjpeg_baseline.py e restituisce risultati.

    Lo script ferma temporaneamente jonny5-mediamtx, acquisisce ~300 frame
    MJPEG via rpicam-vid a 1280x720@30, calcola statistiche inter-frame e
    stima la latenza secondo la metodologia del Cap.10. Lo script riavvia
    sempre jonny5-mediamtx al termine, anche in caso di errore.

    Effetto collaterale: durante il test (~12-15 s) la pipeline video VR
    risulta indisponibile. Il client deve mostrare un avviso preventivo.
    """
    try:
        if not os.path.exists(_MJPEG_BASELINE_SCRIPT):
            await requester_ws.send(json.dumps({
                "type": "mjpeg_baseline_error",
                "reason": "script_missing",
                "path": _MJPEG_BASELINE_SCRIPT,
            }))
            return

        # Esecuzione bloccante in thread executor per non bloccare l'event loop
        try:
            await requester_ws.send(json.dumps({
                "type": "mjpeg_baseline_status",
                "phase": "starting",
                "message": "Stop di MediaMTX e avvio rpicam-vid in modalita' MJPEG...",
            }))
        except Exception:
            pass

        # Parametri opzionali del profilo da testare. Default = baseline storico
        # del Cap.10 (1280x720@30). Valori ammessi: profilo equivalente ai 4
        # profili MediaMTX (lowlatency/zoomfriendly/inspection/maxres) oppure
        # parametri custom (width, height, fps).
        try:
            width   = int(data.get("width", 1280))
            height  = int(data.get("height", 720))
            fps     = int(data.get("fps", 30))
            n_frames = int(data.get("target_frames", 300))
            label   = str(data.get("label", "baseline"))
        except (TypeError, ValueError):
            width, height, fps, n_frames, label = 1280, 720, 30, 300, "baseline"

        # Sanity: limiti ragionevoli
        n_frames = max(50, min(1000, n_frames))

        cmd = ["python3", _MJPEG_BASELINE_SCRIPT,
               "--width", str(width), "--height", str(height),
               "--fps", str(fps), "--target-frames", str(n_frames),
               "--label", label]

        def _run() -> dict:
            proc = subprocess.run(
                cmd,
                capture_output=True, text=True,
                timeout=_MJPEG_BASELINE_TIMEOUT_S,
            )
            if proc.stdout:
                try:
                    return json.loads(proc.stdout)
                except json.JSONDecodeError:
                    return {"status": "parse_error", "stdout": proc.stdout[-1000:], "stderr": proc.stderr[-500:]}
            return {"status": "no_output", "stderr": proc.stderr[-500:] if proc.stderr else ""}

        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, _run)

        await requester_ws.send(json.dumps({
            "type": "mjpeg_baseline_result",
            "result": result,
        }))
        logger.info("mjpeg_baseline test completato (status=%s, n=%s)",
                    result.get("status"), result.get("n_frames"))

    except asyncio.TimeoutError:
        try:
            await requester_ws.send(json.dumps({
                "type": "mjpeg_baseline_error",
                "reason": "timeout",
                "timeout_s": _MJPEG_BASELINE_TIMEOUT_S,
            }))
        except Exception:
            pass
        # Tentativo di ripristino mediamtx in caso di timeout (lo script
        # avrebbe dovuto farlo da solo, ma è bene assicurarsi)
        try:
            subprocess.run(["sudo", "-n", "/bin/systemctl", "start", "jonny5-mediamtx.service"],
                           capture_output=True, timeout=5.0)
        except Exception:
            pass
    except Exception as e:
        logger.exception("mjpeg_baseline test errore: %s", e)
        try:
            await requester_ws.send(json.dumps({
                "type": "mjpeg_baseline_error",
                "reason": "exception",
                "detail": str(e),
            }))
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Campionamento carico computazionale (CPU/RAM/temperatura) per pagina Test.
# Usato in parallelo ai test latenza MediaMTX per misurare il carico per profilo.
# ---------------------------------------------------------------------------

_THERMAL_ZONE_PATH = "/sys/class/thermal/thermal_zone0/temp"
_PROC_STAT_PATH    = "/proc/stat"
_PROC_MEMINFO_PATH = "/proc/meminfo"


def _read_cpu_jiffies() -> tuple[int, int] | None:
    """Ritorna (total_jiffies, idle_jiffies) dalla prima riga di /proc/stat.

    Idle jiffies = idle + iowait. CPU% si calcola come delta su due letture
    spaziate temporalmente: cpu_pct = 100 * (1 - delta_idle / delta_total).
    """
    try:
        with open(_PROC_STAT_PATH, "r") as f:
            line = f.readline()
        parts = line.split()
        # parts[0] == "cpu"; parts[1..] = user nice system idle iowait irq softirq steal guest guest_nice
        if not parts or parts[0] != "cpu":
            return None
        vals = [int(x) for x in parts[1:]]
        if len(vals) < 5:
            return None
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        total = sum(vals)
        return total, idle
    except Exception:
        return None


def _read_ram_pct() -> float | None:
    """RAM utilizzata in % = (MemTotal - MemAvailable) / MemTotal * 100."""
    try:
        mem_total = None
        mem_avail = None
        with open(_PROC_MEMINFO_PATH, "r") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    mem_total = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    mem_avail = int(line.split()[1])
                if mem_total is not None and mem_avail is not None:
                    break
        if mem_total is None or mem_avail is None or mem_total <= 0:
            return None
        return 100.0 * (mem_total - mem_avail) / mem_total
    except Exception:
        return None


def _read_temp_c() -> float | None:
    """Temperatura CPU/SoC in °C dal thermal zone (milli-gradi)."""
    try:
        with open(_THERMAL_ZONE_PATH, "r") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return None


def _aggregate(samples: list[float]) -> dict:
    if not samples:
        return {"mean": None, "min": None, "max": None, "std": None, "n": 0}
    n = len(samples)
    mean = sum(samples) / n
    mn = min(samples)
    mx = max(samples)
    if n > 1:
        var = sum((x - mean) ** 2 for x in samples) / n
        std = var ** 0.5
    else:
        std = 0.0
    return {"mean": mean, "min": mn, "max": mx, "std": std, "n": n}


async def _handle_start_system_load_sampling(requester_ws, data: dict) -> None:
    """Campiona CPU/RAM/temperatura a ~1 Hz per duration_s secondi.

    Schema atteso da client:
      type=start_system_load_sampling
      duration_s=12        (default; clamp [3, 60])
      interval_s=1.0       (default; clamp [0.5, 3.0])
      label="lowlatency"   (opzionale, libero - es. profilo che si sta testando)

    Output messages:
      system_load_status   (phase, message) all'avvio
      system_load_result   (cpu_pct, ram_pct, temp_c, label) al termine
      system_load_error    (reason, detail) in caso di errore
    """
    try:
        duration_s = float(data.get("duration_s", 12.0))
        interval_s = float(data.get("interval_s", 1.0))
        label = str(data.get("label", ""))
    except (TypeError, ValueError):
        duration_s, interval_s, label = 12.0, 1.0, ""
    duration_s = max(3.0, min(60.0, duration_s))
    interval_s = max(0.5, min(3.0, interval_s))

    try:
        await requester_ws.send(json.dumps({
            "type": "system_load_status",
            "phase": "starting",
            "duration_s": duration_s,
            "interval_s": interval_s,
            "label": label,
            "message": f"Campionamento carico ({duration_s:.0f}s, ogni {interval_s:.1f}s)...",
        }))
    except Exception:
        pass

    cpu_samples: list[float] = []
    ram_samples: list[float] = []
    temp_samples: list[float] = []

    t_start = time.monotonic()
    prev = _read_cpu_jiffies()
    await asyncio.sleep(interval_s)

    while time.monotonic() - t_start < duration_s:
        cur = _read_cpu_jiffies()
        if prev is not None and cur is not None:
            d_total = cur[0] - prev[0]
            d_idle  = cur[1] - prev[1]
            if d_total > 0:
                pct = 100.0 * (1.0 - d_idle / d_total)
                cpu_samples.append(max(0.0, min(100.0, pct)))
        prev = cur
        ram = _read_ram_pct()
        if ram is not None:
            ram_samples.append(ram)
        temp = _read_temp_c()
        if temp is not None:
            temp_samples.append(temp)
        await asyncio.sleep(interval_s)

    result = {
        "label": label,
        "duration_s_actual": round(time.monotonic() - t_start, 2),
        "interval_s": interval_s,
        "cpu_pct": _aggregate(cpu_samples),
        "ram_pct": _aggregate(ram_samples),
        "temp_c":  _aggregate(temp_samples),
    }

    try:
        await requester_ws.send(json.dumps({
            "type": "system_load_result",
            "result": result,
        }))
    except Exception:
        pass

    logger.info(
        "system_load result [%s]: CPU mean=%.1f%% (n=%d) RAM mean=%.1f%% temp mean=%.1f°C",
        label or "n/a",
        result["cpu_pct"]["mean"] or 0.0, result["cpu_pct"]["n"],
        result["ram_pct"]["mean"] or 0.0,
        result["temp_c"]["mean"] or 0.0,
    )


# ---------------------------------------------------------------------------
# handle_client — dispatcher principale
# ---------------------------------------------------------------------------

async def handle_client(websocket, path=None):  # path è opzionale nelle nuove versioni di websockets
    remote = getattr(websocket, "remote_address", "?") or "?"
    logger.info("Client connesso: %s (path: %s)", remote, path or "/")
    clients.add(websocket)
    try:
        # Log informazioni sulla connessione SSL se disponibili
        if hasattr(websocket, "secure") and websocket.secure:
            logger.info("Connessione WSS (SSL/TLS) da %s", remote)
        else:
            logger.info("Connessione WS (non crittografata) da %s", remote)

        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError as e:
                logger.warning("JSON non valido: %s", e)
                continue

            # Comando controllo IMU (UART): non è un intent
            if isinstance(data, dict) and data.get("type") == "set_imu":
                await _uart.handle_set_imu(websocket, data)
                continue

            # Cambio modalità VR dalla dashboard (set_vr_mode)
            if isinstance(data, dict) and data.get("type") == "set_vr_mode":
                await _intent.handle_set_vr_mode(websocket, data)
                continue

            # Lettura impostazioni
            if isinstance(data, dict) and data.get("type") == "get_settings":
                await _settings.handle_get_settings(websocket)
                continue

            # Salvataggio impostazioni
            if isinstance(data, dict) and data.get("type") == "save_settings":
                await _settings.handle_save_settings(websocket, data)
                continue

            # Parametri POE — persistenza lato Raspberry (source of truth)
            if isinstance(data, dict) and data.get("type") == "get_poe_params":
                await _poe.handle_get_poe_params(websocket)
                continue

            if isinstance(data, dict) and data.get("type") == "set_poe_params":
                await _poe.handle_set_poe_params(websocket, data)
                continue

            # FK POE (dashboard): stesso modello della IK runtime
            if isinstance(data, dict) and data.get("type") == "compute_fk_poe":
                await _kinematics.handle_compute_fk_poe(websocket, data)
                continue

            # Calibrazione stereo: broadcast a tutti i client connessi (visore + dashboard)
            if isinstance(data, dict) and data.get("type") == "vr_calib":
                await _settings.handle_vr_calib(websocket, data)
                continue

            # Stato zoom VR: il viewer XR notifica il backend a ~10Hz quando l'utente
            # cambia zoom. Il valore viene incluso nei payload di telemetria successivi
            # (campi vr_zoom0 / vr_zoom1) per la visualizzazione su dashboard.
            if isinstance(data, dict) and data.get("type") == "vr_zoom_state":
                try:
                    z0 = data.get("zoom0")
                    z1 = data.get("zoom1")
                    _imu.set_vr_zoom_state(z0, z1)
                except Exception:
                    pass
                continue

            # vr_zoom_command: comando di zoom (delta/set/reset) inviato dalle
            # dashboard. Viene rilanciato a tutti gli altri client: il viewer XR
            # (se connesso) lo applica via changeZoom() e re-emette vr_zoom_state;
            # le altre dashboard lo applicano localmente via CSS scale.
            if isinstance(data, dict) and data.get("type") == "vr_zoom_command":
                try:
                    msg_out = json.dumps({
                        "type": "vr_zoom_command",
                        "action": str(data.get("action", "delta")),
                        "value": float(data.get("value", 0.0)),
                    })
                    for client in list(_ws_core.clients):
                        if client is websocket:
                            continue
                        try:
                            await client.send(msg_out)
                        except Exception:
                            pass
                except Exception:
                    pass
                continue

            # set_video_profile: richiesto da dashboard (settings o vr-live).
            # Aggiorna video_profile in video_pipeline.yaml, riavvia il servizio
            # jonny5-mediamtx in background e propaga cameras_refocus_triggered
            # a tutti i client per indurli a riconnettere WHEP.
            if isinstance(data, dict) and data.get("type") == "set_video_profile":
                asyncio.create_task(_handle_set_video_profile(websocket, data))
                continue

            # set_imu_world_bias: calibrazione IMU dalla dashboard home (bottone
            # "Calibra IMU"). Salva imu_world_bias.json, invalida la cache di
            # ws_handlers_imu, broadcasta imu_world_bias_updated ai client.
            if isinstance(data, dict) and data.get("type") == "set_imu_world_bias":
                asyncio.create_task(_handle_set_imu_world_bias(websocket, data))
                continue

            # start_mjpeg_baseline_test: pagina Test della dashboard. Esegue uno
            # script Python (tools/measure_mjpeg_baseline.py) che riproduce sul
            # Pi la misura della pipeline MJPEG baseline del Cap.10 e restituisce
            # le statistiche al client. Il test ferma temporaneamente
            # jonny5-mediamtx per liberare la camera, lo riavvia al termine.
            if isinstance(data, dict) and data.get("type") == "start_mjpeg_baseline_test":
                asyncio.create_task(_handle_start_mjpeg_baseline_test(websocket, data))
                continue

            # start_system_load_sampling: pagina Test della dashboard. Campiona
            # CPU/RAM/temperatura a ~1 Hz per duration_s secondi e restituisce
            # un singolo system_load_result al termine. Usato in parallelo ai
            # test latenza MediaMTX per misurare il carico computazionale per
            # profilo (alimenta la colonna "CPU%" della tabella comparativa).
            if isinstance(data, dict) and data.get("type") == "start_system_load_sampling":
                asyncio.create_task(_handle_start_system_load_sampling(websocket, data))
                continue

            # ws_ping: echo immediato per misura round-trip WebSocket reale dalla
            # pagina Test. Riproduce la metodologia di Tab.10.3 della tesi
            # (latenza di controllo = WS round-trip). Il client invia ts_client
            # (timestamp performance.now() del browser) + id univoco; il server
            # risponde con ws_pong contenente gli stessi campi (echo). Il client
            # calcola RTT = performance.now() - ts_client (matching su id).
            if isinstance(data, dict) and data.get("type") == "ws_ping":
                try:
                    await websocket.send(json.dumps({
                        "type": "ws_pong",
                        "ts_client": data.get("ts_client"),
                        "id": data.get("id"),
                        "ts_server": time.time(),
                    }))
                except Exception:
                    pass
                continue

            # vr_session_refresh / vr_session_state: il viewer XR pubblica
            # session.frameRate dell'HMD (Quest 1=72, Quest 2=90/120 Hz nativi)
            # ogni 2 s mentre la sessione XR è attiva. Broadcast a tutti gli
            # altri client (dashboard pagina Test) per visualizzazione live
            # del refresh HMD reale, distinto dal monitor del PC che apre la
            # pagina dashboard. Usato per validare la spec "60-72 Hz visore"
            # del Cap.6/Cap.10 della tesi.
            if isinstance(data, dict) and data.get("type") in ("vr_session_refresh", "vr_session_state", "vr_video_latency"):
                try:
                    msg_out = json.dumps(data)
                    for client in list(clients):
                        if client is websocket:
                            continue
                        try:
                            await client.send(msg_out)
                        except Exception:
                            pass
                except Exception:
                    pass
                continue

            # Mappatura controller aggiornata dalla dashboard /controllers:
            # broadcast live a tutti gli altri client (viewer XR + altre dashboard).
            if isinstance(data, dict) and data.get("type") == "controller_mappings_updated":
                await _settings.handle_controller_mappings_updated(websocket, data)
                continue

            # Trigger refocus camere: il viewer XR notifica gli altri client
            # cosi' che possano riconnettere i propri stream WebRTC dopo il
            # restart MediaMTX (~3s delay tipico).
            if isinstance(data, dict) and data.get("type") == "cameras_refocus_triggered":
                await _settings.handle_cameras_refocus_triggered(websocket, data)
                continue

            # Applicazione offset meccanici al firmware via SET_OFFSETS UART
            if isinstance(data, dict) and data.get("type") == "apply_offsets":
                await _settings.handle_apply_offsets(websocket, data)
                continue

            # Restituisce la configurazione PWM persistita.
            if isinstance(data, dict) and data.get("type") == "get_pwm_config":
                cfg = _uart.load_persisted_pwm_config()
                try:
                    await websocket.send(json.dumps({"type": "pwm_config", "config": cfg}))
                except Exception:
                    pass
                continue

            # Salva e applica configurazione PWM servo al firmware.
            if isinstance(data, dict) and data.get("type") == "save_pwm_config":
                cfg = data.get("config", {})
                saved = _uart.save_pwm_config(cfg)
                ok = await _uart.apply_pwm_config_now(cfg)
                try:
                    await websocket.send(json.dumps({"type": "pwm_config_applied", "ok": ok, "saved": saved}))
                except Exception:
                    pass
                continue

            # Applica al firmware la config IMU-VR persistita su routing_config.json.
            if isinstance(data, dict) and data.get("type") == "apply_saved_vr_config":
                ok = await _uart.apply_persisted_vr_config_to_firmware()
                logger.info("[VR_CONFIG][APPLY] apply_saved_vr_config richiesto da client -> ok=%s", ok)
                try:
                    await websocket.send(json.dumps({"type": "vr_config_applied", "ok": ok}))
                except Exception:
                    pass
                continue

            # Comandi UART ENABLE / STOP / STATUS? / SAFE / RESET + HOME / PARK / TELEOPPOSE / SETPOSE
            if isinstance(data, dict) and data.get("type") == "uart":
                await _uart.handle_uart_cmd(websocket, data)
                continue

            if isinstance(data, dict) and data.get("type") == "self_test":
                await _uart.handle_self_test(websocket, data)
                continue

            # --- Intent VR ---
            global _intent_log_counter
            _intent_log_counter += 1
            if _intent_log_counter % LOG_INTENT_EVERY_N == 0:
                logger.info(
                    "Intent ricevuto: mode=%s joy_x=%s joy_y=%s pitch=%s yaw=%s intensity=%s buttons_L=0x%04x buttons_R=0x%04x camctrl=%s",
                    data.get("mode"),
                    data.get("joy_x"),
                    data.get("joy_y"),
                    data.get("pitch"),
                    data.get("yaw"),
                    data.get("intensity"),
                    int(data.get("buttons_left",  0) or 0) & 0xFFFF,
                    int(data.get("buttons_right", 0) or 0) & 0xFFFF,
                    data.get("camctrl"),
                )

            intent, err = _intent.validate_and_build_intent(data)
            if err:
                logger.warning("Validazione fallita: %s", err)
                continue

            # Mode=5: HEAD OVERFLOW ASSIST (config: routing_config headAssist).
            if int(intent.get("mode", -1)) == 5:
                _process_head_assist_mode(intent)
                # Mantieni comunque latest_intent aggiornato per telemetria/debug.
                with shared_state.lock:
                    shared_state.latest_intent = intent
                shared_state.write_intent_to_file(intent)
                continue

            # Log minimale throttled: include mode + heartbeat + camctrl (se presente)
            if _intent_log_counter % LOG_INTENT_EVERY_N == 0:
                if intent.get("camctrl") is not None:
                    logger.info("Intent validato: mode=%d hb=%d camctrl=%s",
                                intent["mode"], intent.get("heartbeat", 0), intent["camctrl_payload"])
                else:
                    logger.info("Intent validato: mode=%d hb=%d",
                                intent["mode"], intent.get("heartbeat", 0))

            # Se HEAD mode dashboard è attivo: preserva mode=3, grip e buttons dal task heartbeat,
            # ma aggiorna i quaternioni del visore (fondamentali per il closed loop HEAD).
            if _intent.get_dashboard_head_active() and int(intent.get("mode", -1)) in (3, 4, 5):
                # HEAD "sticky": non interrompere il loop su frame intent transitori
                # (race comune: un frame mode=2 subito dopo set_vr_mode=3).
                # L'uscita da HEAD deve avvenire in modo esplicito con set_vr_mode.
                intent["mode"]          = 3
                intent["grip"]          = 1
                intent["buttons_left"]  = 0x0002
                intent["buttons_right"] = 0x0002

            old_mode = -1
            with shared_state.lock:
                prev = shared_state.latest_intent
                if isinstance(prev, dict):
                    try:
                        old_mode = int(prev.get("mode", -1))
                    except (TypeError, ValueError):
                        old_mode = -1
                shared_state.latest_intent = intent
            shared_state.write_intent_to_file(intent)

            mode_i = int(intent.get("mode", -1))
            # Se arrivano intent MANUAL espliciti, interrompi eventuale heartbeat HEAD
            # che altrimenti riscrive buttons_left=0x0002 e contamina il controllo ROLL.
            if mode_i == 2:
                try:
                    _intent.stop_head_mode("auto-stop on manual intent")
                except Exception:
                    pass
            if mode_i in (3, 4, 5) and mode_i != old_mode:
                asyncio.create_task(_uart.apply_vr_config_on_intent_head_hybrid_entry(mode_i))

            # Log diagnostico 4 livelli: quaternione ricevuto dal visore (solo mode=3, ogni 5° messaggio)
            if intent.get("mode") == 3:
                global _vr_input_log_count
                _vr_input_log_count += 1
                if _vr_input_log_count % 5 == 0:
                    logger.info(
                        "[VR-INPUT] qvis=(%.3f %.3f %.3f %.3f)",
                        intent["quat_w"], intent["quat_x"], intent["quat_y"], intent["quat_z"],
                    )

    except websockets.exceptions.ConnectionClosed as e:
        logger.info("Client disconnesso: %s (code: %s, reason: %s)", remote, e.code, e.reason)
        raise
    except websockets.exceptions.InvalidMessage as e:
        logger.warning("Messaggio non valido da %s: %s", remote, e)
    except Exception as e:
        logger.exception("Errore gestione client %s: %s", remote, e)
    finally:
        try:
            clients.discard(websocket)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

async def main():
    # [RPi-1.0] Inizializza asyncio.Event nei moduli handler dentro il loop attivo.
    # Deve precedere qualsiasi uso degli Event (task, callback, coroutine).
    _intent.init_events()
    _uart.init_events()

    # Registra il loop asyncio nel modulo UART (per callback threadsafe)
    _uart.set_main_loop(asyncio.get_running_loop())

    # Registra callback per righe non-solicitate (SETPOSE_DONE)
    uart_manager.set_unsolicited_callback(_uart.on_uart_unsolicited)

    # Certificati: prefer runtime config, con fallback legacy durante la transizione.
    requested_cert, requested_key, cert_file, key_file = _resolve_tls_pair()
    ssl_ctx   = None
    if os.path.exists(cert_file) and os.path.exists(key_file):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
        if cert_file != requested_cert or key_file != requested_key:
            logger.warning(
                "TLS fallback attivo: richiesto=(%s, %s) risolto=(%s, %s)",
                requested_cert,
                requested_key,
                cert_file,
                key_file,
            )
        logger.info("SSL/WSS abilitato (cert: %s)", cert_file)
    else:
        logger.warning(
            "Certificati SSL non trovati (%s, %s) - solo WS supportato",
            requested_cert,
            requested_key,
        )

    proto = "wss" if ssl_ctx else "ws"
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT, ping_interval=20, ping_timeout=10, ssl=ssl_ctx):
        logger.info("WS-TELEOP in ascolto su %s://0.0.0.0:%s (nessun invio SPI)", proto, WS_PORT)
        asyncio.create_task(_imu.feedback_loop())
        asyncio.create_task(_imu.imu_debug_loop(_uart.get_robot_state))
        asyncio.create_task(_uart.startup_imuon())
        asyncio.create_task(_uart.robot_state_poll_loop())
        asyncio.create_task(_uart.apply_vr_config_at_startup())
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
