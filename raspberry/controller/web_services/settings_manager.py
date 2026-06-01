"""
NOTE [RPI-SAFE-REFACTOR-PHASE2]
- Modulo analizzato, nessuna modifica funzionale.
- Responsabilità documentate come CORE / UTILITY / DIAGNOSTIC / LEGACY.

settings_manager.py — Gestione persistenza impostazioni JONNY5-4.0

Legge e scrive settings.json sul filesystem del Pi.

## Spazio angolare virtuale

Tutta la UI e settings.json lavorano in "gradi virtuali" dove 90 = HOME meccanica
di ogni giunto, indipendentemente dall'offset fisico reale.

Formula di conversione:
  fisico[i]  = offset[i] + (virtuale[i] - 90)
  virtuale[i] = fisico[i] - offset[i] + 90

Esempio con offset BASE = 104°:
  virtuale=90  → fisico=104  (HOME)
  virtuale=100 → fisico=114  (+10° dal centro)
  virtuale=80  → fisico=94   (-10° dal centro)

Gli offset meccanici (campo "offsets") restano sempre in gradi fisici reali.
Il firmware STM32 riceve sempre gradi fisici; la conversione da spazio virtuale
avviene nel layer Raspberry WebSocket/HTTP prima dell'invio dei comandi UART.

I valori di default fisici del firmware (servo_control.c):
  servo_offset_deg = {104, 107, 77, 88, 95, 104}   (HOME)
  servo_teleop_deg = {104, 107, 124, 88, 126, 104}  (PARK / VR)

La lettura runtime usa il path ufficiale sotto config_runtime.
Eventuali env legacy non influenzano più il path attivo.
"""

import json
import logging
import os
import threading
from controller.web_services import runtime_config_paths as rcfg

logger = logging.getLogger("settings_manager")

# Valori di default del firmware (servo_control.c) — usati per calcolare i default virtuali
_OFFSETS_FACTORY = [104, 107, 77, 88, 95, 104]   # servo_offset_deg
_TELEOP_FACTORY  = [104, 107, 124, 88, 126, 104]  # servo_teleop_deg

# Valori di default in gradi VIRTUALI (90 = HOME meccanica di ogni giunto).
# home è sempre [90,90,90,90,90,90] per definizione.
# park/vr sono calcolati dalla formula: virtuale = fisico - offset + 90
DEFAULTS: dict = {
    # Offset meccanici (gradi fisici reali — servo_offset_deg nel firmware)
    "offsets": list(_OFFSETS_FACTORY),

    # Verso di rotazione per ogni giunto (+1 = normale, -1 = invertito)
    "dirs": [1, 1, 1, 1, 1, 1],

    # Pose predefinite in gradi VIRTUALI (90 = HOME)
    "home": [90, 90, 90, 90, 90, 90],
    "park": [_TELEOP_FACTORY[i] - _OFFSETS_FACTORY[i] + 90 for i in range(6)],
    "vr":   [_TELEOP_FACTORY[i] - _OFFSETS_FACTORY[i] + 90 for i in range(6)],

    # Parametri di moto
    "vel_max": 80,      # °/s — default; il firmware accetta fino a 120 °/s
    "profile": "RTR5",  # RTR3 | RTR5 | BB | BCB

    # Sequenza DEMO — lista di step, ognuno con angoli virtuali, vel e profilo.
    # Eseguita lato Raspberry: il server invia SETPOSE uno alla volta
    # aspettando SETPOSE_DONE prima di procedere al passo successivo.
    # Angoli in spazio virtuale (90 = HOME meccanica).
    "demo_steps": [
        {"angles": [90, 90, 90, 90, 90, 90],   "vel": 40, "profile": "RTR5"},  # HOME
        {"angles": [90, 123, 143, 90, 90, 90],  "vel": 28, "profile": "RTR3"},  # Spalla alta
        {"angles": [46, 103, 123, 71, 95, 76],  "vel": 42, "profile": "BCB"},   # Base sx
        {"angles": [90, 53,  63, 90, 75, 90],   "vel": 30, "profile": "RTR5"},  # Posa bassa
        {"angles": [136, 103, 123, 107, 85, 106],"vel": 45, "profile": "BCB"},  # Base dx
        {"angles": [90, 113, 168, 90, 90, 90],  "vel": 28, "profile": "RTR3"},  # Estesa frontale
        {"angles": [121, 78, 103, 112, 80, 66], "vel": 58, "profile": "RTR5"},  # Gesto rapido dx
        {"angles": [56,  78, 103, 65,  80, 116],"vel": 58, "profile": "RTR5"},  # Gesto rapido sx
        {"angles": [90, 90, 90, 90, 90, 90],    "vel": 40, "profile": "RTR5"},  # HOME (loop)
    ],

    # Info sistema (readonly in UI)
    "ws_port": 8557,
}

_LABELS = ["B", "S", "G", "Y", "P", "R"]
_PROFILES = ("RTR3", "RTR5", "BB", "BCB")

_lock = threading.Lock()
_warned_legacy_env_override = False
_cache_path: str | None = None
_cache_mtime: float | None = None
_cache_data: dict | None = None


# ---------------------------------------------------------------------------
# Conversione spazio angolare virtuale ↔ fisico
# ---------------------------------------------------------------------------

def virtual_to_physical(virtual_angles: list, offsets: list, dirs: list | None = None) -> list:
    """
    Converte 6 angoli virtuali (90=HOME) in angoli fisici (0-180).
    Formula con verso di rotazione dir[i] ∈ {-1, +1}:
        fisico[i] = offset[i] + dir[i] * (virtuale[i] - 90)
    Il risultato è clampato in [5, 175] (range sicuro firmware).
    """
    if dirs is None:
        dirs = [1] * 6
    out: list[int] = []
    for i in range(6):
        dir_i = dirs[i] if i < len(dirs) else 1
        dir_i = -1 if dir_i < 0 else 1
        val = offsets[i] + dir_i * (virtual_angles[i] - 90)
        out.append(int(max(5, min(175, val))))
    return out


# NOTE [RPi-0.2]:
# Questa funzione non risulta usata nel codebase Python (vedi REPORT RPi-0.1),
# ma potrebbe essere richiamata da client esterni custom.
# Per ora viene mantenuta come legacy; candidata a rimozione in uno step futuro
# dopo test end-to-end della dashboard.
def physical_to_virtual(physical_angles: list, offsets: list, dirs: list | None = None) -> list:
    """
    Inversa: angoli fisici (0-180) → virtuali (90=HOME).
    Formula inversa:
        virtuale[i] = (fisico[i] - offset[i]) / dir[i] + 90
    """
    if dirs is None:
        dirs = [1] * 6
    out: list[float] = []
    for i in range(6):
        dir_i = dirs[i] if i < len(dirs) else 1
        dir_i = -1 if dir_i < 0 else 1
        if dir_i == 0:
            dir_i = 1
        out.append((physical_angles[i] - offsets[i]) / dir_i + 90)
    return out


def load() -> dict:
    """Carica settings dal path runtime ufficiale (fallback legacy disabilitato)."""
    path = rcfg.get_runtime_config_read_path("j5_settings")
    global _warned_legacy_env_override, _cache_path, _cache_mtime, _cache_data
    env_name = rcfg.get_runtime_only_legacy_env_var("j5_settings")
    if env_name and os.environ.get(env_name) and not _warned_legacy_env_override:
        logger.warning("[settings] %s è impostata ma ignorata (runtime-only read attivo)", env_name)
        _warned_legacy_env_override = True
    with _lock:
        if not os.path.exists(path):
            raise RuntimeError(f"[settings] runtime config mancante: {path} (fallback legacy disabilitato)")
        try:
            mtime = os.path.getmtime(path)
            if path == _cache_path and _cache_mtime == mtime and isinstance(_cache_data, dict):
                return dict(_cache_data)
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            merged = dict(DEFAULTS)
            merged.update(data)
            _cache_path = path
            _cache_mtime = mtime
            _cache_data = dict(merged)
            logger.debug("[settings] caricato da %s", path)
            return dict(merged)
        except Exception as e:
            raise RuntimeError(f"[settings] errore lettura runtime config {path}: {e}") from e


def save(settings: dict) -> bool:
    """Valida e salva settings.json. Ritorna True se OK, False se validazione o I/O fallisce."""
    global _cache_path, _cache_mtime, _cache_data
    validated, err = validate(settings)
    if err:
        logger.warning("[settings] validazione fallita: %s", err)
        return False
    with _lock:
        try:
            ok = rcfg.save_runtime_json("j5_settings", validated, mirror_legacy=False)
            if ok:
                _cache_path = None
                _cache_mtime = None
                _cache_data = None
                logger.info("[settings] salvato in %s", rcfg.get_runtime_config_write_path("j5_settings"))
                return True
            logger.warning("[settings] errore scrittura %s", rcfg.get_runtime_config_write_path("j5_settings"))
            return False
        except Exception as e:
            logger.warning("[settings] errore scrittura %s: %s", rcfg.get_runtime_config_write_path("j5_settings"), e)
            return False


def validate(data: dict) -> tuple[dict, str | None]:
    """Valida i campi di settings. Ritorna (dict_validato, errore_stringa|None)."""
    out = dict(DEFAULTS)

    for key in ("offsets", "home", "park", "vr"):
        raw = data.get(key)
        if raw is not None:
            if not isinstance(raw, list) or len(raw) != 6:
                return {}, f"'{key}' deve essere una lista di 6 valori interi"
            try:
                vals = [int(v) for v in raw]
            except (TypeError, ValueError):
                return {}, f"'{key}' contiene valori non interi"
            if not all(0 <= v <= 180 for v in vals):
                return {}, f"'{key}' contiene valori fuori range [0, 180]"
            out[key] = vals

    dirs = data.get("dirs")
    if dirs is not None:
        if not isinstance(dirs, list) or len(dirs) != 6:
            return {}, "'dirs' deve essere una lista di 6 valori interi (-1 o 1)"
        try:
            vals = [int(v) for v in dirs]
        except (TypeError, ValueError):
            return {}, "'dirs' contiene valori non interi"
        if not all(v in (-1, 1) for v in vals):
            return {}, "'dirs' deve contenere solo -1 o 1"
        out["dirs"] = vals

    vel = data.get("vel_max")
    if vel is not None:
        try:
            vel = int(vel)
        except (TypeError, ValueError):
            return {}, "'vel_max' deve essere un intero"
        if not (1 <= vel <= 120):
            return {}, "'vel_max' deve essere in [1, 120]"
        out["vel_max"] = vel

    profile = data.get("profile")
    if profile is not None:
        if profile not in _PROFILES:
            return {}, f"'profile' deve essere uno di {_PROFILES}"
        out["profile"] = profile

    demo_steps = data.get("demo_steps")
    if demo_steps is not None:
        if not isinstance(demo_steps, list) or len(demo_steps) == 0:
            return {}, "'demo_steps' deve essere una lista non vuota"
        validated_steps = []
        for i, step in enumerate(demo_steps):
            if not isinstance(step, dict):
                return {}, f"demo_steps[{i}] deve essere un oggetto"
            angles = step.get("angles")
            if not isinstance(angles, list) or len(angles) != 6:
                return {}, f"demo_steps[{i}].angles deve essere una lista di 6 interi"
            try:
                angles = [int(v) for v in angles]
            except (TypeError, ValueError):
                return {}, f"demo_steps[{i}].angles contiene valori non interi"
            if not all(0 <= v <= 180 for v in angles):
                return {}, f"demo_steps[{i}].angles contiene valori fuori range [0, 180]"
            try:
                vel = int(step.get("vel", 40))
            except (TypeError, ValueError):
                return {}, f"demo_steps[{i}].vel deve essere un intero"
            if not (1 <= vel <= 120):
                return {}, f"demo_steps[{i}].vel deve essere in [1, 120]"
            prof = step.get("profile", "RTR5")
            if prof not in _PROFILES:
                return {}, f"demo_steps[{i}].profile deve essere uno di {_PROFILES}"
            validated_steps.append({"angles": angles, "vel": vel, "profile": prof})
        out["demo_steps"] = validated_steps

    # ws_port è readonly — non accettiamo modifiche dal client
    out["ws_port"] = DEFAULTS["ws_port"]

    return out, None
