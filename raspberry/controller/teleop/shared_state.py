"""
NOTE [RPI-SAFE-REFACTOR-PHASE1]
- Modulo analizzato, nessuna modifica funzionale.
- Marcatura delle funzioni CORE / UTILITY / DIAGNOSTIC / LEGACY.
- Obiettivo: documentazione interna per futura FASE 2.

Stato condiviso tra WS-TELEOP e SPI J5VR TX.
- In-process: latest_intent + lock (threading).
- Cross-process: file con lock (fcntl) per due processi separati.
"""

import json
import os
import threading

# fcntl è disponibile solo su Unix/Linux (non su Windows)
if os.name != "nt":
    import fcntl
else:
    fcntl = None  # type: ignore

latest_intent = None
lock = threading.Lock()

# File per IPC tra processo ws_teleop_server e processo spi_j5vr_tx.
# /dev/shm è tmpfs (RAM-backed) su Linux: elimina latenza SD card a 100 Hz.
_INTENT_FILE    = os.environ.get("J5VR_INTENT_FILE",    "/dev/shm/j5vr_latest_intent.json")
_FEEDBACK_FILE  = os.environ.get("J5VR_FEEDBACK_FILE",  "/dev/shm/j5vr_feedback.json")
_TELEMETRY_FILE = os.environ.get("J5VR_TELEMETRY_FILE", "/dev/shm/j5vr_telemetry.json")


def _flock(f, flag):
    """Applica fcntl.flock se disponibile (Unix only)."""
    if fcntl is not None and hasattr(f, "fileno"):
        fcntl.flock(f.fileno(), flag)


def _write_json_atomic(path: str, data: dict, *, sync: bool = True) -> None:
    """Scrive data su path in modo atomico (write su tmp + os.replace).
    sync=False skips fsync for lower latency on volatile data.
    Su /dev/shm (tmpfs/RAM) fsync è sempre forzato a False: è un no-op
    costoso su filesystem in memoria."""
    if path.startswith("/dev/shm"):
        sync = False
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(data, f)
            f.flush()
            if sync:
                os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except Exception:
            pass


# Write batching: fsync every Nth write (volatile /tmp data)
_intent_write_count: int = 0
_telemetry_write_count: int = 0
_INTENT_FSYNC_EVERY: int = 10
_TELEMETRY_FSYNC_EVERY: int = 10


def write_intent_to_file(intent):
    """Scrive l'intent su file. Fsync solo ogni 10 write per ridurre latenza."""
    global _intent_write_count
    _intent_write_count += 1
    do_sync = (_intent_write_count % _INTENT_FSYNC_EVERY) == 0
    _write_json_atomic(_INTENT_FILE, intent, sync=do_sync)


_intent_cache: dict | None = None
_intent_cache_mtime: float = 0.0


def read_intent_from_file():
    """Legge l'ultimo intent dal file con cache mtime (come telemetry)."""
    global _intent_cache, _intent_cache_mtime
    try:
        if not os.path.isfile(_INTENT_FILE):
            return None
        mtime = os.path.getmtime(_INTENT_FILE)
        if mtime == _intent_cache_mtime and _intent_cache is not None:
            return _intent_cache
        with open(_INTENT_FILE) as f:
            _flock(f, fcntl.LOCK_SH if fcntl else 0)
            try:
                data = json.load(f)
            finally:
                _flock(f, fcntl.LOCK_UN if fcntl else 0)
        _intent_cache = data
        _intent_cache_mtime = mtime
        return data
    except Exception:
        return None


def write_feedback_to_file(feedback: dict):
    """Scrive feedback (ACK/eventi) su file e aggiorna la cache in-process."""
    global _feedback_cache, _feedback_cache_mtime
    _write_json_atomic(_FEEDBACK_FILE, feedback)
    _feedback_cache = feedback
    try:
        _feedback_cache_mtime = os.path.getmtime(_FEEDBACK_FILE)
    except Exception:
        pass


_feedback_cache: "dict | None" = None
_feedback_cache_mtime: float = 0.0


def read_feedback_from_file():
    """Legge l'ultimo feedback con cache mtime: rilegge il file solo se è cambiato."""
    global _feedback_cache, _feedback_cache_mtime
    try:
        if not os.path.isfile(_FEEDBACK_FILE):
            return None
        mtime = os.path.getmtime(_FEEDBACK_FILE)
        if mtime == _feedback_cache_mtime and _feedback_cache is not None:
            return _feedback_cache
        with open(_FEEDBACK_FILE) as f:
            _flock(f, fcntl.LOCK_SH if fcntl else 0)
            try:
                data = json.load(f)
            finally:
                _flock(f, fcntl.LOCK_UN if fcntl else 0)
        _feedback_cache = data
        _feedback_cache_mtime = mtime
        return data
    except Exception:
        return None


_telemetry_cache: dict = {}
_telemetry_cache_mtime: float = 0.0


def is_telemetry_fresh(max_age_s: float = 1.5) -> bool:
    """Check if telemetry file was updated recently, using cached mtime."""
    if _telemetry_cache_mtime <= 0:
        return False
    import time as _t
    return (_t.time() - _telemetry_cache_mtime) <= max_age_s


def is_intent_fresh(max_age_s: float = 1.5) -> bool:
    """Check if intent file was updated recently, using cached mtime."""
    if _intent_cache_mtime <= 0:
        return False
    import time as _t
    return (_t.time() - _intent_cache_mtime) <= max_age_s


def get_intent_cache_mtime() -> float:
    """Restituisce il mtime (epoch) dell'ultimo intent letto da file.
    API pubblica — evita import di _intent_cache_mtime (variabile privata)."""
    return _intent_cache_mtime


def write_telemetry_to_file(telemetry: dict):
    """Scrive telemetria IMU su file per debug UI e aggiorna la cache in-process."""
    global _telemetry_cache, _telemetry_cache_mtime, _telemetry_write_count
    _telemetry_write_count += 1
    do_sync = (_telemetry_write_count % _TELEMETRY_FSYNC_EVERY) == 0
    _write_json_atomic(_TELEMETRY_FILE, telemetry, sync=do_sync)
    _telemetry_cache = telemetry
    try:
        _telemetry_cache_mtime = os.path.getmtime(_TELEMETRY_FILE)
    except Exception:
        pass


def read_telemetry_from_file():
    """Legge l'ultima telemetria con cache mtime: rilegge il file solo se è cambiato dall'ultima lettura."""
    global _telemetry_cache, _telemetry_cache_mtime
    try:
        if not os.path.isfile(_TELEMETRY_FILE):
            return None
        mtime = os.path.getmtime(_TELEMETRY_FILE)
        if mtime == _telemetry_cache_mtime and _telemetry_cache:
            return _telemetry_cache
        with open(_TELEMETRY_FILE) as f:
            _flock(f, fcntl.LOCK_SH if fcntl else 0)
            try:
                data = json.load(f)
            finally:
                _flock(f, fcntl.LOCK_UN if fcntl else 0)
        _telemetry_cache = data
        _telemetry_cache_mtime = mtime
        return data
    except Exception:
        return None


