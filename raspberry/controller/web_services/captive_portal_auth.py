import json
import os
import time
from typing import Dict


_AUTH_TTL_S = 60.0   # 60 s — il keepalive HTTP rinnova ogni 2 s mentre viewer è aperto; 60 s bastano per la navigazione HTTP→HTTPS ma il captive torna entro 1 min dopo disconnessione
_AUTH_FILE = os.environ.get(
    "CAPTIVE_PORTAL_AUTH_FILE",
    "/tmp/jonny5-captive-portal/authenticated_clients.json",
)


def _ensure_parent_dir():
    parent = os.path.dirname(_AUTH_FILE)
    if parent:
        os.makedirs(parent, exist_ok=True)


def _load_raw() -> Dict[str, float]:
    try:
        with open(_AUTH_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            return {str(k): float(v) for k, v in data.items()}
    except FileNotFoundError:
        return {}
    except Exception:
        return {}
    return {}


def _save_raw(data: Dict[str, float]):
    _ensure_parent_dir()
    tmp = f"{_AUTH_FILE}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f)
    os.replace(tmp, _AUTH_FILE)


def prune_expired(now: float | None = None) -> Dict[str, float]:
    now = time.time() if now is None else float(now)
    data = _load_raw()
    cleaned = {ip: expiry for ip, expiry in data.items() if float(expiry) >= now}
    if cleaned != data:
        _save_raw(cleaned)
    return cleaned


def mark_authenticated(ip: str, ttl_s: float = _AUTH_TTL_S):
    now = time.time()
    data = prune_expired(now)
    data[str(ip)] = now + float(ttl_s)
    _save_raw(data)


def is_authenticated(ip: str) -> bool:
    now = time.time()
    data = prune_expired(now)
    return float(data.get(str(ip), 0.0)) >= now
