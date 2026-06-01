"""
poe_params_manager.py — Persistenza parametri POE (assi di vite S + matrice home M).

Stesso schema JSON del frontend: {"S": [[6 floats] x 6], "M": [[4 floats] x 4]}
Storage: JSON su filesystem (path configurabile via J5_POE_PARAMS_FILE).

Cache numpy in-memory aggiornata su load/save per evitare I/O ad ogni valutazione FK/IK.
"""

from __future__ import annotations

import copy
import json
import logging
import os
import threading
from typing import Any

import numpy as np
from controller.web_services import runtime_config_paths as rcfg

logger = logging.getLogger("poe_params_manager")

_lock = threading.Lock()

# Default allineato a ik_solver.POE_SCREWS / POE_M e al frontend (metri).
POE_DEFAULT: dict[str, Any] = {
    "S": [
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.094, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.154, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.311, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 0.311, 0.0],
    ],
    "M": [
        [1.0, 0.0, 0.0, 0.060],
        [0.0, 1.0, 0.0, 0.000],
        [0.0, 0.0, 1.0, 0.311],
        [0.0, 0.0, 0.0, 1.000],
    ],
}

_np_S: np.ndarray | None = None
_np_M: np.ndarray | None = None
_seed_hash: int = 0
_warned_legacy_env_override = False


def _validate_poe(obj: Any) -> tuple[dict[str, list] | None, str | None]:
    if not isinstance(obj, dict):
        return None, "POE deve essere un oggetto con S e M"
    S_raw = obj.get("S")
    M_raw = obj.get("M")
    if not isinstance(S_raw, list) or len(S_raw) != 6:
        return None, "S deve essere una lista di 6 righe"
    if not isinstance(M_raw, list) or len(M_raw) != 4:
        return None, "M deve essere una matrice 4x4 (4 righe)"
    S_out: list[list[float]] = []
    for i, row in enumerate(S_raw):
        if not isinstance(row, (list, tuple)) or len(row) != 6:
            return None, f"S[{i}] deve avere 6 elementi"
        r: list[float] = []
        for j, v in enumerate(row):
            try:
                x = float(v)
            except (TypeError, ValueError):
                return None, f"S[{i}][{j}] non numerico"
            if x != x or x in (float("inf"), float("-inf")):
                return None, f"S[{i}][{j}] non finito"
            r.append(x)
        S_out.append(r)
    M_out: list[list[float]] = []
    for i, row in enumerate(M_raw):
        if not isinstance(row, (list, tuple)) or len(row) != 4:
            return None, f"M[{i}] deve avere 4 elementi"
        r = []
        for j, v in enumerate(row):
            try:
                x = float(v)
            except (TypeError, ValueError):
                return None, f"M[{i}][{j}] non numerico"
            if x != x or x in (float("inf"), float("-inf")):
                return None, f"M[{i}][{j}] non finito"
            r.append(x)
        M_out.append(r)
    return {"S": S_out, "M": M_out}, None


def _set_cache_from_struct(struct: dict[str, list]) -> None:
    global _np_S, _np_M, _seed_hash
    S = np.asarray(struct["S"], dtype=float)
    M = np.asarray(struct["M"], dtype=float)
    _np_S = S
    _np_M = M
    _seed_hash = hash(S.tobytes()) ^ hash(M.tobytes())


def _default_struct_copy() -> dict[str, list]:
    return copy.deepcopy(POE_DEFAULT)


def load() -> dict[str, Any]:
    """
    Carica S, M come liste nested. Aggiorna cache numpy.
    persisted: True se esisteva un file JSON sul disco (anche se contenuto invalido → default).
    """
    path = rcfg.get_runtime_config_read_path("j5_poe_params")
    global _warned_legacy_env_override
    env_name = rcfg.get_runtime_only_legacy_env_var("j5_poe_params")
    if env_name and os.environ.get(env_name) and not _warned_legacy_env_override:
        logger.warning("[poe] %s è impostata ma ignorata (runtime-only read attivo)", env_name)
        _warned_legacy_env_override = True
    with _lock:
        if not os.path.isfile(path):
            raise RuntimeError(f"[poe] runtime config mancante: {path} (fallback legacy disabilitato)")
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            validated, err = _validate_poe(data)
            if err or validated is None:
                raise RuntimeError(f"[poe] runtime config invalida ({path}): {err}")
            _set_cache_from_struct(validated)
            logger.info("[poe] caricato da %s", path)
            return {"S": validated["S"], "M": validated["M"], "persisted": True}
        except Exception as e:
            if isinstance(e, RuntimeError):
                raise
            raise RuntimeError(f"[poe] errore lettura runtime config {path}: {e}") from e


def save(obj: Any) -> bool:
    """Valida e salva JSON. Aggiorna cache numpy."""
    validated, err = _validate_poe(obj)
    if err or validated is None:
        logger.warning("[poe] validazione fallita: %s", err)
        return False
    write_path = rcfg.get_runtime_config_write_path("j5_poe_params")
    with _lock:
        try:
            ok = rcfg.save_runtime_json("j5_poe_params", validated, mirror_legacy=False)
            if ok:
                _set_cache_from_struct(validated)
                logger.info("[poe] salvato in %s", write_path)
                return True
            logger.warning("[poe] errore scrittura %s", write_path)
            return False
        except Exception as e:
            logger.warning("[poe] errore scrittura %s: %s", write_path, e)
            return False


def get_screws_m_numpy() -> tuple[np.ndarray, np.ndarray]:
    """
    S 6x6, M 4x4 per FK/IK. Usa cache; se cache vuota, load() una volta.
    """
    with _lock:
        if _np_S is not None and _np_M is not None:
            return _np_S, _np_M
    load()
    with _lock:
        assert _np_S is not None and _np_M is not None
        return _np_S, _np_M


def get_seed_signature() -> tuple:
    """Chiave per invalidare seed IK quando cambia il modello POE."""
    get_screws_m_numpy()
    with _lock:
        return ("POE", _seed_hash)
