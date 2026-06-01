"""
runtime_config_paths.py — Centralized runtime config path resolution.

Safe-mode strategy:
1) Runtime configs live under config_runtime.
2) Runtime-only keys read only from config_runtime.
3) Legacy paths remain only where explicitly retained for controlled compatibility.
"""

from __future__ import annotations

import json
import os
import tempfile
from typing import Any


_WEB_SERVICES_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROLLER_DIR = os.path.dirname(_WEB_SERVICES_DIR)
_RPI5_ROOT = os.path.dirname(_CONTROLLER_DIR)

_CONFIG_RUNTIME_DIR = os.path.join(_RPI5_ROOT, "config_runtime")
_RUNTIME_ONLY_READ_KEYS = {"j5_settings", "j5_poe_params", "video_pipeline", "routing_config", "imu_ee_mount", "pwm_config", "imu_world_bias", "imu_home_ref", "controller_mappings"}
_RUNTIME_ONLY_ENV_VARS = {
    "j5_settings": "J5_SETTINGS_FILE",
    "j5_poe_params": "J5_POE_PARAMS_FILE",
}


def _legacy_settings_path() -> str:
    return os.environ.get("J5_SETTINGS_FILE", os.path.join(_RPI5_ROOT, "j5_settings.json"))


def _legacy_poe_path() -> str:
    return os.environ.get("J5_POE_PARAMS_FILE", os.path.join(_RPI5_ROOT, "j5_poe_params.json"))


_CONFIG_SPECS: dict[str, dict[str, Any]] = {
    "j5_settings": {
        "runtime_rel": "robot/j5_settings.json",
        "legacy_getter": _legacy_settings_path,
    },
    "routing_config": {
        "runtime_rel": "robot/routing_config.json",
        "legacy_abs": os.path.join(_CONTROLLER_DIR, "routing_config.json"),
    },
    "controller_mappings": {
        # Runtime mapping (mode, event) -> action. Modificabile da dashboard
        # /controllers e broadcast a tutti i client via WS controller_mappings_updated.
        "runtime_rel": "robot/controller_mappings.json",
    },
    "j5_poe_params": {
        "runtime_rel": "kinematics/j5_poe_params.json",
        "legacy_getter": _legacy_poe_path,
    },
    "imu_ee_mount": {
        # Supported in runtime mapping for progressive migration.
        # Current repository evidence shows usage primarily in diagnostic tools
        # (e.g. controller.imu_analytics.validate_imu_vs_ee), not in the active WS/SPI live control path.
        "runtime_rel": "imu/imu_ee_mount.json",
        "legacy_abs": os.path.join(_RPI5_ROOT, "config", "imu_ee_mount.json"),
    },
    "video_pipeline": {
        "runtime_rel": "video/video_pipeline.yaml",
        "legacy_abs": os.path.join(_RPI5_ROOT, "config", "video_pipeline.yaml"),
    },
    "pwm_config": {
        "runtime_rel": "robot/pwm_config.json",
    },
    "imu_world_bias": {
        # BNO085 Rotation-Vector world-frame yaw bias (magnetometer-dependent).
        # Optional: if the runtime file is absent the validation layer falls back to
        # identity (backward-compatible — same behaviour as before this artefact existed).
        # This is deliberately SEPARATED from imu_ee_mount so the mechanical mount stays
        # a fixed chip-to-EE rotation while the magnetometer-driven yaw reference is
        # refreshable independently. Used only by the validation / analytics tools; the
        # operational teleop / VR / WS runtime path does NOT consult this file.
        "runtime_rel": "imu/imu_world_bias.json",
    },
    "imu_home_ref": {
        # "Zero at HOME" observability-layer quaternion: snapshot of the residual
        # rotation (R_observed · R_fk_conj) taken when the operator presses the
        # "Azzera IMU @ HOME" button on the FK/IK dashboard. Used ONLY by the
        # compare/IMU-vs-FK visualization layer to cancel a stale BNO085 yaw
        # reference without rerunning the full mount/world_bias calibration.
        #
        # Shape: { "quat_wxyz": [w,x,y,z], "calibrated_at": ISO8601, "fk_pose_mm": [...] }
        # Optional: if missing → pipeline collapses to R_ee = R_wb^-1 · R_imu · R_mount^-1
        # (identical pre-fix behaviour). Does NOT affect operational teleop / VR /
        # WS control / SPI / firmware paths.
        "runtime_rel": "imu/imu_home_ref.json",
    },
    "webrtc_calibration": {
        "runtime_rel": "vr/webrtc-calibration.json",
        "legacy_abs": os.path.join(_RPI5_ROOT, "web", "vr", "webrtc-calibration.json"),
    },
    "tls_cert": {
        "runtime_rel": "tls/webrtc.crt",
        "legacy_abs": os.path.join(_CONTROLLER_DIR, "certs", "webrtc.crt"),
    },
    "tls_key": {
        "runtime_rel": "tls/webrtc.key",
        "legacy_abs": os.path.join(_CONTROLLER_DIR, "certs", "webrtc.key"),
    },
}


def list_supported_keys() -> list[str]:
    return sorted(_CONFIG_SPECS.keys())


def _spec(config_key: str) -> dict[str, Any]:
    if config_key not in _CONFIG_SPECS:
        raise KeyError(f"Unsupported runtime config key: {config_key}")
    return _CONFIG_SPECS[config_key]


def get_runtime_config_path(config_key: str) -> str:
    return os.path.join(_CONFIG_RUNTIME_DIR, _spec(config_key)["runtime_rel"])


def get_legacy_config_path(config_key: str) -> str:
    s = _spec(config_key)
    if "legacy_abs" in s:
        return s["legacy_abs"]
    return s["legacy_getter"]()


def get_runtime_config_read_path(config_key: str) -> str:
    runtime_path = get_runtime_config_path(config_key)
    if config_key in _RUNTIME_ONLY_READ_KEYS:
        return runtime_path
    if os.path.isfile(runtime_path):
        return runtime_path
    return get_legacy_config_path(config_key)


def resolve_existing_config_path(config_key: str, env_var: str | None = None) -> str:
    """
    Risolve un file opzionale preferendo:
    1) override env esplicito,
    2) path runtime,
    3) path legacy.

    Se nessun file esiste, ritorna comunque il path preferito per produrre errori chiari a valle.
    """
    preferred = os.environ.get(env_var) if env_var else None
    candidates: list[str] = []
    for raw in (preferred, get_runtime_config_path(config_key), get_legacy_config_path(config_key)):
        if not raw:
            continue
        path = os.path.abspath(raw)
        if path not in candidates:
            candidates.append(path)
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0] if candidates else get_runtime_config_path(config_key)


def is_runtime_only_read_key(config_key: str) -> bool:
    return config_key in _RUNTIME_ONLY_READ_KEYS


def get_runtime_only_legacy_env_var(config_key: str) -> str | None:
    return _RUNTIME_ONLY_ENV_VARS.get(config_key)


def get_runtime_config_write_path(config_key: str) -> str:
    return get_runtime_config_path(config_key)


def ensure_parent_dir(path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)


def _write_text_atomic(path: str, text: str) -> None:
    ensure_parent_dir(path)
    fd, tmp_path = tempfile.mkstemp(prefix=".tmp_cfg_", dir=os.path.dirname(path), text=True)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def load_runtime_json(config_key: str, default: Any = None) -> Any:
    path = get_runtime_config_read_path(config_key)
    if not os.path.isfile(path):
        return default
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def _validate_routing_config_shape(cfg: Any) -> None:
    if not isinstance(cfg, dict):
        raise RuntimeError("[routing_config] payload non valido: atteso oggetto JSON")

    required_top = ("pbState", "pbEn", "limits")
    for key in required_top:
        if key not in cfg:
            raise RuntimeError(f"[routing_config] chiave obbligatoria mancante: {key}")

    pb_state = cfg.get("pbState")
    pb_en = cfg.get("pbEn")
    limits = cfg.get("limits")
    if not isinstance(pb_state, dict):
        raise RuntimeError("[routing_config] pbState non valido: atteso oggetto")
    if not isinstance(pb_en, dict):
        raise RuntimeError("[routing_config] pbEn non valido: atteso oggetto")
    if not isinstance(limits, dict):
        raise RuntimeError("[routing_config] limits non valido: atteso oggetto")

    axes = ("roll", "pitch", "yaw")
    for axis in axes:
        row = pb_state.get(axis)
        if not isinstance(row, dict):
            raise RuntimeError(f"[routing_config] pbState.{axis} mancante/non valido")
        if "src" not in row or "sign" not in row:
            raise RuntimeError(f"[routing_config] pbState.{axis} richiede src/sign")
        if axis not in pb_en:
            raise RuntimeError(f"[routing_config] pbEn.{axis} mancante")

    joints = ("base", "spalla", "gomito", "yaw", "pitch", "roll")
    for joint in joints:
        row = limits.get(joint)
        if not isinstance(row, dict):
            raise RuntimeError(f"[routing_config] limits.{joint} mancante/non valido")
        if "min" not in row or "max" not in row:
            raise RuntimeError(f"[routing_config] limits.{joint} richiede min/max")
        try:
            mn = float(row["min"])
            mx = float(row["max"])
        except Exception as e:
            raise RuntimeError(f"[routing_config] limits.{joint} min/max non numerici") from e
        if not (0.0 <= mn < mx <= 180.0):
            raise RuntimeError(f"[routing_config] limits.{joint} fuori range: min={mn} max={mx}")


def validate_routing_config_shape(cfg: Any) -> None:
    _validate_routing_config_shape(cfg)


def _validate_imu_ee_mount_shape(cfg: Any) -> None:
    if not isinstance(cfg, dict):
        raise RuntimeError("[imu_ee_mount] payload non valido: atteso oggetto JSON")

    quat = cfg.get("quat_wxyz")
    rpy = cfg.get("rpy_deg")

    quat_ok = isinstance(quat, list) and len(quat) == 4
    rpy_ok = isinstance(rpy, list) and len(rpy) == 3
    if not quat_ok and not rpy_ok:
        raise RuntimeError("[imu_ee_mount] schema non valido: richiesto quat_wxyz[4] oppure rpy_deg[3]")

    if quat_ok:
        try:
            [float(v) for v in quat]
        except Exception as e:
            raise RuntimeError("[imu_ee_mount] quat_wxyz contiene valori non numerici") from e
    if rpy_ok:
        try:
            [float(v) for v in rpy]
        except Exception as e:
            raise RuntimeError("[imu_ee_mount] rpy_deg contiene valori non numerici") from e


def validate_imu_ee_mount_shape(cfg: Any) -> None:
    _validate_imu_ee_mount_shape(cfg)


def _validate_imu_world_bias_shape(cfg: Any) -> None:
    if not isinstance(cfg, dict):
        raise RuntimeError("[imu_world_bias] payload non valido: atteso oggetto JSON")
    quat = cfg.get("quat_wxyz")
    rpy = cfg.get("rpy_deg")
    quat_ok = isinstance(quat, list) and len(quat) == 4
    rpy_ok = isinstance(rpy, list) and len(rpy) == 3
    if not quat_ok and not rpy_ok:
        raise RuntimeError("[imu_world_bias] schema non valido: richiesto quat_wxyz[4] oppure rpy_deg[3]")
    if quat_ok:
        try:
            [float(v) for v in quat]
        except Exception as e:
            raise RuntimeError("[imu_world_bias] quat_wxyz contiene valori non numerici") from e
    if rpy_ok:
        try:
            [float(v) for v in rpy]
        except Exception as e:
            raise RuntimeError("[imu_world_bias] rpy_deg contiene valori non numerici") from e


def validate_imu_world_bias_shape(cfg: Any) -> None:
    _validate_imu_world_bias_shape(cfg)


def _validate_imu_home_ref_shape(cfg: Any) -> None:
    """Schema dell'offset HOME (quat_wxyz obbligatorio, metadati opzionali)."""
    if not isinstance(cfg, dict):
        raise RuntimeError("[imu_home_ref] payload non valido: atteso oggetto JSON")
    quat = cfg.get("quat_wxyz")
    if not isinstance(quat, list) or len(quat) != 4:
        raise RuntimeError("[imu_home_ref] schema non valido: richiesto quat_wxyz[4]")
    try:
        [float(v) for v in quat]
    except Exception as e:
        raise RuntimeError("[imu_home_ref] quat_wxyz contiene valori non numerici") from e


def validate_imu_home_ref_shape(cfg: Any) -> None:
    _validate_imu_home_ref_shape(cfg)


def load_imu_ee_mount_strict() -> dict:
    """Carica imu_ee_mount runtime-only con guardrail esplicito su presenza/schema."""
    path = get_runtime_config_path("imu_ee_mount")
    if not os.path.isfile(path):
        raise RuntimeError(f"[imu_ee_mount] runtime config mancante: {path} (fallback legacy disabilitato)")
    try:
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except Exception as e:
        raise RuntimeError(f"[imu_ee_mount] errore parsing JSON: {path}: {e}") from e
    _validate_imu_ee_mount_shape(cfg)
    return cfg


def load_routing_config_strict() -> dict:
    """Carica routing_config runtime-only con guardrail esplicito su presenza/schema."""
    path = get_runtime_config_path("routing_config")
    if not os.path.isfile(path):
        raise RuntimeError(f"[routing_config] runtime config mancante: {path} (fallback legacy disabilitato)")
    try:
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except Exception as e:
        raise RuntimeError(f"[routing_config] errore parsing JSON: {path}: {e}") from e
    _validate_routing_config_shape(cfg)
    return cfg


def save_runtime_json(config_key: str, data: Any, mirror_legacy: bool = False) -> bool:
    try:
        payload = json.dumps(data, indent=2, ensure_ascii=False)
        target = get_runtime_config_write_path(config_key)
        _write_text_atomic(target, payload)
        if mirror_legacy:
            _write_text_atomic(get_legacy_config_path(config_key), payload)
        return True
    except Exception:
        return False


def _parse_simple_yaml(text: str) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for line in text.splitlines():
        stripped = line.split("#", 1)[0].strip()
        if not stripped or ":" not in stripped:
            continue
        key, raw = stripped.split(":", 1)
        key = key.strip()
        val = raw.strip()
        if val.lower() in ("true", "false"):
            out[key] = val.lower() == "true"
            continue
        try:
            if "." in val:
                out[key] = float(val)
            else:
                out[key] = int(val)
            continue
        except ValueError:
            pass
        out[key] = val
    return out


def _dump_simple_yaml(data: dict[str, Any]) -> str:
    lines: list[str] = []
    for k, v in data.items():
        if isinstance(v, bool):
            vv = "true" if v else "false"
        else:
            vv = str(v)
        lines.append(f"{k}: {vv}")
    return "\n".join(lines) + "\n"


def load_runtime_yaml(config_key: str, default: Any = None) -> Any:
    path = get_runtime_config_read_path(config_key)
    if not os.path.isfile(path):
        return default
    try:
        with open(path, "r", encoding="utf-8") as f:
            return _parse_simple_yaml(f.read())
    except Exception:
        return default


def save_runtime_yaml(config_key: str, data: dict[str, Any], mirror_legacy: bool = False) -> bool:
    try:
        payload = _dump_simple_yaml(data)
        target = get_runtime_config_write_path(config_key)
        _write_text_atomic(target, payload)
        if mirror_legacy:
            _write_text_atomic(get_legacy_config_path(config_key), payload)
        return True
    except Exception:
        return False
