#!/usr/bin/env python3
"""
Validate IMU orientation against predicted EE orientation.

Pipeline:
1) Read live telemetry (imu quaternion + servo angles) from WS.
2) Reconstruct EE orientation from joint angles using runtime POE FK.
3) Align EE frame to IMU frame with a fixed mounting transform.
4) Compute rotational error metrics and save CSV + JSON summary.
"""

import argparse
import asyncio
import csv
import json
import math
import ssl
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R
import websockets

from controller.web_services import ik_solver, settings_manager, runtime_config_paths as rcfg

# raspberry5 root (this file: .../controller/imu_analytics/validate_imu_vs_ee.py)
RPI5 = Path(__file__).resolve().parents[2]

DEFAULT_OUT_DIR = RPI5 / "logs" / "imu_validation"
DEFAULT_MOUNT_CFG = Path(rcfg.get_runtime_config_path("imu_ee_mount"))


@dataclass
class Sample:
    t_s: float
    servo_deg: list[float]
    q_ee_xyzw: np.ndarray
    q_imu_xyzw: np.ndarray
    q_pred_imu_xyzw: np.ndarray
    q_err_xyzw: np.ndarray
    theta_err_deg: float
    yaw_err_deg: float
    pitch_err_deg: float
    roll_err_deg: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _quat_wxyz_to_xyzw(q_wxyz: list[float]) -> np.ndarray:
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)


def _quat_xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)


def _mount_rotation_from_config(cfg: dict[str, Any]) -> R:
    if isinstance(cfg, dict):
        if isinstance(cfg.get("quat_wxyz"), list) and len(cfg["quat_wxyz"]) == 4:
            q_xyzw = _quat_wxyz_to_xyzw([float(v) for v in cfg["quat_wxyz"]])
            return R.from_quat(q_xyzw)
        if isinstance(cfg.get("rpy_deg"), list) and len(cfg["rpy_deg"]) == 3:
            r, p, y = [float(v) for v in cfg["rpy_deg"]]
            return R.from_euler("ZYX", [y, p, r], degrees=True)
    raise RuntimeError("[imu_ee_mount] schema non valido: richiesto quat_wxyz[4] oppure rpy_deg[3]")


def _world_bias_rotation() -> R:
    """Return the BNO085 world-frame yaw-reference bias as a Rotation.

    Physical-measurement model used by the validation / analytics layer only:
        R_imu = R_world_bias · R_ee · R_mount
    where R_world_bias absorbs the BNO085 Rotation-Vector's magnetometer-derived
    yaw reference (which drifts with magnetometer state), and R_mount is the
    purely mechanical chip-to-EE rigid offset.

    Backward-compatible: if `config_runtime/imu/imu_world_bias.json` is absent
    or malformed, this returns the IDENTITY rotation. Validators then collapse
    back to the historical single-rotation model `R_imu = R_ee · R_mount`.

    This function is deliberately confined to the validation path. Operational
    teleop / VR / WS / SPI / firmware code does NOT consult this file.
    """
    try:
        cfg = rcfg.load_runtime_json("imu_world_bias", default=None)
    except Exception:
        return R.identity()
    if not isinstance(cfg, dict):
        return R.identity()
    try:
        rcfg.validate_imu_world_bias_shape(cfg)
    except Exception:
        return R.identity()
    if isinstance(cfg.get("quat_wxyz"), list) and len(cfg["quat_wxyz"]) == 4:
        q_xyzw = _quat_wxyz_to_xyzw([float(v) for v in cfg["quat_wxyz"]])
        return R.from_quat(q_xyzw)
    if isinstance(cfg.get("rpy_deg"), list) and len(cfg["rpy_deg"]) == 3:
        r, p, y = [float(v) for v in cfg["rpy_deg"]]
        return R.from_euler("ZYX", [y, p, r], degrees=True)
    return R.identity()


def _extract_joint_physical_deg(msg: dict[str, Any]) -> list[float] | None:
    keys = ("servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R")
    if any(k not in msg for k in keys):
        return None
    return [float(msg[k]) for k in keys]


def _extract_imu_quat_xyzw(msg: dict[str, Any]) -> np.ndarray | None:
    keys = ("imu_q_w", "imu_q_x", "imu_q_y", "imu_q_z")
    if any(k not in msg for k in keys):
        return None
    qw = float(msg["imu_q_w"])
    qx = float(msg["imu_q_x"])
    qy = float(msg["imu_q_y"])
    qz = float(msg["imu_q_z"])
    return _quat_wxyz_to_xyzw([qw, qx, qy, qz])


def _ee_rotation_from_servo_physical_deg(servo_deg: list[float], offsets: list[int], dirs: list[int]) -> R:
    virtual_deg = settings_manager.physical_to_virtual(servo_deg, offsets, dirs)
    q_math_deg = np.asarray(virtual_deg, dtype=float) - 90.0
    T = ik_solver.forward_kinematics_poe(q_math_deg)
    return R.from_matrix(T[:3, :3])


def _error_metrics(r_pred_imu: R, r_meas_imu: R) -> tuple[np.ndarray, float, float, float, float]:
    r_err = r_pred_imu.inv() * r_meas_imu
    q_err_xyzw = r_err.as_quat()
    q_err_wxyz = _quat_xyzw_to_wxyz(q_err_xyzw)
    w = _clamp(abs(float(q_err_wxyz[0])), -1.0, 1.0)
    theta_err_deg = math.degrees(2.0 * math.acos(w))
    yaw_err, pitch_err, roll_err = r_err.as_euler("ZYX", degrees=True)
    return q_err_xyzw, theta_err_deg, float(yaw_err), float(pitch_err), float(roll_err)


def _theta_stats(values: list[float]) -> dict[str, float]:
    arr = np.array(values, dtype=float)
    return {
        "theta_err_deg_mean": float(statistics.fmean(values)),
        "theta_err_deg_std": float(statistics.pstdev(values)) if len(values) > 1 else 0.0,
        "theta_err_deg_max": float(max(values)),
        "theta_err_deg_p95": float(np.percentile(arr, 95)),
    }


async def run(args: argparse.Namespace) -> tuple[list[Sample], dict[str, Any]]:
    cfg = settings_manager.load()
    offsets = cfg.get("offsets", settings_manager.DEFAULTS["offsets"])
    dirs = cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
    if bool(args.ignore_mount):
        r_mount = R.identity()
        r_world_bias = R.identity()
    else:
        mount_path = Path(args.mount_config).resolve()
        runtime_mount_path = Path(rcfg.get_runtime_config_path("imu_ee_mount")).resolve()
        if mount_path == runtime_mount_path:
            mount_cfg = rcfg.load_imu_ee_mount_strict()
        else:
            if not mount_path.exists():
                raise RuntimeError(f"[imu_ee_mount] mount config mancante: {mount_path}")
            try:
                mount_cfg = json.loads(mount_path.read_text(encoding="utf-8"))
            except Exception as e:
                raise RuntimeError(f"[imu_ee_mount] errore parsing JSON: {mount_path}: {e}") from e
            rcfg.validate_imu_ee_mount_shape(mount_cfg)
        r_mount = _mount_rotation_from_config(mount_cfg)
        # Optional world-bias rotation (identity if runtime config absent → backward-compatible).
        r_world_bias = _world_bias_rotation()

    ssl_ctx = None
    if args.url.startswith("wss://"):
        ssl_ctx = ssl.create_default_context()
        if args.insecure:
            ssl_ctx.check_hostname = False
            ssl_ctx.verify_mode = ssl.CERT_NONE

    period = 1.0 / max(1.0, float(args.rate))
    start = time.monotonic()
    next_t = start
    samples: list[Sample] = []
    theta_identity: list[float] = []

    async with websockets.connect(args.url, ssl=ssl_ctx, ping_interval=20, ping_timeout=20) as ws:
        while time.monotonic() - start <= float(args.duration):
            raw = await ws.recv()
            msg = json.loads(raw)
            if msg.get("type") != "telemetry":
                continue
            if not bool(msg.get("imu_valid", False)):
                continue
            now = time.monotonic()
            if now < next_t:
                continue
            next_t += period

            servo = _extract_joint_physical_deg(msg)
            q_imu_xyzw = _extract_imu_quat_xyzw(msg)
            if servo is None or q_imu_xyzw is None:
                continue

            r_ee = _ee_rotation_from_servo_physical_deg(servo, offsets, dirs)
            r_meas_imu = R.from_quat(q_imu_xyzw)
            r_pred_imu = r_world_bias * r_ee * r_mount

            q_err_xyzw, theta, yaw_e, pitch_e, roll_e = _error_metrics(r_pred_imu, r_meas_imu)
            if bool(args.compare_identity) and not bool(args.ignore_mount):
                _, theta_id, _, _, _ = _error_metrics(r_ee, r_meas_imu)
                theta_identity.append(theta_id)
            samples.append(
                Sample(
                    t_s=now - start,
                    servo_deg=servo,
                    q_ee_xyzw=r_ee.as_quat(),
                    q_imu_xyzw=q_imu_xyzw,
                    q_pred_imu_xyzw=r_pred_imu.as_quat(),
                    q_err_xyzw=q_err_xyzw,
                    theta_err_deg=theta,
                    yaw_err_deg=yaw_e,
                    pitch_err_deg=pitch_e,
                    roll_err_deg=roll_e,
                )
            )

    if not samples:
        raise RuntimeError("No valid samples collected (check IMU validity / WS telemetry).")

    th = [s.theta_err_deg for s in samples]
    stats = _theta_stats(th)
    summary = {
        "samples": len(samples),
        "duration_s": float(args.duration),
        "rate_hz_target": float(args.rate),
        **stats,
        "mount_config": str(args.mount_config),
        "mount_used": "identity" if bool(args.ignore_mount) else "configured",
        "ignore_mount": bool(args.ignore_mount),
    }
    if theta_identity:
        summary["identity_baseline"] = _theta_stats(theta_identity)
    return samples, summary


def _write_outputs(samples: list[Sample], summary: dict[str, Any], out_dir: Path, stem: str) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"{stem}.csv"
    json_path = out_dir / f"{stem}.summary.json"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "timestamp_s",
                "servo_deg_B",
                "servo_deg_S",
                "servo_deg_G",
                "servo_deg_Y",
                "servo_deg_P",
                "servo_deg_R",
                "q_ee_w",
                "q_ee_x",
                "q_ee_y",
                "q_ee_z",
                "q_imu_w",
                "q_imu_x",
                "q_imu_y",
                "q_imu_z",
                "q_pred_imu_w",
                "q_pred_imu_x",
                "q_pred_imu_y",
                "q_pred_imu_z",
                "q_err_w",
                "q_err_x",
                "q_err_y",
                "q_err_z",
                "theta_err_deg",
                "yaw_err_deg",
                "pitch_err_deg",
                "roll_err_deg",
            ]
        )
        for s in samples:
            q_ee_wxyz = _quat_xyzw_to_wxyz(s.q_ee_xyzw)
            q_imu_wxyz = _quat_xyzw_to_wxyz(s.q_imu_xyzw)
            q_pred_wxyz = _quat_xyzw_to_wxyz(s.q_pred_imu_xyzw)
            q_err_wxyz = _quat_xyzw_to_wxyz(s.q_err_xyzw)
            w.writerow(
                [
                    f"{s.t_s:.6f}",
                    *[f"{v:.6f}" for v in s.servo_deg],
                    *[f"{v:.9f}" for v in q_ee_wxyz],
                    *[f"{v:.9f}" for v in q_imu_wxyz],
                    *[f"{v:.9f}" for v in q_pred_wxyz],
                    *[f"{v:.9f}" for v in q_err_wxyz],
                    f"{s.theta_err_deg:.6f}",
                    f"{s.yaw_err_deg:.6f}",
                    f"{s.pitch_err_deg:.6f}",
                    f"{s.roll_err_deg:.6f}",
                ]
            )
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return csv_path, json_path


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Validate IMU orientation vs predicted EE orientation.")
    p.add_argument("--url", default="wss://10.42.0.1:8557", help="WebSocket URL (ws:// or wss://)")
    p.add_argument("--duration", type=float, default=20.0, help="Acquisition duration in seconds")
    p.add_argument("--rate", type=float, default=20.0, help="Sampling rate in Hz")
    p.add_argument("--mount-config", default=str(DEFAULT_MOUNT_CFG), help="Mount config JSON path")
    p.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="Output directory for CSV/JSON")
    p.add_argument("--stem", default="", help="Optional output filename stem")
    p.add_argument("--insecure", action="store_true", help="Disable TLS verification for wss://")
    p.add_argument("--ignore-mount", action="store_true", help="Use identity mount instead of config file")
    p.add_argument("--compare-identity", action="store_true", help="Also compute identity-mount baseline on same samples")
    return p


def main() -> int:
    args = build_argparser().parse_args()
    if not args.stem:
        args.stem = time.strftime("imu_vs_ee_%Y%m%d_%H%M%S")
    samples, summary = asyncio.run(run(args))
    csv_path, json_path = _write_outputs(samples, summary, Path(args.out_dir), args.stem)
    print(f"samples={summary['samples']}")
    print(f"theta_mean_deg={summary['theta_err_deg_mean']:.4f}")
    print(f"theta_std_deg={summary['theta_err_deg_std']:.4f}")
    print(f"theta_max_deg={summary['theta_err_deg_max']:.4f}")
    print(f"theta_p95_deg={summary['theta_err_deg_p95']:.4f}")
    if "identity_baseline" in summary:
        ib = summary["identity_baseline"]
        print(f"identity_theta_mean_deg={ib['theta_err_deg_mean']:.4f}")
        print(f"identity_theta_p95_deg={ib['theta_err_deg_p95']:.4f}")
    print(f"csv={csv_path}")
    print(f"summary={json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
