"""
self_test_imu.py — Canonical backend entrypoint for IMU self-test.

This module centralizes:
1) axis/direction self-test execution,
2) dynamic-response extraction from motion transients,
3) IMU vibration peak measurement,
4) simplified modal test,
5) canonical summary/report generation for the dashboard path.

For manual diagnostics, call `run_axis_direction_test` or `run_self_test_imu` from
Python (same package layout as the WS server). The dashboard path invokes
`run_self_test_imu` directly — no subprocess and no separate CLI wrapper.
"""

import asyncio
import csv
import json
import ssl
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Awaitable, Callable

import numpy as np
from scipy.spatial.transform import Rotation as R
import websockets

_RPI5_DIR = Path(__file__).resolve().parents[2]

from controller.web_services import runtime_config_paths as rcfg
from controller.imu_analytics.imu_dynamic_motion_peaks import telemetry_msgs_phase_move, telemetry_msgs_to_quat_rows
from controller.imu_analytics.imu_modal_test import aggregate_modal_peaks, run_imu_modal_test
from controller.imu_analytics.imu_vibration_peaks import measure_vibration_peaks_hz, top_vibration_peaks_hz
from controller.imu_analytics import run_real_imu_ee_validation as real_run
from controller.imu_analytics import validate_imu_vs_ee as val


StatusEmitter = Callable[[str], Awaitable[None]] | None

DEFAULT_OUT_DIR = _RPI5_DIR / "logs" / "imu_validation"
DEFAULT_MOUNT_CFG = Path(val.DEFAULT_MOUNT_CFG)

# Finestra C_phase_move spesso <2s a 20Hz → meno di 48 campioni; PSD locale
# resta sensata da ~16+ campioni.
_PHASE_MOVE_MIN_SAMPLES = 20

POSES_ROLL: list[tuple[str, list[int]]] = [
    ("home_ref", [90, 90, 90, 90, 90, 90]),
    ("roll_pos", [90, 90, 90, 90, 90, 120]),
    ("roll_neg", [90, 90, 90, 90, 90, 60]),
]

POSES_ALL_AXES: list[tuple[str, list[int]]] = [
    ("home_ref", [90, 90, 90, 90, 90, 90]),
    ("yaw_pos", [90, 90, 90, 120, 90, 90]),
    ("yaw_neg", [90, 90, 90, 60, 90, 90]),
    ("pitch_pos", [90, 90, 90, 90, 110, 90]),
    ("pitch_neg", [90, 90, 90, 90, 70, 90]),
    ("roll_pos", [90, 90, 90, 90, 90, 120]),
    ("roll_neg", [90, 90, 90, 90, 90, 60]),
]

POSES_MAIN_ARM: list[tuple[str, list[int]]] = [
    ("home_ref", [90, 90, 90, 90, 90, 90]),
    ("base_pos", [120, 90, 90, 90, 90, 90]),
    ("base_neg", [60, 90, 90, 90, 90, 90]),
    ("spalla_pos", [90, 120, 90, 90, 90, 90]),
    ("spalla_neg", [90, 60, 90, 90, 90, 90]),
    ("gomito_pos", [90, 90, 120, 90, 90, 90]),
    ("gomito_neg", [90, 90, 60, 90, 90, 90]),
]

POSES_SELF_TEST: list[tuple[str, list[int]]] = [
    ("home_ref", [90, 90, 90, 90, 90, 90]),
    ("base_pos", [120, 90, 90, 90, 90, 90]),
    ("base_neg", [60, 90, 90, 90, 90, 90]),
    ("spalla_pos", [90, 120, 90, 90, 90, 90]),
    ("spalla_neg", [90, 60, 90, 90, 90, 90]),
    ("gomito_pos", [90, 90, 120, 90, 90, 90]),
    ("gomito_neg", [90, 90, 60, 90, 90, 90]),
    ("yaw_pos", [90, 90, 90, 120, 90, 90]),
    ("yaw_neg", [90, 90, 90, 60, 90, 90]),
    ("pitch_pos", [90, 90, 90, 90, 110, 90]),
    ("pitch_neg", [90, 90, 90, 90, 70, 90]),
    ("roll_pos", [90, 90, 90, 90, 90, 120]),
    ("roll_neg", [90, 90, 90, 90, 90, 60]),
]

AXIS_NAMES = ("x", "y", "z")


@dataclass
class RollDirectionSample:
    pose_name: str
    t_s: float
    phi_pred: np.ndarray
    phi_imu: np.ndarray
    dot_product: float
    angle_between_deg: float


async def _maybe_emit_status(emit_status: StatusEmitter, text: str) -> None:
    if emit_status is not None:
        await emit_status(text)


def _axis_key_from_pose_name(pose_name: str) -> str:
    if pose_name.startswith("base_"):
        return "base"
    if pose_name.startswith("spalla_"):
        return "spalla"
    if pose_name.startswith("gomito_"):
        return "gomito"
    if pose_name.startswith("yaw_"):
        return "yaw"
    if pose_name.startswith("pitch_"):
        return "pitch"
    if pose_name.startswith("roll_"):
        return "roll"
    return "other"


def _pose_group_label(pose_name: str) -> str:
    if pose_name.startswith("base_"):
        return "BASE"
    if pose_name.startswith("spalla_"):
        return "SHOULDER"
    if pose_name.startswith("gomito_"):
        return "ELBOW"
    if pose_name.startswith("yaw_"):
        return "YAW"
    if pose_name.startswith("pitch_"):
        return "PITCH"
    if pose_name.startswith("roll_"):
        return "ROLL"
    return pose_name.upper()


def _safe_angle_deg(v1: np.ndarray, v2: np.ndarray) -> float:
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 < 1e-12 or n2 < 1e-12:
        return 0.0
    c = float(np.dot(v1, v2) / (n1 * n2))
    c = max(-1.0, min(1.0, c))
    return float(np.degrees(np.arccos(c)))


def _dominant_axis(phi: np.ndarray) -> tuple[str, int, float]:
    idx = int(np.argmax(np.abs(phi)))
    value = float(phi[idx])
    sign = 0
    if value > 1e-12:
        sign = 1
    elif value < -1e-12:
        sign = -1
    return AXIS_NAMES[idx], sign, value


def _axis_conclusion(axis_name: str, rows: list[dict[str, Any]], weak_observability: bool = False) -> str:
    if len(rows) < 2:
        return f"{axis_name.upper()} direction: insufficient data."
    both_dot_pos = all(r["dot_product"] > 0.0 for r in rows)
    both_dot_neg = all(r["dot_product"] < 0.0 for r in rows)
    both_sign_ok = all(bool(r["sign_consistent"]) for r in rows)
    both_sign_bad = all(not bool(r["sign_consistent"]) for r in rows)
    alpha_mean = statistics.fmean(float(r["angle_between_deg"]) for r in rows)
    coupling = any(float(r["cross_axis_ratio"]) > 0.45 for r in rows)
    score_mean = statistics.fmean(float(r["directional_consistency_score"]) for r in rows)
    pred_norm_mean = statistics.fmean(float(np.linalg.norm(np.array(r["phi_pred_mean"], dtype=float))) for r in rows)
    imu_norm_mean = statistics.fmean(float(np.linalg.norm(np.array(r["phi_imu_mean"], dtype=float))) for r in rows)
    if weak_observability and (pred_norm_mean < 0.10 or imu_norm_mean < 0.10 or score_mean < 0.75):
        return f"{axis_name.upper()}: INCONCLUSIVE / WEAK OBSERVABILITY"
    if both_dot_pos and both_sign_ok:
        if coupling or alpha_mean > 35.0:
            return f"{axis_name.upper()}: BROADLY CONSISTENT BUT WITH CROSS-AXIS COUPLING"
        return f"{axis_name.upper()}: CONSISTENT"
    if both_dot_neg or (both_sign_bad and alpha_mean > 90.0):
        return f"{axis_name.upper()}: INVERTED / STRONGLY INCONSISTENT"
    if both_dot_pos:
        return f"{axis_name.upper()}: BROADLY CONSISTENT BUT WITH CROSS-AXIS COUPLING"
    return f"{axis_name.upper()}: MIXED / AMBIGUOUS"


def _overall_conclusion(axis_conclusions: dict[str, str], *, main_arm_mode: bool = False, self_test_mode: bool = False) -> str:
    vals = list(axis_conclusions.values())
    if vals and all(v.endswith("CONSISTENT") for v in vals):
        if self_test_mode:
            return "ALL TESTED AXES DIRECTIONALLY CONSISTENT"
        return "MAIN ARM AXES DIRECTIONALLY CONSISTENT" if main_arm_mode else "ALL WRIST AXES DIRECTIONALLY CONSISTENT"
    if any("INVERTED" in v for v in vals):
        return "SOME AXES REQUIRE SIGN REVIEW"
    if axis_conclusions.get("base", "").endswith("CONSISTENT") and axis_conclusions.get("spalla", "").endswith("CONSISTENT") and "INCONCLUSIVE" in axis_conclusions.get("gomito", ""):
        return "BASE/SPALLA CONSISTENT, GOMITO INCONCLUSIVE"
    if any("CROSS-AXIS COUPLING" in v for v in vals):
        return "AXES CONSISTENT BUT AFFECTED BY POSE-DEPENDENT COUPLING"
    return "SOME AXES REQUIRE SIGN REVIEW"


def _load_mount_rotation(mount_config: str | Path) -> dict[str, Any]:
    mount_path = Path(mount_config).resolve()
    runtime_mount_path = Path(rcfg.get_runtime_config_path("imu_ee_mount")).resolve()
    if mount_path == runtime_mount_path:
        return rcfg.load_imu_ee_mount_strict()
    if not mount_path.exists():
        raise RuntimeError(f"[imu_ee_mount] mount config mancante: {mount_path}")
    try:
        mount_cfg = json.loads(mount_path.read_text(encoding="utf-8"))
    except Exception as e:
        raise RuntimeError(f"[imu_ee_mount] errore parsing JSON: {mount_path}: {e}") from e
    rcfg.validate_imu_ee_mount_shape(mount_cfg)
    return mount_cfg


def _select_poses(*, self_test: bool, main_arm: bool, all_axes: bool) -> list[tuple[str, list[int]]]:
    if self_test:
        return POSES_SELF_TEST
    if main_arm:
        return POSES_MAIN_ARM
    if all_axes:
        return POSES_ALL_AXES
    return POSES_ROLL


def _write_axis_outputs(
    out_dir: Path,
    stem: str,
    sample_rows: list[RollDirectionSample],
    pose_results: list[dict[str, Any]],
    summary: dict[str, Any],
) -> tuple[Path, Path, Path]:
    run_dir = out_dir / stem
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "samples.csv"
    json_path = run_dir / "summary.json"
    report_path = run_dir / "report.md"

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pose_name",
                "timestamp_s",
                "phi_pred_x",
                "phi_pred_y",
                "phi_pred_z",
                "phi_imu_x",
                "phi_imu_y",
                "phi_imu_z",
                "dot_product",
                "angle_between_deg",
            ]
        )
        for s in sample_rows:
            w.writerow(
                [
                    s.pose_name,
                    f"{s.t_s:.6f}",
                    f"{float(s.phi_pred[0]):.9f}",
                    f"{float(s.phi_pred[1]):.9f}",
                    f"{float(s.phi_pred[2]):.9f}",
                    f"{float(s.phi_imu[0]):.9f}",
                    f"{float(s.phi_imu[1]):.9f}",
                    f"{float(s.phi_imu[2]):.9f}",
                    f"{s.dot_product:.9f}",
                    f"{s.angle_between_deg:.6f}",
                ]
            )

    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    lines = [
        "# Wrist Direction Consistency Report",
        "",
        "## Procedure",
        f"- home_settle_s: {summary['procedure']['home_settle_s']}",
        f"- imu_settle_s: {summary['procedure']['imu_settle_s']}",
        f"- pose_settle_s: {summary['procedure']['pose_settle_s']}",
        f"- sample_seconds: {summary['procedure']['sample_seconds']}",
        f"- rate_hz: {summary['procedure']['rate_hz']}",
        "",
        "## Pose Diagnostics",
        "",
        "| pose_name | dot_product | angle_between_deg | dominant_axis_pred | dominant_axis_imu | sign_consistent | cross_axis_ratio |",
        "|---|---:|---:|---|---|---|---:|",
    ]
    for r in pose_results:
        lines.append(
            f"| {r['pose_name']} | {r['dot_product']:.6f} | {r['angle_between_deg']:.3f} | "
            f"{r['dominant_axis_pred']} | {r['dominant_axis_imu']} | {r['sign_consistent']} | {r['cross_axis_ratio']:.3f} |"
        )
    if "axis_conclusions" in summary:
        lines.extend(["", "## Axis Conclusions"])
        for k in ("yaw", "pitch", "roll", "base", "spalla", "gomito"):
            if k in summary["axis_conclusions"]:
                lines.append(f"- {summary['axis_conclusions'][k]}")
    lines.extend(["", "## Conclusion", summary["conclusion"], ""])
    dr = summary.get("dynamic_response")
    if isinstance(dr, dict):
        lines.extend(
            [
                "",
                "## Dynamic response (from self-test motion transients)",
                f"- strategy: {dr.get('strategy', {})}",
                f"- top_peaks_hz: {dr.get('top_peaks_hz', [])}",
                f"- valid_segments: {dr.get('valid_segments', 0)}",
                f"- discarded_or_weak_segments: {dr.get('discarded_or_weak_segments', 0)}",
                "",
            ]
        )
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return csv_path, json_path, report_path


def _plot_phi_means(run_dir: Path, pose_results: list[dict[str, Any]]) -> Path:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return run_dir / "phi_comparison.png"
    labels = [p["pose_name"] for p in pose_results]
    pred = np.array([p["phi_pred_mean"] for p in pose_results], dtype=float)
    imu = np.array([p["phi_imu_mean"] for p in pose_results], dtype=float)
    x = np.arange(len(labels))
    width = 0.12
    fig = plt.figure(figsize=(11, 4.8))
    ax = fig.add_subplot(111)
    ax.bar(x - 2.5 * width, pred[:, 0], width=width, label="pred_x")
    ax.bar(x - 1.5 * width, pred[:, 1], width=width, label="pred_y")
    ax.bar(x - 0.5 * width, pred[:, 2], width=width, label="pred_z")
    ax.bar(x + 0.5 * width, imu[:, 0], width=width, label="imu_x")
    ax.bar(x + 1.5 * width, imu[:, 1], width=width, label="imu_y")
    ax.bar(x + 2.5 * width, imu[:, 2], width=width, label="imu_z")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Rotation Vector Component [rad]")
    ax.set_title("Mean Rotation Vectors: Model vs IMU")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(ncol=3, fontsize=9)
    fig.tight_layout()
    out = run_dir / "phi_comparison.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


async def run_axis_direction_test(
    *,
    url: str = "ws://127.0.0.1:8557",
    insecure: bool = False,
    home_settle: float = 3.0,
    imu_settle: float = 5.0,
    pose_settle: float = 2.0,
    transient_seconds: float = 6.0,
    sample_seconds: float = 4.0,
    rate: float = 20.0,
    profile: str = "RTR3",
    vel_deg_s: float = 30.0,
    out_dir: str | Path = DEFAULT_OUT_DIR,
    stem: str = "",
    mount_config: str | Path = DEFAULT_MOUNT_CFG,
    all_axes: bool = False,
    main_arm: bool = False,
    self_test: bool = False,
    emit_status: StatusEmitter = None,
    emit_final_completed: bool = True,
) -> dict[str, Any]:
    cfg = val.settings_manager.load()
    offsets = cfg.get("offsets", val.settings_manager.DEFAULTS["offsets"])
    dirs = cfg.get("dirs", val.settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
    mount_cfg = _load_mount_rotation(mount_config)
    r_mount = val._mount_rotation_from_config(mount_cfg)
    # Optional world-frame yaw bias (identity if absent → backward-compatible).
    r_world_bias = val._world_bias_rotation()

    ssl_ctx = None
    if url.startswith("wss://"):
        ssl_ctx = ssl.create_default_context()
        if insecure:
            ssl_ctx.check_hostname = False
            ssl_ctx.verify_mode = ssl.CERT_NONE

    poses = _select_poses(self_test=self_test, main_arm=main_arm, all_axes=all_axes)
    base_t = time.monotonic()
    rows: list[RollDirectionSample] = []
    pose_results: list[dict[str, Any]] = []
    transient_segments: list[dict[str, Any]] = []

    async with websockets.connect(url, ssl=ssl_ctx, ping_interval=20, ping_timeout=20) as ws:
        runner = real_run.WSRunner(ws)

        await _maybe_emit_status(emit_status, "Preparing...")
        await runner.send_uart_wait("SAFE", timeout=10.0)
        en = await runner.send_uart_wait("ENABLE", timeout=35.0)
        if not bool(en.get("ok", False)):
            raise RuntimeError(f"ENABLE failed: {en}")
        await _maybe_emit_status(emit_status, "Going HOME...")
        home_resp = await runner.send_uart_wait("HOME", timeout=20.0)
        if not bool(home_resp.get("ok", False)):
            raise RuntimeError(f"HOME failed: {home_resp}")
        await runner.wait_setpose_done(timeout=20.0)
        await asyncio.sleep(float(home_settle))

        await _maybe_emit_status(emit_status, "Zeroing IMU...")
        home_msgs = await runner.collect_telemetry(float(imu_settle), float(rate))
        real_run._annotate_time(base_t, home_msgs)
        home_quats = [val._extract_imu_quat_xyzw(m) for m in home_msgs if bool(m.get("imu_valid", False))]
        home_quats = [q for q in home_quats if q is not None]
        if not home_quats:
            raise RuntimeError("No valid IMU samples while capturing HOME zero")
        q_zero_xyzw = real_run._mean_quat_xyzw(home_quats)
        r_home_imu_zero = R.from_quat(q_zero_xyzw)

        home_last = next((m for m in reversed(home_msgs) if bool(m.get("imu_valid", False))), None)
        if home_last is None:
            raise RuntimeError("No valid HOME telemetry sample")
        _, _, r_home_pred_abs = real_run._telemetry_to_rotations(home_last, offsets, dirs, r_mount, r_world_bias)

        current_label = ""
        for pose_name, pose_virtual in poses:
            next_label = _pose_group_label(pose_name)
            if pose_name != "home_ref" and next_label != current_label:
                await _maybe_emit_status(emit_status, f"Testing {next_label}...")
                current_label = next_label
            cmd = f"SETPOSE {' '.join(str(v) for v in pose_virtual)} {int(vel_deg_s)} {profile}"
            if self_test and pose_name != "home_ref":
                async with websockets.connect(url, ssl=ssl_ctx, ping_interval=20, ping_timeout=20) as ws_tel:
                    tel_runner = real_run.WSRunner(ws_tel)
                    col_task = asyncio.create_task(tel_runner.collect_telemetry(float(transient_seconds), float(rate)))
                    await asyncio.sleep(0.05)
                    t_send = time.monotonic()
                    try:
                        pose_resp = await runner.send_uart_wait(cmd, timeout=20.0)
                        if not bool(pose_resp.get("ok", False)):
                            col_task.cancel()
                            try:
                                await col_task
                            except asyncio.CancelledError:
                                pass
                            raise RuntimeError(f"Pose command failed for {pose_name}: {pose_resp}")
                        await runner.wait_setpose_done(timeout=20.0)
                        t_done = time.monotonic()
                        transient_msgs = await col_task
                    except Exception:
                        if not col_task.done():
                            col_task.cancel()
                            try:
                                await col_task
                            except asyncio.CancelledError:
                                pass
                        raise
                real_run._annotate_time(base_t, transient_msgs)
                move_msgs = telemetry_msgs_phase_move(transient_msgs, base_t=base_t, t_send_mono=t_send, t_done_mono=t_done)
                qrows = telemetry_msgs_to_quat_rows(move_msgs)
                peaks_tr: list[float] = []
                perr_tr: str | None = None
                pre_used = "raw"
                if len(qrows) >= _PHASE_MOVE_MIN_SAMPLES:
                    peaks_tr, perr_tr = top_vibration_peaks_hz(
                        qrows,
                        top_n=5,
                        min_hz=0.25,
                        prominence_frac=0.045,
                        detrend_omega_linear=False,
                        min_samples=_PHASE_MOVE_MIN_SAMPLES,
                    )
                    if not peaks_tr:
                        peaks_fb, perr_fb = top_vibration_peaks_hz(
                            qrows,
                            top_n=5,
                            min_hz=0.25,
                            prominence_frac=0.045,
                            detrend_omega_linear=True,
                            min_samples=_PHASE_MOVE_MIN_SAMPLES,
                        )
                        if peaks_fb:
                            peaks_tr, pre_used, perr_tr = peaks_fb, "detrend_linear_fallback", ""
                else:
                    perr_tr = f"phase_move samples {len(qrows)} < {_PHASE_MOVE_MIN_SAMPLES}"
                seg_status = "ok"
                if len(qrows) < _PHASE_MOVE_MIN_SAMPLES:
                    seg_status = "insufficient_samples"
                elif not peaks_tr:
                    seg_status = "no_peaks"
                transient_segments.append(
                    {
                        "pose_name": pose_name,
                        "axis": _axis_key_from_pose_name(pose_name),
                        "segment": "C_phase_move",
                        "signal": "omega_norm",
                        "preprocess": pre_used,
                        "frequencies_hz": peaks_tr,
                        "sample_count": len(qrows),
                        "status": seg_status,
                        "error": (perr_tr or "") if not peaks_tr else "",
                    }
                )
            else:
                pose_resp = await runner.send_uart_wait(cmd, timeout=20.0)
                if not bool(pose_resp.get("ok", False)):
                    raise RuntimeError(f"Pose command failed for {pose_name}: {pose_resp}")
                await runner.wait_setpose_done(timeout=20.0)
            await asyncio.sleep(float(pose_settle))
            pose_msgs = await runner.collect_telemetry(float(sample_seconds), float(rate))
            real_run._annotate_time(base_t, pose_msgs)

            phi_pred_samples: list[np.ndarray] = []
            phi_imu_samples: list[np.ndarray] = []
            for msg in pose_msgs:
                if not bool(msg.get("imu_valid", False)):
                    continue
                _, r_imu_raw, r_pred_abs = real_run._telemetry_to_rotations(msg, offsets, dirs, r_mount, r_world_bias)
                r_pred_rel = r_home_pred_abs.inv() * r_pred_abs
                r_imu_rel = r_home_imu_zero.inv() * r_imu_raw
                phi_pred = r_pred_rel.as_rotvec()
                phi_imu = r_imu_rel.as_rotvec()
                dot = float(np.dot(phi_pred, phi_imu))
                ang = _safe_angle_deg(phi_pred, phi_imu)
                phi_pred_samples.append(phi_pred)
                phi_imu_samples.append(phi_imu)
                rows.append(
                    RollDirectionSample(
                        pose_name=pose_name,
                        t_s=float(msg.get("_sample_t_s", 0.0)),
                        phi_pred=phi_pred,
                        phi_imu=phi_imu,
                        dot_product=dot,
                        angle_between_deg=ang,
                    )
                )
            if not phi_pred_samples:
                raise RuntimeError(f"No valid IMU samples for {pose_name}")
            pred_mean = np.mean(np.array(phi_pred_samples), axis=0)
            imu_mean = np.mean(np.array(phi_imu_samples), axis=0)
            dot_mean = float(np.dot(pred_mean, imu_mean))
            alpha = _safe_angle_deg(pred_mean, imu_mean)
            pred_axis, pred_sign, _ = _dominant_axis(pred_mean)
            imu_axis, imu_sign, _ = _dominant_axis(imu_mean)
            pred_dom_idx = AXIS_NAMES.index(pred_axis)
            imu_on_pred_axis = float(imu_mean[pred_dom_idx])
            pred_on_pred_axis = float(pred_mean[pred_dom_idx])
            sign_consistent = (pred_on_pred_axis * imu_on_pred_axis) > 0.0
            cross_axis_ratio = 0.0
            pred_norm = float(np.linalg.norm(pred_mean))
            if pred_norm > 1e-12:
                cross_axis_ratio = float(np.linalg.norm(np.delete(imu_mean, pred_dom_idx)) / max(1e-12, abs(imu_on_pred_axis)))
            pose_results.append(
                {
                    "pose_name": pose_name,
                    "target_virtual_deg": pose_virtual,
                    "phi_pred_mean": [float(v) for v in pred_mean],
                    "phi_imu_mean": [float(v) for v in imu_mean],
                    "dot_product": dot_mean,
                    "angle_between_deg": alpha,
                    "dominant_axis_pred": pred_axis,
                    "dominant_axis_imu": imu_axis,
                    "sign_pred": pred_sign,
                    "sign_imu": imu_sign,
                    "sign_consistent": bool(sign_consistent),
                    "directional_consistency_score": float(dot_mean / max(1e-12, np.linalg.norm(pred_mean) * np.linalg.norm(imu_mean))),
                    "cross_axis_ratio": cross_axis_ratio,
                }
            )

    axis_groups = {
        "yaw": [r for r in pose_results if r["pose_name"] in ("yaw_pos", "yaw_neg")],
        "pitch": [r for r in pose_results if r["pose_name"] in ("pitch_pos", "pitch_neg")],
        "roll": [r for r in pose_results if r["pose_name"] in ("roll_pos", "roll_neg")],
        "base": [r for r in pose_results if r["pose_name"] in ("base_pos", "base_neg")],
        "spalla": [r for r in pose_results if r["pose_name"] in ("spalla_pos", "spalla_neg")],
        "gomito": [r for r in pose_results if r["pose_name"] in ("gomito_pos", "gomito_neg")],
    }
    axis_conclusions: dict[str, str] = {}
    for k, v in axis_groups.items():
        if v:
            axis_conclusions[k] = _axis_conclusion(k, v, weak_observability=(k == "gomito"))
    conclusion = _overall_conclusion(axis_conclusions if axis_conclusions else {"roll": _axis_conclusion("roll", axis_groups["roll"])}, main_arm_mode=main_arm, self_test_mode=self_test)

    run_stem = stem or time.strftime("roll_direction_test_%Y%m%d_%H%M%S")
    dynamic_response_peaks_hz: list[float] = []
    dynamic_response_block: dict[str, Any] | None = None
    if self_test:
        dyn_lists = [s["frequencies_hz"] for s in transient_segments if s.get("frequencies_hz")]
        dynamic_response_peaks_hz = aggregate_modal_peaks(dyn_lists, top_n=3) if dyn_lists else []
        valid_n = sum(1 for s in transient_segments if s.get("status") == "ok" and (s.get("frequencies_hz") or []))
        discarded_n = len(transient_segments) - valid_n
        dynamic_response_block = {
            "title": "Dynamic response (from self-test motion transients)",
            "strategy": {
                "segment": "C_phase_move",
                "signal": "omega_norm",
                "preprocess_default": "raw",
                "preprocess_note": "detrend_linear on ||omega|| only as fallback if raw PSD yields no peaks",
                "phase_move_min_samples": _PHASE_MOVE_MIN_SAMPLES,
            },
            "top_peaks_hz": list(dynamic_response_peaks_hz),
            "valid_segments": int(valid_n),
            "discarded_or_weak_segments": int(discarded_n),
            "per_pose": transient_segments,
        }

    summary: dict[str, Any] = {
        "procedure": {
            "home_settle_s": float(home_settle),
            "imu_settle_s": float(imu_settle),
            "pose_settle_s": float(pose_settle),
            "sample_seconds": float(sample_seconds),
            "transient_seconds": float(transient_seconds) if self_test else 0.0,
            "rate_hz": float(rate),
            "profile": profile,
            "vel_deg_s": float(vel_deg_s),
            "mount_config": str(mount_config),
        },
        "poses": pose_results,
        "axis_conclusions": axis_conclusions,
        "conclusion": conclusion,
        "dynamic_response_peaks_hz": dynamic_response_peaks_hz,
        "dynamic_response": dynamic_response_block,
    }

    axis_run_dir = Path(out_dir) / run_stem
    summary["artifacts"] = {
        "run_dir": str(axis_run_dir),
        "samples_csv": str(axis_run_dir / "samples.csv"),
        "summary_json": str(axis_run_dir / "summary.json"),
        "report_md": str(axis_run_dir / "report.md"),
        "phi_plot_png": str(axis_run_dir / "phi_comparison.png"),
    }
    csv_path, json_path, report_path = _write_axis_outputs(Path(out_dir), run_stem, rows, pose_results, summary)
    plot_path = _plot_phi_means(axis_run_dir, pose_results)
    if emit_final_completed:
        await _maybe_emit_status(emit_status, "Completed")
    return {
        "csv_path": str(csv_path),
        "json_path": str(json_path),
        "report_path": str(report_path),
        "plot_path": str(plot_path),
        "summary": summary,
    }


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    n = len(s)
    m = n // 2
    if n % 2:
        return float(s[m])
    return float((s[m - 1] + s[m]) / 2.0)


def _classify_self_test_axis(axis_name: str, rows: list[dict[str, Any]], tool_conclusion: str = "") -> dict[str, object]:
    tc_u = (tool_conclusion or "").upper()
    scores = [float(r.get("directional_consistency_score", 0.0)) for r in rows]
    score_mean = _mean(scores)
    score_median = _median(scores)
    score_min = min(scores) if scores else 0.0
    score_max = max(scores) if scores else 0.0
    cross_max = max([float(r.get("cross_axis_ratio", 0.0)) for r in rows], default=0.0)
    angle_mean = _mean([float(r.get("angle_between_deg", 0.0)) for r in rows])
    signs = [bool(r.get("sign_consistent")) for r in rows]
    sign_all = all(signs)
    sign_any = any(signs)
    sign_split = len(signs) >= 2 and sign_any and (not sign_all)
    dominant_all = all(str(r.get("dominant_axis_pred", "")) == str(r.get("dominant_axis_imu", "")) for r in rows)
    pred_norm_mean = _mean(
        [sum(float(v) * float(v) for v in r.get("phi_pred_mean", [0.0, 0.0, 0.0])) ** 0.5 for r in rows]
    )
    imu_norm_mean = _mean(
        [sum(float(v) * float(v) for v in r.get("phi_imu_mean", [0.0, 0.0, 0.0])) ** 0.5 for r in rows]
    )
    coupling = "CROSS-AXIS COUPLING" in tc_u or "POSE-DEPENDENT COUPLING" in tc_u

    classification = "OK"
    if "INVERTED" in tc_u:
        classification = "FAIL"
    elif axis_name == "yaw" and sign_split and "INVERTED" not in tc_u:
        if score_max >= 0.88 and score_min <= 0.55:
            classification = "WARNING"
        elif score_median >= 0.22 or (score_max >= 0.75 and score_min < 0.0):
            classification = "INCONCLUSIVE"
        else:
            classification = "FAIL"
    elif axis_name == "gomito":
        if "INCONCLUSIVE" in tc_u or "WEAK OBSERVABILITY" in tc_u or pred_norm_mean < 0.08 or imu_norm_mean < 0.08:
            classification = "INCONCLUSIVE"
        elif score_mean < 0.58 and not coupling:
            classification = "FAIL"
        elif score_mean < 0.80:
            classification = "WARNING" if coupling and sign_all else "INCONCLUSIVE"
        elif score_mean < 0.90 or cross_max > 0.30 or angle_mean > 18.0:
            classification = "INCONCLUSIVE"
        else:
            classification = "OK"
    elif axis_name in ("pitch", "roll") and (not dominant_all) and coupling and sign_all and cross_max > 0.30:
        if score_mean >= 0.28:
            classification = "WARNING"
        elif score_mean >= 0.16:
            classification = "INCONCLUSIVE"
        else:
            classification = "FAIL"
    elif not sign_all:
        classification = "FAIL"
    elif not dominant_all:
        if coupling and sign_all and score_mean >= 0.70:
            classification = "WARNING"
        elif coupling and sign_all and score_mean >= 0.45:
            classification = "INCONCLUSIVE"
        elif score_mean < 0.72:
            classification = "FAIL"
        else:
            classification = "WARNING"
    elif score_mean < 0.76:
        classification = "WARNING" if coupling and sign_all else "FAIL"
    elif cross_max > 0.35 or score_mean < 0.93 or angle_mean > 20.0:
        classification = "WARNING"

    out: dict[str, object] = {
        "classification": classification,
        "tool_conclusion": tool_conclusion,
        "score_mean": score_mean,
        "score_median": score_median,
        "cross_axis_max": cross_max,
        "angle_mean_deg": angle_mean,
        "sign_consistent_all": sign_all,
        "dominant_axis_consistent_all": dominant_all,
        "pose_count": len(rows),
    }
    if axis_name == "gomito":
        out["note"] = "Observability is indirect because IMU is mounted on the wrist."
    return out


def _self_test_global_result(axis_status: dict[str, dict[str, object]]) -> str:
    vals = [str(v.get("classification", "")) for v in axis_status.values()]
    if any(v == "FAIL" for v in vals):
        return "TEST FAILED"
    if any(v == "INCONCLUSIVE" for v in vals):
        return "TEST INCONCLUSIVE"
    if any(v == "WARNING" for v in vals):
        return "TEST OK WITH WARNINGS"
    return "TEST OK"


def _self_test_display_message(result: str, tool_conclusion: str) -> str:
    tc = (tool_conclusion or "").strip()
    if result == "TEST FAILED":
        return f"{tc} | Verdict: {result}." if tc else result
    if result == "TEST INCONCLUSIVE":
        return f"{tc} | Verdict: {result} (see per-axis classifications)." if tc else result
    if result == "TEST OK WITH WARNINGS":
        return f"{tc} | Verdict: {result}." if tc else result
    if result == "TEST OK":
        return f"{tc} | Verdict: {result}." if tc else result
    return tc or result


def _write_canonical_outputs(run_dir: str, payload: dict[str, Any]) -> tuple[str, str]:
    out_dir = Path(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_json = out_dir / "summary.json"
    report_md = out_dir / "report.md"
    summary_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    lines = [
        "# Self-Test Report",
        "",
        f"- result: {payload.get('result', '-')}",
        f"- message: {payload.get('message', '-')}",
        "",
        "## Axes",
        "",
        "| axis | classification | score_mean | angle_mean_deg | cross_axis_max |",
        "|---|---|---:|---:|---:|",
    ]
    for axis in ("base", "spalla", "gomito", "yaw", "pitch", "roll"):
        row = (payload.get("axes") or {}).get(axis)
        if not row:
            continue
        lines.append(
            f"| {axis} | {row.get('classification', '-')} | {float(row.get('score_mean', 0.0)):.3f} | "
            f"{float(row.get('angle_mean_deg', 0.0)):.3f} | {float(row.get('cross_axis_max', 0.0)):.3f} |"
        )
        note = row.get("note")
        if note:
            lines.append(f"| {axis} note | {note} |  |  |  |")

    dpeaks = payload.get("dynamic_response_peaks_hz") or []
    derr = (payload.get("dynamic_response_error") or "").strip()
    dr = payload.get("dynamic_response")
    vpeaks = payload.get("imu_vibration_peaks_hz") or []
    verr = (payload.get("imu_vibration_error") or "").strip()
    mpeaks = payload.get("imu_modal_peaks_hz") or []
    merr = (payload.get("imu_modal_error") or "").strip()
    lines.extend(["", "## Dynamic response (from self-test motion transients)"])
    if isinstance(dr, dict):
        strat = dr.get("strategy") or {}
        lines.extend(
            [
                f"- segment: {strat.get('segment', '-')}",
                f"- signal: {strat.get('signal', '-')}",
                f"- preprocess: {strat.get('preprocess_default', '-')} ({strat.get('preprocess_note', '')})",
                f"- top_peaks_hz: {dr.get('top_peaks_hz', dpeaks if dpeaks else '-')}",
                f"- valid_segments: {dr.get('valid_segments', '-')}",
                f"- discarded_or_weak_segments: {dr.get('discarded_or_weak_segments', '-')}",
                f"- note: {derr if derr else 'ok'}",
            ]
        )
    else:
        lines.extend(
            [
                f"- peaks_hz: {dpeaks if dpeaks else '-'}",
                f"- note: {derr if derr else 'ok'}",
            ]
        )
    lines.extend(
        [
            "",
            "## IMU vibration (experimental)",
            f"- peaks_hz: {vpeaks if vpeaks else '-'}",
            f"- note: {verr if verr else 'ok'}",
            "",
            "## IMU modal test (micro-excitation, simplified)",
            f"- peaks_hz: {mpeaks if mpeaks else '-'}",
            f"- note: {merr if merr else 'ok'}",
            "",
            "## Artifacts",
            f"- summary_json: {payload.get('summary_json', '-')}",
            f"- report_md: {payload.get('report_md', '-')}",
            f"- axis_summary_json: {(payload.get('artifacts') or {}).get('axis_summary_json', '-')}",
            f"- axis_report_md: {(payload.get('artifacts') or {}).get('axis_report_md', '-')}",
            f"- axis_samples_csv: {(payload.get('artifacts') or {}).get('axis_samples_csv', '-')}",
            f"- axis_plot_png: {(payload.get('artifacts') or {}).get('axis_plot_png', '-')}",
            "",
        ]
    )
    report_md.write_text("\n".join(lines), encoding="utf-8")
    return str(summary_json), str(report_md)


def build_failed_self_test_payload(run_dir: str, message: str) -> dict[str, Any]:
    payload = {
        "result": "TEST FAILED",
        "message": message,
        "axes": {},
        "run_dir": run_dir,
        "dynamic_response_peaks_hz": [],
        "dynamic_response_error": "",
        "dynamic_response": None,
        "imu_vibration_peaks_hz": [],
        "imu_vibration_error": "",
        "imu_modal_peaks_hz": [],
        "imu_modal_error": "",
        "artifacts": {},
    }
    payload["summary_json"] = str(Path(run_dir) / "summary.json")
    payload["report_md"] = str(Path(run_dir) / "report.md")
    summary_json, report_md = _write_canonical_outputs(run_dir, payload)
    payload["summary_json"] = summary_json
    payload["report_md"] = report_md
    return payload


async def run_self_test_imu(run_dir: str, emit_status: StatusEmitter = None) -> dict[str, Any]:
    axis_out = await run_axis_direction_test(
        url="wss://127.0.0.1:8557",
        insecure=True,
        home_settle=3.0,
        imu_settle=5.0,
        pose_settle=2.0,
        transient_seconds=6.0,
        sample_seconds=4.0,
        rate=20.0,
        profile="RTR3",
        vel_deg_s=30.0,
        out_dir=Path(run_dir),
        stem="self_test_axes",
        mount_config=DEFAULT_MOUNT_CFG,
        self_test=True,
        emit_status=emit_status,
        emit_final_completed=False,
    )

    axis_summary = axis_out["summary"]
    poses = list(axis_summary.get("poses", []))
    axis_pairs = {
        "base": ("base_pos", "base_neg"),
        "spalla": ("spalla_pos", "spalla_neg"),
        "gomito": ("gomito_pos", "gomito_neg"),
        "yaw": ("yaw_pos", "yaw_neg"),
        "pitch": ("pitch_pos", "pitch_neg"),
        "roll": ("roll_pos", "roll_neg"),
    }
    axis_status: dict[str, dict[str, object]] = {}
    tool_conclusions = dict(axis_summary.get("axis_conclusions", {}))
    for axis_name, pair in axis_pairs.items():
        rows = [p for p in poses if p.get("pose_name") in pair]
        if rows:
            axis_status[axis_name] = _classify_self_test_axis(axis_name, rows, str(tool_conclusions.get(axis_name, "")))

    result = _self_test_global_result(axis_status)
    message = _self_test_display_message(result, str(axis_summary.get("conclusion", "")))
    d_peaks = list(axis_summary.get("dynamic_response_peaks_hz") or [])
    d_err = ""
    if not d_peaks and bool(axis_summary.get("dynamic_response")):
        segs = (axis_summary.get("dynamic_response") or {}).get("per_pose") or []
        if segs and all(not (s.get("frequencies_hz") or []) for s in segs):
            d_err = "dynamic response: no PSD peaks in C_phase_move segments (IMU rate / move duration)"

    await _maybe_emit_status(emit_status, "Recording IMU vibration (experimental, ~12s)...")
    v_peaks: list[float] = []
    v_err = ""
    try:
        v_peaks, v_err_note = await measure_vibration_peaks_hz()
        if v_err_note:
            v_err = v_err_note
    except Exception as ex:
        v_err = str(ex)

    m_peaks: list[float] = []
    m_err = ""
    try:
        m_peaks, m_err_note = await run_imu_modal_test(emit_status)
        if m_err_note:
            m_err = m_err_note
    except Exception as ex:
        m_err = str(ex)

    payload: dict[str, Any] = {
        "result": result,
        "message": message,
        "axes": axis_status,
        "run_dir": run_dir,
        "dynamic_response_peaks_hz": d_peaks,
        "dynamic_response_error": d_err,
        "dynamic_response": axis_summary.get("dynamic_response"),
        "imu_vibration_peaks_hz": v_peaks,
        "imu_vibration_error": v_err,
        "imu_modal_peaks_hz": m_peaks,
        "imu_modal_error": m_err,
        "artifacts": {
            "axis_summary_json": axis_out["json_path"],
            "axis_report_md": axis_out["report_path"],
            "axis_samples_csv": axis_out["csv_path"],
            "axis_plot_png": axis_out["plot_path"],
        },
    }
    payload["summary_json"] = str(Path(run_dir) / "summary.json")
    payload["report_md"] = str(Path(run_dir) / "report.md")
    summary_json, report_md = _write_canonical_outputs(run_dir, payload)
    payload["summary_json"] = summary_json
    payload["report_md"] = report_md
    return payload
