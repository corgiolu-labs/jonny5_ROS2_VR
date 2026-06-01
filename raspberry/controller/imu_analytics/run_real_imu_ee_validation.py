#!/usr/bin/env python3
"""
Canonical real-world IMU vs EE validation procedure.

Sequence:
1) Bring robot to HOME and wait motion complete.
2) Wait mechanical settle.
3) Capture IMU zero/reference quaternion in HOME.
4) Wait IMU settle.
5) Execute one or more test poses.
6) For each pose, wait motion complete, settle, sample telemetry, and compute
   orientation error using quaternion-relative comparison.
"""

import argparse
import asyncio
import csv
import json
import ssl
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R
import websockets

from controller.imu_analytics import validate_imu_vs_ee as val


DEFAULT_OUT_DIR = val.RPI5 / "logs" / "imu_validation"

POSE_KEYS = {
    "BASE": 0,
    "B": 0,
    "SPALLA": 1,
    "SHOULDER": 1,
    "S": 1,
    "GOMITO": 2,
    "ELBOW": 2,
    "G": 2,
    "YAW": 3,
    "Y": 3,
    "PITCH": 4,
    "P": 4,
    "ROLL": 5,
    "R": 5,
}


@dataclass
class TelemetrySample:
    t_s: float
    servo_deg: list[float]
    q_imu_xyzw: np.ndarray
    q_pred_rel_xyzw: np.ndarray
    q_imu_rel_xyzw: np.ndarray
    theta_err_deg: float
    yaw_err_deg: float
    pitch_err_deg: float
    roll_err_deg: float
    theta_err_identity_deg: float | None


def _mean_quat_xyzw(quats: list[np.ndarray]) -> np.ndarray:
    q_ref = np.array(quats[0], dtype=float)
    q_ref /= np.linalg.norm(q_ref)
    acc = np.zeros(4, dtype=float)
    for q in quats:
        qn = np.array(q, dtype=float)
        qn /= max(1e-12, np.linalg.norm(qn))
        if float(np.dot(qn, q_ref)) < 0.0:
            qn = -qn
        acc += qn
    acc /= max(1e-12, np.linalg.norm(acc))
    return acc


def _theta_stats(values: list[float]) -> dict[str, float]:
    arr = np.array(values, dtype=float)
    return {
        "mean_deg": float(statistics.fmean(values)),
        "std_deg": float(statistics.pstdev(values)) if len(values) > 1 else 0.0,
        "max_deg": float(max(values)),
        "p95_deg": float(np.percentile(arr, 95)),
    }


def _build_basic_test_set() -> list[tuple[str, list[int]]]:
    return [
        ("home_ref", [90, 90, 90, 90, 90, 90]),
        ("yaw_pos", [90, 90, 90, 120, 90, 90]),
        ("yaw_neg", [90, 90, 90, 60, 90, 90]),
        ("pitch_pos", [90, 90, 90, 90, 110, 90]),
        ("pitch_neg", [90, 90, 90, 90, 70, 90]),
        ("roll_pos", [90, 90, 90, 90, 90, 120]),
        ("roll_neg", [90, 90, 90, 90, 90, 60]),
        ("combined_1", [90, 90, 90, 115, 105, 110]),
        ("combined_2", [90, 90, 90, 65, 75, 70]),
    ]


def _parse_single_pose(spec: str) -> tuple[str, list[int]]:
    pose = [90, 90, 90, 90, 90, 90]
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"Single pose token invalid: {item}")
        key, raw = item.split("=", 1)
        idx = POSE_KEYS.get(key.strip().upper())
        if idx is None:
            raise ValueError(f"Unknown pose key: {key}")
        pose[idx] = int(round(float(raw.strip())))
    return ("single_pose", pose)


def _axis_error_means(samples: list[TelemetrySample]) -> dict[str, float]:
    return {
        "yaw_err_mean_deg": float(statistics.fmean([s.yaw_err_deg for s in samples])),
        "pitch_err_mean_deg": float(statistics.fmean([s.pitch_err_deg for s in samples])),
        "roll_err_mean_deg": float(statistics.fmean([s.roll_err_deg for s in samples])),
    }


class WSRunner:
    def __init__(self, ws):
        self.ws = ws

    async def _recv_json(self, timeout: float = 2.0) -> dict[str, Any]:
        raw = await asyncio.wait_for(self.ws.recv(), timeout=timeout)
        return json.loads(raw)

    async def send_uart_wait(self, cmd: str, timeout: float = 25.0) -> dict[str, Any]:
        await self.ws.send(json.dumps({"type": "uart", "cmd": cmd}))
        prefix = cmd.split()[0].upper()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = await self._recv_json(timeout=min(2.0, end - time.monotonic()))
            if msg.get("type") == "uart_response" and str(msg.get("cmd", "")).upper() == prefix:
                return msg
        raise TimeoutError(f"uart_response timeout for {cmd}")

    async def wait_setpose_done(self, timeout: float = 20.0) -> dict[str, Any]:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = await self._recv_json(timeout=min(2.0, end - time.monotonic()))
            if msg.get("type") == "setpose_done":
                return msg
        raise TimeoutError("setpose_done timeout")

    async def collect_telemetry(self, duration_s: float, rate_hz: float) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        end = time.monotonic() + duration_s
        period = 1.0 / max(1.0, rate_hz)
        next_t = time.monotonic()
        while time.monotonic() < end:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = await self._recv_json(timeout=min(2.0, max(0.05, remaining)))
            except (TimeoutError, asyncio.TimeoutError):
                break  # fine finestra di raccolta, restituisce i campioni acquisiti
            if msg.get("type") != "telemetry":
                continue
            now = time.monotonic()
            if now < next_t:
                continue
            next_t += period
            msg["_recv_mono"] = float(now)
            out.append(msg)
        return out


def _telemetry_to_rotations(
    msg: dict[str, Any],
    offsets: list[int],
    dirs: list[int],
    r_mount: R,
    r_world_bias: R | None = None,
) -> tuple[list[float], R, R]:
    """Turn one telemetry message into (servo_deg, r_imu_measured, r_pred_absolute).

    r_pred_absolute = R_world_bias · R_ee · R_mount

    r_world_bias defaults to identity when not supplied, preserving the historical
    single-rotation model (R_imu = R_ee · R_mount) used by callers that do not yet
    pass a world-bias. The validation layer supplies it when the optional
    imu_world_bias config is present.
    """
    servo = val._extract_joint_physical_deg(msg)
    q_imu = val._extract_imu_quat_xyzw(msg)
    if servo is None or q_imu is None:
        raise ValueError("Telemetry sample missing servo or IMU quaternion")
    r_ee = val._ee_rotation_from_servo_physical_deg(servo, offsets, dirs)
    r_imu = R.from_quat(q_imu)
    if r_world_bias is None:
        r_pred_abs = r_ee * r_mount
    else:
        r_pred_abs = r_world_bias * r_ee * r_mount
    return servo, r_imu, r_pred_abs


def _summarize_pose(
    telemetry_msgs: list[dict[str, Any]],
    offsets: list[int],
    dirs: list[int],
    r_mount: R,
    r_home_imu_zero: R,
    r_home_pred_abs: R,
    compare_identity: bool,
    r_world_bias: R | None = None,
) -> tuple[list[TelemetrySample], dict[str, Any]]:
    samples: list[TelemetrySample] = []
    for msg in telemetry_msgs:
        if not bool(msg.get("imu_valid", False)):
            continue
        servo, r_imu_raw, r_pred_abs = _telemetry_to_rotations(msg, offsets, dirs, r_mount, r_world_bias)
        r_imu_rel = r_home_imu_zero.inv() * r_imu_raw
        r_pred_rel = r_home_pred_abs.inv() * r_pred_abs
        _, theta, yaw_e, pitch_e, roll_e = val._error_metrics(r_pred_rel, r_imu_rel)
        theta_identity = None
        if compare_identity:
            r_pred_abs_id = val._ee_rotation_from_servo_physical_deg(servo, offsets, dirs)
            r_pred_rel_id = r_home_pred_abs.inv() * r_pred_abs_id
            _, theta_identity, _, _, _ = val._error_metrics(r_pred_rel_id, r_imu_rel)
        samples.append(
            TelemetrySample(
                t_s=float(msg.get("_sample_t_s", 0.0)),
                servo_deg=servo,
                q_imu_xyzw=r_imu_raw.as_quat(),
                q_pred_rel_xyzw=r_pred_rel.as_quat(),
                q_imu_rel_xyzw=r_imu_rel.as_quat(),
                theta_err_deg=theta,
                yaw_err_deg=yaw_e,
                pitch_err_deg=pitch_e,
                roll_err_deg=roll_e,
                theta_err_identity_deg=theta_identity,
            )
        )
    if not samples:
        raise RuntimeError("No valid IMU samples for pose summary")
    theta = [s.theta_err_deg for s in samples]
    summary = {
        "samples": len(samples),
        **_theta_stats(theta),
        "final_servo_deg": samples[-1].servo_deg,
    }
    if compare_identity:
        theta_id = [float(s.theta_err_identity_deg) for s in samples if s.theta_err_identity_deg is not None]
        if theta_id:
            summary["identity_baseline"] = _theta_stats(theta_id)
    return samples, summary


def _annotate_time(base_t: float, telemetry_msgs: list[dict[str, Any]]) -> None:
    for msg in telemetry_msgs:
        recv = msg.get("_recv_mono")
        if recv is not None:
            msg["_sample_t_s"] = float(recv) - base_t
        else:
            msg["_sample_t_s"] = time.monotonic() - base_t


def _write_outputs(
    out_dir: Path,
    stem: str,
    pose_rows: list[tuple[str, TelemetrySample]],
    summary: dict[str, Any],
) -> tuple[Path, Path, Path, Path, Path]:
    run_dir = out_dir / stem
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "samples.csv"
    json_path = run_dir / "summary.json"
    pose_csv_path = run_dir / "pose_summary.csv"
    report_md_path = run_dir / "report.md"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pose_name",
                "timestamp_s",
                "servo_deg_B",
                "servo_deg_S",
                "servo_deg_G",
                "servo_deg_Y",
                "servo_deg_P",
                "servo_deg_R",
                "q_pred_rel_w",
                "q_pred_rel_x",
                "q_pred_rel_y",
                "q_pred_rel_z",
                "q_imu_rel_w",
                "q_imu_rel_x",
                "q_imu_rel_y",
                "q_imu_rel_z",
                "theta_err_deg",
                "yaw_err_deg",
                "pitch_err_deg",
                "roll_err_deg",
                "theta_err_identity_deg",
            ]
        )
        for pose_name, s in pose_rows:
            q_pred_wxyz = val._quat_xyzw_to_wxyz(s.q_pred_rel_xyzw)
            q_imu_wxyz = val._quat_xyzw_to_wxyz(s.q_imu_rel_xyzw)
            w.writerow(
                [
                    pose_name,
                    f"{s.t_s:.6f}",
                    *[f"{v:.6f}" for v in s.servo_deg],
                    *[f"{v:.9f}" for v in q_pred_wxyz],
                    *[f"{v:.9f}" for v in q_imu_wxyz],
                    f"{s.theta_err_deg:.6f}",
                    f"{s.yaw_err_deg:.6f}",
                    f"{s.pitch_err_deg:.6f}",
                    f"{s.roll_err_deg:.6f}",
                    "" if s.theta_err_identity_deg is None else f"{s.theta_err_identity_deg:.6f}",
                ]
            )
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    with pose_csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "pose_name",
                "samples",
                "mean_deg",
                "std_deg",
                "p95_deg",
                "max_deg",
                "yaw_err_mean_deg",
                "pitch_err_mean_deg",
                "roll_err_mean_deg",
                "identity_mean_deg",
            ]
        )
        for pose_name, data in summary["poses"].items():
            identity_mean = ""
            if "identity_baseline" in data:
                identity_mean = f"{data['identity_baseline']['mean_deg']:.6f}"
            w.writerow(
                [
                    pose_name,
                    data["samples"],
                    f"{data['mean_deg']:.6f}",
                    f"{data['std_deg']:.6f}",
                    f"{data['p95_deg']:.6f}",
                    f"{data['max_deg']:.6f}",
                    f"{data['yaw_err_mean_deg']:.6f}",
                    f"{data['pitch_err_mean_deg']:.6f}",
                    f"{data['roll_err_mean_deg']:.6f}",
                    identity_mean,
                ]
            )

    md_lines = [
        "# IMU vs End-Effector Validation Report",
        "",
        "## Procedure",
        f"- home_settle_s: {summary['procedure']['home_settle_s']}",
        f"- imu_settle_s: {summary['procedure']['imu_settle_s']}",
        f"- pose_settle_s: {summary['procedure']['pose_settle_s']}",
        f"- sample_seconds: {summary['procedure']['sample_seconds']}",
        f"- rate_hz: {summary['procedure']['rate_hz']}",
        f"- profile: {summary['procedure']['profile']}",
        f"- vel_deg_s: {summary['procedure']['vel_deg_s']}",
        "",
        "## Per-pose Summary",
        "",
        "| pose_name | samples | mean_deg | std_deg | p95_deg | max_deg | identity_mean_deg |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for pose_name, data in summary["poses"].items():
        identity_mean = data.get("identity_baseline", {}).get("mean_deg")
        identity_str = "-" if identity_mean is None else f"{identity_mean:.4f}"
        md_lines.append(
            f"| {pose_name} | {data['samples']} | {data['mean_deg']:.4f} | {data['std_deg']:.4f} | "
            f"{data['p95_deg']:.4f} | {data['max_deg']:.4f} | {identity_str} |"
        )
    md_lines.extend(
        [
            "",
            "## Overall",
            f"- mean_deg: {summary['overall']['mean_deg']:.4f}",
            f"- std_deg: {summary['overall']['std_deg']:.4f}",
            f"- p95_deg: {summary['overall']['p95_deg']:.4f}",
            f"- max_deg: {summary['overall']['max_deg']:.4f}",
        ]
    )
    if "overall_identity_baseline" in summary:
        md_lines.extend(
            [
                "",
                "## Identity Baseline",
                f"- mean_deg: {summary['overall_identity_baseline']['mean_deg']:.4f}",
                f"- std_deg: {summary['overall_identity_baseline']['std_deg']:.4f}",
                f"- p95_deg: {summary['overall_identity_baseline']['p95_deg']:.4f}",
                f"- max_deg: {summary['overall_identity_baseline']['max_deg']:.4f}",
            ]
        )
    report_md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    return run_dir, csv_path, json_path, pose_csv_path, report_md_path


async def run(args: argparse.Namespace) -> tuple[Path, Path, dict[str, Any]]:
    cfg = val.settings_manager.load()
    offsets = cfg.get("offsets", val.settings_manager.DEFAULTS["offsets"])
    dirs = cfg.get("dirs", val.settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
    # Load mount config: prior versions of this tool passed Path directly to
    # _mount_rotation_from_config (which expects a dict) and crashed. Load the
    # dict here through the same strict/legacy path that validate_imu_vs_ee uses.
    if bool(args.compare_identity_only):
        r_mount = R.identity()
    else:
        import json as _json
        from controller.web_services import runtime_config_paths as _rcfg
        mount_path = Path(args.mount_config).resolve()
        runtime_mount_path = Path(_rcfg.get_runtime_config_path("imu_ee_mount")).resolve()
        if mount_path == runtime_mount_path:
            mount_cfg = _rcfg.load_imu_ee_mount_strict()
        else:
            if not mount_path.exists():
                raise RuntimeError(f"[imu_ee_mount] mount config mancante: {mount_path}")
            mount_cfg = _json.loads(mount_path.read_text(encoding="utf-8"))
            _rcfg.validate_imu_ee_mount_shape(mount_cfg)
        r_mount = val._mount_rotation_from_config(mount_cfg)
    # Optional world-bias rotation (validation-path only; identity if file absent).
    r_world_bias = R.identity() if bool(args.compare_identity_only) else val._world_bias_rotation()

    ssl_ctx = None
    if args.url.startswith("wss://"):
        ssl_ctx = ssl.create_default_context()
        if args.insecure:
            ssl_ctx.check_hostname = False
            ssl_ctx.verify_mode = ssl.CERT_NONE

    poses: list[tuple[str, list[int]]] = []
    if args.single_pose:
        poses.append(_parse_single_pose(args.single_pose))
    elif args.test_set == "basic":
        poses.extend(_build_basic_test_set())
    else:
        poses.extend(_build_basic_test_set())

    pose_rows: list[tuple[str, TelemetrySample]] = []
    pose_summaries: dict[str, Any] = {}
    base_t = time.monotonic()

    async with websockets.connect(args.url, ssl=ssl_ctx, ping_interval=20, ping_timeout=20) as ws:
        runner = WSRunner(ws)

        await runner.send_uart_wait("SAFE", timeout=10.0)
        en = await runner.send_uart_wait("ENABLE", timeout=35.0)
        if not bool(en.get("ok", False)):
            raise RuntimeError(f"ENABLE failed: {en}")

        home_resp = await runner.send_uart_wait("HOME", timeout=20.0)
        if not bool(home_resp.get("ok", False)):
            raise RuntimeError(f"HOME failed: {home_resp}")
        await runner.wait_setpose_done(timeout=20.0)
        await asyncio.sleep(float(args.home_settle))

        home_msgs = await runner.collect_telemetry(float(args.imu_settle), float(args.rate))
        _annotate_time(base_t, home_msgs)
        home_quats = [val._extract_imu_quat_xyzw(m) for m in home_msgs if bool(m.get("imu_valid", False))]
        home_quats = [q for q in home_quats if q is not None]
        if not home_quats:
            raise RuntimeError("No valid IMU samples while capturing HOME zero")
        q_zero_xyzw = _mean_quat_xyzw(home_quats)
        r_home_imu_zero = R.from_quat(q_zero_xyzw)

        home_last = next((m for m in reversed(home_msgs) if bool(m.get("imu_valid", False))), None)
        if home_last is None:
            raise RuntimeError("No valid HOME telemetry sample")
        _, _, r_home_pred_abs = _telemetry_to_rotations(home_last, offsets, dirs, r_mount, r_world_bias)

        for pose_name, pose_virtual in poses:
            cmd = f"SETPOSE {' '.join(str(v) for v in pose_virtual)} {int(args.vel_deg_s)} {args.profile}"
            pose_resp = await runner.send_uart_wait(cmd, timeout=20.0)
            if not bool(pose_resp.get("ok", False)):
                raise RuntimeError(f"Pose command failed for {pose_name}: {pose_resp}")
            await runner.wait_setpose_done(timeout=20.0)
            await asyncio.sleep(float(args.pose_settle))
            pose_msgs = await runner.collect_telemetry(float(args.sample_seconds), float(args.rate))
            _annotate_time(base_t, pose_msgs)
            samples, stats = _summarize_pose(
                pose_msgs,
                offsets,
                dirs,
                r_mount,
                r_home_imu_zero,
                r_home_pred_abs,
                bool(args.compare_identity),
                r_world_bias,
            )
            pose_rows.extend((pose_name, s) for s in samples)
            pose_summaries[pose_name] = {
                "target_virtual_deg": pose_virtual,
                **stats,
                **_axis_error_means(samples),
            }

        overall_theta = [s.theta_err_deg for _, s in pose_rows]
        summary = {
            "procedure": {
                "home_settle_s": float(args.home_settle),
                "imu_settle_s": float(args.imu_settle),
                "pose_settle_s": float(args.pose_settle),
                "sample_seconds": float(args.sample_seconds),
                "rate_hz": float(args.rate),
                "profile": args.profile,
                "vel_deg_s": float(args.vel_deg_s),
                "mount_config": str(args.mount_config),
                "test_set": args.test_set,
            },
            "home_zero_quat_wxyz": [float(v) for v in val._quat_xyzw_to_wxyz(q_zero_xyzw)],
            "pose_order": [name for name, _ in poses],
            "poses": pose_summaries,
            "overall": _theta_stats(overall_theta),
        }
        if args.compare_identity:
            id_theta = [float(s.theta_err_identity_deg) for _, s in pose_rows if s.theta_err_identity_deg is not None]
            if id_theta:
                summary["overall_identity_baseline"] = _theta_stats(id_theta)

        stem = args.stem or time.strftime("real_imu_ee_validation_%Y%m%d_%H%M%S")
        run_dir = Path(args.out_dir) / stem
        summary["artifacts"] = {
            "run_dir": str(run_dir),
            "samples_csv": str(run_dir / "samples.csv"),
            "summary_json": str(run_dir / "summary.json"),
            "pose_summary_csv": str(run_dir / "pose_summary.csv"),
            "report_md": str(run_dir / "report.md"),
        }
        run_dir, csv_path, json_path, pose_csv_path, report_md_path = _write_outputs(Path(args.out_dir), stem, pose_rows, summary)
        return csv_path, json_path, summary


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Run canonical real IMU vs EE validation procedure.")
    p.add_argument("--url", default="ws://127.0.0.1:8557", help="WebSocket URL")
    p.add_argument("--insecure", action="store_true", help="Disable TLS verification for wss://")
    p.add_argument("--home-settle", type=float, default=3.0, help="Extra settle after HOME and motion complete")
    p.add_argument("--imu-settle", type=float, default=5.0, help="Sampling window in HOME to capture IMU zero")
    p.add_argument("--pose-settle", type=float, default=2.0, help="Extra settle after each pose motion complete")
    p.add_argument("--sample-seconds", type=float, default=4.0, help="Acquisition duration for each pose")
    p.add_argument("--rate", type=float, default=20.0, help="Sampling rate in Hz")
    p.add_argument("--profile", default="RTR3", help="Planner profile for test poses")
    p.add_argument("--vel-deg-s", type=float, default=30.0, help="Velocity in deg/s for test poses")
    p.add_argument("--single-pose", default="", help='Single pose, e.g. "YAW=120,PITCH=90,ROLL=90"')
    p.add_argument("--test-set", default="basic", choices=["basic"], help="Named test-set to execute")
    p.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="Output directory")
    p.add_argument("--stem", default="", help="Optional output file stem")
    p.add_argument("--mount-config", default=str(val.DEFAULT_MOUNT_CFG), help="Mount config JSON path")
    p.add_argument("--use-mounted-calibration", action="store_true", help="Alias for using mount config (default behavior)")
    p.add_argument("--compare-identity", action="store_true", help="Also compute identity-mount baseline")
    p.add_argument("--compare-identity-only", action="store_true", help="Force identity mount as primary path")
    return p


def main() -> int:
    args = build_argparser().parse_args()
    csv_path, json_path, summary = asyncio.run(run(args))
    print(f"run_dir={summary['artifacts']['run_dir']}")
    print(f"csv={csv_path}")
    print(f"summary={json_path}")
    print(f"pose_summary_csv={summary['artifacts']['pose_summary_csv']}")
    print(f"report_md={summary['artifacts']['report_md']}")
    print(f"overall_mean_deg={summary['overall']['mean_deg']:.4f}")
    print(f"overall_p95_deg={summary['overall']['p95_deg']:.4f}")
    print(f"overall_max_deg={summary['overall']['max_deg']:.4f}")
    for pose_name, data in summary["poses"].items():
        print(
            f"pose={pose_name} mean_deg={data['mean_deg']:.4f} "
            f"p95_deg={data['p95_deg']:.4f} max_deg={data['max_deg']:.4f}"
        )
    if "overall_identity_baseline" in summary:
        ib = summary["overall_identity_baseline"]
        print(f"identity_mean_deg={ib['mean_deg']:.4f}")
        print(f"identity_p95_deg={ib['p95_deg']:.4f}")
        print(f"identity_max_deg={ib['max_deg']:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
