"""
identify_joint_signs.py — Joint-sign identification via POE model + IMU.

For each of the 6 joints, commands a small symmetric perturbation (+Δ, -Δ
virtual degrees) around HOME, measures the end-effector rotation with the
wrist-IMU, compares it against the POE forward-kinematics prediction
computed with dirs=[+1]*6 (canonical POE convention, independent of the
currently configured dirs), and classifies the sign as CORRECT / INVERTED /
INCONCLUSIVE.

Rationale
---------
The POE model in ik_solver treats q_math_i = virtual_i - 90 as a signed
rotation around screw S_i. The runtime dirs[] exists only to compensate for
physical joints that are mechanically mounted with inverted direction
relative to S_i. So the only quantity that matters is: for each joint i,
does a positive physical displacement from its offset produce a rotation
aligned with S_i (sign = +1) or opposite (sign = -1)?

By feeding the *physical servo angles actually achieved* (read from telemetry)
into the existing FK helper with dirs=[+1]*6, the prediction becomes the raw
POE answer "what rotation would a +displacement produce if the joint were
mounted in canonical direction". Comparing that to the measured IMU rotation
vector gives the answer directly, with no iteration over dirs[] candidates.

Relative rotation (R_minus^-1 · R_plus) is used as the comparison quantity:
it doubles SNR, cancels IMU quaternion drift over the short test window, and
cancels the fixed IMU-to-end-effector mount rotation (same mount applied to
both pred and meas, so it drops out of the log-map direction comparison).

Operational
-----------
python3 -m controller.imu_analytics.identify_joint_signs \
    --url wss://localhost:8443/ws --insecure \
    --delta-deg 15 --samples-per-pose 30 --rate 50 --settle 0.8 \
    --out-dir logs/imu_validation

Output
------
logs/imu_validation/joint_signs_<TS>/summary.json
logs/imu_validation/joint_signs_<TS>/report.md

This script does NOT modify j5_settings.json. It only PROPOSES a corrected
dirs[] vector; applying it remains a manual step.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import ssl
import statistics
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import numpy as np
import websockets
from scipy.spatial.transform import Rotation as R

from controller.imu_analytics import run_real_imu_ee_validation as real_run
from controller.imu_analytics import validate_imu_vs_ee as val
from controller.web_services import runtime_config_paths as rcfg
from controller.web_services import settings_manager

JOINT_NAMES = ["base", "spalla", "gomito", "yaw", "pitch", "roll"]
HOME_VIRTUAL = [90, 90, 90, 90, 90, 90]

# Observability thresholds (see Phase 1 analysis).
MIN_PRED_MAGNITUDE_RAD = 0.10  # ~5.7°, below this the prediction is too small to classify
MIN_MEAS_MAGNITUDE_RAD = 0.10  # ~5.7°, below this the robot did not actually move
CLEAR_COS_THRESHOLD = 0.80  # |cos| above this → CORRECT/INVERTED; below → INCONCLUSIVE


@dataclass
class JointSignResult:
    joint_index: int  # 1..6
    joint_name: str
    virtual_plus: list[int]
    virtual_minus: list[int]
    phys_plus: list[float]
    phys_minus: list[float]
    phi_pred: list[float]  # rotation vector (rad) in IMU frame
    phi_meas: list[float]
    pred_magnitude_rad: float
    meas_magnitude_rad: float
    cosine: float
    verdict: str  # CORRECT | INVERTED | INCONCLUSIVE
    verdict_reason: str
    confidence: float  # |cosine| if verdict clear; else 0
    samples_plus: int
    samples_minus: int
    proposed_dir: int | None  # +1 | -1 | None (when INCONCLUSIVE, keep current)
    current_dir: int


def _resolve_mount_rotation(mount_arg: str | None, ignore_mount: bool) -> R:
    """Load mount rotation matrix or return identity."""
    if ignore_mount:
        return R.identity()
    if mount_arg:
        mount_path = Path(mount_arg).resolve()
    else:
        mount_path = Path(rcfg.get_runtime_config_path("imu_ee_mount")).resolve()
    runtime_path = Path(rcfg.get_runtime_config_path("imu_ee_mount")).resolve()
    if mount_path == runtime_path:
        mount_cfg = rcfg.load_imu_ee_mount_strict()
    else:
        if not mount_path.exists():
            raise RuntimeError(f"mount config missing: {mount_path}")
        mount_cfg = json.loads(mount_path.read_text(encoding="utf-8"))
        rcfg.validate_imu_ee_mount_shape(mount_cfg)
    return val._mount_rotation_from_config(mount_cfg)


async def _go_home_and_settle(runner: real_run.WSRunner, home_settle: float) -> None:
    """HOME + settle. Raises on UART failure."""
    resp = await runner.send_uart_wait("HOME", timeout=20.0)
    if not bool(resp.get("ok", False)):
        raise RuntimeError(f"HOME failed: {resp}")
    await runner.wait_setpose_done(timeout=20.0)
    await asyncio.sleep(home_settle)


async def _setpose_and_collect(
    runner: real_run.WSRunner,
    virtual: list[int],
    *,
    settle_s: float,
    sample_s: float,
    rate_hz: float,
    vel_deg_s: int,
    profile: str,
) -> tuple[list[dict[str, Any]], float]:
    """Send a SETPOSE, wait for motion complete, settle, collect telemetry window."""
    cmd = f"SETPOSE {' '.join(str(v) for v in virtual)} {int(vel_deg_s)} {profile}"
    resp = await runner.send_uart_wait(cmd, timeout=25.0)
    if not bool(resp.get("ok", False)):
        raise RuntimeError(f"SETPOSE failed for {virtual}: {resp}")
    await runner.wait_setpose_done(timeout=25.0)
    await asyncio.sleep(settle_s)
    t0 = time.monotonic()
    msgs = await runner.collect_telemetry(sample_s, rate_hz)
    return msgs, t0


def _mean_pose_rotation(
    msgs: list[dict[str, Any]],
    offsets: list[int],
    r_mount: R,
    r_world_bias: R | None = None,
) -> tuple[R, R, list[float], int]:
    """From a telemetry window, average IMU rotation and compute POE-predicted rotation
    using dirs=[+1]*6 (canonical, ignoring runtime dirs). Returns (r_meas, r_pred, mean_phys, n).

    r_world_bias (if supplied) composes as: r_pred = R_world_bias · R_ee · R_mount.
    Defaults to identity for backward compatibility."""
    dirs_test = [1, 1, 1, 1, 1, 1]
    imu_quats: list[np.ndarray] = []
    phys_samples: list[list[float]] = []
    for msg in msgs:
        if not bool(msg.get("imu_valid", False)):
            continue
        try:
            servo, r_imu, r_pred_abs = real_run._telemetry_to_rotations(msg, offsets, dirs_test, r_mount, r_world_bias)
        except ValueError:
            continue
        imu_quats.append(r_imu.as_quat())
        phys_samples.append(list(servo))
    if not imu_quats:
        raise RuntimeError("no valid IMU samples in collection window")
    q_mean_xyzw = real_run._mean_quat_xyzw(imu_quats)
    r_imu_mean = R.from_quat(q_mean_xyzw)
    phys_arr = np.asarray(phys_samples, dtype=float)
    phys_mean = [float(x) for x in phys_arr.mean(axis=0)]
    # Predicted rotation from the mean physical angles, using canonical dirs=[+1]*6
    r_ee_pred = val._ee_rotation_from_servo_physical_deg(phys_mean, offsets, dirs_test)
    if r_world_bias is None:
        r_pred_mean = r_ee_pred * r_mount
    else:
        r_pred_mean = r_world_bias * r_ee_pred * r_mount
    return r_imu_mean, r_pred_mean, phys_mean, len(imu_quats)


def _relative_rotvec(r_minus: R, r_plus: R) -> np.ndarray:
    return (r_minus.inv() * r_plus).as_rotvec()


def _classify(
    phi_pred: np.ndarray,
    phi_meas: np.ndarray,
) -> tuple[str, str, float, float, float]:
    """Return (verdict, reason, cosine, |pred|, |meas|)."""
    mag_pred = float(np.linalg.norm(phi_pred))
    mag_meas = float(np.linalg.norm(phi_meas))
    if mag_pred < MIN_PRED_MAGNITUDE_RAD:
        return "INCONCLUSIVE", f"weak_predicted_observability (|pred|={mag_pred:.3f} rad)", 0.0, mag_pred, mag_meas
    if mag_meas < MIN_MEAS_MAGNITUDE_RAD:
        return "INCONCLUSIVE", f"weak_measured_response (|meas|={mag_meas:.3f} rad)", 0.0, mag_pred, mag_meas
    cos = float(np.dot(phi_pred, phi_meas) / (mag_pred * mag_meas))
    if cos >= CLEAR_COS_THRESHOLD:
        return "CORRECT", f"cos={cos:+.3f} >= +{CLEAR_COS_THRESHOLD}", cos, mag_pred, mag_meas
    if cos <= -CLEAR_COS_THRESHOLD:
        return "INVERTED", f"cos={cos:+.3f} <= -{CLEAR_COS_THRESHOLD}", cos, mag_pred, mag_meas
    return "INCONCLUSIVE", f"coupling_or_noise (cos={cos:+.3f} within [-{CLEAR_COS_THRESHOLD},+{CLEAR_COS_THRESHOLD}])", cos, mag_pred, mag_meas


async def _test_one_joint(
    runner: real_run.WSRunner,
    joint_idx: int,
    *,
    delta_deg: int,
    offsets: list[int],
    dirs_cur: list[int],
    r_mount: R,
    vel_deg_s: int,
    profile: str,
    home_settle: float,
    pose_settle: float,
    sample_s: float,
    rate_hz: float,
    r_world_bias: R | None = None,
) -> JointSignResult:
    """Execute the sign test on a single joint index (0..5)."""
    virt_plus = list(HOME_VIRTUAL)
    virt_plus[joint_idx] = 90 + int(delta_deg)
    virt_minus = list(HOME_VIRTUAL)
    virt_minus[joint_idx] = 90 - int(delta_deg)

    # +delta pose
    msgs_p, _ = await _setpose_and_collect(
        runner, virt_plus, settle_s=pose_settle, sample_s=sample_s,
        rate_hz=rate_hz, vel_deg_s=vel_deg_s, profile=profile,
    )
    r_imu_p, r_pred_p, phys_p, n_p = _mean_pose_rotation(msgs_p, offsets, r_mount, r_world_bias)

    # return to HOME (safety)
    await _go_home_and_settle(runner, home_settle)

    # -delta pose
    msgs_m, _ = await _setpose_and_collect(
        runner, virt_minus, settle_s=pose_settle, sample_s=sample_s,
        rate_hz=rate_hz, vel_deg_s=vel_deg_s, profile=profile,
    )
    r_imu_m, r_pred_m, phys_m, n_m = _mean_pose_rotation(msgs_m, offsets, r_mount, r_world_bias)

    # back to HOME
    await _go_home_and_settle(runner, home_settle)

    phi_pred = _relative_rotvec(r_pred_m, r_pred_p)
    phi_meas = _relative_rotvec(r_imu_m, r_imu_p)
    verdict, reason, cos, mag_pred, mag_meas = _classify(phi_pred, phi_meas)

    if verdict == "CORRECT":
        proposed = +1
    elif verdict == "INVERTED":
        proposed = -1
    else:
        proposed = None  # caller will keep current dir for the proposal

    return JointSignResult(
        joint_index=joint_idx + 1,
        joint_name=JOINT_NAMES[joint_idx],
        virtual_plus=list(virt_plus),
        virtual_minus=list(virt_minus),
        phys_plus=phys_p,
        phys_minus=phys_m,
        phi_pred=[float(x) for x in phi_pred],
        phi_meas=[float(x) for x in phi_meas],
        pred_magnitude_rad=mag_pred,
        meas_magnitude_rad=mag_meas,
        cosine=cos,
        verdict=verdict,
        verdict_reason=reason,
        confidence=abs(cos) if verdict != "INCONCLUSIVE" else 0.0,
        samples_plus=n_p,
        samples_minus=n_m,
        proposed_dir=proposed,
        current_dir=int(dirs_cur[joint_idx]),
    )


def _write_report(out_dir: Path, summary: dict[str, Any]) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_json = out_dir / "summary.json"
    report_md = out_dir / "report.md"
    summary_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    lines: list[str] = []
    lines.append("# Joint Sign Identification Report")
    lines.append("")
    lines.append(f"- timestamp_utc: {summary.get('timestamp_utc')}")
    lines.append(f"- url: {summary.get('url')}")
    lines.append(f"- delta_deg: {summary.get('delta_deg')}")
    lines.append(f"- offsets: {summary.get('offsets')}")
    lines.append(f"- dirs_current: {summary.get('dirs_current')}")
    lines.append(f"- dirs_proposed: {summary.get('dirs_proposed')}")
    lines.append(f"- dirs_confidence: {summary.get('dirs_confidence')}")
    lines.append(f"- global_verdict: {summary.get('global_verdict')}")
    lines.append("")
    lines.append("| joint | cur | proposed | verdict | cosine | |pred| rad | |meas| rad | phi_pred (x,y,z) | phi_meas (x,y,z) |")
    lines.append("|---|---:|---:|---|---:|---:|---:|---|---|")
    for r_ in summary.get("joints", []):
        phi_p = r_["phi_pred"]
        phi_m = r_["phi_meas"]
        lines.append(
            f"| {r_['joint_name']} ({r_['joint_index']}) | {r_['current_dir']:+d} | "
            f"{('+1' if r_['proposed_dir']==1 else ('-1' if r_['proposed_dir']==-1 else '?'))} | "
            f"{r_['verdict']} | {r_['cosine']:+.3f} | "
            f"{r_['pred_magnitude_rad']:.3f} | {r_['meas_magnitude_rad']:.3f} | "
            f"[{phi_p[0]:+.3f},{phi_p[1]:+.3f},{phi_p[2]:+.3f}] | "
            f"[{phi_m[0]:+.3f},{phi_m[1]:+.3f},{phi_m[2]:+.3f}] |"
        )
    lines.append("")
    lines.append("## Interpretation")
    lines.append("")
    lines.append("- CORRECT  : raw POE (dirs=+1) agrees with IMU → keep proposed_dir (=+1) or current, whichever was configured to yield this agreement.")
    lines.append("- INVERTED : raw POE disagrees sign-wise → proposed_dir is the mathematically correct setting (flip from current in dirs[]).")
    lines.append("- INCONCLUSIVE: weak observability or cross-axis coupling. Do not change dirs[] for this joint without a second diagnostic run.")
    lines.append("")
    report_md.write_text("\n".join(lines), encoding="utf-8")
    return summary_json, report_md


async def run(args: argparse.Namespace) -> dict[str, Any]:
    cfg = settings_manager.load()
    offsets = cfg.get("offsets", settings_manager.DEFAULTS["offsets"])
    dirs_cur = cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1] * 6))
    r_mount = _resolve_mount_rotation(args.mount_config, bool(args.ignore_mount))
    # Optional world-frame yaw bias (identity if absent → backward-compatible).
    r_world_bias = R.identity() if bool(args.ignore_mount) else val._world_bias_rotation()

    ssl_ctx = None
    if args.url.startswith("wss://"):
        ssl_ctx = ssl.create_default_context()
        if args.insecure:
            ssl_ctx.check_hostname = False
            ssl_ctx.verify_mode = ssl.CERT_NONE

    async with websockets.connect(args.url, ssl=ssl_ctx, ping_interval=20, ping_timeout=20) as ws:
        runner = real_run.WSRunner(ws)
        await runner.send_uart_wait("SAFE", timeout=10.0)
        en = await runner.send_uart_wait("ENABLE", timeout=35.0)
        if not bool(en.get("ok", False)):
            raise RuntimeError(f"ENABLE failed: {en}")
        await _go_home_and_settle(runner, float(args.home_settle))

        joint_results: list[JointSignResult] = []
        joints = [int(j) - 1 for j in (args.joints.split(",") if args.joints else "1,2,3,4,5,6".split(","))]
        for idx in joints:
            if not (0 <= idx <= 5):
                continue
            res = await _test_one_joint(
                runner, idx,
                delta_deg=int(args.delta_deg),
                offsets=offsets, dirs_cur=dirs_cur, r_mount=r_mount,
                vel_deg_s=int(args.vel_deg_s), profile=args.profile,
                home_settle=float(args.home_settle),
                pose_settle=float(args.pose_settle),
                sample_s=float(args.sample_seconds),
                rate_hz=float(args.rate),
                r_world_bias=r_world_bias,
            )
            joint_results.append(res)

        # Return to SAFE at the end.
        try:
            await runner.send_uart_wait("SAFE", timeout=10.0)
        except Exception:
            pass

    # Build proposed dirs[6]: take proposed_dir when not None; else keep current.
    dirs_proposed: list[int] = list(dirs_cur)
    dirs_confidence: list[float] = [0.0] * 6
    for r_ in joint_results:
        if r_.proposed_dir is not None:
            dirs_proposed[r_.joint_index - 1] = int(r_.proposed_dir)
        dirs_confidence[r_.joint_index - 1] = round(float(r_.confidence), 3)

    n_inc = sum(1 for r_ in joint_results if r_.verdict == "INCONCLUSIVE")
    n_inv = sum(1 for r_ in joint_results if r_.verdict == "INVERTED")
    if n_inc > 0:
        global_verdict = "PARTIAL"
    elif n_inv > 0:
        global_verdict = "SIGN_MISMATCH_DETECTED"
    else:
        global_verdict = "ALL_CORRECT"

    summary = {
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "url": args.url,
        "delta_deg": int(args.delta_deg),
        "offsets": list(offsets),
        "dirs_current": list(dirs_cur),
        "dirs_proposed": dirs_proposed,
        "dirs_confidence": dirs_confidence,
        "global_verdict": global_verdict,
        "joints": [asdict(r_) for r_ in joint_results],
        "config": {
            "vel_deg_s": int(args.vel_deg_s),
            "profile": args.profile,
            "home_settle_s": float(args.home_settle),
            "pose_settle_s": float(args.pose_settle),
            "sample_seconds": float(args.sample_seconds),
            "rate_hz": float(args.rate),
            "thresholds": {
                "min_pred_magnitude_rad": MIN_PRED_MAGNITUDE_RAD,
                "min_meas_magnitude_rad": MIN_MEAS_MAGNITUDE_RAD,
                "clear_cos_threshold": CLEAR_COS_THRESHOLD,
            },
        },
    }

    ts_stem = time.strftime("joint_signs_%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) / ts_stem
    summary_json, report_md = _write_report(out_dir, summary)
    summary["artifacts"] = {"summary_json": str(summary_json), "report_md": str(report_md)}
    return summary


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", required=True, help="WS endpoint, e.g. wss://localhost:8443/ws or ws://localhost:8557")
    p.add_argument("--insecure", action="store_true", help="Skip TLS verify for wss://")
    p.add_argument("--delta-deg", type=int, default=15, help="Perturbation amplitude (virtual deg), default 15")
    p.add_argument("--joints", type=str, default="1,2,3,4,5,6", help="Comma list of joint indices 1..6 to test (default all)")
    p.add_argument("--home-settle", type=float, default=1.2, help="Seconds to settle after HOME")
    p.add_argument("--pose-settle", type=float, default=0.8, help="Seconds to settle after SETPOSE before sampling")
    p.add_argument("--sample-seconds", type=float, default=0.6, help="Seconds of telemetry to average per pose")
    p.add_argument("--rate", type=float, default=50.0, help="Telemetry collection rate (Hz)")
    p.add_argument("--vel-deg-s", type=int, default=30, help="SETPOSE velocity (deg/s)")
    p.add_argument("--profile", type=str, default="S_curve", help="SETPOSE velocity profile (e.g. S_curve, linear)")
    p.add_argument("--out-dir", type=str, default="logs/imu_validation", help="Root directory for reports")
    p.add_argument("--mount-config", type=str, default=None, help="Path to imu_ee_mount.json; defaults to runtime config")
    p.add_argument("--ignore-mount", action="store_true", help="Use identity for mount rotation")
    return p


def main() -> None:
    args = _build_parser().parse_args()
    summary = asyncio.run(run(args))
    print(json.dumps({
        "global_verdict": summary["global_verdict"],
        "dirs_current": summary["dirs_current"],
        "dirs_proposed": summary["dirs_proposed"],
        "dirs_confidence": summary["dirs_confidence"],
        "artifacts": summary["artifacts"],
        "joints": [
            {"joint": j["joint_name"], "verdict": j["verdict"], "cosine": round(j["cosine"], 3),
             "current_dir": j["current_dir"], "proposed_dir": j["proposed_dir"]}
            for j in summary["joints"]
        ],
    }, indent=2))


if __name__ == "__main__":
    main()
