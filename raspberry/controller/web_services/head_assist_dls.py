"""
head_assist_dls.py — position-based ASSIST via Damped Least Squares IK.

Alternative to rate-based head_assist.step_mode5_head_assist. Selected at
runtime via routing_config.json -> "assistMode": "dls" (default "rate").

Design:
  - Same signature as step_mode5_head_assist -> drop-in in
    ws_server._process_head_assist_mode dispatch.
  - Same HeadAssistState for grip/grace continuity.
  - Wrist (Y/P/R) untouched -> caller routes these via firmware HEAD pipe.
  - Target mapping (relative): identity head quat -> target = HOME_ee (no motion).
      target = HOME_ee + gainM * (head_dir_base - forward_base_at_home)
  - DLS one-shot resolved-rate per frame:
      dq = J^T (J J^T + lambda^2 I)^-1 * dx
  - Adaptive lambda (Nakamura/Hanafusa): lambda^2 = lambdaMax^2 * max(0, 1 - manip/thr)^2.

Offline prototype validated in prototype_dls_arm_ik.py (this repo root):
  identity_hold -> err=0 mm, step=0 deg.
  pitch_sine_10_0.4Hz -> err_2nd_half=21 mm, no phase lag.
  extreme configurations saturate cleanly at joint limits, no NaN/explosions.
"""
import logging
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

from controller.web_services import head_assist as _ha
from controller.web_services import ik_solver
from controller.web_services import settings_manager

logger = logging.getLogger("head_assist_dls")

_FORWARD_BASE = np.array([1.0, 0.0, 0.0], dtype=float)
_HOME_EE_M: np.ndarray | None = None  # lazy cache


def _home_ee_m() -> np.ndarray:
    global _HOME_EE_M
    if _HOME_EE_M is None:
        T = np.asarray(ik_solver.forward_kinematics_poe([0.0] * 6), dtype=float)
        _HOME_EE_M = T[:3, 3].copy()
    return _HOME_EE_M


def _head_quat_to_direction(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    Rh = R.from_quat([qx, qy, qz, qw]).as_matrix()
    return Rh @ _FORWARD_BASE


def _target_from_head_quat(head_wxyz, gain_m: float) -> np.ndarray:
    qw, qx, qy, qz = head_wxyz
    d = _head_quat_to_direction(qw, qx, qy, qz)
    return _home_ee_m() + float(gain_m) * (d - _FORWARD_BASE)


def parse_assist_dls_cfg(raw: dict | None) -> dict:
    """Parse routing_config 'assistDls' sub-key with defaults + clamps."""
    r = raw if isinstance(raw, dict) else {}

    def _f(key: str, dflt: float) -> float:
        try:
            return float(r.get(key, dflt))
        except (TypeError, ValueError):
            return float(dflt)

    # Defaults validated on the live robot (tuning campaign 2026-04-21,
    # see ai/reports/DLS_ASSIST_TUNING_SUMMARY_*.md). Keep manipThresh at
    # 1e-3 to prevent the damping-unlock / pitch_down-excursion anomaly
    # observed at manipThresh=5e-4 and gainM>=0.12.
    return {
        "gainM":           max(0.02, min(0.50, _f("gainM", 0.15))),
        "lambdaMax":       max(0.001, min(1.0, _f("lambdaMax", 0.12))),
        "manipThresh":     max(1e-6, _f("manipThresh", 1e-3)),
        "maxDqDegPerTick": max(0.1, _f("maxDqDegPerTick", 2.0)),
        "maxDxMmPerTick":  max(1.0, _f("maxDxMmPerTick", 15.0)),
        # Null-space bias toward q=0 (HOME joints). Needed because HOME is
        # kinematically singular for 3-DoF position IK: multiple joint configs
        # yield EE at HOME_ee. Without this, joint state drifts into
        # near-limit configurations even when EE tracks HOME correctly.
        "nullSpaceGain":   max(0.0, min(1.0, _f("nullSpaceGain", 0.20))),
    }


def _fk_arm_pos(q6_math_rad: np.ndarray) -> np.ndarray:
    T = np.asarray(ik_solver.forward_kinematics_poe(np.degrees(q6_math_rad)), dtype=float)
    return T[:3, 3]


def _jacobian_pos_arm(q6_math_rad: np.ndarray, eps: float = 1e-5) -> np.ndarray:
    p0 = _fk_arm_pos(q6_math_rad)
    J = np.zeros((3, 3), dtype=float)
    for i in range(3):
        qp = q6_math_rad.copy()
        qp[i] += eps
        J[:, i] = (_fk_arm_pos(qp) - p0) / eps
    return J


def _dls_step(
    q3_rad: np.ndarray,
    q456_rad: np.ndarray,
    p_target_m: np.ndarray,
    cfg: dict,
    joint_lo_rad: np.ndarray,
    joint_hi_rad: np.ndarray,
) -> tuple[np.ndarray, dict]:
    q6 = np.zeros(6, dtype=float)
    q6[:3] = q3_rad
    q6[3:] = q456_rad

    p_cur = _fk_arm_pos(q6)
    J = _jacobian_pos_arm(q6)

    dx = p_target_m - p_cur
    max_dx = float(cfg["maxDxMmPerTick"]) / 1000.0
    mag = float(np.linalg.norm(dx))
    if mag > max_dx:
        dx = dx * (max_dx / mag)

    JJT = J @ J.T
    manip = math.sqrt(max(0.0, float(np.linalg.det(JJT))))
    thr = float(cfg["manipThresh"])
    lam_max = float(cfg["lambdaMax"])
    if manip < thr:
        k = 1.0 - manip / thr
        lam_sq = (lam_max * k) ** 2
    else:
        lam_sq = 0.0

    I3 = np.eye(3)
    try:
        J_pinv_dls = J.T @ np.linalg.solve(JJT + lam_sq * I3, I3)
    except np.linalg.LinAlgError:
        J_pinv_dls = J.T @ np.linalg.solve(JJT + (lam_max ** 2) * I3, I3)
    dq_primary = J_pinv_dls @ dx

    # Null-space projection: moves q toward q_pref = 0 (HOME) without
    # disturbing EE position. P_null = I - J_pinv_dls @ J projects into
    # the null-space of the Cartesian task.
    ns_gain = float(cfg.get("nullSpaceGain", 0.0))
    if ns_gain > 0.0:
        P_null = I3 - J_pinv_dls @ J
        dq_null = ns_gain * (P_null @ (-q3_rad))
    else:
        dq_null = np.zeros(3, dtype=float)

    dq = dq_primary + dq_null

    max_dq = math.radians(float(cfg["maxDqDegPerTick"]))
    dq = np.clip(dq, -max_dq, max_dq)
    q_next = np.clip(q3_rad + dq, joint_lo_rad, joint_hi_rad)

    info = {
        "manip": manip,
        "lambda_sq": lam_sq,
        "dx_cart_mm": mag * 1000.0,
        "err_target_mm": float(np.linalg.norm(_fk_arm_pos(
            np.concatenate([q_next, q456_rad])
        ) - p_target_m) * 1000.0),
    }
    return q_next, info


def _load_offsets_dirs() -> tuple[list[float], list[int]]:
    cfg = settings_manager.load()
    offsets = list(cfg.get("offsets", settings_manager.DEFAULTS["offsets"]))
    dirs = list(cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1] * 6)))
    return offsets, dirs


_debug_log_counter = 0
_DEBUG_LOG_EVERY_N = 25  # ~0.5 Hz at 50 Hz tick rate


def step_dls_head_assist(
    *,
    raw_grip_active: bool,
    physical_six: list[float] | None,
    head_quat_wxyz: tuple[float, float, float, float] | None,
    limits_src: dict,
    ha: dict,
    ha_dls: dict,
    state: _ha.HeadAssistState,
    now: float,
) -> tuple[list[int] | None, bool, bool, int]:
    """Per-tick position-based ASSIST via DLS. Same return shape as step_mode5_head_assist."""
    lim = _ha._joint_limits_dict(limits_src)
    grip_active = _ha._resolve_assist_active(
        raw_grip_active=raw_grip_active,
        state=state,
        ha=ha,
        now=now,
    )

    if not grip_active:
        state.filt_b = state.filt_s = state.filt_g = None
        state.last_head_rpy_deg = None
        state.last_ts = 0.0
        hold = state.last_arm_physical is not None
        return state.last_arm_physical, False, hold, state.target_id

    if not physical_six or len(physical_six) < 6 or head_quat_wxyz is None:
        hold = state.last_arm_physical is not None
        return state.last_arm_physical, True, hold, state.target_id

    try:
        offsets, dirs = _load_offsets_dirs()
        virt = np.asarray(
            settings_manager.physical_to_virtual(list(physical_six), offsets, dirs),
            dtype=float,
        ).reshape(6)
    except Exception:
        hold = state.last_arm_physical is not None
        return state.last_arm_physical, True, hold, state.target_id

    q_math_rad = np.radians(virt - 90.0)
    q3_cur = q_math_rad[:3].copy()
    q456 = q_math_rad[3:].copy()

    lo_deg = np.array(
        [lim["base"]["min"], lim["spalla"]["min"], lim["gomito"]["min"]],
        dtype=float,
    )
    hi_deg = np.array(
        [lim["base"]["max"], lim["spalla"]["max"], lim["gomito"]["max"]],
        dtype=float,
    )
    lo_rad = np.radians(lo_deg - 90.0)
    hi_rad = np.radians(hi_deg - 90.0)

    target_m = _target_from_head_quat(head_quat_wxyz, ha_dls["gainM"])
    q3_next, info = _dls_step(q3_cur, q456, target_m, ha_dls, lo_rad, hi_rad)

    virt_next = virt.copy()
    virt_next[0] = float(np.clip(math.degrees(q3_next[0]) + 90.0, lim["base"]["min"],   lim["base"]["max"]))
    virt_next[1] = float(np.clip(math.degrees(q3_next[1]) + 90.0, lim["spalla"]["min"], lim["spalla"]["max"]))
    virt_next[2] = float(np.clip(math.degrees(q3_next[2]) + 90.0, lim["gomito"]["min"], lim["gomito"]["max"]))

    phys_next = settings_manager.virtual_to_physical(list(virt_next), offsets, dirs)
    arm = [
        int(round(float(phys_next[0]))),
        int(round(float(phys_next[1]))),
        int(round(float(phys_next[2]))),
    ]
    state.last_arm_physical = arm
    state.target_id = (state.target_id + 1) & 0xFFFF
    state.last_ts = now

    global _debug_log_counter
    _debug_log_counter += 1
    if _debug_log_counter % _DEBUG_LOG_EVERY_N == 0:
        try:
            logger.info(
                "[HEAD-ASSIST-DLS] target_mm=%s cur_virt=[%.1f,%.1f,%.1f] next_virt=[%.1f,%.1f,%.1f] "
                "manip=%.5f lam2=%.5f err_target=%.1fmm dx=%.1fmm",
                [round(float(v) * 1000.0, 1) for v in target_m],
                float(virt[0]), float(virt[1]), float(virt[2]),
                float(virt_next[0]), float(virt_next[1]), float(virt_next[2]),
                info["manip"], info["lambda_sq"],
                info["err_target_mm"], info["dx_cart_mm"],
            )
        except Exception:
            pass

    return arm, True, False, state.target_id
