"""
ik_solver.py — POE Forward/Inverse Kinematics for JONNY5-4.0

FK: Product of Exponentials with screws from poe_params_manager.
IK: scipy.optimize.least_squares (TRF) on 6D or 3D residuals.

Angle convention:
  - Internal (solver): radians, 0 = HOME (servo at 90 deg physical)
  - External (WS API): degrees physical [0-180], 90 = HOME
"""

import math
import logging
import threading

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from controller.web_services import poe_params_manager as _poe_mgr
from controller.web_services import runtime_config_paths as rcfg

logger = logging.getLogger("ik_solver")

# ---------------------------------------------------------------------------
# Joint limits
# ---------------------------------------------------------------------------
JOINT_MIN_DEG = 5.0
JOINT_MAX_DEG = 175.0
JOINT_MIN_RAD = math.radians(JOINT_MIN_DEG - 90.0)
JOINT_MAX_RAD = math.radians(JOINT_MAX_DEG - 90.0)
_JOINT_NAMES = ["base", "spalla", "gomito", "yaw", "pitch", "roll"]

# Solver tuning
W_POS = 1.0 / 50.0
W_ORI = 1.0
DEFAULT_MAX_POS_ERROR_MM = 20.0
DEFAULT_MAX_ORI_ERROR_DEG = 25.0

SOLVER_POE = "POE"

# POE default screws/M (overridden at runtime by poe_params_manager)
POE_SCREWS = np.array([
    [0, 0, 1, 0,      0,     0],
    [0, 1, 0, -0.094, 0,     0],
    [0, 1, 0, -0.154, 0,     0],
    [0, 0, 1, 0,      0,     0],
    [0, 1, 0, -0.311, 0,     0],
    [1, 0, 0, 0,      0.311, 0],
], dtype=float)
POE_M = np.array([
    [1, 0, 0, 0.060],
    [0, 1, 0, 0.000],
    [0, 0, 1, 0.311],
    [0, 0, 0, 1.000],
], dtype=float)

# ---------------------------------------------------------------------------
# Seed cache (warm-start across consecutive IK calls)
# ---------------------------------------------------------------------------
_seed_lock = threading.Lock()
_seed_cache: dict[tuple, np.ndarray] = {}


def _get_cached_seed(sig: tuple | None, lo: np.ndarray, hi: np.ndarray):
    if sig is None:
        return None
    with _seed_lock:
        c = _seed_cache.get(sig)
    if c is None or np.shape(c) != (6,):
        return None
    return np.clip(np.asarray(c, dtype=float), lo, hi)


def _store_cached_seed(sig: tuple | None, q: np.ndarray):
    if sig is None:
        return
    with _seed_lock:
        _seed_cache[sig] = np.asarray(q, dtype=float)


# ---------------------------------------------------------------------------
# POE Forward Kinematics
# ---------------------------------------------------------------------------

def skew3(w):
    w = np.asarray(w, dtype=float).ravel()
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], dtype=float)


def matrix_exp6(se3):
    W = se3[:3, :3]
    v = se3[:3, 3]
    wt = np.array([W[2, 1], W[0, 2], W[1, 0]], dtype=float)
    th = float(np.linalg.norm(wt))
    T = np.eye(4, dtype=float)
    if th < 1e-12:
        T[:3, 3] = v
        return T
    Wn = W / th
    R = np.eye(3) + math.sin(th) * Wn + (1 - math.cos(th)) * (Wn @ Wn)
    G = np.eye(3) * th + (1 - math.cos(th)) * Wn + (th - math.sin(th)) * (Wn @ Wn)
    T[:3, :3] = R
    T[:3, 3] = G @ (v / th)
    return T


def _fk_poe_math(q_rad):
    """FK in math space (rad, q=0 ↔ virtual 90 deg)."""
    S, M = _poe_mgr.get_screws_m_numpy()
    q = np.asarray(q_rad, dtype=float).ravel()
    T = np.eye(4, dtype=float)
    for i in range(6):
        se3 = np.zeros((4, 4), dtype=float)
        se3[:3, :3] = skew3(S[i, :3])
        se3[:3, 3] = S[i, 3:]
        T = T @ matrix_exp6(se3 * float(q[i]))
    return T @ M


def forward_kinematics_poe(q_deg):
    """FK from math-space degrees. Returns 4x4 SE(3) in meters."""
    return _fk_poe_math(np.radians(np.asarray(q_deg, dtype=float).ravel()))


def compute_fk_poe_virtual_deg(angles_virtual) -> dict:
    """FK from virtual degrees (0-180, HOME=90). Returns pose in mm + RPY deg."""
    if not isinstance(angles_virtual, (list, tuple)) or len(angles_virtual) != 6:
        return {"ok": False, "error": "need 6 angles"}
    try:
        virt = [float(angles_virtual[i]) for i in range(6)]
    except (TypeError, ValueError):
        return {"ok": False, "error": "non-numeric"}
    q_math = np.array([v - 90.0 for v in virt], dtype=float)
    try:
        T = forward_kinematics_poe(q_math)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    p_mm = T[:3, 3] * 1000.0
    eul = Rotation.from_matrix(T[:3, :3]).as_euler("ZYX", degrees=True)
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()
    return {
        "ok": True,
        "x_mm": round(float(p_mm[0]), 3), "y_mm": round(float(p_mm[1]), 3),
        "z_mm": round(float(p_mm[2]), 3),
        "roll_deg": round(float(eul[2]), 3), "pitch_deg": round(float(eul[1]), 3),
        "yaw_deg": round(float(eul[0]), 3),
        "quat_xyzw": [round(float(quat[i]), 6) for i in range(4)],
    }


def pose_from_T(T):
    return np.asarray(T[:3, 3], dtype=float) * 1000.0, Rotation.from_matrix(T[:3, :3]).as_rotvec()


# ---------------------------------------------------------------------------
# Joint bounds
# ---------------------------------------------------------------------------

def _merge_limits(limits, mn, mx):
    if not isinstance(limits, dict):
        return
    for i, jn in enumerate(_JOINT_NAMES):
        row = limits.get(jn)
        if not isinstance(row, dict):
            continue
        try:
            lo = float(row.get("min", mn[i]))
            hi = float(row.get("max", mx[i]))
        except (TypeError, ValueError):
            continue
        if 0.0 <= lo < hi <= 180.0:
            mn[i] = lo
            mx[i] = hi


def _load_joint_limits():
    mn = [JOINT_MIN_DEG] * 6
    mx = [JOINT_MAX_DEG] * 6
    try:
        cfg = rcfg.load_routing_config_strict()
        _merge_limits(cfg.get("limits"), mn, mx)
        return mn, mx
    except Exception:
        pass
    try:
        cfg = rcfg.load_runtime_json("routing_config", default={}) or {}
        if isinstance(cfg, dict):
            _merge_limits(cfg.get("limits"), mn, mx)
    except Exception:
        pass
    return mn, mx


def _bounds(joint_min=None, joint_max=None):
    if joint_min is None or joint_max is None:
        mn, mx = _load_joint_limits()
    else:
        mn = [float(v) for v in joint_min]
        mx = [float(v) for v in joint_max]
    lo = np.array([math.radians(v - 90) for v in mn], dtype=float)
    hi = np.array([math.radians(v - 90) for v in mx], dtype=float)
    return lo, hi, mn, mx


# ---------------------------------------------------------------------------
# Residual functions
# ---------------------------------------------------------------------------

def _residual_full(q, T_target_m):
    T = _fk_poe_math(q)
    p_err = (T_target_m[:3, 3] - T[:3, 3]) * 1000.0
    o_err = Rotation.from_matrix(T_target_m[:3, :3] @ T[:3, :3].T).as_rotvec()
    return np.concatenate([W_POS * p_err, 0.3 * W_ORI * o_err])


def _run_least_squares(residual_fn, seeds, bounds, args, *, ftol=1e-8, xtol=1e-8, gtol=1e-8, max_nfev=900):
    """Run least_squares over multiple seeds, return (best_result, total_nfev)."""
    best, best_cost, nfev = None, float("inf"), 0
    for q0 in seeds:
        try:
            res = least_squares(
                residual_fn, q0, args=args, bounds=bounds, method="trf",
                ftol=ftol, xtol=xtol, gtol=gtol, max_nfev=max_nfev, verbose=0,
            )
            nfev += int(res.nfev)
            if res.cost < best_cost:
                best_cost = res.cost
                best = res
        except Exception:
            pass
    return best, nfev


def _q_to_virtual_deg(q, mn, mx):
    return [float(np.clip(math.degrees(float(q[i])) + 90, mn[i], mx[i])) for i in range(len(q))]


# ---------------------------------------------------------------------------
# Public IK solvers
# ---------------------------------------------------------------------------

def solve_numeric_poe(
    x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, *,
    reset_solver=False, max_pos_error_mm=DEFAULT_MAX_POS_ERROR_MM,
    max_ori_error_deg=DEFAULT_MAX_ORI_ERROR_DEG,
    joint_min_deg=None, joint_max_deg=None,
) -> dict:
    """Full 6-DOF IK: target pose (mm, deg) → all 6 joint angles."""
    lo, hi, mn, mx = _bounds(joint_min_deg, joint_max_deg)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYX", [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix()
    T[:3, 3] = np.array([x_mm, y_mm, z_mm]) / 1000.0

    sig = _poe_mgr.get_seed_signature()
    if reset_solver:
        seeds = [np.zeros(6, dtype=float)]
    else:
        seeds = []
        c = _get_cached_seed(sig, lo, hi)
        if c is not None:
            seeds.append(c)
        seeds.append(np.zeros(6, dtype=float))
        d = math.radians(15.0)
        seeds.append(np.clip(np.array([0, +d, -d, 0, 0, 0], dtype=float), lo, hi))
        seeds.append(np.clip(np.array([0, -d, +d, 0, 0, 0], dtype=float), lo, hi))

    best, nfev = _run_least_squares(_residual_full, seeds, (lo, hi), (T,), max_nfev=3000)

    if best is None:
        return {"angles_deg": [90.0]*6, "reachable": False,
                "error_pos": -1, "error_ori": -1, "iterations": nfev, "message": "failed"}

    q = np.asarray(best.x, dtype=float)
    Tf = forward_kinematics_poe(np.degrees(q))
    e_pos = float(np.linalg.norm((T[:3, 3] - Tf[:3, 3]) * 1000))
    e_ori = float(np.linalg.norm(Rotation.from_matrix(T[:3, :3] @ Tf[:3, :3].T).as_rotvec()))
    ok = e_pos <= max_pos_error_mm
    if ok:
        _store_cached_seed(sig, q)
    return {
        "angles_deg": _q_to_virtual_deg(q, mn, mx), "reachable": ok,
        "error_pos": round(e_pos, 3), "error_ori": round(math.degrees(e_ori), 3),
        "iterations": nfev, "message": "OK" if ok else f"err={e_pos:.1f}mm",
    }


def solve_arm_position_poe(
    x_mm, y_mm, z_mm, wrist_deg_virtual, *,
    preferred_angles_deg=None, reset_solver=False,
    max_pos_error_mm=DEFAULT_MAX_POS_ERROR_MM,
    joint_min_deg=None, joint_max_deg=None,
) -> dict:
    """3-DOF arm IK: target tool position (mm) → base/shoulder/elbow. Wrist fixed."""
    if not isinstance(wrist_deg_virtual, (list, tuple)) or len(wrist_deg_virtual) != 3:
        return {**_ARM_FAIL, "message": "need 3 wrist angles"}
    lo, hi, mn, mx = _bounds(joint_min_deg, joint_max_deg)
    q456 = np.array([math.radians(float(v) - 90) for v in wrist_deg_virtual], dtype=float)
    tgt = np.array([x_mm, y_mm, z_mm], dtype=float) / 1000.0
    seeds = _build_seeds_3dof(preferred_angles_deg, lo[:3], hi[:3], reset=reset_solver)

    best, nfev = _run_least_squares(_residual_arm_pos, seeds, (lo[:3], hi[:3]), (q456, tgt))
    if best is None:
        fw = [float(np.clip(v, mn[3+i], mx[3+i])) for i, v in enumerate(wrist_deg_virtual)]
        return {**_ARM_FAIL, "angles_deg": [90]*3 + fw, "arm_angles_deg": [90]*3,
                "iterations": nfev, "message": "failed"}

    q = np.concatenate([best.x, q456])
    e = float(np.linalg.norm((tgt - _fk_poe_math(q)[:3, 3]) * 1000))
    ok = e <= max_pos_error_mm
    a = _q_to_virtual_deg(q, mn, mx)
    return {"angles_deg": a, "arm_angles_deg": a[:3], "reachable": ok,
            "error_pos": round(e, 3), "error_ori": 0.0, "iterations": nfev,
            "message": "OK" if ok else f"err={e:.1f}mm", "solver_used": SOLVER_POE_ARM_POSITION}


def solve_arm_wrist_center_poe(
    x_mm, y_mm, z_mm, wrist_deg_virtual, *,
    preferred_angles_deg=None, reset_solver=False,
    max_pos_error_mm=DEFAULT_MAX_POS_ERROR_MM,
    joint_min_deg=None, joint_max_deg=None,
    wrist_center_to_tool_tool_m=(0.06, 0.0, 0.0),
    realtime=False,
) -> dict:
    """3-DOF arm IK on wrist-center. realtime=True: 1 seed, lax tolerances."""
    if not isinstance(wrist_deg_virtual, (list, tuple)) or len(wrist_deg_virtual) != 3:
        return {**_ARM_FAIL, "message": "need 3 wrist angles"}
    lo, hi, mn, mx = _bounds(joint_min_deg, joint_max_deg)
    q456 = np.array([math.radians(float(v) - 90) for v in wrist_deg_virtual], dtype=float)
    tgt = np.array([x_mm, y_mm, z_mm], dtype=float) / 1000.0
    wc_off = np.array(wrist_center_to_tool_tool_m, dtype=float).ravel()
    seeds = _build_seeds_3dof(preferred_angles_deg, lo[:3], hi[:3],
                               reset=reset_solver, realtime=realtime)
    kw = dict(ftol=1e-5, xtol=1e-5, gtol=1e-5, max_nfev=180) if realtime else \
         dict(ftol=1e-8, xtol=1e-8, gtol=1e-8, max_nfev=900)

    best, nfev = _run_least_squares(
        _residual_arm_wc, seeds, (lo[:3], hi[:3]), (q456, tgt, wc_off), **kw)
    if best is None:
        fw = [float(np.clip(v, mn[3+i], mx[3+i])) for i, v in enumerate(wrist_deg_virtual)]
        return {**_ARM_FAIL, "angles_deg": [90]*3 + fw, "arm_angles_deg": [90]*3,
                "iterations": nfev, "message": "failed"}

    q = np.concatenate([best.x, q456])
    Tf = _fk_poe_math(q)
    wc = Tf[:3, 3] - (Tf[:3, :3] @ wc_off)
    e = float(np.linalg.norm((tgt - wc) * 1000))
    ok = e <= max_pos_error_mm
    a = _q_to_virtual_deg(q, mn, mx)
    return {"angles_deg": a, "arm_angles_deg": a[:3], "reachable": ok,
            "error_pos": round(e, 3), "error_ori": 0.0, "iterations": nfev,
            "message": "OK" if ok else f"err={e:.1f}mm", "solver_used": SOLVER_POE_ARM_POSITION}


def solve(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, **kw) -> dict:
    """Main entry point: full 6-DOF POE IK."""
    return solve_numeric_poe(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, **kw)
