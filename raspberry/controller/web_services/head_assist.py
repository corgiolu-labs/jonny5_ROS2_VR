"""
HEAD OVERFLOW ASSIST — mode=5 (ex IK MODE).

Polso Y/P/R: solo pipeline HEAD sul firmware; qui si leggono i margini dai limiti
routing_config e si assiste B/S/G con incrementi morbidi verso il centro polso.

NOTA OPERATIVA (thesis freeze 2026-05-17)
=========================================
Questo modulo implementa la variante **rate-based** dell'ASSIST mode, attivata
quando ``routing_config.assistMode == "rate"``. La variante attualmente
selezionata per default in produzione è invece quella **position-based DLS**
in ``head_assist_dls.py`` (selezionata da ``assistMode == "dls"``).

Il modulo è mantenuto come:
  1. legacy fallback rapidamente riattivabile in caso di anomalia sulla
     pipeline DLS;
  2. raccolta di utility condivise (``relief_signed``, ``parse_head_assist_cfg``,
     ``HeadAssistState``, mappa limiti) importate sia da ``ws_server.py`` sia
     da ``head_assist_dls.py``.

Non rimuovere senza prima ripulire le dipendenze in ``head_assist_dls.py``
e in ``ws_server.py``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
import math
from math import copysign

import numpy as np
from controller.web_services import ik_solver, poe_params_manager, settings_manager

logger = logging.getLogger("ws_teleop")

_LIMIT_ORDER = ("base", "spalla", "gomito", "yaw", "pitch", "roll")

# Reference reach used to normalise the yaw→BASE kinematic scaling.
# At this reach (metres) the gain is unchanged; below it BASE is amplified,
# above it BASE is attenuated so the wrist centre moves at a consistent
# Cartesian speed regardless of arm extension.
_REACH_REF_M: float = 0.15

# ============================================================
# A/B TEST FLAG — invert GOMITO delta in ASSIST pitch remap only.
# Localized experiment: verify whether inverting dg (GOMITO) improves
# vertical wrist-centre tracking. Default False (no change).
# Toggled manually for A/B sessions; meant to be reverted after experiment.
# Scope: compute_head_motion_follow_physical only. NO OTHER EFFECT.
# ============================================================
TEST_INVERT_ELBOW: bool = False

# Rate-limited debug log counter (shared state across all assist ticks).
_ASSIST_DEBUG_LOG_EVERY_N: int = 25   # ~250 ms @ 100 Hz assist rate
_assist_debug_log_counter: int = 0

# Upper cap on the reach scaling factor. Measured extreme-workspace tests
# (verify_assist_extremes.py, 2026-04-19) showed that at very folded poses
# (reach_xy ≈ 0.046 m, "ALL DOWN") the raw 1/reach_xy term produced scale
# factors up to 3.0×, making ALL DOWN ~24% more responsive than ALL UP
# (reach_xy ≈ 0.063 m, scale 2.4×) on the yaw channel and inflating the
# coupling toward the wrist ROLL servo. Capping scale at the ALL UP level
# flattens the asymmetry (ALL UP ↔ ALL DOWN) to ≤5% without changing the
# response at poses where reach_xy ≥ _REACH_REF_M / _REACH_SCALE_CAP
# (≈0.063 m). At more extended configurations the cap is inactive and the
# scaling law (1/reach_xy) operates unchanged.
_REACH_SCALE_CAP: float = 2.4


def relief_signed(
    angle: float,
    mn: float,
    mx: float,
    warn_margin_deg: float,
    crit_margin_deg: float,
) -> tuple[float, int]:
    """
    Ritorna (relief, zona) con relief in [-1, 1] circa:
      >0 spinge l'angolo verso l'alto (via dal minimo), <0 verso il basso (via dal max).
    zona: 0=libera, 1=warning, 2=critica.
    warn_margin_deg / crit_margin_deg: distanza (gradi) dal limite oltre cui si attiva
    (es. warn=12 -> sotto min+12 siamo ancora in warning).
    """
    if mx <= mn + 1e-6:
        return 0.0, 0
    warn_margin_deg = max(1e-3, float(warn_margin_deg))
    crit_margin_deg = max(1e-3, min(float(crit_margin_deg), warn_margin_deg - 1e-3))

    m_lo = float(angle) - float(mn)
    m_hi = float(mx) - float(angle)

    if m_lo <= m_hi:
        if m_lo >= warn_margin_deg:
            return 0.0, 0
        if m_lo > crit_margin_deg:
            t = (warn_margin_deg - m_lo) / max(warn_margin_deg - crit_margin_deg, 1e-6)
            return 0.42 * t, 1
        t = (crit_margin_deg - m_lo) / max(crit_margin_deg, 1e-6)
        return 0.42 + 0.58 * min(1.0, t), 2

    if m_hi >= warn_margin_deg:
        return 0.0, 0
    if m_hi > crit_margin_deg:
        t = (warn_margin_deg - m_hi) / max(warn_margin_deg - crit_margin_deg, 1e-6)
        return -0.42 * t, 1
    t = (crit_margin_deg - m_hi) / max(crit_margin_deg, 1e-6)
    return -0.42 - 0.58 * min(1.0, t), 2


def _f(cfg: dict, path: list[str], default: float) -> float:
    d: object = cfg
    for k in path:
        if not isinstance(d, dict):
            return default
        d = d.get(k)  # type: ignore[assignment]
    try:
        return float(d)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


def _axis_pair(cfg: dict, name: str) -> tuple[float, float]:
    block = cfg.get(name) if isinstance(cfg.get(name), dict) else {}
    assert isinstance(block, dict)
    warn = max(2.0, _f(block, ["warnDeg"], 12.0))
    crit = max(0.5, _f(block, ["critDeg"], 5.0))
    if crit >= warn:
        crit = max(0.5, warn - 1.0)
    return warn, crit


def parse_head_assist_cfg(raw: dict | None) -> dict:
    """Normalizza headAssist dopo merge_vr_config_with_defaults."""
    r = raw if isinstance(raw, dict) else {}
    yaw_w, yaw_c = _axis_pair(r, "yaw")
    pitch_w, pitch_c = _axis_pair(r, "pitch")
    roll_w, roll_c = _axis_pair(r, "roll")
    en = r.get("assistEnable") if isinstance(r.get("assistEnable"), dict) else {}
    assert isinstance(en, dict)
    split = r.get("pitchSplit") if isinstance(r.get("pitchSplit"), dict) else {}
    assert isinstance(split, dict)
    rs = r.get("rollSplit") if isinstance(r.get("rollSplit"), dict) else {}
    assert isinstance(rs, dict)
    sp = float(split.get("spalla", 0.55))
    sg = float(split.get("gomito", 0.45))
    ssum = sp + sg
    if ssum < 1e-6:
        sp, sg = 0.55, 0.45
    else:
        sp, sg = sp / ssum, sg / ssum
    rsp = float(rs.get("spalla", 0.5))
    rsg = float(rs.get("gomito", 0.5))
    rsum = rsp + rsg
    if rsum < 1e-6:
        rsp, rsg = 0.5, 0.5
    else:
        rsp, rsg = rsp / rsum, rsg / rsum

    return {
        "enabled": bool(r.get("enabled", True)),
        "yawWarn": yaw_w,
        "yawCrit": yaw_c,
        "pitchWarn": pitch_w,
        "pitchCrit": pitch_c,
        "rollWarn": roll_w,
        "rollCrit": roll_c,
        "assistYaw": bool(en.get("yaw", True)),
        "assistPitch": bool(en.get("pitch", True)),
        "assistRoll": bool(en.get("roll", False)),
        "signYaw": -1.0 if int(r.get("signYaw", 1)) == -1 else 1.0,
        "signPitch": -1.0 if int(r.get("signPitch", 1)) == -1 else 1.0,
        "signRoll": -1.0 if int(r.get("signRoll", 1)) == -1 else 1.0,
        "gainBase": max(0.0, _f(r, ["gainBase"], 0.48)),
        "gainSpalla": max(0.0, _f(r, ["gainSpalla"], 0.36)),
        "gainGomito": max(0.0, _f(r, ["gainGomito"], 0.30)),
        "gainRollArm": max(0.0, _f(r, ["gainRollArm"], 0.10)),
        "critGainMul": max(1.0, _f(r, ["critGainMul"], 1.75)),
        "pitchSplitSpalla": sp,
        "pitchSplitGomito": sg,
        "rollSplitSpalla": rsp,
        "rollSplitGomito": rsg,
        "assistAlpha": max(0.02, min(0.98, _f(r, ["assistAlpha"], 0.30))),
        "freeFollowAlpha": max(0.02, min(0.98, _f(r, ["freeFollowAlpha"], 0.16))),
        "maxStepDegPerTick": max(0.05, _f(r, ["maxStepDegPerTick"], 1.32)),
        "reliefDeadband": max(0.0, _f(r, ["reliefDeadband"], 0.015)),
        "releaseGraceSec": max(0.0, _f(r, ["releaseGraceMs"], 220.0) / 1000.0),
        "armReliefGainMul": max(1.0, _f(r, ["armReliefGainMul"], 2.2)),
        "armAssistAlphaMul": max(1.0, _f(r, ["armAssistAlphaMul"], 2.0)),
        "minSpallaStepDeg": max(0.0, _f(r, ["minSpallaStepDeg"], 0.45)),
        "minGomitoStepDeg": max(0.0, _f(r, ["minGomitoStepDeg"], 0.35)),
        "headMotionFollow": bool(r.get("headMotionFollow", True)),
        "headMotionDeadbandDeg": max(0.0, _f(r, ["headMotionDeadbandDeg"], 0.18)),
        "headMotionGainYaw": max(0.0, _f(r, ["headMotionGainYaw"], 1.0)),
        "headMotionGainPitch": max(0.0, _f(r, ["headMotionGainPitch"], 1.25)),
        "headMotionMaxStepDegPerTick": max(0.05, _f(r, ["headMotionMaxStepDegPerTick"], max(1.8, _f(r, ["maxStepDegPerTick"], 1.32)))),
        "pitchReachEnabled": bool(r.get("pitchReachEnabled", True)),
        "pitchReachBias": max(0.0, min(1.0, _f(r, ["pitchReachBias"], 0.82))),
    }


@dataclass
class HeadAssistState:
    filt_b: float | None = None
    filt_s: float | None = None
    filt_g: float | None = None
    last_arm_physical: list[int] | None = None
    target_id: int = 0
    log_counter: int = 0
    last_ts: float = 0.0
    assist_latched: bool = False
    last_raw_grip_ts: float = 0.0
    grace_active: bool = False
    last_head_rpy_deg: tuple[float, float, float] | None = None


def _clamp(x: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, x))


def _apply_min_signed_step(value: float, floor_abs: float) -> float:
    if floor_abs <= 0.0 or abs(value) < 1e-9 or abs(value) >= floor_abs:
        return value
    return copysign(floor_abs, value)


def _quat_to_rpy_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def _wrap_delta_deg(value: float) -> float:
    x = float(value)
    while x > 180.0:
        x -= 360.0
    while x < -180.0:
        x += 360.0
    return x


def _joint_limits_dict(limits: dict) -> dict[str, dict[str, int]]:
    out: dict[str, dict[str, int]] = {
        "base": {"min": 20, "max": 160},
        "spalla": {"min": 20, "max": 160},
        "gomito": {"min": 20, "max": 160},
        "yaw": {"min": 20, "max": 160},
        "pitch": {"min": 30, "max": 140},
        "roll": {"min": 30, "max": 140},
    }
    if not isinstance(limits, dict):
        return out
    for joint in _LIMIT_ORDER:
        row = limits.get(joint)
        if not isinstance(row, dict):
            continue
        try:
            mn = int(row.get("min", out[joint]["min"]))
            mx = int(row.get("max", out[joint]["max"]))
        except (TypeError, ValueError):
            continue
        if 0 <= mn < mx <= 180:
            out[joint] = {"min": mn, "max": mx}
    return out


def _axis_point_from_screw(screw_row: np.ndarray) -> np.ndarray | None:
    w = np.asarray(screw_row[:3], dtype=float).reshape(3)
    v = np.asarray(screw_row[3:], dtype=float).reshape(3)
    wn = float(np.dot(w, w))
    if wn <= 1e-9:
        return None
    return np.cross(w, v) / wn


def _shoulder_origin_m() -> np.ndarray:
    try:
        screws, _ = poe_params_manager.get_screws_m_numpy()
        if len(screws) >= 2:
            q = _axis_point_from_screw(np.asarray(screws[1], dtype=float))
            if q is not None and np.all(np.isfinite(q)):
                return np.asarray(q, dtype=float).reshape(3)
    except Exception:
        pass
    return np.array([0.0, 0.0, 0.094], dtype=float)


def _physical_limits_to_virtual(
    lim: dict[str, dict[str, int]],
    offsets: list[float],
    dirs: list[int],
) -> tuple[list[float], list[float]]:
    out_min: list[float] = []
    out_max: list[float] = []
    for i, joint in enumerate(_LIMIT_ORDER):
        off = float(offsets[i])
        dir_i = int(dirs[i]) if i < len(dirs) else 1
        dir_i = -1 if dir_i < 0 else 1
        mn_p = float(lim[joint]["min"])
        mx_p = float(lim[joint]["max"])
        v1 = (mn_p - off) / float(dir_i) + 90.0
        v2 = (mx_p - off) / float(dir_i) + 90.0
        out_min.append(min(v1, v2))
        out_max.append(max(v1, v2))
    return out_min, out_max


def _wrist_center_from_physical(
    physical_six: list[float],
    offsets: list[float],
    dirs: list[int],
) -> tuple[np.ndarray, np.ndarray]:
    virt = np.asarray(
        settings_manager.physical_to_virtual(list(physical_six), list(offsets), list(dirs)),
        dtype=float,
    ).reshape(6)
    q_math_deg = virt - 90.0
    T = np.asarray(ik_solver.forward_kinematics_poe(q_math_deg), dtype=float).reshape(4, 4)
    p_tool = np.asarray(T[:3, 3], dtype=float).reshape(3)
    R_tool = np.asarray(T[:3, :3], dtype=float).reshape(3, 3)
    p_wc = p_tool - (R_tool @ np.array([0.06, 0.0, 0.0], dtype=float))
    return virt, p_wc


def _pitch_reach_remap_sg(
    *,
    physical_six: list[float],
    lim: dict[str, dict[str, int]],
    ha: dict,
    pitch_relief: float,
    ds_pitch: float,
    dg_pitch: float,
) -> tuple[float, float] | None:
    if not ha.get("pitchReachEnabled", True):
        return None

    v_static = np.array([float(ds_pitch), float(dg_pitch)], dtype=float)
    if float(np.linalg.norm(v_static)) <= 1e-6:
        return None

    try:
        cfg = settings_manager.load()
        offsets = list(cfg.get("offsets", settings_manager.DEFAULTS["offsets"]))
        dirs = list(cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1])))

        _, p_wc = _wrist_center_from_physical(list(physical_six), offsets, dirs)
        shoulder = _shoulder_origin_m()
        radius_xz = np.array(
            [float(p_wc[0] - shoulder[0]), float(p_wc[2] - shoulder[2])],
            dtype=float,
        )
        radius_norm = float(np.linalg.norm(radius_xz))
        if radius_norm <= 1e-6:
            return None

        tangent = np.array([radius_xz[1], -radius_xz[0]], dtype=float) / radius_norm

        phys_base = [float(v) for v in physical_six]
        jac_cols: list[np.ndarray] = []
        for phys_idx, joint_name in ((1, "spalla"), (2, "gomito")):
            base_val = float(phys_base[phys_idx])
            mn = float(lim[joint_name]["min"])
            mx = float(lim[joint_name]["max"])
            plus_val = min(mx, base_val + 1.0)
            minus_val = max(mn, base_val - 1.0)
            if abs(plus_val - minus_val) <= 1e-6:
                return None

            plus_pose = list(phys_base)
            minus_pose = list(phys_base)
            plus_pose[phys_idx] = plus_val
            minus_pose[phys_idx] = minus_val

            _, p_wc_plus = _wrist_center_from_physical(plus_pose, offsets, dirs)
            _, p_wc_minus = _wrist_center_from_physical(minus_pose, offsets, dirs)
            deriv = (
                np.array(
                    [float(p_wc_plus[0] - p_wc_minus[0]), float(p_wc_plus[2] - p_wc_minus[2])],
                    dtype=float,
                )
                * 1000.0
                / float(plus_val - minus_val)
            )
            jac_cols.append(deriv)

        J = np.column_stack(jac_cols)
        if J.shape != (2, 2):
            return None

        cart_mag_mm = float(np.linalg.norm(J @ v_static))
        if cart_mag_mm <= 1e-3:
            return None

        q_forward = np.linalg.lstsq(J, tangent * cart_mag_mm, rcond=None)[0]
        q_backward = np.linalg.lstsq(J, -tangent * cart_mag_mm, rcond=None)[0]
        d_forward = J @ q_forward
        d_backward = J @ q_backward

        # Scegli il verso in base all'effetto cartesiano locale sul wrist-center:
        # - vicino al limite basso di pitch (look up / polso "alto") vogliamo favorire
        #   la risalita del wrist-center
        #   -> preferiamo il candidato con dz piu' positivo
        # - vicino al limite alto di pitch (look down / polso "basso") vogliamo favorire
        #   la discesa del wrist-center
        #   -> preferiamo il candidato con dz piu' negativo
        if float(pitch_relief) > 0.0:
            v_reach = q_forward if float(d_forward[1]) >= float(d_backward[1]) else q_backward
        else:
            v_reach = q_forward if float(d_forward[1]) <= float(d_backward[1]) else q_backward

        bias = float(ha.get("pitchReachBias", 0.82))
        v_blend = ((1.0 - bias) * v_static) + (bias * v_reach)
        return float(v_blend[0]), float(v_blend[1])
    except Exception as exc:
        logger.debug("[HEAD-ASSIST] pitch reach remap fallback: %s", exc)
        return None


def _head_pitch_follow_remap_sg(
    *,
    physical_six: list[float],
    lim: dict[str, dict[str, int]],
    pitch_cmd: float,
    ds_pitch: float,
    dg_pitch: float,
    max_joint_step_deg: float,
) -> tuple[float, float] | None:
    """
    Remap del follow pitch della testa in spazio cartesiano locale del wrist-center.

    Mantiene la magnitudine della split statica, ma impone un verso verticale
    coerente del wrist-center:
      pitch_cmd < 0 -> dz < 0
      pitch_cmd > 0 -> dz > 0
    """
    v_static = np.array([float(ds_pitch), float(dg_pitch)], dtype=float)
    if abs(float(pitch_cmd)) <= 1e-6 or float(np.linalg.norm(v_static)) <= 1e-6:
        return None

    try:
        cfg = settings_manager.load()
        offsets = list(cfg.get("offsets", settings_manager.DEFAULTS["offsets"]))
        dirs = list(cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1])))

        phys_base = [float(v) for v in physical_six]
        jac_cols: list[np.ndarray] = []
        for phys_idx, joint_name in ((1, "spalla"), (2, "gomito")):
            base_val = float(phys_base[phys_idx])
            mn = float(lim[joint_name]["min"])
            mx = float(lim[joint_name]["max"])
            plus_val = min(mx, base_val + 1.0)
            minus_val = max(mn, base_val - 1.0)
            if abs(plus_val - minus_val) <= 1e-6:
                return None

            plus_pose = list(phys_base)
            minus_pose = list(phys_base)
            plus_pose[phys_idx] = plus_val
            minus_pose[phys_idx] = minus_val

            _, p_wc_plus = _wrist_center_from_physical(plus_pose, offsets, dirs)
            _, p_wc_minus = _wrist_center_from_physical(minus_pose, offsets, dirs)
            deriv = (
                np.array(
                    [float(p_wc_plus[0] - p_wc_minus[0]), float(p_wc_plus[2] - p_wc_minus[2])],
                    dtype=float,
                )
                * 1000.0
                / float(plus_val - minus_val)
            )
            jac_cols.append(deriv)

        J = np.column_stack(jac_cols)
        if J.shape != (2, 2):
            return None

        cart_mag_mm = float(np.linalg.norm(J @ v_static))
        if cart_mag_mm <= 1e-3:
            return None

        desired_cart = np.array([0.0, math.copysign(cart_mag_mm, float(pitch_cmd))], dtype=float)
        q_vertical = np.linalg.lstsq(J, desired_cart, rcond=None)[0]
        d_vertical = J @ q_vertical
        if abs(float(d_vertical[1])) <= 1e-6:
            return None
        if float(d_vertical[1]) * float(pitch_cmd) < 0.0:
            return None
        target_peak = max(1e-6, min(float(max_joint_step_deg), float(np.max(np.abs(v_static)))))
        q_peak = max(float(abs(q_vertical[0])), float(abs(q_vertical[1])), 1e-6)
        if q_peak > target_peak:
            q_vertical = q_vertical * (target_peak / q_peak)
        _, p_wc_0 = _wrist_center_from_physical(phys_base, offsets, dirs)
        phys_next = list(phys_base)
        phys_next[1] += float(q_vertical[0])
        phys_next[2] += float(q_vertical[1])
        _, p_wc_1 = _wrist_center_from_physical(phys_next, offsets, dirs)
        dz_mm = float((p_wc_1[2] - p_wc_0[2]) * 1000.0)
        if abs(dz_mm) <= 1e-3:
            return None
        if dz_mm * float(pitch_cmd) < 0.0:
            return None
        return float(q_vertical[0]), float(q_vertical[1])
    except Exception as exc:
        logger.debug("[HEAD-ASSIST] head pitch follow remap fallback: %s", exc)
        return None


def compute_arm_assist_physical(
    *,
    physical_six: list[float],
    lim: dict[str, dict[str, int]],
    ha: dict,
) -> tuple[float, float, float, float]:
    """
    Ritorna (delta_b, delta_s, delta_g, mag_relief) in gradi fisici;
    mag_relief = |r_y|+|r_p|+|r_r| (prima del deadband).
    """
    py = float(physical_six[3])
    pp = float(physical_six[4])
    pr = float(physical_six[5])

    r_y, z_y = 0.0, 0
    if ha["assistYaw"]:
        r_y, z_y = relief_signed(py, lim["yaw"]["min"], lim["yaw"]["max"], ha["yawWarn"], ha["yawCrit"])
    r_p, z_p = 0.0, 0
    if ha["assistPitch"]:
        r_p, z_p = relief_signed(pp, lim["pitch"]["min"], lim["pitch"]["max"], ha["pitchWarn"], ha["pitchCrit"])
    r_r, z_r = 0.0, 0
    if ha["assistRoll"]:
        r_r, z_r = relief_signed(pr, lim["roll"]["min"], lim["roll"]["max"], ha["rollWarn"], ha["rollCrit"])

    db = (
        ha["signYaw"]
        * r_y
        * ha["gainBase"]
        * (ha["critGainMul"] if z_y >= 2 else 1.0)
    )
    mul_p = ha["critGainMul"] if z_p >= 2 else 1.0
    ds_pitch = ha["signPitch"] * r_p * ha["gainSpalla"] * mul_p * ha["pitchSplitSpalla"]
    dg_pitch = ha["signPitch"] * r_p * ha["gainGomito"] * mul_p * ha["pitchSplitGomito"]
    mul_r = ha["critGainMul"] if z_r >= 2 else 1.0
    ds_roll = ha["signRoll"] * r_r * ha["gainRollArm"] * mul_r * ha["rollSplitSpalla"]
    dg_roll = ha["signRoll"] * r_r * ha["gainRollArm"] * mul_r * ha["rollSplitGomito"]

    mag_r = abs(r_y) + abs(r_p) + abs(r_r)
    if mag_r < ha["reliefDeadband"]:
        db = ds_pitch = dg_pitch = ds_roll = dg_roll = 0.0
    else:
        arm_mul = ha["armReliefGainMul"]
        ds_pitch *= arm_mul
        dg_pitch *= arm_mul
        ds_roll *= arm_mul
        dg_roll *= arm_mul

    reach_remap = None
    if abs(r_p) > 1e-6 and (abs(ds_pitch) > 1e-6 or abs(dg_pitch) > 1e-6):
        reach_remap = _pitch_reach_remap_sg(
            physical_six=physical_six,
            lim=lim,
            ha=ha,
            pitch_relief=r_p,
            ds_pitch=ds_pitch,
            dg_pitch=dg_pitch,
        )
    if reach_remap is not None:
        ds_pitch, dg_pitch = reach_remap

    ds = ds_pitch + ds_roll
    dg = dg_pitch + dg_roll

    mstep = ha["maxStepDegPerTick"]
    db = _clamp(db, -mstep, mstep)
    ds = _clamp(ds, -mstep, mstep)
    dg = _clamp(dg, -mstep, mstep)
    return db, ds, dg, mag_r


def compute_head_motion_follow_physical(
    *,
    physical_six: list[float],
    lim: dict[str, dict[str, int]],
    ha: dict,
    head_delta_rpy_deg: tuple[float, float, float] | None,
) -> tuple[float, float, float, float]:
    if not ha.get("headMotionFollow", True) or head_delta_rpy_deg is None:
        return 0.0, 0.0, 0.0, 0.0

    _roll_d, pitch_d, yaw_d = (float(v) for v in head_delta_rpy_deg)
    dead = float(ha["headMotionDeadbandDeg"])
    hy = yaw_d if abs(yaw_d) >= dead else 0.0
    hp = pitch_d if abs(pitch_d) >= dead else 0.0

    # Yaw → BASE with kinematic reach scaling.
    # The BASE joint is the azimuth axis: a delta of Δθ_base rotates the wrist
    # centre through an arc of reach_xy * Δθ_base_rad.  Scaling db by
    # REACH_REF / reach_xy keeps the wrist-centre Cartesian speed constant
    # across different arm configurations (extended vs folded).
    db = ha["signYaw"] * hy * ha["gainBase"] * ha["headMotionGainYaw"]
    if abs(db) > 1e-6:
        try:
            cfg = settings_manager.load()
            offsets = list(cfg.get("offsets", settings_manager.DEFAULTS["offsets"]))
            dirs = list(cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1])))
            _, p_wc = _wrist_center_from_physical(list(physical_six), offsets, dirs)
            shoulder = _shoulder_origin_m()
            reach_xy_m = float(np.linalg.norm(
                [float(p_wc[0]) - float(shoulder[0]), float(p_wc[1]) - float(shoulder[1])]
            ))
            db *= min(_REACH_SCALE_CAP, _REACH_REF_M / max(reach_xy_m, 0.05))
        except Exception:
            pass  # keep unscaled db on FK failure

    # Pitch → SPALLA + GOMITO via local Jacobian (see _head_pitch_follow_remap_sg).
    # Head roll is NOT mapped to arm joints: roll of the headset controls the
    # wrist ROLL servo (HEAD pipeline), not the arm configuration.
    hp_drive = -float(hp)
    pitch_cmd = ha["signPitch"] * hp_drive
    ds = pitch_cmd * ha["gainSpalla"] * ha["pitchSplitSpalla"] * ha["headMotionGainPitch"]
    dg = pitch_cmd * ha["gainGomito"] * ha["pitchSplitGomito"] * ha["headMotionGainPitch"]
    follow_remap = None
    if abs(hp) > 1e-6 and (abs(ds) > 1e-6 or abs(dg) > 1e-6):
        follow_remap = _head_pitch_follow_remap_sg(
            physical_six=physical_six,
            lim=lim,
            pitch_cmd=pitch_cmd,
            ds_pitch=ds,
            dg_pitch=dg,
            max_joint_step_deg=float(ha["headMotionMaxStepDegPerTick"]),
        )
    if follow_remap is not None:
        ds, dg = follow_remap
    elif abs(hp) > 1e-6:
        # Local Jacobian could not produce the correct vertical direction;
        # suppress rather than apply the old static mapping that could invert.
        ds = 0.0
        dg = 0.0

    # A/B TEST: optional elbow sign inversion (default False, no-op).
    # Applied AFTER remap & static fallback so it flips whichever value
    # is going to be sent — covers both paths uniformly.
    if TEST_INVERT_ELBOW:
        dg = -dg

    mstep = float(ha["headMotionMaxStepDegPerTick"])
    db = _clamp(db, -mstep, mstep)
    ds = _clamp(ds, -mstep, mstep)
    dg = _clamp(dg, -mstep, mstep)
    mag = abs(hy) + abs(hp)

    # Rate-limited debug log (~4 Hz). Disabled via counter; zero cost when
    # no pitch delta is active (the typical idle case). Safe to keep even
    # after experiment: emits only during active assist.
    global _assist_debug_log_counter
    if abs(hp) > 1e-6 or abs(hy) > 1e-6:
        _assist_debug_log_counter += 1
        if _assist_debug_log_counter % _ASSIST_DEBUG_LOG_EVERY_N == 0:
            try:
                # Reach + scale diagnostic (compute without modifying state)
                cfg = settings_manager.load()
                offsets = list(cfg.get("offsets", settings_manager.DEFAULTS["offsets"]))
                dirs = list(cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1,1,1,1,1,1])))
                _, p_wc = _wrist_center_from_physical(list(physical_six), offsets, dirs)
                shoulder = _shoulder_origin_m()
                reach_xy = float(np.linalg.norm(
                    [float(p_wc[0]) - float(shoulder[0]), float(p_wc[1]) - float(shoulder[1])]
                ))
                raw_scale = _REACH_REF_M / max(reach_xy, 0.05)
                effective_scale = min(_REACH_SCALE_CAP, raw_scale)
                used_remap = (follow_remap is not None)
                logger.info(
                    "[ASSIST DEBUG] pitch_delta=%+.2f yaw_delta=%+.2f ds=%+.3f dg=%+.3f db=%+.3f "
                    "reach_xy=%.3f raw_scale=%.2f eff_scale=%.2f remap=%s invert_elbow=%s",
                    float(hp), float(hy), float(ds), float(dg), float(db),
                    reach_xy, raw_scale, effective_scale,
                    "Jac" if used_remap else ("static" if abs(hp) > 1e-6 else "none"),
                    TEST_INVERT_ELBOW,
                )
            except Exception:
                pass

    return db, ds, dg, mag


def _resolve_assist_active(
    *,
    raw_grip_active: bool,
    state: HeadAssistState,
    ha: dict,
    now: float,
) -> bool:
    if raw_grip_active:
        if not state.assist_latched:
            logger.info("[HEAD-ASSIST] engaged raw deadman")
        state.assist_latched = True
        state.last_raw_grip_ts = now
        state.grace_active = False
        return True

    if state.assist_latched:
        grace_s = ha["releaseGraceSec"]
        elapsed = max(0.0, now - state.last_raw_grip_ts)
        if elapsed <= grace_s:
            if not state.grace_active:
                logger.info("[HEAD-ASSIST] grace hold %.0fms after raw deadman drop", elapsed * 1000.0)
                state.grace_active = True
            return True
        logger.info("[HEAD-ASSIST] released after %.0fms grace window", grace_s * 1000.0)

    state.assist_latched = False
    state.grace_active = False
    return False


def step_mode5_head_assist(
    *,
    raw_grip_active: bool,
    physical_six: list[float] | None,
    head_rpy_deg: tuple[float, float, float] | None,
    limits_src: dict,
    ha: dict,
    state: HeadAssistState,
    now: float,
) -> tuple[list[int] | None, bool, bool, int]:
    """
    Un tick della modalità assistita. Ritorna:
      (arm_physical [B,S,G] o None, grip_active, hold_active, target_id)
    """
    lim = _joint_limits_dict(limits_src)
    grip_active = _resolve_assist_active(
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

    if not physical_six or len(physical_six) < 6:
        hold = state.last_arm_physical is not None
        return state.last_arm_physical, True, hold, state.target_id

    b0 = float(physical_six[0])
    s0 = float(physical_six[1])
    g0 = float(physical_six[2])

    if state.filt_b is None:
        state.filt_b, state.filt_s, state.filt_g = b0, s0, g0
        state.last_head_rpy_deg = head_rpy_deg
        state.target_id = (state.target_id + 1) & 0xFFFF
        state.last_ts = now
        arm = [int(round(b0)), int(round(s0)), int(round(g0))]
        state.last_arm_physical = arm
        return arm, True, False, state.target_id

    head_delta = None
    if head_rpy_deg is not None and state.last_head_rpy_deg is not None:
        pr, pp, py = state.last_head_rpy_deg
        cr, cp, cy = head_rpy_deg
        head_delta = (
            _wrap_delta_deg(cr - pr),
            _wrap_delta_deg(cp - pp),
            _wrap_delta_deg(cy - py),
        )
    state.last_head_rpy_deg = head_rpy_deg

    db_rel, ds_rel, dg_rel, mag_r = compute_arm_assist_physical(
        physical_six=physical_six,
        lim=lim,
        ha=ha,
    )
    db_head, ds_head, dg_head, mag_h = compute_head_motion_follow_physical(
        physical_six=physical_six,
        lim=lim,
        ha=ha,
        head_delta_rpy_deg=head_delta,
    )

    db = db_rel + db_head
    ds = ds_rel + ds_head
    dg = dg_rel + dg_head

    alpha = ha["assistAlpha"]
    arm_alpha = min(0.98, max(alpha, alpha * ha["armAssistAlphaMul"]))
    motion_active = mag_h > 0.0
    assist_active = mag_r >= ha["reliefDeadband"] or motion_active
    if assist_active:
        visible_step_floor = 0.51 / max(arm_alpha, 1e-6)
        ds = _apply_min_signed_step(ds, max(ha["minSpallaStepDeg"], visible_step_floor))
        dg = _apply_min_signed_step(dg, max(ha["minGomitoStepDeg"], visible_step_floor))
        mstep = max(float(ha["maxStepDegPerTick"]), float(ha["headMotionMaxStepDegPerTick"]))
        ds = _clamp(ds, -mstep, mstep)
        dg = _clamp(dg, -mstep, mstep)
        db = _clamp(db, -mstep, mstep)

    t_b = _clamp(b0 + db, lim["base"]["min"], lim["base"]["max"])
    t_s = _clamp(s0 + ds, lim["spalla"]["min"], lim["spalla"]["max"])
    t_g = _clamp(g0 + dg, lim["gomito"]["min"], lim["gomito"]["max"])

    f_b = float(state.filt_b)
    f_s = float(state.filt_s)
    f_g = float(state.filt_g)

    if not assist_active:
        ff = ha["freeFollowAlpha"]
        f_b = ff * b0 + (1.0 - ff) * f_b
        f_s = ff * s0 + (1.0 - ff) * f_s
        f_g = ff * g0 + (1.0 - ff) * f_g
    else:
        f_b = alpha * t_b + (1.0 - alpha) * f_b
        f_s = arm_alpha * t_s + (1.0 - arm_alpha) * f_s
        f_g = arm_alpha * t_g + (1.0 - arm_alpha) * f_g

    f_b = _clamp(f_b, lim["base"]["min"], lim["base"]["max"])
    f_s = _clamp(f_s, lim["spalla"]["min"], lim["spalla"]["max"])
    f_g = _clamp(f_g, lim["gomito"]["min"], lim["gomito"]["max"])

    state.filt_b, state.filt_s, state.filt_g = f_b, f_s, f_g
    state.last_ts = now

    arm = [int(round(f_b)), int(round(f_s)), int(round(f_g))]
    state.last_arm_physical = arm
    state.target_id = (state.target_id + 1) & 0xFFFF

    state.log_counter += 1
    if state.log_counter % 25 == 0:
        logger.info(
            "[HEAD-ASSIST] arm=%s d=(%.3f,%.3f,%.3f) head=(%.3f,%.3f,%.3f) mag_r=%.3f mag_h=%.3f",
            arm,
            db,
            ds,
            dg,
            db_head,
            ds_head,
            dg_head,
            mag_r,
            mag_h,
        )

    return arm, True, False, state.target_id
