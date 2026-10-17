"""
head_assist_gimbal.py - ASSIST mode (5) as a gimbal: camera orientation in the world = head orientation, 1:1.

Why (measured 2026-10-18 on the robot with the Quest XR app, legacy "rate" assist):
  * the firmware HEAD pipeline drives the wrist from the head error with gains 2 / 3 / 1 and a 3 deg dead zone:
    a 17 deg head turn turned the camera by 54 deg;
  * the wrist follows the head per axis in the world frame while the backend moves the arm: every arm motion
    adds to the camera rotation, the arm ran into its limits (base 135, wrist pitch 120);
  * with the forearm pitched down, yawing the wrist servo tilts the horizon (camera roll 0 -> 37 deg).

What this does instead (per intent, mode 5, deadman held):
  1. engage (both grips pressed): HEADZERO is sent to the firmware and the camera orientation R_cam_ref (POE FK
     of the current joints) and the head orientation are captured;
  2. target camera orientation  R_target = R_head * R_head_ref^T * R_cam_ref   (robot base frame);
  3. wrist (yaw / pitch / roll) by exact orientation IK for the ACTUAL arm joints:
       R = Rz(q1) Ry(q2) Ry(q3) Rz(q4) Ry(q5) Rx(q6)   (POE screw axes z, y, y, z, y, x; M has no rotation)
       W = (Rz(q1) Ry(q2+q3))^T R_target  ->  ZYX: q4 = atan2(W10, W00), q5 = asin(-W20), q6 = atan2(W21, W22)
  4. arm assist: wrist yaw beyond +-BAND moves the base, wrist pitch beyond +-BAND moves the elbow (then the
     shoulder), a few deg per tick, so the wrist drifts back to its centre while it keeps the camera on target;
  5. the firmware HEAD pipeline is fed a synthetic quaternion that makes it produce exactly those wrist angles:
     it extracts per-axis twists (2*atan2(|q_i|, w)), applies signs, EMA, soft dead zone, src routing and gains
     (all read from the same config that builds SET_VR_PARAMS), so q_err is built by inverting that chain and
     sent as q_err * q_ref (q_ref = the quaternion the firmware latched after HEADZERO).
Release the grips: nothing moves (firmware), re-engage anywhere without jumps (new references).

Head POSITION (2026-10-18, intents with head_px/py/pz from the Quest XR app): standing, the user crouches or
bends and the camera goes down / forward with them. The head displacement since engage (yaw-referenced, scaled by
gimbalPosition.scale) moves the wrist centre in the arm plane (height, reach) by a 2-link IK on shoulder + elbow
(height weighted more than reach: the arm is nearly vertical, it cannot go straight down without some reach).
In that case the shoulder / elbow are driven by the position and no longer by the wrist-pitch overflow; the
base still helps the wrist yaw. Without head position (web viewer) the old pitch assist is used.

Selected with routing_config.json "assistMode": "gimbal" (other values: "rate" legacy, "dls").
"""
from __future__ import annotations

import asyncio
import logging
import math
import time

import numpy as np

from controller.web_services import head_assist as _ha
from controller.web_services import settings_manager

logger = logging.getLogger("ws_teleop")

BAND_DEG = 15.0          # wrist free band before the arm helps
ARM_RATE = 14.0          # 1/s: the overflow decays as exp(-ARM_RATE * t) (was 7: too slow on the robot)
ARM_MAX_DEG_S = 100.0    # arm assist speed cap (firmware caps spalla/gomito with tg-vel-*-head, now 90 deg/s)
# time based, so the behaviour does not depend on the intent rate (60/s from the Quest XR app, 25/s web)
WRIST_MARGIN_DEG = 1.0   # stay this far from the wrist hard limits

# head position -> wrist centre (see the module docstring)
POS_SCALE_DEFAULT = 0.7      # robot metres per head metre (crouching 30 cm lowers the camera 21 cm)
POS_WC_Z_MIN = 0.17          # m, lowest wrist centre (camera still clear of the base / table)
POS_REACH_MIN = -0.02        # m, do not lean back behind the base axis
POS_JUMP_M = 0.25            # head position jump in one intent (recenter, tracking loss): re-reference
POS_W_REACH = 0.25           # IK weight of the reach error vs the height error
POS_POSTURE_W = 2e-4         # pull towards the engage posture (m^2 / rad^2): picks the "bow forward" branch
POS_LM_DAMP = 1e-3           # Levenberg-Marquardt damping (near-straight arm = singular)

# XR (x right, y up, z back) -> robot base (x forward, y left, z up)
_C = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _quat_to_mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def cam_rotation(q_math_rad) -> np.ndarray:
    q1, q2, q3, q4, q5, q6 = q_math_rad
    return _rz(q1) @ _ry(q2 + q3) @ _rz(q4) @ _ry(q5) @ _rx(q6)


_ARM_GEOM = None


def _arm_geom():
    """(h_shoulder, L1, L2) in m from the POE screws (shoulder, elbow, wrist pitch axis heights at HOME)."""
    global _ARM_GEOM
    if _ARM_GEOM is None:
        h = [0.1343, 0.1943, 0.381225]
        try:
            from controller.web_services import poe_params_manager
            screws, _ = poe_params_manager.get_screws_m_numpy()
            pts = [_ha._axis_point_from_screw(np.asarray(screws[i], dtype=float)) for i in (1, 2, 4)]
            if all(p is not None and np.all(np.isfinite(p)) for p in pts):
                h = [float(p[2]) for p in pts]
        except Exception:
            pass
        _ARM_GEOM = (h[0], h[1] - h[0], h[2] - h[1])
    return _ARM_GEOM


def wc_rz(q2: float, q3: float) -> tuple[float, float]:
    """Wrist centre (reach along the arm plane, height) for shoulder / elbow math angles (rad, Ry: + = forward)."""
    h0, l1, l2 = _arm_geom()
    return (l1 * math.sin(q2) + l2 * math.sin(q2 + q3), h0 + l1 * math.cos(q2) + l2 * math.cos(q2 + q3))


def ik_rz(r_t: float, z_t: float, q2: float, q3: float, lo, hi, q_pref=None) -> tuple[float, float]:
    """Shoulder / elbow (rad) that bring the wrist centre to (r_t, z_t): Levenberg-Marquardt on the weighted
    position error plus a small pull towards q_pref (the posture at engage). The arm is nearly straight (a
    singular configuration for a 2-link arm): without damping and the posture term the solver jumped to the
    "shoulder back, elbow forward" branch and ran into the limits. Best effort when out of reach (height first)."""
    _, l1, l2 = _arm_geom()
    w = np.array([POS_W_REACH, 1.0])
    q = np.array([q2, q3], dtype=float)
    qp = np.array(q_pref if q_pref is not None else (q2, q3), dtype=float)
    rho = POS_POSTURE_W
    for _ in range(25):
        r, z = wc_rz(q[0], q[1])
        e = np.array([r_t - r, z_t - z]) * w
        c12, s12 = math.cos(q[0] + q[1]), math.sin(q[0] + q[1])
        j = np.array([[l1 * math.cos(q[0]) + l2 * c12, l2 * c12],
                      [-l1 * math.sin(q[0]) - l2 * s12, -l2 * s12]]) * w[:, None]
        g = j.T @ e - rho * (q - qp)
        if float(np.max(np.abs(g))) < 1e-6:
            break
        dq = np.linalg.solve(j.T @ j + (rho + POS_LM_DAMP) * np.eye(2), g)
        q = np.clip(q + np.clip(dq, -0.08, 0.08), lo, hi)
    return float(q[0]), float(q[1])


def bow_for_height(z_t: float, q2r: float, q3r: float, lo, hi) -> tuple[float, float]:
    """"Bow": shoulder and elbow bend forward together (q2 = q2r + s, q3 = q3r + s) until the wrist centre is at
    height z_t, like a person lowering the head. s is limited by the joint limits and by a straight-up forearm
    (q2 + q3 >= 0), where the height is maximum; in that range the height decreases with s, so bisection."""
    s_lo = max(lo[0] - q2r, lo[1] - q3r, -(q2r + q3r) / 2.0)
    s_hi = min(hi[0] - q2r, hi[1] - q3r)
    if s_hi <= s_lo:
        return q2r, q3r

    def z_of(sv):
        return wc_rz(q2r + sv, q3r + sv)[1]

    if z_t >= z_of(s_lo):
        sv = s_lo
    elif z_t <= z_of(s_hi):
        sv = s_hi
    else:
        a, b = s_lo, s_hi
        for _ in range(30):
            m = 0.5 * (a + b)
            if z_of(m) > z_t:
                a = m
            else:
                b = m
        sv = 0.5 * (a + b)
    return q2r + sv, q3r + sv


def wrist_pitch_roll_for_yaw(q_arm_rad, r_target, q4) -> tuple[float, float]:
    """Wrist pitch / roll when the wrist yaw is held at q4 (yaw at its limit): the camera keeps the WORLD pitch and
    roll of the target (optical axis elevation, level horizon) and only stops turning in yaw. With the forearm
    tilted the wrist yaw axis is not vertical, so the full-yaw solution clipped in yaw tilted the camera by up to
    25 deg in pitch / roll. A few Gauss-Newton steps on (pitch, roll) from the plain decomposition."""
    q1, q2, q3 = q_arm_rad
    base = _rz(q1) @ _ry(q2 + q3) @ _rz(q4)
    m = base.T @ r_target
    x = np.array([math.asin(max(-1.0, min(1.0, -m[2, 0]))), math.atan2(m[2, 1], m[2, 2])])
    _, tp, tr = _rot_to_ypr(r_target)

    def err(v):
        _, p, r = _rot_to_ypr(base @ _ry(v[0]) @ _rx(v[1]))
        return np.array([p - tp, math.atan2(math.sin(r - tr), math.cos(r - tr))])

    h = 1e-4
    for _ in range(6):
        e = err(x)
        if abs(e[0]) + abs(e[1]) < 1e-5:
            break
        j = np.column_stack([(err(x + d) - e) / h for d in (np.array([h, 0.0]), np.array([0.0, h]))])
        try:
            dx = np.linalg.solve(j.T @ j + 1e-6 * np.eye(2), j.T @ e)
        except np.linalg.LinAlgError:
            break
        x = x - np.clip(dx, -0.5, 0.5)
    return float(x[0]), float(x[1])


def wrist_ik(q_arm_rad, r_target) -> tuple[float, float, float]:
    q1, q2, q3 = q_arm_rad
    w = (_rz(q1) @ _ry(q2 + q3)).T @ r_target
    q5 = math.asin(max(-1.0, min(1.0, -w[2, 0])))
    q4 = math.atan2(w[1, 0], w[0, 0])
    q6 = math.atan2(w[2, 1], w[2, 2])
    return q4, q5, q6


# ---------------------------------------------------------------------------------------------------------------
# firmware HEAD pipeline model (j5vr_head.c) - parameters as sent with SET_VR_PARAMS
# ---------------------------------------------------------------------------------------------------------------

class HeadPipe:
    """gains (yaw, pitch, roll servo), sensitivity, dead zone, signs per head axis (yaw=z, pitch=y, roll=x twists),
    src routing (roll servo <- err[src_roll], pitch servo <- err[src_pitch], yaw servo <- err[src_yaw]),
    err = [yaw_err, pitch_err, roll_err]."""

    def __init__(self, params: list[str]):
        f = [float(v) for v in params[:16]]
        i = [int(float(v)) for v in params[16:19]]
        self.gain = {"yaw": f[0], "pitch": f[1], "roll": f[2]}
        self.deadzone = f[5]
        self.sens = max(0.25, min(4.0, f[12])) if f[12] > 0 else 1.0
        self.sign = [1.0 if f[13] >= 0 else -1.0, 1.0 if f[14] >= 0 else -1.0, 1.0 if f[15] >= 0 else -1.0]
        self.src = {"roll": i[0], "pitch": i[1], "yaw": i[2]}

    def quat_for_servo_deltas(self, d_yaw: float, d_pitch: float, d_roll: float):
        """q_err (w, x, y, z) that makes the firmware move the wrist servos by these PHYSICAL degree deltas."""
        err = [0.0, 0.0, 0.0]  # [yaw_err, pitch_err, roll_err] after sign, before EMA / dead zone
        for servo, delta in (("yaw", d_yaw), ("pitch", d_pitch), ("roll", d_roll)):
            g = self.gain[servo] * self.sens
            if abs(g) < 1e-6:
                continue
            v = delta / g
            if abs(v) > 1e-6:
                v = math.copysign(abs(v) + self.deadzone, v)   # invert the soft dead zone
            err[self.src[servo]] = v
        # undo the per-axis sign, map to twists: yaw_err <- z, pitch_err <- y, roll_err <- x
        tz = math.radians(err[0] * self.sign[0])
        ty = math.radians(err[1] * self.sign[1])
        tx = math.radians(err[2] * self.sign[2])
        lim = math.radians(170.0)
        tx, ty, tz = (max(-lim, min(lim, t)) for t in (tx, ty, tz))
        kx, ky, kz = math.tan(tx / 2), math.tan(ty / 2), math.tan(tz / 2)
        w = 1.0 / math.sqrt(1.0 + kx * kx + ky * ky + kz * kz)
        return (w, w * kx, w * ky, w * kz)


# ---------------------------------------------------------------------------------------------------------------
# state machine
# ---------------------------------------------------------------------------------------------------------------

class GimbalState:
    def __init__(self):
        self.phase = "idle"          # idle -> zeroing -> track
        self.q_hold = (1.0, 0.0, 0.0, 0.0)   # quaternion sent while not tracking (constant)
        self.q_ref = (1.0, 0.0, 0.0, 0.0)    # what the firmware latched after HEADZERO
        self.zero_ok_at = 0.0
        self.zero_task = None
        self.center_phys = None      # wrist servo positions (physical) at HEADZERO
        self.r_cam_ref = None
        self.r_head_ref = None
        self.arm_v = None            # arm target, virtual deg (B, S, G)
        self.last_arm_phys = None
        self.arm_cmd_phys = None     # last arm target sent while tracking (physical, float)
        self.p_ref = None            # head position (XR, m) at engage
        self.p_last = None
        self.rz_ref = None           # wrist centre (reach, height) at engage
        self.q23_ref = None          # shoulder / elbow (rad) at engage: preferred posture for the IK
        self.dpsi = 0.0              # camera yaw - head yaw at engage (maps head displacement to the robot)
        self.pos_dbg = None
        self.target_id = 0
        self.last_t = 0.0
        self.last_log = 0.0


_state = GimbalState()


def notify_headzero_external():
    """Any HEADZERO not issued by us resets the firmware reference: re-zero at the next engage."""
    if _state.phase == "track":
        logger.info("[GIMBAL] external HEADZERO: re-engage needed")
    _state.phase = "idle"


async def _send_headzero(uart_manager):
    ok, resp = await uart_manager.send_uart_command("HEADZERO", timeout_s=1.0)
    if ok:
        _state.zero_ok_at = time.monotonic()
    else:
        logger.warning("[GIMBAL] HEADZERO failed: %s", resp)
        _state.phase = "idle"


_JOINTS = ("base", "spalla", "gomito", "yaw", "pitch", "roll")


def _virt_to_phys_f(v: float, offset, d) -> float:
    """virtual -> physical without rounding, same formula and [5, 175] clamp as settings_manager."""
    s = -1.0 if float(d) < 0 else 1.0
    return max(5.0, min(175.0, float(offset) + s * (float(v) - 90.0)))


def _virtual_limits(lim: dict, offsets, dirs):
    """Physical joint limits -> (virtual min[6], virtual max[6]); virtual = (phys - offset) / dir + 90."""
    lo, hi = [], []
    for i, j in enumerate(_JOINTS):
        d = float(dirs[i]) if i < len(dirs) and float(dirs[i]) != 0.0 else 1.0
        off = float(offsets[i]) if i < len(offsets) else 90.0
        a = (float(lim[j]["min"]) - off) / d + 90.0
        b = (float(lim[j]["max"]) - off) / d + 90.0
        lo.append(min(a, b))
        hi.append(max(a, b))
    return lo, hi


def _head_pos(intent: dict):
    try:
        p = np.array([float(intent["head_px"]), float(intent["head_py"]), float(intent["head_pz"])])
    except (KeyError, TypeError, ValueError):
        return None
    return p if np.all(np.isfinite(p)) else None


def step(intent: dict, *, physical_six, limits_src: dict, vr_params: list[str], uart_manager, grip: bool,
         pos_cfg: dict | None = None):
    """Returns (arm_physical[3] | None, grip_active, hold_active, target_id) and rewrites intent quat_* ."""
    st = _state
    lim = _ha._joint_limits_dict(limits_src)

    def send_quat(q):
        intent["quat_w"], intent["quat_x"], intent["quat_y"], intent["quat_z"] = (float(v) for v in q)

    if not grip or not physical_six or len(physical_six) < 6:
        if st.phase != "idle":
            logger.info("[GIMBAL] released")
        st.phase = "idle"
        send_quat(st.q_hold)
        return st.last_arm_phys, False, st.last_arm_phys is not None, st.target_id

    cfg = settings_manager.load()
    offsets = cfg.get("offsets", settings_manager.DEFAULTS["offsets"])
    dirs = cfg.get("dirs", settings_manager.DEFAULTS.get("dirs", [1] * 6))
    physical_six = [float(v) for v in physical_six[:6]]
    if st.phase == "track" and st.arm_cmd_phys is not None:
        # The telemetry reports the arm servos in WHOLE degrees (uint8 = round of the firmware's float position).
        # With the arm now moving smoothly, the wrist compensation computed from those values advanced in 1 deg
        # steps (camera yaw wobble of ~0.5 deg on slow turns). Best estimate of the real position: our last
        # float target, kept within +-0.5 deg of the reported value (exact when the arm keeps up, falls back to
        # the telemetry edge when the firmware rate limit leaves it behind).
        for i in range(3):
            t = physical_six[i]
            physical_six[i] = min(t + 0.5, max(t - 0.5, float(st.arm_cmd_phys[i])))
    virt = np.asarray(settings_manager.physical_to_virtual(list(physical_six), offsets, dirs), dtype=float)
    qm = np.radians(virt - 90.0)
    hw, hx, hy, hz = (float(intent.get(k, d)) for k, d in
                      (("quat_w", 1.0), ("quat_x", 0.0), ("quat_y", 0.0), ("quat_z", 0.0)))
    r_head = _C @ _quat_to_mat(hw, hx, hy, hz) @ _C.T

    if st.phase == "idle":
        # engage: freeze the firmware wrist (constant quaternion) and zero its reference
        st.phase = "zeroing"
        st.arm_cmd_phys = None
        st.zero_ok_at = 0.0
        st.zero_task = asyncio.get_event_loop().create_task(_send_headzero(uart_manager))
        st.arm_v = virt[:3].copy()
        logger.info("[GIMBAL] engage: HEADZERO, arm virt %s", [round(float(v), 1) for v in virt[:3]])
    if st.phase == "zeroing":
        send_quat(st.q_hold)
        # a couple of intents after the OK so the firmware has latched q_hold as its reference
        if st.zero_ok_at and time.monotonic() - st.zero_ok_at > 0.08:
            st.q_ref = st.q_hold
            st.center_phys = [float(physical_six[3]), float(physical_six[4]), float(physical_six[5])]
            st.r_cam_ref = cam_rotation(qm)
            st.r_head_ref = r_head
            st.p_ref = _head_pos(intent)
            st.p_last = st.p_ref
            st.rz_ref = wc_rz(qm[1], qm[2])
            st.q23_ref = (float(qm[1]), float(qm[2]))
            st.dpsi = (math.atan2(st.r_cam_ref[1, 0], st.r_cam_ref[0, 0])
                       - math.atan2(st.r_head_ref[1, 0], st.r_head_ref[0, 0]))
            st.phase = "track"
            st.last_t = 0.0
            logger.info("[GIMBAL] tracking: wrist centre phys %s", [round(float(v), 1) for v in st.center_phys])
        arm_phys = [round(float(v), 2) for v in physical_six[:3]]
        st.last_arm_phys = arm_phys
        return arm_phys, True, True, st.target_id

    # ---- track
    r_target = r_head @ st.r_head_ref.T @ st.r_cam_ref
    q4, q5, q6 = wrist_ik(qm[:3], r_target)
    w_deg = np.degrees([q4, q5, q6]) + 90.0          # wrist virtual
    # routing_config limits are PHYSICAL servo degrees (the firmware clamps there): convert them to virtual with
    # offsets / dirs. Using them as virtual was wrong for the reversed joints (pitch dir -1: physical 35..120 is
    # virtual 60..145), the IK then believed the wrist reached angles the firmware had clipped.
    v_lo, v_hi = _virtual_limits(lim, offsets, dirs)
    w_lo = [v_lo[i] + WRIST_MARGIN_DEG for i in (3, 4, 5)]
    w_hi = [v_hi[i] - WRIST_MARGIN_DEG for i in (3, 4, 5)]
    w_cmd = [float(np.clip(w_deg[i], w_lo[i], w_hi[i])) for i in range(3)]
    if abs(w_cmd[0] - w_deg[0]) > 0.05:
        # wrist yaw at its limit (and the base too, or still catching up): re-solve pitch / roll for that yaw
        p5, p6 = wrist_pitch_roll_for_yaw(qm[:3], r_target, math.radians(w_cmd[0] - 90.0))
        w_cmd[1] = float(np.clip(math.degrees(p5) + 90.0, w_lo[1], w_hi[1]))
        w_cmd[2] = float(np.clip(math.degrees(p6) + 90.0, w_lo[2], w_hi[2]))

    # arm assist: bring the wrist back inside +-BAND (virtual deg around 90)
    def overflow(v):
        d = v - 90.0
        return d - max(-BAND_DEG, min(BAND_DEG, d))

    now = time.monotonic()
    dt = min(0.1, max(0.0, now - st.last_t)) if st.last_t else 0.0
    st.last_t = now
    frac = 1.0 - math.exp(-ARM_RATE * dt)
    max_step = ARM_MAX_DEG_S * dt
    a_lo = v_lo[:3]
    a_hi = v_hi[:3]
    ov_yaw = overflow(float(w_deg[0]))
    ov_pitch = overflow(float(w_deg[1]))
    step_b = max(-max_step, min(max_step, frac * ov_yaw))
    st.arm_v[0] = float(np.clip(st.arm_v[0] + step_b, a_lo[0], a_hi[0]))
    pcfg = pos_cfg or {}
    p_now = _head_pos(intent) if pcfg.get("enable", True) else None
    st.pos_dbg = None
    if p_now is not None and st.p_ref is not None:
        # ---- head position -> wrist centre (shoulder + elbow)
        if st.p_last is not None and float(np.linalg.norm(p_now - st.p_last)) > POS_JUMP_M:
            jump = float(np.linalg.norm(p_now - st.p_last))
            st.p_ref = st.p_ref + (p_now - st.p_last)          # recenter / tracking jump: keep the arm where it is
            logger.info("[GIMBAL] head position jump %.2f m: re-referenced", jump)
        st.p_last = p_now
        scale = float(pcfg.get("scale", POS_SCALE_DEFAULT))
        d = _rz(st.dpsi) @ (_C @ (p_now - st.p_ref))          # robot frame, head forward = camera forward at engage
        q1 = qm[0]
        dr = scale * float(d[0] * math.cos(q1) + d[1] * math.sin(q1))
        dz = scale * float(d[2])
        lo = [math.radians(a_lo[1] - 90.0), math.radians(a_lo[2] - 90.0)]
        hi = [math.radians(a_hi[1] - 90.0), math.radians(a_hi[2] - 90.0)]
        # height -> "bow" posture (down and a bit forward, like a person); the head's forward displacement adds
        # reach on top of the bow, solved around it
        z_t = max(POS_WC_Z_MIN, st.rz_ref[1] + dz)
        q2b, q3b = bow_for_height(z_t, st.q23_ref[0], st.q23_ref[1], lo, hi)
        r_bow = wc_rz(q2b, q3b)[0]
        h0, l1, l2 = _arm_geom()
        r_max = math.sqrt(max(0.0, (0.98 * (l1 + l2)) ** 2 - (z_t - h0) ** 2))   # stretched arm at that height
        r_t = min(r_max, max(POS_REACH_MIN, r_bow + dr))
        if abs(dr) > 1e-4:
            q2t, q3t = ik_rz(r_t, z_t, q2b, q3b, lo, hi, q_pref=(q2b, q3b))
        else:
            q2t, q3t = q2b, q3b
        for k, qt in ((1, q2t), (2, q3t)):
            delta = math.degrees(qt) + 90.0 - st.arm_v[k]
            st.arm_v[k] = float(np.clip(st.arm_v[k] + max(-max_step, min(max_step, frac * delta)),
                                        a_lo[k], a_hi[k]))
        st.pos_dbg = (dr, dz, r_t, z_t)
    else:
        step_p = max(-max_step, min(max_step, frac * ov_pitch))
        g_new = float(np.clip(st.arm_v[2] + step_p, a_lo[2], a_hi[2]))
        rest = step_p - (g_new - st.arm_v[2])            # what the elbow could not take goes to the shoulder
        st.arm_v[2] = g_new
        st.arm_v[1] = float(np.clip(st.arm_v[1] + rest, a_lo[1], a_hi[1]))

    full_v = list(virt)
    full_v[0:3] = [float(v) for v in st.arm_v]
    full_v[3:6] = w_cmd
    # arm targets with the fraction (the SPI frame carries centidegrees): settings_manager.virtual_to_physical
    # truncates to whole degrees, which made the arm follow the head in 1 deg steps
    arm_phys = [round(_virt_to_phys_f(full_v[i], offsets[i], dirs[i]), 2) for i in range(3)]
    wrist_phys = [float(v) for v in
                  [offsets[i] + (dirs[i] if dirs[i] >= 0 else -1) * (w_cmd[i - 3] - 90.0) for i in (3, 4, 5)]]

    pipe = HeadPipe(vr_params)
    q_err = pipe.quat_for_servo_deltas(wrist_phys[0] - st.center_phys[0], wrist_phys[1] - st.center_phys[1],
                                       wrist_phys[2] - st.center_phys[2])
    q_send = _qmul(q_err, st.q_ref)
    send_quat(q_send)
    st.q_hold = q_send   # on release / re-engage the firmware keeps exactly this pose (no jump)
    st.last_arm_phys = arm_phys
    st.arm_cmd_phys = arm_phys
    st.target_id = (st.target_id + 1) & 0xFFFF

    if now - st.last_log >= 1.0:
        st.last_log = now
        fk = np.degrees(_rot_to_ypr(cam_rotation(np.radians(np.asarray(full_v) - 90.0))))
        tg = np.degrees(_rot_to_ypr(r_target))
        pos_txt = ""
        if st.pos_dbg is not None:
            dr, dz, r_t, z_t = st.pos_dbg
            r_c, z_c = wc_rz(math.radians(st.arm_v[1] - 90.0), math.radians(st.arm_v[2] - 90.0))
            pos_txt = " pos d[r z] %+.0f %+.0f mm wc tgt [%.0f %.0f] cmd [%.0f %.0f] mm" % (
                dr * 1000, dz * 1000, r_t * 1000, z_t * 1000, r_c * 1000, z_c * 1000)
        logger.info("[GIMBAL] target ypr %s cam(cmd) ypr %s wrist virt %s%s arm virt %s dt %.0f ms%s",
                    [round(float(v), 1) for v in tg], [round(float(v), 1) for v in fk],
                    [round(float(v), 1) for v in w_cmd],
                    " (clamped)" if any(abs(w_cmd[i] - w_deg[i]) > 0.05 for i in range(3)) else "",
                    [round(float(v), 1) for v in st.arm_v], dt * 1000.0, pos_txt)
    return arm_phys, True, False, st.target_id


def _rot_to_ypr(r):
    return (math.atan2(r[1, 0], r[0, 0]), math.asin(max(-1.0, min(1.0, -r[2, 0]))), math.atan2(r[2, 1], r[2, 2]))
