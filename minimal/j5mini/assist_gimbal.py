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

Point at an object (2026-10-18, request_point, from tools/vision_service.py via ws_server): the right trigger in
ASSIST asks the vision service for the object under the crosshair and its 3D position in the cam0 frame. The plan:
base towards the object, shoulder / elbow stretch the wrist centre along the shoulder -> object line (maximum
extension, or POINT_STANDOFF_M from the object centre when it is closer), wrist so that cam0's optical axis goes
through the object centre with a level horizon. The pose is reached with a minimum-jerk interpolation (deadman
held: releasing the grips stops it, like every motion in mode 5); then the references are re-latched at the new
pose and the head keeps steering from there without a jump.

Head motion prediction (2026-10-18, routing_config "gimbalPrediction"): the IMU campaign measured ~200-250 ms
between the firmware servo targets and the real wrist motion (command -> firmware only 41 ms). The target camera
orientation is computed from the head orientation PREDICTED lead_s ahead: angular velocity of the head (finite
difference over PRED_WIN_S of the intent arrival times, smoothed), applied for lead_s, faded out below a few deg/s
(no amplified micro-motion while holding still) and capped at max_deg (no big overshoot when the head stops).

Spring on yaw stops (2026-10-18, user): turning the head and stopping, base + wrist yaw went past and came back.
Two causes, both handled: (1) the prediction kept going ~0.1 s after the stop -> the yaw part (about the vertical)
has its own lead (yaw_lead_s) and the prediction is braked by the speed over the last PRED_SHORT_S; (2) the wrist
compensated the base as if it were already at its target, but the base (whole arm) is slower than the wrist ->
the wrist IK uses a first-order model of the real base position (base_tau_s).

IMU trim (routing_config "gimbalImuTrim"): with the firmware pose and the IMU still, the gravity direction measured
by the wrist BNO085 (mounting + base tilt from config_runtime/imu/imu_ee_mount.json) is compared with the one the
kinematics predicts; the difference phi = g_meas x g_fk (tool frame, pitch / roll part only: yaw is not observable
from gravity and the magnetometer yaw is unreliable) is low-passed (tau_s), with a dead band and a cap, and the
target is commanded as R_target . exp(-trim). The campaign measured 2.7 deg RMS (p95 5.5) of such static error.

Selected with routing_config.json "assistMode": "gimbal" (other values: "rate" legacy, "dls").

Horizon lock (26/10/2026, user): with set_horizon_lock(True) the target keeps the camera's optical axis where
the head points but turns it about that axis until the image's horizontal axis is perpendicular to the vertical
(base tilt from imu_ee_mount.json); the IMU trim then makes the real camera match it with the gravity measured by
the wrist IMU. Blended in over LEVEL_RAMP_S (on toggle and at every engage), suspended when looking almost straight
up or down (roll undefined).

j5mini (26/10/2026): ported from raspberry/controller/web_services/head_assist_gimbal.py (branch
feature/video-direct-rtp), the ASSIST the robot used ("assistMode": "gimbal"). Only the glue changed: offsets / dirs,
the screw axes, the IMU sample and the HEADZERO sender come from j5mini (configure()), not from the old Pi modules.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time

import numpy as np

logger = logging.getLogger("j5mini.assist")


# ---------------------------------------------------------------------------------------------------------------
# j5mini glue: what the old Pi modules (head_assist, settings_manager, poe_params_manager, the telemetry file and
# the UART manager) provided
# ---------------------------------------------------------------------------------------------------------------

_LIMIT_ORDER = ("base", "spalla", "gomito", "yaw", "pitch", "roll")
_glue = {"telemetry": lambda: {}, "screws": None}


def configure(offsets, dirs, screws=None, telemetry=None, mount_path=None, campaign_dir=None):
    """offsets / dirs: j5mini settings; screws: 6 screw axes [w, v] (dashboard kinematics.json "S", m);
    telemetry(): the latest firmware telemetry dict (imu_valid, imu_q_*)."""
    global _MOUNT_PATH, CAMPAIGN_DIR, _ARM_GEOM
    settings_manager.current = {"offsets": [float(v) for v in offsets], "dirs": [1 if float(d) >= 0 else -1 for d in dirs]}
    if screws is not None:
        _glue["screws"] = [list(map(float, s)) for s in screws]
        _ARM_GEOM = None
    if telemetry is not None:
        _glue["telemetry"] = telemetry
    if mount_path:
        _MOUNT_PATH = mount_path
    if campaign_dir:
        CAMPAIGN_DIR = campaign_dir


class settings_manager:  # noqa: N801 - stands in for the old module (three calls)
    DEFAULTS = {"offsets": [90.0] * 6, "dirs": [1] * 6}
    current = {"offsets": [90.0] * 6, "dirs": [1] * 6}

    @staticmethod
    def load():
        return settings_manager.current

    @staticmethod
    def physical_to_virtual(physical, offsets, dirs=None):
        dirs = dirs or [1] * 6
        return [(float(physical[i]) - float(offsets[i])) / (-1.0 if float(dirs[i]) < 0 else 1.0) + 90.0
                for i in range(6)]


class _ha:  # noqa: N801 - the two helpers of the old head_assist module
    @staticmethod
    def _joint_limits_dict(limits):
        out = {"base": {"min": 20, "max": 160}, "spalla": {"min": 20, "max": 160}, "gomito": {"min": 20, "max": 160},
               "yaw": {"min": 20, "max": 160}, "pitch": {"min": 30, "max": 140}, "roll": {"min": 30, "max": 140}}
        if not isinstance(limits, dict):
            return out
        for joint in _LIMIT_ORDER:
            row = limits.get(joint)
            if not isinstance(row, dict):
                continue
            try:
                mn, mx = int(row.get("min", out[joint]["min"])), int(row.get("max", out[joint]["max"]))
            except (TypeError, ValueError):
                continue
            if 0 <= mn < mx <= 180:
                out[joint] = {"min": mn, "max": mx}
        return out

    @staticmethod
    def _axis_point_from_screw(screw_row):
        w = np.asarray(screw_row[:3], dtype=float).reshape(3)
        v = np.asarray(screw_row[3:], dtype=float).reshape(3)
        wn = float(np.dot(w, w))
        if wn <= 1e-9:
            return None
        return np.cross(w, v) / wn

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
PITCH_HELP_MARGIN_DEG = 3.0  # with head position: the arm helps the wrist pitch this close to its limits

# point at an object (request_point)
CAM0_OFF_M = np.array([0.100, 0.0315, 0.0])   # cam0 (left) optical centre from the wrist centre, tool frame (URDF:
                                              # tool 0.057 + camera 0.045 forward, half of the 63 mm baseline left)
POINT_STANDOFF_M = 0.15     # camera -> object centre, closest approach
POINT_REACH_FRAC = 0.98     # "maximum extension" = this fraction of the stretched arm
POINT_SPEED_DEG_S = 40.0    # largest joint motion / duration
POINT_MIN_T_S = 1.5

# head motion prediction (routing_config "gimbalPrediction": enable, lead_s, max_deg)
PRED_LEAD_S = 0.20          # how far ahead (servos ~0.2 s behind; A/B 2026-10-18: pitch lag 125 -> 0 ms)
PRED_WIN_S = 0.08           # angular velocity over this window (intent arrival jitter on Wi-Fi)
PRED_TAU_S = 0.03           # smoothing of the angular velocity
PRED_W_LO, PRED_W_HI = 4.0, 20.0   # deg/s: no prediction below LO, full above HI (smoothstep in between)
PRED_MAX_DEG = 12.0
PRED_YAW_LEAD_S = 0.12      # lead of the yaw part (about the vertical): base + wrist yaw sprang back at stops
PRED_SHORT_S = 0.035        # brake: speed over the last samples, the prediction scales with short / smoothed speed
BASE_TAU_S = 0.12           # first-order lag of the real base (whole arm) behind its target, for the wrist IK

# IMU trim (routing_config "gimbalImuTrim": enable, tau_s, deadband_deg, max_deg)
TRIM_TAU_S = 1.0
TRIM_DEADBAND_DEG = 0.7
TRIM_MAX_DEG = 6.0
TRIM_STILL_S = 0.4          # firmware pose and IMU (nearly) unchanged for this long
TRIM_STILL_DEG = 1.5        # the telemetry reports whole servo degrees: any settling is >= 1 deg
TRIM_SERVO_LAG_S = 0.2      # the IMU is compared with the firmware pose this much earlier (servo lag, campaign)
_MOUNT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imu_ee_mount.json")
_mount_cache = {"mtime": None, "M": None, "T": None}

# horizon lock
LEVEL_RAMP_S = 0.5          # blend in / out
LEVEL_MIN_HORIZ = 0.3       # |up x optical axis| below this (within ~17 deg of vertical): no roll to level

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
            screws = _glue["screws"]
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
        self.z_off = 0.0             # height reference offset (m): end-of-range ratchet + pitch help
        self.dpsi = 0.0              # camera yaw - head yaw at engage (maps head displacement to the robot)
        self.pos_dbg = None
        self.target_id = 0
        self.last_t = 0.0
        self.last_log = 0.0
        self.last_full_v = None      # last commanded pose, virtual deg [6] (planning base for request_point)
        self.v_lims = None           # (virtual lo[6], virtual hi[6]) of the last step
        self.goal = None             # request_point motion in progress
        self.goal_done = None        # info of the last reached goal, taken by ws_server (pop_goal_done)
        self.head_hist = []          # (t, R_head) of the last ~0.3 s, for the prediction
        self.w_f = np.zeros(3)       # smoothed head angular velocity, rad/s (robot base frame)
        self.w_t = 0.0
        self.pred_deg = 0.0
        self.base_est = None         # modelled real base position (rad), see BASE_TAU_S
        self.trim = np.zeros(3)      # IMU trim (rotation vector, tool frame, rad)
        self.trim_hist = []          # (t, firmware virt[6], R_imu) for the stillness test
        self.trim_t = 0.0
        self.horizon = False         # horizon lock requested
        self.level_u = 0.0           # 0 = head roll, 1 = level (blended)
        self.level_t = 0.0


_state = GimbalState()


def set_horizon_lock(on: bool) -> bool:
    _state.horizon = bool(on)
    logger.info("[GIMBAL] horizon lock %s", "on" if on else "off")
    return _state.horizon


def horizon_lock() -> bool:
    return _state.horizon


def _level(st, r, now):
    """r turned about its optical axis (column 0) towards a level horizon by st.level_u (see the docstring)."""
    want = 1.0 if st.horizon else 0.0
    dt = min(0.1, max(0.0, now - st.level_t)) if st.level_t else 0.0
    st.level_t = now
    k = dt / LEVEL_RAMP_S
    st.level_u += max(-k, min(k, want - st.level_u))
    if st.level_u <= 1e-3:
        return r
    _, tilt = _imu_mount()
    up = tilt.T @ np.array([0.0, 0.0, 1.0]) if tilt is not None else np.array([0.0, 0.0, 1.0])
    x = r[:, 0]
    y = np.cross(up, x)
    n = float(np.linalg.norm(y))
    if n < LEVEL_MIN_HORIZ:
        return r
    y = y / n
    r_lvl = np.column_stack([x, y, np.cross(x, y)])
    return r @ _exp_map(st.level_u * _log_map(r.T @ r_lvl))


# ---------------------------------------------------------------------------------------------------------------
# measurement campaign (2026-10-18): desired / commanded / firmware / IMU orientation per intent, to decide whether a
# closed loop on the wrist IMU is worth it (tools/imu_campaign_report.py). Switched on / off with the ws message
# {"type": "imu_campaign", "on": true|false, "label": "..."}; nothing changes in the motion.
# ---------------------------------------------------------------------------------------------------------------

CAMPAIGN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "imu_campaign")
CAMPAIGN_HZ = 30.0
_campaign = {"f": None, "path": None, "last": 0.0, "n": 0, "label": ""}


def set_campaign(on: bool, label: str = "") -> dict:
    c = _campaign
    if c["f"] is not None:
        try:
            c["f"].close()
        except OSError:
            pass
        logger.info("[CAMPAIGN] stopped: %s (%d samples)", c["path"], c["n"])
        c["f"] = None
    if on:
        os.makedirs(CAMPAIGN_DIR, exist_ok=True)
        c["path"] = os.path.abspath(os.path.join(CAMPAIGN_DIR, time.strftime("%Y%m%d_%H%M%S") + ".jsonl"))
        c["f"] = open(c["path"], "a", buffering=1 << 16)
        c["n"] = 0
        c["label"] = label
        logger.info("[CAMPAIGN] recording to %s", c["path"])
    return {"on": c["f"] is not None, "path": c["path"], "n": c["n"]}


def set_campaign_label(label: str) -> None:
    _campaign["label"] = label
    logger.info("[CAMPAIGN] label %s", label)


def _campaign_log(st, intent, full_v, r_target, physical_tel, r_target_raw=None):
    c = _campaign
    if c["f"] is None:
        return
    now = time.monotonic()
    if now - c["last"] < 1.0 / CAMPAIGN_HZ:
        return
    c["last"] = now
    imu = None
    try:
        t = _glue["telemetry"]()
        if t.get("imu_valid"):
            imu = [t.get("imu_q_w"), t.get("imu_q_x"), t.get("imu_q_y"), t.get("imu_q_z"), t.get("imu_sample_counter")]
    except (TypeError, ValueError):
        pass
    rec = {"t": round(now, 4), "wall": round(time.time(), 3), "label": c["label"],
           "goal": st.goal is not None,
           "head_q": [round(float(intent.get(k, 0.0)), 5) for k in ("quat_w_in", "quat_x_in", "quat_y_in", "quat_z_in")],
           "head_p": [intent.get("head_px"), intent.get("head_py"), intent.get("head_pz")],
           "cmd_v": [round(float(v), 3) for v in full_v],
           "tel_phys": [round(float(v), 2) for v in physical_tel],
           "target_R": None if r_target is None else [round(float(v), 5) for v in np.asarray(r_target).reshape(-1)],
           "target_raw_R": None if r_target_raw is None else [round(float(v), 5) for v in np.asarray(r_target_raw).reshape(-1)],
           "pred_deg": round(float(st.pred_deg), 2),
           "trim_deg": [round(math.degrees(float(v)), 3) for v in st.trim],
           "base_est": None if st.base_est is None else round(math.degrees(st.base_est) + 90.0, 3),
           "imu": imu}
    try:
        c["f"].write(json.dumps(rec) + "\n")
        c["n"] += 1
        if c["n"] % 30 == 0:
            c["f"].flush()
    except OSError:
        pass


def _log_map(r):
    """Rotation vector (rad) of a rotation matrix."""
    c = max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return v * (a / (2.0 * math.sin(a)))


def _exp_map(v):
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * (kx @ kx)


def _predict_head(st, r_head, now, cfg):
    """Head orientation lead_s ahead (see the module docstring); r_head itself when disabled / still."""
    cfg = cfg or {}
    hist = st.head_hist
    hist.append((now, r_head))
    while hist and now - hist[0][0] > 0.3:
        hist.pop(0)
    if not cfg.get("enable", True):
        st.pred_deg = 0.0
        return r_head
    old = None
    for t_o, r_o in hist:
        if now - t_o >= PRED_WIN_S:
            old = (t_o, r_o)
    if old is None:
        return r_head
    w = (r_head @ old[1].T)
    w = _log_map(w) / max(1e-3, now - old[0])
    dt = min(0.1, max(0.0, now - st.w_t)) if st.w_t else 0.0
    st.w_t = now
    a = 1.0 - math.exp(-dt / PRED_TAU_S) if dt > 0 else 1.0
    st.w_f = st.w_f + a * (w - st.w_f)
    speed = math.degrees(float(np.linalg.norm(st.w_f)))
    x = max(0.0, min(1.0, (speed - PRED_W_LO) / (PRED_W_HI - PRED_W_LO)))
    fade = x * x * (3.0 - 2.0 * x)
    # brake: when the head stops, the smoothed velocity lags ~0.1 s; the speed over the last few samples drops at
    # once and scales the prediction down
    brake = 1.0
    if cfg.get("brake", True):
        short = None
        for t_o, r_o in hist:
            if now - t_o >= PRED_SHORT_S:
                short = (t_o, r_o)
        if short is not None:
            s_short = math.degrees(float(np.linalg.norm(_log_map(r_head @ short[1].T)))) / max(1e-3, now - short[0])
            brake = min(1.0, s_short / max(1e-6, speed))
    # yaw (about the vertical) with its own, shorter lead
    w_v = np.array([0.0, 0.0, st.w_f[2]])
    w_h = st.w_f - w_v
    rv = (w_h * float(cfg.get("lead_s", PRED_LEAD_S)) + w_v * float(cfg.get("yaw_lead_s", PRED_YAW_LEAD_S))) \
        * fade * brake
    ang = math.degrees(float(np.linalg.norm(rv)))
    cap = float(cfg.get("max_deg", PRED_MAX_DEG))
    if ang > cap:
        rv = rv * (cap / ang)
        ang = cap
    st.pred_deg = ang
    if ang < 1e-3:
        return r_head
    return _exp_map(rv) @ r_head


def _imu_mount():
    """(M, T): IMU mounting (R_imu = W . R_tool . M) and base tilt rotation, from imu_ee_mount.json."""
    c = _mount_cache
    try:
        mt = os.path.getmtime(_MOUNT_PATH)
    except OSError:
        return None, None
    if mt != c["mtime"]:
        try:
            with open(_MOUNT_PATH) as f:
                d = json.load(f)
            c["M"] = _quat_to_mat(*d["quat_wxyz"])
            tx, ty = (math.radians(float(v)) for v in d.get("base_tilt_deg", [0.0, 0.0])[:2])
            c["T"] = _exp_map(np.array([tx, ty, 0.0]))
            c["mtime"] = mt
        except (OSError, ValueError, KeyError, TypeError):
            return None, None
    return c["M"], c["T"]


def _read_imu():
    try:
        t = _glue["telemetry"]()
        if t.get("imu_valid"):
            return _quat_to_mat(t["imu_q_w"], t["imu_q_x"], t["imu_q_y"], t["imu_q_z"])
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def _imu_trim(st, virt_fw, now, cfg):
    """Update the IMU trim (see the module docstring) and return it as a rotation matrix to apply on the right of
    the target."""
    cfg = cfg or {}
    if not cfg.get("enable", False):
        st.trim = np.zeros(3)
        return np.eye(3)
    m, tilt = _imu_mount()
    r_imu = _read_imu()
    if m is not None and r_imu is not None:
        h = st.trim_hist
        h.append((now, np.asarray(virt_fw, dtype=float), r_imu))
        while h and now - h[0][0] > TRIM_STILL_S + TRIM_SERVO_LAG_S + 0.2:
            h.pop(0)
        # firmware pose TRIM_SERVO_LAG_S ago (where the servos are now) and the window before it: nearly constant;
        # IMU over the last TRIM_STILL_S: nearly constant
        lagged = [x for x in h if now - x[0] >= TRIM_SERVO_LAG_S]
        win_fw = [x for x in lagged if now - x[0] <= TRIM_SERVO_LAG_S + TRIM_STILL_S]
        win_imu = [x for x in h if now - x[0] <= TRIM_STILL_S]
        if lagged and len(win_fw) >= 3 and len(win_imu) >= 3 and now - win_imu[0][0] >= TRIM_STILL_S * 0.7:
            v = np.array([x[1] for x in win_fw])
            imu_still = all(math.degrees(float(np.linalg.norm(_log_map(win_imu[0][2].T @ x[2])))) < TRIM_STILL_DEG
                            for x in win_imu)
            if float(np.max(np.ptp(v, axis=0))) < TRIM_STILL_DEG and imu_still:
                z = np.array([0.0, 0.0, 1.0])
                r_fk = cam_rotation(np.radians(np.asarray(lagged[-1][1], dtype=float) - 90.0))
                g_fk = r_fk.T @ (tilt.T @ z)
                g_m = m @ (r_imu.T @ z)
                phi = np.cross(g_m, g_fk)                        # tool frame, perpendicular to gravity
                dt = min(0.1, max(0.0, now - st.trim_t)) if st.trim_t else 0.0
                st.trim_t = now
                if math.degrees(float(np.linalg.norm(phi - st.trim))) > float(cfg.get("deadband_deg", TRIM_DEADBAND_DEG)):
                    a = 1.0 - math.exp(-dt / float(cfg.get("tau_s", TRIM_TAU_S))) if dt > 0 else 0.0
                    st.trim = st.trim + a * (phi - st.trim)
                cap = math.radians(float(cfg.get("max_deg", TRIM_MAX_DEG)))
                n = float(np.linalg.norm(st.trim))
                if n > cap:
                    st.trim = st.trim * (cap / n)
            else:
                st.trim_t = 0.0
    return _exp_map(-st.trim)


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
         pos_cfg: dict | None = None, pred_cfg: dict | None = None, trim_cfg: dict | None = None):
    """Returns (arm_physical[3] | None, grip_active, hold_active, target_id) and rewrites intent quat_* ."""
    st = _state
    lim = _ha._joint_limits_dict(limits_src)

    def send_quat(q):
        intent["quat_w"], intent["quat_x"], intent["quat_y"], intent["quat_z"] = (float(v) for v in q)

    for k in ("quat_w", "quat_x", "quat_y", "quat_z"):
        intent[k + "_in"] = intent.get(k, 1.0 if k == "quat_w" else 0.0)
    tel_raw = [float(v) for v in physical_six[:6]] if physical_six and len(physical_six) >= 6 else None

    if not grip or not physical_six or len(physical_six) < 6:
        if st.phase != "idle":
            logger.info("[GIMBAL] released")
        if st.goal is not None:
            logger.info("[GIMBAL] point: cancelled (grips released)")
            st.goal = None
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
        st.head_hist = []
        st.w_f = np.zeros(3)
        st.w_t = 0.0
        st.base_est = None
        st.trim_hist = []
        st.trim_t = 0.0
        st.level_u = 0.0             # the horizon lock blends in again from the engaged pose
        st.level_t = 0.0
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
            st.z_off = 0.0
            st.dpsi = (math.atan2(st.r_cam_ref[1, 0], st.r_cam_ref[0, 0])
                       - math.atan2(st.r_head_ref[1, 0], st.r_head_ref[0, 0]))
            st.phase = "track"
            st.last_t = 0.0
            logger.info("[GIMBAL] tracking: wrist centre phys %s", [round(float(v), 1) for v in st.center_phys])
        arm_phys = [round(float(v), 2) for v in physical_six[:3]]
        st.last_arm_phys = arm_phys
        return arm_phys, True, True, st.target_id

    v_lo, v_hi = _virtual_limits(lim, offsets, dirs)
    st.v_lims = (v_lo, v_hi)

    # ---- point at an object: minimum-jerk motion to the planned pose, the head is ignored meanwhile
    if st.goal is not None:
        g = st.goal
        now = time.monotonic()
        st.last_t = now
        s = min(1.0, (now - g["t0"]) / g["T"])
        e = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
        st.arm_v = [g["a0"][i] + (g["a1"][i] - g["a0"][i]) * e for i in range(3)]
        w_cmd = [g["w0"][i] + (g["w1"][i] - g["w0"][i]) * e for i in range(3)]
        full_v = list(virt)
        full_v[0:3] = list(st.arm_v)
        full_v[3:6] = w_cmd
        if s >= 1.0:
            # new references at the reached pose: the head steers from here (no jump), as after an engage
            qg = np.radians(np.asarray(full_v) - 90.0)
            st.r_cam_ref = cam_rotation(qg)
            st.r_head_ref = r_head
            st.p_ref = _head_pos(intent)
            st.p_last = st.p_ref
            st.rz_ref = wc_rz(qg[1], qg[2])
            st.q23_ref = (float(qg[1]), float(qg[2]))
            st.z_off = 0.0
            st.dpsi = (math.atan2(st.r_cam_ref[1, 0], st.r_cam_ref[0, 0])
                       - math.atan2(st.r_head_ref[1, 0], st.r_head_ref[0, 0]))
            st.goal_done = dict(g["info"], t=now)
            st.goal = None
            logger.info("[GIMBAL] point: reached arm virt %s wrist virt %s", [round(v, 1) for v in st.arm_v],
                        [round(v, 1) for v in w_cmd])
        out = _emit(st, intent, full_v, offsets, dirs, vr_params, send_quat)
        _campaign_log(st, intent, full_v, None, tel_raw)
        return out

    # ---- track
    now_p = time.monotonic()
    r_target_raw = r_head @ st.r_head_ref.T @ st.r_cam_ref
    r_head_p = _predict_head(st, r_head, now_p, pred_cfg)
    virt_fw = settings_manager.physical_to_virtual(list(tel_raw), offsets, dirs) if tel_raw else list(virt)
    r_target = _level(st, r_head_p @ st.r_head_ref.T @ st.r_cam_ref, now_p) @ _imu_trim(st, virt_fw, now_p, trim_cfg)
    # wrist IK on the modelled REAL base position: the base (whole arm) lags its target more than the wrist
    tau_b = float((pred_cfg or {}).get("base_tau_s", BASE_TAU_S))
    if st.base_est is None or tau_b <= 0:
        st.base_est = float(qm[0])
    else:
        dtb = min(0.1, max(0.0, now_p - st.last_t)) if st.last_t else 0.0
        st.base_est += (float(qm[0]) - st.base_est) * (1.0 - math.exp(-dtb / tau_b))
    qm_w = np.array(qm, dtype=float)
    qm_w[0] = st.base_est
    q4, q5, q6 = wrist_ik(qm_w[:3], r_target)
    w_deg = np.degrees([q4, q5, q6]) + 90.0          # wrist virtual
    # routing_config limits are PHYSICAL servo degrees (the firmware clamps there): convert them to virtual with
    # offsets / dirs. Using them as virtual was wrong for the reversed joints (pitch dir -1: physical 35..120 is
    # virtual 60..145), the IK then believed the wrist reached angles the firmware had clipped.
    w_lo = [v_lo[i] + WRIST_MARGIN_DEG for i in (3, 4, 5)]
    w_hi = [v_hi[i] - WRIST_MARGIN_DEG for i in (3, 4, 5)]
    w_cmd = [float(np.clip(w_deg[i], w_lo[i], w_hi[i])) for i in range(3)]
    if abs(w_cmd[0] - w_deg[0]) > 0.05:
        # wrist yaw at its limit (and the base too, or still catching up): re-solve pitch / roll for that yaw
        p5, p6 = wrist_pitch_roll_for_yaw(qm_w[:3], r_target, math.radians(w_cmd[0] - 90.0))
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
        q2r, q3r = st.q23_ref
        z_t = st.rz_ref[1] + dz + st.z_off
        # ratchet at the ends of the reachable height: the excess is dropped from the reference, so the arm
        # answers as soon as the head reverses (engaged crouched and stood up: the target went 50 cm above the
        # straight arm and crouching again did nothing until the head came back below that point)
        z_top = wc_rz(*bow_for_height(10.0, q2r, q3r, lo, hi))[1]
        z_bot = max(POS_WC_Z_MIN, wc_rz(*bow_for_height(-10.0, q2r, q3r, lo, hi))[1])
        if z_t > z_top:
            st.z_off -= z_t - z_top
            z_t = z_top
        elif z_t < z_bot:
            st.z_off += z_bot - z_t
            z_t = z_bot
        q2b, q3b = bow_for_height(z_t, q2r, q3r, lo, hi)
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
        # orientation first: when the wrist pitch reaches its limit, the elbow (then the shoulder) helps as in the
        # orientation-only assist, and the height target follows the new posture (looking down with the arm
        # straight: the wrist alone stopped at its 40 deg)
        ov_lim = float(w_deg[1]) - float(np.clip(w_deg[1], w_lo[1] + PITCH_HELP_MARGIN_DEG,
                                                 w_hi[1] - PITCH_HELP_MARGIN_DEG))
        if abs(ov_lim) > 1e-3:
            step_p = max(-max_step, min(max_step, frac * ov_lim))
            g_new = float(np.clip(st.arm_v[2] + step_p, a_lo[2], a_hi[2]))
            rest = step_p - (g_new - st.arm_v[2])
            st.arm_v[2] = g_new
            st.arm_v[1] = float(np.clip(st.arm_v[1] + rest, a_lo[1], a_hi[1]))
            z_new = wc_rz(math.radians(st.arm_v[1] - 90.0), math.radians(st.arm_v[2] - 90.0))[1]
            st.z_off += z_new - z_t
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
    arm_phys, _, _, _ = _emit(st, intent, full_v, offsets, dirs, vr_params, send_quat)
    _campaign_log(st, intent, full_v, r_target, tel_raw, r_target_raw)

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
        logger.info("[GIMBAL] target ypr %s cam(cmd) ypr %s wrist virt %s%s arm virt %s dt %.0f ms pred %.1f deg%s",
                    [round(float(v), 1) for v in tg], [round(float(v), 1) for v in fk],
                    [round(float(v), 1) for v in w_cmd],
                    " (clamped)" if any(abs(w_cmd[i] - w_deg[i]) > 0.05 for i in range(3)) else "",
                    [round(float(v), 1) for v in st.arm_v], dt * 1000.0, st.pred_deg, pos_txt)
    return arm_phys, True, False, st.target_id


def _emit(st, intent, full_v, offsets, dirs, vr_params, send_quat):
    """Arm target (physical, with the fraction) + the synthetic head quaternion for the wrist part of full_v."""
    # arm targets with the fraction (the SPI frame carries centidegrees): settings_manager.virtual_to_physical
    # truncates to whole degrees, which made the arm follow the head in 1 deg steps
    arm_phys = [round(_virt_to_phys_f(full_v[i], offsets[i], dirs[i]), 2) for i in range(3)]
    wrist_phys = [float(v) for v in
                  [offsets[i] + (dirs[i] if dirs[i] >= 0 else -1) * (full_v[i] - 90.0) for i in (3, 4, 5)]]
    pipe = HeadPipe(vr_params)
    q_err = pipe.quat_for_servo_deltas(wrist_phys[0] - st.center_phys[0], wrist_phys[1] - st.center_phys[1],
                                       wrist_phys[2] - st.center_phys[2])
    q_send = _qmul(q_err, st.q_ref)
    send_quat(q_send)
    st.q_hold = q_send   # on release / re-engage the firmware keeps exactly this pose (no jump)
    st.last_arm_phys = arm_phys
    st.arm_cmd_phys = arm_phys
    st.last_full_v = [float(v) for v in full_v]
    st.target_id = (st.target_id + 1) & 0xFFFF
    return arm_phys, True, False, st.target_id


# ---------------------------------------------------------------------------------------------------------------
# point at an object (tools/vision_service.py)
# ---------------------------------------------------------------------------------------------------------------

def plan_point(full_v, v_lims, p_cam_mm):
    """Pose (virtual deg [6]) that points cam0 at the object p_cam_mm = [X right, Y down, Z forward] of cam0 seen
    from the pose full_v, as close to it as the arm allows (POINT_STANDOFF_M at best).

    The wrist yaw axis runs along the forearm and the wrist pitch only spans about -55..+40 deg, so the forearm
    tilt and the camera pitch are coupled (camera pitch = forearm tilt + wrist pitch in the arm plane): stretching
    the arm flat towards an object at camera height would leave the camera looking at the floor. So instead of
    "wrist centre on the shoulder -> object line" all shoulder / elbow poses are searched (1 deg grid) for the
    camera closest to the object whose required wrist pitch is reachable; the best few get the full wrist IK and
    the first one that really points cam0 at the object wins. Returns (pose_v, info) or (None, info with "why")."""
    v_lo, v_hi = v_lims
    q = np.radians(np.asarray(full_v, dtype=float) - 90.0)
    r_now = cam_rotation(q)
    r_c, z_c = wc_rz(q[1], q[2])
    c_now = np.array([r_c * math.cos(q[0]), r_c * math.sin(q[0]), z_c])
    cam = c_now + r_now @ CAM0_OFF_M
    x, y, z = (float(v) / 1000.0 for v in p_cam_mm)
    p = cam + r_now @ np.array([z, -x, -y])            # image right = tool -y, image down = tool -z
    info = {"p_base_mm": [round(float(v) * 1000) for v in p]}
    # base towards the object
    b_v = float(np.clip(math.degrees(math.atan2(p[1], p[0])) + 90.0, v_lo[0], v_hi[0]))
    q1 = math.radians(b_v - 90.0)
    pr = p[0] * math.cos(q1) + p[1] * math.sin(q1)      # object in the arm plane (reach, height)
    pz = float(p[2])
    h0, l1, l2 = _arm_geom()
    fwd = float(CAM0_OFF_M[0])
    # grid over shoulder / elbow (virtual deg, inside the limits)
    s_v = np.arange(math.ceil(v_lo[1]), math.floor(v_hi[1]) + 1, 1.0)
    e_v = np.arange(math.ceil(v_lo[2]), math.floor(v_hi[2]) + 1, 1.0)
    S, E = np.meshgrid(np.radians(s_v - 90.0), np.radians(e_v - 90.0), indexing="ij")
    wr = l1 * np.sin(S) + l2 * np.sin(S + E)
    wz = h0 + l1 * np.cos(S) + l2 * np.cos(S + E)
    dr, dzz = pr - wr, pz - wz
    dwc = np.hypot(dr, dzz)                              # wrist centre -> object
    pitch = np.arctan2(-dzz, dr)                         # camera pitch towards the object (+ = down), in the plane
    q5 = pitch - (S + E)                                 # forearm tilt S+E (Ry, + = forward/down for the tool x)
    p_lo = math.radians(v_lo[4] - 90.0 + WRIST_MARGIN_DEG + 2.0)
    p_hi = math.radians(v_hi[4] - 90.0 - WRIST_MARGIN_DEG - 2.0)
    dist_cam = dwc - fwd                                 # camera roughly on the wrist centre -> object line
    ok = (q5 >= p_lo) & (q5 <= p_hi) & (wz >= POS_WC_Z_MIN) & (dr > fwd + 0.02) & (dist_cam >= POINT_STANDOFF_M)
    if not np.any(ok):
        return None, dict(info, why="nessuna posa del braccio riesce a puntare l'oggetto")
    move = np.hypot(S - q[1], E - q[2])
    cost = np.where(ok, (dist_cam - POINT_STANDOFF_M) + 0.02 * move, np.inf)
    order = np.argsort(cost, axis=None)[:40]
    best = None
    for k in order:
        i, j = np.unravel_index(int(k), cost.shape)
        if not np.isfinite(cost[i, j]):
            break
        q2, q3 = float(S[i, j]), float(E[i, j])
        c_t = np.array([float(wr[i, j]) * math.cos(q1), float(wr[i, j]) * math.sin(q1), float(wz[i, j])])
        d = p - c_t
        dh = math.hypot(d[0], d[1])
        yaw = math.atan2(d[1], d[0]) - math.asin(max(-1.0, min(1.0, float(CAM0_OFF_M[1]) / max(dh, 0.05))))
        r_t = _rz(yaw) @ _ry(math.atan2(-d[2], dh))
        q4, q5w, q6 = wrist_ik((q1, q2, q3), r_t)
        w = [math.degrees(t) + 90.0 for t in (q4, q5w, q6)]
        if not all(v_lo[3 + n] + WRIST_MARGIN_DEG <= w[n] <= v_hi[3 + n] - WRIST_MARGIN_DEG for n in range(3)):
            continue
        pose = [b_v, math.degrees(q2) + 90.0, math.degrees(q3) + 90.0] + w
        qp = np.radians(np.asarray(pose) - 90.0)
        r_p = cam_rotation(qp)
        to_p = p - (c_t + r_p @ CAM0_OFF_M)
        err = math.degrees(math.acos(max(-1.0, min(1.0, float(r_p[:, 0] @ to_p) / max(1e-6, float(np.linalg.norm(to_p)))))))
        if err > 4.0:
            continue
        best = (pose, err, float(np.linalg.norm(to_p)))
        break
    if best is None:
        return None, dict(info, why="il polso non riesce a puntare l'oggetto da nessuna posa vicina")
    pose, err, dist_after = best
    info.update(err_deg=round(err, 1), dist_after_mm=round(dist_after * 1000),
                base_limited=abs(b_v - (math.degrees(math.atan2(p[1], p[0])) + 90.0)) > 0.5)
    return pose, info


def request_point(p_cam_mm, label=None):
    """Start the motion towards an object (ws_server, vision_result with aim). Only while tracking (grips held)."""
    st = _state
    if st.phase != "track" or st.last_full_v is None or st.v_lims is None:
        return False, {"why": "tieni premuti i due grip (ASSIST attivo)"}
    pose, info = plan_point(st.last_full_v, st.v_lims, p_cam_mm)
    if pose is None:
        return False, info
    a0, w0 = list(st.last_full_v[0:3]), list(st.last_full_v[3:6])
    span = max(abs(pose[i] - st.last_full_v[i]) for i in range(6))
    T = max(POINT_MIN_T_S, span / POINT_SPEED_DEG_S)
    info.update(label=label, T_s=round(T, 2), pose_v=[round(v, 1) for v in pose])
    st.goal = {"a0": a0, "a1": pose[0:3], "w0": w0, "w1": pose[3:6], "t0": time.monotonic(), "T": T, "info": info}
    st.goal_done = None
    logger.info("[GIMBAL] point %s: object base %s mm -> pose virt %s in %.1f s (axis err %.1f deg, %d mm away%s%s)",
                label, info["p_base_mm"], info["pose_v"], T, info["err_deg"], info["dist_after_mm"],
                ", base at its limit" if info.get("base_limited") else "", "")
    return True, info


def pop_goal_done():
    st = _state
    d, st.goal_done = st.goal_done, None
    return d


def _rot_to_ypr(r):
    return (math.atan2(r[1, 0], r[0, 0]), math.asin(max(-1.0, min(1.0, -r[2, 0]))), math.atan2(r[2, 1], r[2, 2]))
