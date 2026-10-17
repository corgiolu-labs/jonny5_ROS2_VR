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
ARM_GAIN = 0.25          # fraction of the overflow absorbed per tick
ARM_MAX_STEP_DEG = 2.0   # per intent (40 ms in mode 5) -> 50 deg/s
WRIST_MARGIN_DEG = 1.0   # stay this far from the wrist hard limits

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
        self.target_id = 0
        self.log_n = 0


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


def step(intent: dict, *, physical_six, limits_src: dict, vr_params: list[str], uart_manager, grip: bool):
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
    virt = np.asarray(settings_manager.physical_to_virtual(list(physical_six), offsets, dirs), dtype=float)
    qm = np.radians(virt - 90.0)
    hw, hx, hy, hz = (float(intent.get(k, d)) for k, d in
                      (("quat_w", 1.0), ("quat_x", 0.0), ("quat_y", 0.0), ("quat_z", 0.0)))
    r_head = _C @ _quat_to_mat(hw, hx, hy, hz) @ _C.T

    if st.phase == "idle":
        # engage: freeze the firmware wrist (constant quaternion) and zero its reference
        st.phase = "zeroing"
        st.zero_ok_at = 0.0
        st.zero_task = asyncio.get_event_loop().create_task(_send_headzero(uart_manager))
        st.arm_v = virt[:3].copy()
        logger.info("[GIMBAL] engage: HEADZERO, arm virt %s", [round(v, 1) for v in virt[:3]])
    if st.phase == "zeroing":
        send_quat(st.q_hold)
        # a couple of intents after the OK so the firmware has latched q_hold as its reference
        if st.zero_ok_at and time.monotonic() - st.zero_ok_at > 0.08:
            st.q_ref = st.q_hold
            st.center_phys = [float(physical_six[3]), float(physical_six[4]), float(physical_six[5])]
            st.r_cam_ref = cam_rotation(qm)
            st.r_head_ref = r_head
            st.phase = "track"
            logger.info("[GIMBAL] tracking: wrist centre phys %s", [round(v, 1) for v in st.center_phys])
        arm_phys = [int(round(v)) for v in physical_six[:3]]
        st.last_arm_phys = arm_phys
        return arm_phys, True, True, st.target_id

    # ---- track
    r_target = r_head @ st.r_head_ref.T @ st.r_cam_ref
    q4, q5, q6 = wrist_ik(qm[:3], r_target)
    w_deg = np.degrees([q4, q5, q6]) + 90.0          # wrist virtual
    w_lo = [lim["yaw"]["min"] + WRIST_MARGIN_DEG, lim["pitch"]["min"] + WRIST_MARGIN_DEG, lim["roll"]["min"] + WRIST_MARGIN_DEG]
    w_hi = [lim["yaw"]["max"] - WRIST_MARGIN_DEG, lim["pitch"]["max"] - WRIST_MARGIN_DEG, lim["roll"]["max"] - WRIST_MARGIN_DEG]
    w_cmd = [float(np.clip(w_deg[i], w_lo[i], w_hi[i])) for i in range(3)]

    # arm assist: bring the wrist back inside +-BAND (virtual deg around 90)
    def overflow(v):
        d = v - 90.0
        return d - max(-BAND_DEG, min(BAND_DEG, d))

    a_lo = [lim["base"]["min"], lim["spalla"]["min"], lim["gomito"]["min"]]
    a_hi = [lim["base"]["max"], lim["spalla"]["max"], lim["gomito"]["max"]]
    ov_yaw = overflow(float(w_deg[0]))
    ov_pitch = overflow(float(w_deg[1]))
    step_b = max(-ARM_MAX_STEP_DEG, min(ARM_MAX_STEP_DEG, ARM_GAIN * ov_yaw))
    st.arm_v[0] = float(np.clip(st.arm_v[0] + step_b, a_lo[0], a_hi[0]))
    step_p = max(-ARM_MAX_STEP_DEG, min(ARM_MAX_STEP_DEG, ARM_GAIN * ov_pitch))
    g_new = float(np.clip(st.arm_v[2] + step_p, a_lo[2], a_hi[2]))
    rest = step_p - (g_new - st.arm_v[2])            # what the elbow could not take goes to the shoulder
    st.arm_v[2] = g_new
    st.arm_v[1] = float(np.clip(st.arm_v[1] + rest, a_lo[1], a_hi[1]))

    full_v = list(virt)
    full_v[0:3] = [float(v) for v in st.arm_v]
    full_v[3:6] = w_cmd
    phys = settings_manager.virtual_to_physical(full_v, offsets, dirs)
    arm_phys = [int(round(float(v))) for v in phys[:3]]
    wrist_phys = [float(v) for v in
                  [offsets[i] + (dirs[i] if dirs[i] >= 0 else -1) * (w_cmd[i - 3] - 90.0) for i in (3, 4, 5)]]

    pipe = HeadPipe(vr_params)
    q_err = pipe.quat_for_servo_deltas(wrist_phys[0] - st.center_phys[0], wrist_phys[1] - st.center_phys[1],
                                       wrist_phys[2] - st.center_phys[2])
    q_send = _qmul(q_err, st.q_ref)
    send_quat(q_send)
    st.q_hold = q_send   # on release / re-engage the firmware keeps exactly this pose (no jump)
    st.last_arm_phys = arm_phys
    st.target_id = (st.target_id + 1) & 0xFFFF

    st.log_n += 1
    if st.log_n % 25 == 0:
        fk = np.degrees(_rot_to_ypr(cam_rotation(np.radians(np.asarray(full_v) - 90.0))))
        tg = np.degrees(_rot_to_ypr(r_target))
        logger.info("[GIMBAL] target ypr %s cam(cmd) ypr %s wrist virt %s%s arm virt %s",
                    [round(v, 1) for v in tg], [round(v, 1) for v in fk], [round(v, 1) for v in w_cmd],
                    " (clamped)" if any(abs(w_cmd[i] - w_deg[i]) > 0.05 for i in range(3)) else "",
                    [round(v, 1) for v in st.arm_v])
    return arm_phys, True, False, st.target_id


def _rot_to_ypr(r):
    return (math.atan2(r[1, 0], r[0, 0]), math.asin(max(-1.0, min(1.0, -r[2, 0]))), math.atan2(r[2, 1], r[2, 2]))
