#!/usr/bin/env python3
"""
verify_isolated_joints.py — pure isolated-joint diagnostic (no ASSIST).

For each configuration, commanded via SETPOSE (bypasses ASSIST logic):
  1. Go to HOME, capture baseline (IMU pitch, FK wc_z, FK tool_z, servo readback).
  2. Move ONE or BOTH of SPALLA/GOMITO by a fixed virtual delta.
  3. Wait for SETPOSE_DONE + settle, re-capture.
  4. Compute signed deltas vs HOME baseline.

Directly answers: when I command SPALLA+Δ (or GOMITO+Δ, or combined), does
the wrist-center go down (Δfk_wc_z < 0) or up (Δfk_wc_z > 0) AND does the
IMU pitch (with full calib chain applied) go up (ΔIMU_pitch > 0) as the
user expects the mechanical mounting to dictate?

NO firmware / NO architecture / NO tuning: just SETPOSE and observation.
"""
import asyncio, json, math, ssl, time, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"

# Tests: (label, [B, S, G, Y, P, R])
# Δ vs HOME of +15° on one or both of SPALLA/GOMITO (keeping wrist fixed at 90).
DELTA = 15
HOME = [90, 90, 90, 90, 90, 90]
TESTS = [
    ("HOME",                 HOME),
    ("SHOULDER +15",         [90, 90+DELTA, 90, 90, 90, 90]),
    ("SHOULDER -15",         [90, 90-DELTA, 90, 90, 90, 90]),
    ("ELBOW +15",            [90, 90, 90+DELTA, 90, 90, 90]),
    ("ELBOW -15",            [90, 90, 90-DELTA, 90, 90, 90]),
    ("S+15 & G+15",          [90, 90+DELTA, 90+DELTA, 90, 90, 90]),
    ("S+15 & G-15",          [90, 90+DELTA, 90-DELTA, 90, 90, 90]),
    ("S-15 & G+15",          [90, 90-DELTA, 90+DELTA, 90, 90, 90]),
    ("S-15 & G-15",          [90, 90-DELTA, 90-DELTA, 90, 90, 90]),
]

SETTLE_S   = 1.0
SAMPLES_N  = 30
MOTION_TMO = 25.0

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]
def r_from_quat_wxyz(q_wxyz): return R.from_quat(q_wxyz_to_xyzw(q_wxyz))

def load_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

async def await_setpose_done(ws, tmo=25.0):
    t_end = time.monotonic() + tmo
    while time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError: continue
        except: return False
        try: m = json.loads(raw)
        except: continue
        if m.get("type") == "setpose_done": return True
    return False

async def collect_snapshot(ws, r_wb_inv, r_mount_inv, r_home_inv, n):
    """Average n valid telemetry samples. Returns dict with means, or None."""
    servos = {k: [] for k in ("servo_deg_B","servo_deg_S","servo_deg_G","servo_deg_Y","servo_deg_P","servo_deg_R")}
    fk = {k: [] for k in ("fk_live_x_mm","fk_live_y_mm","fk_live_z_mm","fk_live_yaw","fk_live_pitch","fk_live_roll",
                           "fk_live_wc_x_mm","fk_live_wc_y_mm","fk_live_wc_z_mm")}
    imu = {"yaw": [], "pitch": [], "roll": []}
    t_end = time.monotonic() + 4.0
    while (len(servos["servo_deg_B"]) < n) and time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError: continue
        except: return None
        try: m = json.loads(raw)
        except: continue
        if m.get("type") != "telemetry": continue
        if "servo_deg_B" not in m: continue
        try:
            sv = [float(m[k]) for k in servos]
            if any(v < 5 or v > 175 for v in sv): continue
        except: continue
        for k in servos: servos[k].append(float(m[k]))
        if m.get("fk_live_valid"):
            for k in fk:
                if m.get(k) is not None: fk[k].append(float(m[k]))
        if m.get("imu_valid") is True and m.get("imu_q_w") is not None:
            q = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
            r_ee = r_home_inv * r_wb_inv * r_from_quat_wxyz(q) * r_mount_inv
            ypr = r_ee.as_euler("ZYX", degrees=True)
            imu["yaw"].append(float(ypr[0])); imu["pitch"].append(float(ypr[1])); imu["roll"].append(float(ypr[2]))
    if not servos["servo_deg_B"]: return None
    def mean(xs): return sum(xs)/len(xs) if xs else None
    out = {}
    for k,v in servos.items(): out[k] = mean(v)
    for k,v in fk.items():
        if v: out[k] = mean(v)
    for k,v in imu.items(): out["imu_"+k] = mean(v) if v else None
    out["n_imu"] = len(imu["yaw"])
    return out

async def main():
    print("="*92)
    print("ISOLATED-JOINT PHYSICAL TEST — SHOULDER / ELBOW effect on wrist-center Z and IMU pitch")
    print("="*92)

    calib = load_calib()
    r_mount = r_from_quat_wxyz(calib["mount"]["quat_wxyz"])
    r_wb    = r_from_quat_wxyz(calib["world_bias"]["quat_wxyz"])
    r_home  = r_from_quat_wxyz(calib["home"]["quat_wxyz"]) if calib["home"]["present"] else R.identity()
    r_mount_inv = r_mount.inv(); r_wb_inv = r_wb.inv(); r_home_inv = r_home.inv()

    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
        # Safety: SAFE → ENABLE
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"})); await asyncio.sleep(0.5)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        en_ok = False; te = time.monotonic()+30.0
        while time.monotonic() < te:
            try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError: continue
            try: m = json.loads(raw)
            except: continue
            if m.get("type")=="uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok = bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok: return

        results = []
        for label, pose in TESTS:
            print(f"\n→ {label:20s}  pose={pose}")
            cmd = f"SETPOSE {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]} 22 RTR5"
            await ws.send(json.dumps({"type":"uart","cmd":cmd}))
            if not await await_setpose_done(ws, MOTION_TMO):
                print(f"  WARN: SETPOSE_DONE timeout")
            await asyncio.sleep(SETTLE_S)
            snap = await collect_snapshot(ws, r_wb_inv, r_mount_inv, r_home_inv, SAMPLES_N)
            if snap is None:
                print(f"  ERR: no valid samples"); continue
            results.append((label, pose, snap))
            print(f"  IMU pitch = {snap.get('imu_pitch', 'N/A'):+7.2f}°   FK wc_z = {snap.get('fk_live_wc_z_mm', 'N/A'):+7.2f} mm   "
                  f"FK tool_z = {snap.get('fk_live_z_mm', 'N/A'):+7.2f} mm   (n_imu={snap['n_imu']})")

        # Return to HOME (safety)
        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.5)

    # ============================================================
    # Analysis
    # ============================================================
    home_snap = results[0][2] if results and results[0][0] == "HOME" else None
    if home_snap is None:
        print("No HOME baseline — cannot compute deltas"); return

    imu_pitch_home = home_snap.get("imu_pitch")
    wc_z_home      = home_snap.get("fk_live_wc_z_mm")
    tool_z_home    = home_snap.get("fk_live_z_mm")
    fk_pitch_home  = home_snap.get("fk_live_pitch")

    print("\n" + "="*92)
    print(f"Baseline @ HOME: IMU pitch = {imu_pitch_home:+6.2f}°   FK wc_z = {wc_z_home:+7.2f} mm   "
          f"FK tool_z = {tool_z_home:+7.2f} mm   FK pitch = {fk_pitch_home:+6.2f}°")
    print("="*92)

    def interp(dwc_z, dimu_pitch):
        """Return (wc_direction, matches_user_hypothesis) strings."""
        if dwc_z is None: return "—", "—"
        wc_dir = "DOWN" if dwc_z < -1 else ("UP" if dwc_z > 1 else "≈ same")
        # User hypothesis: IMU pitch MORE POSITIVE ↔ wc DOWN
        if dimu_pitch is None: return wc_dir, "—"
        match = "match user-hyp" if (dimu_pitch > 0 and dwc_z < 0) or (dimu_pitch < 0 and dwc_z > 0) else "opposite user-hyp"
        if abs(dwc_z) < 1 or abs(dimu_pitch) < 1: match = "negligible"
        return wc_dir, match

    print(f"\n{'Test':20s}  {'ΔS':>6s}  {'ΔG':>6s}  {'ΔIMU_pit':>9s}  {'ΔFK_pit':>9s}  {'ΔFK_wc_z':>10s}  {'ΔFK_tool_z':>11s}   Interp")
    print("-"*120)
    for label, pose, snap in results[1:]:
        dS = pose[1] - HOME[1]; dG = pose[2] - HOME[2]
        dimu_p = (snap.get("imu_pitch") or 0) - (imu_pitch_home or 0) if snap.get("imu_pitch") is not None and imu_pitch_home is not None else None
        dfk_p  = (snap.get("fk_live_pitch") or 0) - (fk_pitch_home or 0) if snap.get("fk_live_pitch") is not None and fk_pitch_home is not None else None
        dwc_z  = (snap.get("fk_live_wc_z_mm") or 0) - (wc_z_home or 0) if snap.get("fk_live_wc_z_mm") is not None and wc_z_home is not None else None
        dtool_z= (snap.get("fk_live_z_mm") or 0) - (tool_z_home or 0) if snap.get("fk_live_z_mm") is not None and tool_z_home is not None else None
        wc_dir, match = interp(dwc_z, dimu_p)
        def f(x, w=9, p=2): return f"{x:+{w}.{p}f}" if x is not None else "    —    "
        print(f"{label:20s}  {dS:+6d}  {dG:+6d}  {f(dimu_p)}  {f(dfk_p)}  {f(dwc_z, 10, 2)}  {f(dtool_z, 11, 2)}   {wc_dir:8s}  {match}")

    print("\n" + "="*92)
    print("Interpretation key:")
    print("="*92)
    print("  User hypothesis: IMU pitch MORE POSITIVE (ΔIMU_pit > 0) ↔ wrist-center DOWN (ΔFK_wc_z < 0).")
    print("  'match user-hyp'   → ΔIMU_pit and ΔFK_wc_z have opposite signs (pitch+ means wc-)")
    print("  'opposite user-hyp'→ ΔIMU_pit and ΔFK_wc_z have SAME sign (pitch+ means wc+)")
    print("  'negligible'       → one of the changes is < 1 mm / 1°")

asyncio.run(main())
