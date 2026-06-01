#!/usr/bin/env python3
"""
verify_imu_alignment.py — HOME & pose sweep, compute Δ(IMU − FK) for each pose.

Answers the questions:
  1. At HOME, is the IMU base-frame orientation (R_ee derived via
     R_world_bias^-1 · R_imu · R_mount^-1) consistent with FK live orientation?
  2. Across different poses, is the Δ constant (→ missing HOME zero-ing) or
     orientation-dependent (→ wrong frame transform) or drifting (→ world_bias
     stale)?

Procedure per pose:
  • send SETPOSE (virtual angles) → wait for SETPOSE_DONE or timeout
  • settle ~500 ms
  • average N telemetry frames (FK + transformed IMU)
  • record Δ
Analysis at the end cross-correlates the Δ across poses.

Read-only analysis: NO firmware change, NO math change.
"""
import asyncio, json, math, ssl, time, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS  = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"

POSES = [
    ("HOME",        [90,  90,  90,  90,  90,  90]),
    ("Arm ext A",   [90, 100,  80,  90,  90,  90]),
    ("Yaw -30",     [90,  90,  90,  60,  90,  90]),
    ("Pitch -15",   [90,  90,  90,  90,  75,  90]),
    ("Mixed",       [105, 100,  85,  75, 100,  95]),
]

SETTLE_S     = 0.6
SAMPLE_COUNT = 40          # averaging window (~0.4 s at 100 Hz)
MOTION_TMO   = 20.0        # seconds; timeout if SETPOSE_DONE never comes

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]

def load_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def r_from_quat_wxyz(q):
    return R.from_quat(q_wxyz_to_xyzw(q))

def wrap_deg(a):
    while a > 180:  a -= 360
    while a < -180: a += 360
    return a

async def drain(ws, dur_s):
    t_end = time.monotonic() + dur_s
    while time.monotonic() < t_end:
        try:
            await asyncio.wait_for(ws.recv(), timeout=0.05)
        except asyncio.TimeoutError:
            return
        except Exception:
            return

async def await_setpose_done(ws, timeout_s):
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        try: m = json.loads(raw)
        except Exception: continue
        if m.get("type") == "setpose_done":
            return True
    return False

async def sample_pose(ws, r_wb_inv, r_mount_inv, n):
    """Collect n telemetry frames with both FK live and IMU quat, return averaged."""
    fk_xs, fk_ys, fk_zs = [], [], []
    fk_ys_deg, fk_ps_deg, fk_rs_deg = [], [], []
    imu_ys_deg, imu_ps_deg, imu_rs_deg = [], [], []
    imu_xs, imu_ys_mm, imu_zs = [], [], []  # tool-tip = fk_wc + R_ee * TOOL_OFFSET
    TOOL = [0.06, 0.0, 0.0]
    collected = 0
    t_end = time.monotonic() + 6.0
    while collected < n and time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError: continue
        try: m = json.loads(raw)
        except Exception: continue
        if m.get("type") != "telemetry": continue
        if m.get("imu_valid") is not True: continue
        if "fk_live_x_mm" not in m or "imu_q_w" not in m: continue
        fk_xs.append(m["fk_live_x_mm"]); fk_ys.append(m["fk_live_y_mm"]); fk_zs.append(m["fk_live_z_mm"])
        fk_ys_deg.append(m["fk_live_yaw"]); fk_ps_deg.append(m["fk_live_pitch"]); fk_rs_deg.append(m["fk_live_roll"])
        q_imu = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
        r_imu = r_from_quat_wxyz(q_imu)
        r_ee  = r_wb_inv * r_imu * r_mount_inv
        ypr = r_ee.as_euler("ZYX", degrees=True)
        imu_ys_deg.append(ypr[0]); imu_ps_deg.append(ypr[1]); imu_rs_deg.append(ypr[2])
        wc_m = [m["fk_live_wc_x_mm"]/1000, m["fk_live_wc_y_mm"]/1000, m["fk_live_wc_z_mm"]/1000]
        tip_m = [wc_m[i] + (r_ee.as_matrix() @ TOOL)[i] for i in range(3)]
        imu_xs.append(tip_m[0]*1000); imu_ys_mm.append(tip_m[1]*1000); imu_zs.append(tip_m[2]*1000)
        collected += 1
    if collected == 0:
        return None
    def avg(xs): return sum(xs)/len(xs)
    return {
        "n": collected,
        "fk":  {"x": avg(fk_xs), "y": avg(fk_ys), "z": avg(fk_zs),
                "yaw": avg(fk_ys_deg), "pitch": avg(fk_ps_deg), "roll": avg(fk_rs_deg)},
        "imu": {"x": avg(imu_xs), "y": avg(imu_ys_mm), "z": avg(imu_zs),
                "yaw": avg(imu_ys_deg), "pitch": avg(imu_ps_deg), "roll": avg(imu_rs_deg)},
    }

async def main():
    print("=" * 80)
    print("IMU vs FK alignment test — HOME + pose sweep")
    print("=" * 80)

    calib = load_calib()
    r_mount = r_from_quat_wxyz(calib["mount"]["quat_wxyz"])
    r_wb    = r_from_quat_wxyz(calib["world_bias"]["quat_wxyz"])
    r_mount_inv = r_mount.inv(); r_wb_inv = r_wb.inv()
    print(f"Loaded calib:")
    print(f"  mount     YPR deg = {tuple(round(v,3) for v in r_mount.as_euler('ZYX', degrees=True))}  present={calib['mount']['present']}")
    print(f"  worldBias YPR deg = {tuple(round(v,3) for v in r_wb.as_euler('ZYX', degrees=True))}  present={calib['world_bias']['present']}")
    print()

    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
        # Safety: SAFE, then ENABLE. ENABLE may take ~20s while PWM ramps.
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"}))
        await drain(ws, 1.0)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        # Wait up to 30 s for ENABLE ack
        en_ok = False
        t_end = time.monotonic() + 30.0
        while time.monotonic() < t_end:
            try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError: continue
            try: m = json.loads(raw)
            except Exception: continue
            if m.get("type") == "uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok = bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok:
            print("ABORT: enable failed")
            return

        results = []
        for name, angles in POSES:
            b,s,g,y,p,r = angles
            # Conservative velocity (25 deg/s) — SETPOSE interpreta "vel_deg_s"
            cmd = f"SETPOSE {b} {s} {g} {y} {p} {r} 25 RTR5"
            await ws.send(json.dumps({"type":"uart","cmd":cmd}))
            print(f"[{name}] SETPOSE {angles} sent — waiting SETPOSE_DONE…")
            done = await await_setpose_done(ws, MOTION_TMO)
            if not done:
                print(f"  WARN: SETPOSE_DONE timeout after {MOTION_TMO}s — using available samples")
            await asyncio.sleep(SETTLE_S)
            await drain(ws, 0.1)
            snap = await sample_pose(ws, r_wb_inv, r_mount_inv, SAMPLE_COUNT)
            if snap is None:
                print(f"  ERR: no valid samples for {name}")
                continue
            results.append((name, angles, snap))

        # Return to HOME
        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await drain(ws, 0.5)

    if not results:
        print("No results.")
        return

    # Table
    print("\n" + "=" * 80)
    print("Per-pose results (averaged over {} samples each)".format(SAMPLE_COUNT))
    print("=" * 80)
    hdr = f"{'pose':12s} {'src':3s} | {'X':>9s} {'Y':>9s} {'Z':>9s} | {'Yaw':>8s} {'Pitch':>8s} {'Roll':>8s}"
    print(hdr); print("-"*len(hdr))
    dxs, dys, dzs, dyaw, dpitch, droll = [], [], [], [], [], []
    for name, angles, s in results:
        fk = s["fk"]; im = s["imu"]
        dx = im["x"] - fk["x"]; dy = im["y"] - fk["y"]; dz = im["z"] - fk["z"]
        dyw= wrap_deg(im["yaw"]   - fk["yaw"])
        dpt= wrap_deg(im["pitch"] - fk["pitch"])
        drl= wrap_deg(im["roll"]  - fk["roll"])
        dxs.append(dx); dys.append(dy); dzs.append(dz)
        dyaw.append(dyw); dpitch.append(dpt); droll.append(drl)
        print(f"{name:12s} FK  | {fk['x']:+9.2f} {fk['y']:+9.2f} {fk['z']:+9.2f} | {fk['yaw']:+8.2f} {fk['pitch']:+8.2f} {fk['roll']:+8.2f}")
        print(f"{name:12s} IMU | {im['x']:+9.2f} {im['y']:+9.2f} {im['z']:+9.2f} | {im['yaw']:+8.2f} {im['pitch']:+8.2f} {im['roll']:+8.2f}")
        print(f"{name:12s} Δ   | {dx:+9.2f} {dy:+9.2f} {dz:+9.2f} | {dyw:+8.2f} {dpt:+8.2f} {drl:+8.2f}")
        print()

    # Analysis: is Δ constant across poses?
    def stat(xs):
        m = sum(xs)/len(xs)
        v = sum((x-m)**2 for x in xs)/len(xs)
        return m, math.sqrt(v), max(abs(x) for x in xs)

    print("=" * 80)
    print("Bias / pose-dependency analysis (how much Δ varies across poses)")
    print("=" * 80)
    for name, series in [
        ("ΔX mm",     dxs), ("ΔY mm",     dys), ("ΔZ mm",     dzs),
        ("ΔYaw °",    dyaw), ("ΔPitch °",  dpitch), ("ΔRoll °",   droll),
    ]:
        mu, sd, mx = stat(series)
        print(f"  {name:12s}  mean={mu:+8.2f}  std={sd:6.3f}  max|.|={mx:8.2f}")

    print()
    print("=" * 80)
    print("Conclusion (automatic)")
    print("=" * 80)
    # Rules:
    #   - If mean(Δrot) ≠ 0 but std(Δrot) small → constant bias (missing HOME zero)
    #   - If std(Δrot) large → pose-dependent (frame transform wrong or mount/world_bias stale)
    #   - If both small → aligned
    mean_rot = [stat(dyaw)[0], stat(dpitch)[0], stat(droll)[0]]
    std_rot  = [stat(dyaw)[1], stat(dpitch)[1], stat(droll)[1]]
    mean_pos = [stat(dxs)[0], stat(dys)[0], stat(dzs)[0]]
    max_bias_rot = max(abs(v) for v in mean_rot)
    max_spread_rot = max(std_rot)
    print(f"  max |mean Δrot| = {max_bias_rot:.2f}°   max std(Δrot) = {max_spread_rot:.2f}°")
    print(f"  mean Δpos (mm)  = X:{mean_pos[0]:+.2f}  Y:{mean_pos[1]:+.2f}  Z:{mean_pos[2]:+.2f}")
    print()
    if max_bias_rot < 2.0 and max_spread_rot < 1.5:
        print("  → IMU correttamente allineata al base frame: Δrot piccolo e costante.")
    elif max_bias_rot >= 2.0 and max_spread_rot < 1.5:
        print("  → BIAS COSTANTE su Δrot. Ipotesi: HOME zero-ing mancante o world_bias")
        print("    stale (magnetometro derivato). Il termine mancante è una rotazione")
        print("    globale a monte, non un errore della pipeline per-posa.")
        print("    Fix minimo: ricalibrazione 'home quat' che aggiunge R_home^-1 al chain:")
        print("        R_ee = R_home^-1 · R_world_bias^-1 · R_imu · R_mount^-1")
    elif max_spread_rot >= 1.5:
        print("  → Δrot POSE-DEPENDENT. Ipotesi: mount calibration errata oppure")
        print("    ordine moltiplicazione R_world_bias/R_mount sbagliato per questa")
        print("    geometria. Non risolvibile con una rotazione globale costante.")
        print("    Fix possibile: ricalibrare mount con sweep di pose (già disponibile:")
        print("    raspberry/controller/imu_analytics/validate_imu_vs_ee.py).")
    else:
        print("  → Condizione ambigua — vedi spread/bias numerici sopra.")

asyncio.run(main())
