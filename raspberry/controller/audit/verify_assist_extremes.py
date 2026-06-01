#!/usr/bin/env python3
"""
verify_assist_extremes.py — ASSIST extreme-workspace tests (TEST 8-11).

Pre-poses selected within measured virtual limits:
  base[35,125] spalla[33,148] gomito[27,142] yaw[45,135] pitch[60,145] roll[52,142]
ALL UP   = [90, 145, 40, 90, 90, 90]   (spalla alta, gomito esteso in avanti)
ALL DOWN = [90, 40,  140, 90, 90, 90]  (spalla bassa, gomito piegato)

Same injection + capture + metrics pipeline as verify_assist_mode.py.
Additional per-test diagnostic: reach_xy (at start), commanded δ/tick, clamp
saturation indicator.

No code change on firmware / routing_config / head_assist.
"""
import asyncio, json, math, ssl, time, csv, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"
OUT_CSV = "/tmp/verify_assist_extremes.csv"

INTENT_HZ = 60
SAMPLE_HZ = 100
SETTLE_S  = 0.9
MOTION_TMO = 25.0

# Same geometry helpers as main script
def q_identity():       return (1.0, 0.0, 0.0, 0.0)
def q_from_yaw_deg(d):  x,y,z,w = R.from_euler("Z", d, degrees=True).as_quat(); return (w,x,y,z)
def q_from_pitch_deg(d): x,y,z,w = R.from_euler("Y", d, degrees=True).as_quat(); return (w,x,y,z)

TESTS = [
    {
        "id": 8, "name": "ALL UP + yaw osc ±8° @ 0.6 Hz",
        "pre_pose": [90, 145, 40, 90, 90, 90],
        "phases": [
            ("baseline", 1.0, lambda t: q_identity()),
            ("osc_yaw",  6.0, lambda t: q_from_yaw_deg(8.0 * math.sin(2*math.pi*0.6*t))),
        ],
    },
    {
        "id": 9, "name": "ALL DOWN + yaw osc ±8° @ 0.6 Hz",
        "pre_pose": [90, 40, 140, 90, 90, 90],
        "phases": [
            ("baseline", 1.0, lambda t: q_identity()),
            ("osc_yaw",  6.0, lambda t: q_from_yaw_deg(8.0 * math.sin(2*math.pi*0.6*t))),
        ],
    },
    {
        "id": 10, "name": "ALL UP + pitch osc ±10° @ 0.5 Hz",
        "pre_pose": [90, 145, 40, 90, 90, 90],
        "phases": [
            ("baseline", 1.0, lambda t: q_identity()),
            ("osc_pitch", 6.0, lambda t: q_from_pitch_deg(10.0 * math.sin(2*math.pi*0.5*t))),
        ],
    },
    {
        "id": 11, "name": "ALL DOWN + pitch osc ±10° @ 0.5 Hz",
        "pre_pose": [90, 40, 140, 90, 90, 90],
        "phases": [
            ("baseline", 1.0, lambda t: q_identity()),
            ("osc_pitch", 6.0, lambda t: q_from_pitch_deg(10.0 * math.sin(2*math.pi*0.5*t))),
        ],
    },
]

def load_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]
def r_from_quat_wxyz(q): return R.from_quat(q_wxyz_to_xyzw(q))
def wrap_deg(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a

state = {
    "target_quat_wxyz": (1.0, 0.0, 0.0, 0.0),
    "target_yaw_cmd_deg": 0.0, "target_pitch_cmd_deg": 0.0, "target_roll_cmd_deg": 0.0,
    "phase_name": "idle", "assist_grip": 0, "injector_run": True,
}

async def intent_injector(ws):
    dt = 1.0 / INTENT_HZ; hb = 0
    while state["injector_run"]:
        qw, qx, qy, qz = state["target_quat_wxyz"]
        hb = (hb + 1) & 0xFFFF
        grip = 1 if state["assist_grip"] else 0
        btn  = 0x0002 if state["assist_grip"] else 0
        await ws.send(json.dumps({
            "mode": 5, "quat_w": qw, "quat_x": qx, "quat_y": qy, "quat_z": qz,
            "grip": grip, "buttons_left": btn, "buttons_right": btn,
            "heartbeat": hb, "joy_x":0,"joy_y":0,"pitch":0,"yaw":0, "intensity": 255,
        }))
        await asyncio.sleep(dt)

async def telemetry_capture(ws, r_wb_inv, r_mount_inv, r_home_inv, sink):
    TOOL = [0.06, 0.0, 0.0]
    t0 = time.monotonic()
    while state["injector_run"]:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError: continue
        except Exception: return
        try: m = json.loads(raw)
        except: continue
        if m.get("type") != "telemetry": continue
        if "servo_deg_B" not in m: continue
        try:
            vals = [float(m.get(k)) for k in ("servo_deg_B","servo_deg_S","servo_deg_G","servo_deg_Y","servo_deg_P","servo_deg_R")]
            if any((v<5 or v>175) for v in vals): continue
        except: continue
        t_rel = time.monotonic() - t0
        row = {
            "t": round(t_rel, 4), "phase": state["phase_name"],
            "cmd_yaw": round(state["target_yaw_cmd_deg"], 3),
            "cmd_pit": round(state["target_pitch_cmd_deg"], 3),
            "servo_B": m.get("servo_deg_B"), "servo_S": m.get("servo_deg_S"),
            "servo_G": m.get("servo_deg_G"), "servo_Y": m.get("servo_deg_Y"),
            "servo_P": m.get("servo_deg_P"), "servo_R": m.get("servo_deg_R"),
        }
        if m.get("fk_live_valid"):
            row.update({
                "fk_x": m.get("fk_live_x_mm"), "fk_y": m.get("fk_live_y_mm"), "fk_z": m.get("fk_live_z_mm"),
                "fk_yaw": m.get("fk_live_yaw"), "fk_pit": m.get("fk_live_pitch"), "fk_rol": m.get("fk_live_roll"),
                "fk_wc_x": m.get("fk_live_wc_x_mm"), "fk_wc_y": m.get("fk_live_wc_y_mm"), "fk_wc_z": m.get("fk_live_wc_z_mm"),
            })
        if m.get("imu_valid") is True and m.get("imu_q_w") is not None:
            qi = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
            r_ee = r_home_inv * r_wb_inv * r_from_quat_wxyz(qi) * r_mount_inv
            ypr = r_ee.as_euler("ZYX", degrees=True)
            row["imu_yaw"] = round(float(ypr[0]), 3)
            row["imu_pit"] = round(float(ypr[1]), 3)
            row["imu_rol"] = round(float(ypr[2]), 3)
        sink.append(row)

async def run_phases(phases):
    t0 = time.monotonic()
    for pname, dur, qfn in phases:
        state["phase_name"] = pname
        t_phase0 = time.monotonic()
        while True:
            t = time.monotonic() - t_phase0
            if t >= dur: break
            q = qfn(t)
            state["target_quat_wxyz"] = q
            r = r_from_quat_wxyz(q)
            y, p, r_ = r.as_euler("ZYX", degrees=True)
            state["target_yaw_cmd_deg"] = float(y)
            state["target_pitch_cmd_deg"] = float(p)
            state["target_roll_cmd_deg"] = float(r_)
            await asyncio.sleep(0.02)
    state["phase_name"] = "idle"

async def await_setpose_done(ws, tmo):
    t_end = time.monotonic() + tmo
    while time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError: continue
        try: m = json.loads(raw)
        except: continue
        if m.get("type") == "setpose_done": return True
    return False

def compute_tracking_metrics(rows, phase_name, sig_key, cmd_key):
    xs = [r for r in rows if r.get("phase")==phase_name and r.get(sig_key) is not None and r.get(cmd_key) is not None]
    if len(xs) < 30: return None
    ts = [r["t"] for r in xs]
    cmd = [r[cmd_key] for r in xs]; sig = [r[sig_key] for r in xs]
    cm = sum(cmd)/len(cmd); sm = sum(sig)/len(sig)
    cmd_c = [v-cm for v in cmd]; sig_c = [v-sm for v in sig]
    cmd_rms = math.sqrt(sum(v*v for v in cmd_c)/len(cmd_c)) or 1e-9
    sig_rms = math.sqrt(sum(v*v for v in sig_c)/len(sig_c))
    dt = (ts[-1] - ts[0]) / max(1, (len(ts)-1))
    max_lag = int(0.5 / max(dt, 1e-3))
    best_lag = 0; best_c = -1e9
    for lag in range(0, max_lag+1):
        num = sum(cmd_c[i]*sig_c[i+lag] for i in range(len(sig_c)-lag))
        c = num / (len(sig_c) - lag)
        if c > best_c: best_c = c; best_lag = lag
    return {
        "n": len(xs), "cmd_rms": round(cmd_rms, 2), "sig_rms": round(sig_rms, 2),
        "amp_ratio": round(sig_rms / cmd_rms, 3), "lag_ms": round(best_lag * dt * 1000, 0),
    }

def coupling_analysis(rows, phase_name, ignore=None):
    xs = [r for r in rows if r.get("phase")==phase_name]
    base_rows = [r for r in rows if r.get("phase")=="baseline"]
    out = {}
    for k in ("servo_B","servo_S","servo_G","servo_Y","servo_P","servo_R"):
        if k == ignore: continue
        bv = [r[k] for r in base_rows if r.get(k) is not None]
        pv = [r[k] for r in xs if r.get(k) is not None]
        if not bv or not pv: continue
        base = sum(bv[-10:]) / min(10, len(bv))
        dev = max(pv, key=lambda v: abs(v-base)) - base
        out[k] = round(dev, 2)
    return out

def start_reach_xy(sink):
    """reach_xy at first baseline sample (wc projected onto base XY minus shoulder origin)."""
    # Shoulder origin ~ (0, 0, 0.094). reach_xy = sqrt((wc_x-0)² + (wc_y-0)²) in m.
    xs = [r for r in sink if r.get("phase")=="baseline" and r.get("fk_wc_x") is not None]
    if not xs: return None
    r0 = xs[0]
    return round(math.hypot(r0["fk_wc_x"]/1000.0, r0["fk_wc_y"]/1000.0), 4)

async def run_test(ws, r_wb_inv, r_mount_inv, r_home_inv, test):
    print(f"\n=== TEST {test['id']} — {test['name']} ===")
    pp = test["pre_pose"]
    await ws.send(json.dumps({"type":"uart","cmd": f"SETPOSE {pp[0]} {pp[1]} {pp[2]} {pp[3]} {pp[4]} {pp[5]} 22 RTR5"}))
    print(f"  pre-pose: {pp}")
    if not await await_setpose_done(ws, MOTION_TMO):
        print("  WARN: SETPOSE_DONE timeout during pre-pose")
    await asyncio.sleep(SETTLE_S)

    state["target_quat_wxyz"] = (1.0, 0.0, 0.0, 0.0)
    state["assist_grip"] = 1
    state["phase_name"] = "pre"
    state["injector_run"] = True

    sink = []
    inj = asyncio.create_task(intent_injector(ws))
    await asyncio.sleep(2.2)  # warmup
    cap = asyncio.create_task(telemetry_capture(ws, r_wb_inv, r_mount_inv, r_home_inv, sink))
    await run_phases(test["phases"])

    state["assist_grip"] = 0
    await asyncio.sleep(0.2)
    state["injector_run"] = False
    inj.cancel(); cap.cancel()
    for t in (inj, cap):
        try: await t
        except (asyncio.CancelledError, Exception): pass
    return sink

async def main():
    print("=" * 92)
    print("ASSIST EXTREME WORKSPACE tests (8-11)")
    print("=" * 92)

    calib = load_calib()
    r_mount = r_from_quat_wxyz(calib["mount"]["quat_wxyz"])
    r_wb    = r_from_quat_wxyz(calib["world_bias"]["quat_wxyz"])
    r_home  = r_from_quat_wxyz(calib["home"]["quat_wxyz"]) if calib["home"]["present"] else R.identity()
    r_mount_inv = r_mount.inv(); r_wb_inv = r_wb.inv(); r_home_inv = r_home.inv()

    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
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

        all_rows = []; test_results = {}
        for t in TESTS:
            sink = await run_test(ws, r_wb_inv, r_mount_inv, r_home_inv, t)
            for r in sink: r["test_id"] = t["id"]
            all_rows += sink
            test_results[t["id"]] = (t, sink)

        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)

    # Save CSV
    keys = sorted({k for r in all_rows for k in r.keys()})
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(all_rows)
    print(f"\nCSV: {OUT_CSV}  ({len(all_rows)} rows)")

    # Reach_xy + REACH_REF
    REACH_REF = 0.15

    def reach_scale(rxy):
        return REACH_REF / max(rxy, 0.05) if rxy else None

    print("\n" + "=" * 92)
    print("METRICS PER TEST")
    print("=" * 92)
    rows_summary = []
    for tid in (8, 9, 10, 11):
        t, sink = test_results[tid]
        rxy = start_reach_xy(sink)
        rsc = reach_scale(rxy)
        print(f"\n--- TEST {tid} — {t['name']} ---")
        print(f"  pre-pose: {t['pre_pose']}")
        print(f"  reach_xy at start: {rxy} m   reach_scale = REACH_REF/reach_xy = {round(rsc,2) if rsc else '—'}")
        phase = "osc_yaw" if tid in (8,9) else "osc_pitch"
        cmd_key = "cmd_yaw" if tid in (8,9) else "cmd_pit"

        # Primary target per test
        if tid in (8,9):
            primary_joints = ("servo_B",)
            related_imu = "imu_yaw"
            related_fk  = "fk_yaw"
        else:
            primary_joints = ("servo_S", "servo_G")
            related_imu = "imu_pit"
            related_fk  = "fk_z"  # FK z is the vertical proxy for pitch motion

        for k in primary_joints + (related_fk, related_imu):
            m = compute_tracking_metrics(sink, phase, k, cmd_key)
            if m:
                print(f"  {k:8s}  amp_ratio={m['amp_ratio']:.3f}  lag={int(m['lag_ms'])}ms  sig_rms={m['sig_rms']}°  cmd_rms={m['cmd_rms']}°")
                rows_summary.append({"test_id": tid, "channel": k, **m, "reach_xy_m": rxy, "reach_scale": rsc})

        cpl = coupling_analysis(sink, phase)
        print(f"  coupling (max |Δ°| vs baseline, all joints): {cpl}")

    # Pairwise comparison UP vs DOWN
    print("\n" + "=" * 92)
    print("COMPARATIVE ANALYSIS — ALL UP vs ALL DOWN")
    print("=" * 92)
    def pair_metric(tid_up, tid_dn, phase, ch, cmd_key):
        m_up = compute_tracking_metrics(test_results[tid_up][1], phase, ch, cmd_key)
        m_dn = compute_tracking_metrics(test_results[tid_dn][1], phase, ch, cmd_key)
        if m_up and m_dn:
            dr = m_up["amp_ratio"] - m_dn["amp_ratio"]
            dl = m_up["lag_ms"] - m_dn["lag_ms"]
            print(f"  {ch:8s} UP amp={m_up['amp_ratio']:.3f} lag={int(m_up['lag_ms'])}ms  |  DOWN amp={m_dn['amp_ratio']:.3f} lag={int(m_dn['lag_ms'])}ms  |  Δ amp={dr:+.3f} Δ lag={dl:+.0f}ms")

    print("\n[Yaw oscillation comparison]")
    for ch in ("servo_B", "imu_yaw", "fk_yaw"):
        pair_metric(8, 9, "osc_yaw", ch, "cmd_yaw")

    print("\n[Pitch oscillation comparison]")
    for ch in ("servo_S", "servo_G", "imu_pit", "fk_z"):
        pair_metric(10, 11, "osc_pitch", ch, "cmd_pit")

    # Reach-scale estimator (theoretical at start)
    r_up_y = start_reach_xy(test_results[8][1]); r_dn_y = start_reach_xy(test_results[9][1])
    r_up_p = start_reach_xy(test_results[10][1]); r_dn_p = start_reach_xy(test_results[11][1])
    print("\n[Reach scaling at start of each test]")
    for label, val in (("UP/yaw", r_up_y), ("DOWN/yaw", r_dn_y), ("UP/pitch", r_up_p), ("DOWN/pitch", r_dn_p)):
        print(f"  {label:10s}  reach_xy={val} m   scale={round(REACH_REF/max(val,0.05),2) if val else '—'}")

asyncio.run(main())
