#!/usr/bin/env python3
"""
verify_elbow_ab.py — A/B test for TEST_INVERT_ELBOW flag in head_assist.py.

Usage:
    python3 verify_elbow_ab.py A      # current flag state → save /tmp/elbow_A.csv
    python3 verify_elbow_ab.py B      # with flag flipped → save /tmp/elbow_B.csv
    python3 verify_elbow_ab.py compare

Each run (A or B) goes through 3 pre-poses × the same head-pitch motion:
  ramp 0 → −15° in 3 s, hold 1 s, ramp back to 0 in 3 s.
Head pitch goes DOWN then UP — simulates "user nods down then returns".
For each pose captures: servo_S, servo_G, fk_wc_z, fk_z, imu_pit timelines.

Expected observables per pose:
  - servo_S trajectory (shoulder response)
  - servo_G trajectory (elbow response)
  - fk_wc_z net descent (mm) during down-phase, net ascent during up-phase
  - lag / asymmetry between down and up phases
"""
import asyncio, json, math, ssl, sys, time, csv
import urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"

POSES = [
    ("CENTRAL", [90,  90,  90,  90, 90, 90]),
    ("ALL_UP",  [90, 145,  40,  90, 90, 90]),
    ("ALL_DOWN",[90,  40, 140,  90, 90, 90]),
]

def q_from_pitch(deg):
    x, y, z, w = R.from_euler("Y", deg, degrees=True).as_quat()
    return (w, x, y, z)

state = {"q": (1.0,0.0,0.0,0.0), "cmd_pit": 0.0, "grip": 0, "run": True, "phase": "idle"}

async def inj(ws):
    hb = 0
    while state["run"]:
        w,x,y,z = state["q"]
        hb = (hb+1) & 0xFFFF
        g = 1 if state["grip"] else 0
        btn = 0x0002 if state["grip"] else 0
        await ws.send(json.dumps({
            "mode":5,"quat_w":w,"quat_x":x,"quat_y":y,"quat_z":z,
            "grip":g,"buttons_left":btn,"buttons_right":btn,
            "heartbeat":hb,"joy_x":0,"joy_y":0,"pitch":0,"yaw":0,"intensity":255,
        }))
        await asyncio.sleep(1/60)

async def cap(ws, sink):
    t0 = time.monotonic()
    while state["run"]:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError: continue
        except: return
        try: m = json.loads(raw)
        except: continue
        if m.get("type") != "telemetry" or "servo_deg_B" not in m: continue
        try:
            vals = [float(m.get(k)) for k in ("servo_deg_B","servo_deg_S","servo_deg_G","servo_deg_Y","servo_deg_P","servo_deg_R")]
            if any((v<5 or v>175) for v in vals): continue
        except: continue
        row = {
            "t": round(time.monotonic()-t0, 4),
            "phase": state["phase"],
            "cmd_pit": state["cmd_pit"],
            "servo_S": m["servo_deg_S"], "servo_G": m["servo_deg_G"],
        }
        if m.get("fk_live_valid"):
            row["fk_wc_z"] = m.get("fk_live_wc_z_mm")
            row["fk_z"] = m.get("fk_live_z_mm")
            row["fk_pit"] = m.get("fk_live_pitch")
        sink.append(row)

async def await_setpose_done(ws, tmo=25):
    te = time.monotonic() + tmo
    while time.monotonic() < te:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError: continue
        try: m = json.loads(raw)
        except: continue
        if m.get("type") == "setpose_done": return True
    return False

async def run_pose(ws, label, pose):
    print(f"  {label} pose={pose}")
    cmd = f"SETPOSE {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]} 22 RTR5"
    await ws.send(json.dumps({"type":"uart","cmd":cmd}))
    await await_setpose_done(ws)
    await asyncio.sleep(0.8)
    state["q"] = (1.0,0.0,0.0,0.0); state["grip"] = 1; state["phase"] = "pre"; state["run"] = True
    sink = []
    tinj = asyncio.create_task(inj(ws))
    await asyncio.sleep(2.0)  # warmup
    tcap = asyncio.create_task(cap(ws, sink))

    # Sinusoidal probe: ±12° @ 0.4 Hz → peak velocity 30°/s (0.5°/tick)
    # well above headMotionDeadbandDeg=0.18°/tick. Baseline 1s, osc 6s.
    async def phase_hold(name, dur, a):
        state["phase"] = name
        state["q"] = q_from_pitch(a); state["cmd_pit"] = a
        await asyncio.sleep(dur)

    async def phase_osc(name, dur, amp_deg, freq_hz):
        state["phase"] = name
        t0 = time.monotonic()
        while time.monotonic()-t0 < dur:
            t = time.monotonic()-t0
            a = amp_deg * math.sin(2*math.pi*freq_hz*t)
            state["q"] = q_from_pitch(a); state["cmd_pit"] = a
            await asyncio.sleep(0.02)

    await phase_hold("baseline", 1.0, 0.0)
    await phase_osc("osc_pitch", 6.0, 12.0, 0.4)
    await phase_hold("settle", 0.5, 0.0)

    state["grip"] = 0; await asyncio.sleep(0.2)
    state["run"] = False
    tinj.cancel(); tcap.cancel()
    for t in (tinj, tcap):
        try: await t
        except (asyncio.CancelledError, Exception): pass
    for r in sink: r["pose_label"] = label
    return sink

def metrics_for_pose(rows, label):
    """For each phase, compute the SIGNED PEAK deviation from the baseline
    end-value. With velocity-based control the arm moves DURING the ramp,
    then rests (zero delta) during the hold. Peak during ramp is what we
    want to measure: 'how far did the arm go during the down motion'.
    Signs: negative peak on Δwc_z during ramp_down = wc descended.
    """
    out = {"pose": label}
    def phase_rows(p): return [r for r in rows if r.get("phase")==p]
    def end_val(r, k):
        for x in reversed(r):
            if x.get(k) is not None: return x[k]
        return None

    baseline = phase_rows("baseline")
    bs = {
        "servo_S": end_val(baseline, "servo_S"),
        "servo_G": end_val(baseline, "servo_G"),
        "fk_wc_z": end_val(baseline, "fk_wc_z"),
        "fk_z":    end_val(baseline, "fk_z"),
    }

    def signed_peak(phase_pr, key, base):
        if base is None: return None
        vals = [r[key] for r in phase_pr if r.get(key) is not None]
        if not vals: return None
        diffs = [v - base for v in vals]
        return max(diffs, key=lambda d: abs(d))

    def rms(vals):
        if not vals: return None
        mean = sum(vals)/len(vals)
        return math.sqrt(sum((v-mean)**2 for v in vals)/len(vals))

    def corr(xs, ys):
        if len(xs) < 10: return None
        mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
        num=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
        dx=math.sqrt(sum((x-mx)**2 for x in xs)); dy=math.sqrt(sum((y-my)**2 for y in ys))
        return num/(dx*dy) if dx*dy > 1e-9 else None

    for ph in ("osc_pitch",):
        pr = phase_rows(ph)
        if not pr: continue
        cmd_p = [r["cmd_pit"] for r in pr if r.get("servo_S") is not None]
        s = [r["servo_S"] for r in pr if r.get("servo_S") is not None]
        g = [r["servo_G"] for r in pr if r.get("servo_G") is not None]
        z = [r["fk_wc_z"] for r in pr if r.get("fk_wc_z") is not None]
        cmd_p_z = [r["cmd_pit"] for r in pr if r.get("fk_wc_z") is not None]
        # Signed peak across phase (positive excursion and negative excursion)
        peak_s_pos = max((v-bs["servo_S"]) for v in s) if s and bs["servo_S"] is not None else None
        peak_s_neg = min((v-bs["servo_S"]) for v in s) if s and bs["servo_S"] is not None else None
        peak_g_pos = max((v-bs["servo_G"]) for v in g) if g and bs["servo_G"] is not None else None
        peak_g_neg = min((v-bs["servo_G"]) for v in g) if g and bs["servo_G"] is not None else None
        peak_z_pos = max((v-bs["fk_wc_z"]) for v in z) if z and bs["fk_wc_z"] is not None else None
        peak_z_neg = min((v-bs["fk_wc_z"]) for v in z) if z and bs["fk_wc_z"] is not None else None
        out[ph] = {
            "rms_cmd_pit": rms(cmd_p),
            "rms_S": rms(s), "rms_G": rms(g), "rms_wc_z": rms(z),
            "corr_cmd_S": corr(cmd_p, s), "corr_cmd_G": corr(cmd_p, g),
            "corr_cmd_wc_z": corr(cmd_p_z, z),
            "peak_S": (peak_s_pos, peak_s_neg),
            "peak_G": (peak_g_pos, peak_g_neg),
            "peak_wc_z": (peak_z_pos, peak_z_neg),
        }
    return out

async def main_run(label_ab):
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

        all_rows = []
        per_pose = {}
        for pose_lbl, pose in POSES:
            rows = await run_pose(ws, pose_lbl, pose)
            all_rows += rows
            per_pose[pose_lbl] = metrics_for_pose(rows, pose_lbl)

        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)

    out_csv = f"/tmp/elbow_{label_ab}.csv"
    keys = sorted({k for r in all_rows for k in r.keys()})
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(all_rows)
    print(f"CSV: {out_csv}   rows={len(all_rows)}")

    print(f"\n=== METRICS {label_ab} (RMS + signed peaks + correlations, phase=osc_pitch) ===")
    for pose_lbl in ("CENTRAL", "ALL_UP", "ALL_DOWN"):
        m = per_pose[pose_lbl]
        o = m.get("osc_pitch")
        if not o:
            print(f"\n--- {pose_lbl} --- no osc data"); continue
        print(f"\n--- {pose_lbl} ---")
        def f(v, w=7, p=2): return f"{v:+{w}.{p}f}" if v is not None else "   —   "
        def fc(v): return f"{v:+6.3f}" if v is not None else "  —   "
        def fp(pair):
            a,b = pair
            return (f"{a:+7.2f}/{b:+7.2f}")
        print(f"  RMS   cmd_pit={f(o['rms_cmd_pit'])}°  servo_S={f(o['rms_S'])}°  servo_G={f(o['rms_G'])}°  fk_wc_z={f(o['rms_wc_z'])}mm")
        print(f"  CORR  cmd↔S={fc(o['corr_cmd_S'])}  cmd↔G={fc(o['corr_cmd_G'])}  cmd↔wc_z={fc(o['corr_cmd_wc_z'])}")
        print(f"  PEAK  servo_S {fp(o['peak_S'])}°  servo_G {fp(o['peak_G'])}°  wc_z {fp(o['peak_wc_z'])}mm")

    import json as _j
    with open(f"/tmp/elbow_{label_ab}.metrics.json", "w") as f:
        _j.dump(per_pose, f, indent=2, default=str)

def main_compare():
    import json as _j
    try:
        A = _j.load(open("/tmp/elbow_A.metrics.json"))
        B = _j.load(open("/tmp/elbow_B.metrics.json"))
    except Exception as e:
        print(f"Missing metrics file: {e}. Run A and B first."); return

    print("=" * 100)
    print("A/B COMPARISON (A = TEST_INVERT_ELBOW=False, B = TEST_INVERT_ELBOW=True)")
    print("=" * 100)
    for pose in ("CENTRAL", "ALL_UP", "ALL_DOWN"):
        a = A.get(pose, {}).get("osc_pitch", {}); b = B.get(pose, {}).get("osc_pitch", {})
        if not a or not b:
            print(f"\n--- {pose} --- missing data"); continue
        print(f"\n--- {pose} ---")
        def f(x, w=7, p=3): return f"{x:+{w}.{p}f}" if x is not None else "  —    "
        def fp(pair):
            if not pair: return "  —  "
            a_,b_ = pair; return f"{a_:+7.2f}/{b_:+7.2f}"
        print(f"  RMS fk_wc_z (mm)   A={f(a.get('rms_wc_z'), 6, 2)}   B={f(b.get('rms_wc_z'), 6, 2)}   ratio B/A = {(b['rms_wc_z']/a['rms_wc_z']) if a.get('rms_wc_z') else 'N/A':.3f}" if a.get('rms_wc_z') else f"  RMS fk_wc_z: A={f(a.get('rms_wc_z'))}  B={f(b.get('rms_wc_z'))}")
        print(f"  corr(cmd,wc_z)     A={f(a.get('corr_cmd_wc_z'))}   B={f(b.get('corr_cmd_wc_z'))}")
        print(f"  corr(cmd,servo_G)  A={f(a.get('corr_cmd_G'))}   B={f(b.get('corr_cmd_G'))}")
        print(f"  corr(cmd,servo_S)  A={f(a.get('corr_cmd_S'))}   B={f(b.get('corr_cmd_S'))}")
        print(f"  peak servo_G +/−   A={fp(a.get('peak_G'))}   B={fp(b.get('peak_G'))}")
        print(f"  peak wc_z    +/−   A={fp(a.get('peak_wc_z'))}   B={fp(b.get('peak_wc_z'))}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: verify_elbow_ab.py {A|B|compare}"); sys.exit(1)
    mode = sys.argv[1]
    if mode in ("A", "B"):
        asyncio.run(main_run(mode))
    elif mode == "compare":
        main_compare()
    else:
        print(f"unknown mode: {mode}"); sys.exit(1)
