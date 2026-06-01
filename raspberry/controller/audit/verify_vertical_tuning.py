#!/usr/bin/env python3
"""
verify_vertical_tuning.py — A/B/C/D vertical-motion tuning test.

Usage:
    python3 verify_vertical_tuning.py {baseline|b1|b2|b3}
    python3 verify_vertical_tuning.py compare

For each label, runs 3 pre-poses (CENTRAL, ALL_UP, ALL_DOWN) with a pitch
sinusoid probe (±12° @ 0.4 Hz), captures servo+FK+IMU, saves per-label
/tmp/vertical_<label>.csv + /tmp/vertical_<label>.metrics.json.

`compare` loads all 4 json files and prints a comparison table.
"""
import asyncio, json, math, ssl, sys, time, csv
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
POSES = [
    ("CENTRAL",  [90,  90,  90, 90, 90, 90]),
    ("ALL_UP",   [90, 145,  40, 90, 90, 90]),
    ("ALL_DOWN", [90,  40, 140, 90, 90, 90]),
]

def q_from_pitch(deg):
    x, y, z, w = R.from_euler("Y", deg, degrees=True).as_quat()
    return (w, x, y, z)

state = {"q": (1.0,0.0,0.0,0.0), "cmd_pit": 0.0, "grip": 0, "run": True, "phase": "idle"}

async def inj(ws):
    hb = 0
    while state["run"]:
        w,x,y,z = state["q"]; hb = (hb+1)&0xFFFF
        g = 1 if state["grip"] else 0; btn = 0x0002 if state["grip"] else 0
        await ws.send(json.dumps({"mode":5,"quat_w":w,"quat_x":x,"quat_y":y,"quat_z":z,"grip":g,
                                  "buttons_left":btn,"buttons_right":btn,"heartbeat":hb,
                                  "joy_x":0,"joy_y":0,"pitch":0,"yaw":0,"intensity":255}))
        await asyncio.sleep(1/60)

async def cap(ws, sink):
    t0 = time.monotonic()
    while state["run"]:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError: continue
        except: return
        try: m = json.loads(raw)
        except: continue
        if m.get("type")!="telemetry" or "servo_deg_B" not in m: continue
        try:
            v = [float(m[k]) for k in ("servo_deg_B","servo_deg_S","servo_deg_G","servo_deg_Y","servo_deg_P","servo_deg_R")]
            if any(x<5 or x>175 for x in v): continue
        except: continue
        row = {"t": round(time.monotonic()-t0,4), "phase": state["phase"], "cmd_pit": state["cmd_pit"],
               "servo_S": m["servo_deg_S"], "servo_G": m["servo_deg_G"]}
        if m.get("fk_live_valid"):
            row["fk_wc_z"] = m.get("fk_live_wc_z_mm"); row["fk_z"] = m.get("fk_live_z_mm")
        sink.append(row)

async def await_sp_done(ws, tmo=25):
    te = time.monotonic()+tmo
    while time.monotonic()<te:
        try: raw = await asyncio.wait_for(ws.recv(),timeout=1.0)
        except asyncio.TimeoutError: continue
        try: m = json.loads(raw)
        except: continue
        if m.get("type")=="setpose_done": return True
    return False

async def run_pose(ws, label, pose):
    print(f"  {label} pose={pose}")
    await ws.send(json.dumps({"type":"uart","cmd":f"SETPOSE {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]} 22 RTR5"}))
    await await_sp_done(ws)
    await asyncio.sleep(0.8)
    state["q"] = (1.0,0.0,0.0,0.0); state["grip"]=1; state["phase"]="pre"; state["run"]=True
    sink = []
    tinj = asyncio.create_task(inj(ws))
    await asyncio.sleep(2.0)  # warmup
    tcap = asyncio.create_task(cap(ws, sink))

    # Hold 1s baseline
    state["phase"]="baseline"; state["q"]=q_from_pitch(0.0); state["cmd_pit"]=0.0
    await asyncio.sleep(1.0)
    # 6 s sinusoid ±12° @ 0.4 Hz
    state["phase"]="osc_pitch"
    t0=time.monotonic()
    while time.monotonic()-t0<6.0:
        t = time.monotonic()-t0
        a = 12.0 * math.sin(2*math.pi*0.4*t)
        state["q"]=q_from_pitch(a); state["cmd_pit"]=a
        await asyncio.sleep(0.02)
    # 0.5 s settle
    state["phase"]="settle"; state["q"]=q_from_pitch(0.0); state["cmd_pit"]=0.0
    await asyncio.sleep(0.5)

    state["grip"]=0; await asyncio.sleep(0.2)
    state["run"]=False
    tinj.cancel(); tcap.cancel()
    for t in (tinj, tcap):
        try: await t
        except (asyncio.CancelledError, Exception): pass
    for r in sink: r["pose_label"]=label
    return sink

def rms(xs):
    if not xs: return None
    m = sum(xs)/len(xs); return math.sqrt(sum((v-m)**2 for v in xs)/len(xs))

def corr(xs, ys):
    if len(xs)<10: return None
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    num=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    dx=math.sqrt(sum((x-mx)**2 for x in xs)); dy=math.sqrt(sum((y-my)**2 for y in ys))
    return num/(dx*dy) if dx*dy>1e-9 else None

def best_lag_ms(ts, cmd, sig):
    if len(ts)<30: return None
    cm=sum(cmd)/len(cmd); sm=sum(sig)/len(sig)
    c_=[v-cm for v in cmd]; s_=[v-sm for v in sig]
    dt=(ts[-1]-ts[0])/max(1,(len(ts)-1))
    max_lag=int(0.8/max(dt,1e-3)); best=-1e9; bl=0
    for lag in range(0,max_lag+1):
        num=sum(c_[i]*s_[i+lag] for i in range(len(s_)-lag))
        c=num/(len(s_)-lag)
        if c>best: best=c; bl=lag
    return round(bl*dt*1000,0)

def metrics_for_pose(rows, label):
    out = {"pose": label}
    osc = [r for r in rows if r["phase"]=="osc_pitch" and r.get("servo_S") is not None]
    if len(osc)<30: return out
    bl = [r for r in rows if r["phase"]=="baseline" and r.get("servo_S") is not None]
    bs_S = bl[-1]["servo_S"] if bl else osc[0]["servo_S"]
    bs_G = bl[-1]["servo_G"] if bl else osc[0]["servo_G"]
    bs_z = (bl[-1].get("fk_wc_z") if bl else osc[0].get("fk_wc_z")) or 0.0

    ts = [r["t"] for r in osc]
    cp = [r["cmd_pit"] for r in osc]
    ss = [r["servo_S"] for r in osc]; sg = [r["servo_G"] for r in osc]
    zs = [r.get("fk_wc_z") for r in osc if r.get("fk_wc_z") is not None]
    cp_z = [r["cmd_pit"] for r in osc if r.get("fk_wc_z") is not None]
    ts_z = [r["t"] for r in osc if r.get("fk_wc_z") is not None]

    out["rms_cmd_pit"] = rms(cp)
    out["rms_S"] = rms(ss); out["rms_G"] = rms(sg)
    out["rms_wc_z"] = rms(zs) if zs else None
    out["corr_cmd_S"] = corr(cp, ss); out["corr_cmd_G"] = corr(cp, sg)
    out["corr_cmd_wc_z"] = corr(cp_z, zs) if zs else None
    out["lag_ms_S"] = best_lag_ms(ts, cp, ss)
    out["lag_ms_G"] = best_lag_ms(ts, cp, sg)
    out["lag_ms_wc_z"] = best_lag_ms(ts_z, cp_z, zs) if zs else None
    out["peak_wc_z_pos"] = max((v-bs_z for v in zs), default=None) if zs else None
    out["peak_wc_z_neg"] = min((v-bs_z for v in zs), default=None) if zs else None
    out["peak_S_pos"] = max((v-bs_S for v in ss), default=None)
    out["peak_S_neg"] = min((v-bs_S for v in ss), default=None)
    out["peak_G_pos"] = max((v-bs_G for v in sg), default=None)
    out["peak_G_neg"] = min((v-bs_G for v in sg), default=None)
    out["amp_ratio_wc_z_per_deg"] = (out["rms_wc_z"]/out["rms_cmd_pit"]) if out["rms_wc_z"] and out["rms_cmd_pit"] else None
    # Asymmetry: |peak_neg| / |peak_pos| in wc_z
    if out["peak_wc_z_pos"] is not None and out["peak_wc_z_neg"] is not None:
        pp = abs(out["peak_wc_z_pos"]); pn = abs(out["peak_wc_z_neg"])
        out["asym_wc_z"] = (pn/pp) if pp > 1e-6 else None
    return out

async def main_run(label):
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"})); await asyncio.sleep(0.5)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        en_ok=False; te=time.monotonic()+30.0
        while time.monotonic()<te:
            try: raw = await asyncio.wait_for(ws.recv(),timeout=1.0)
            except asyncio.TimeoutError: continue
            try: m = json.loads(raw)
            except: continue
            if m.get("type")=="uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok=bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok: return

        all_rows=[]; per_pose={}
        for pl, pose in POSES:
            rows = await run_pose(ws, pl, pose)
            all_rows += rows
            per_pose[pl] = metrics_for_pose(rows, pl)
        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)

    csv_path = f"/tmp/vertical_{label}.csv"
    keys = sorted({k for r in all_rows for k in r.keys()})
    with open(csv_path,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(all_rows)
    json_path = f"/tmp/vertical_{label}.metrics.json"
    with open(json_path,"w") as f:
        json.dump(per_pose, f, indent=2)
    print(f"\nSaved: {csv_path}  {json_path}")

    print(f"\n=== METRICS [{label}] ===")
    for pl in ("CENTRAL","ALL_UP","ALL_DOWN"):
        m = per_pose[pl]
        if not m.get("rms_wc_z") and m.get("rms_wc_z") != 0:
            print(f"\n{pl}: insufficient data"); continue
        def f(v,w=6,p=2): return f"{v:+{w}.{p}f}" if v is not None else "  —  "
        print(f"\n{pl}:")
        print(f"  RMS wc_z = {f(m.get('rms_wc_z'),6,2)} mm   lag_wc_z = {f(m.get('lag_ms_wc_z'),5,0)} ms   amp_ratio = {f(m.get('amp_ratio_wc_z_per_deg'),6,3)} mm/°")
        print(f"  peak wc_z +/− = {f(m.get('peak_wc_z_pos'),6,2)}/{f(m.get('peak_wc_z_neg'),6,2)} mm   asym = {f(m.get('asym_wc_z'),6,2)}")
        print(f"  corr(cmd,S)={f(m.get('corr_cmd_S'))}  corr(cmd,G)={f(m.get('corr_cmd_G'))}  corr(cmd,wc_z)={f(m.get('corr_cmd_wc_z'))}")

def main_compare():
    labels=["baseline","b1","b2","b3"]
    data={}
    for l in labels:
        try: data[l] = json.load(open(f"/tmp/vertical_{l}.metrics.json"))
        except Exception as e: print(f"missing {l}: {e}"); return
    print("="*120)
    print("VERTICAL TUNING COMPARISON — baseline / B1 (gain 2.0) / B2 (relax suppress) / B3 (both)")
    print("="*120)
    for pose in ("CENTRAL","ALL_UP","ALL_DOWN"):
        print(f"\n--- {pose} ---")
        def row(metric, unit=""):
            parts=[]
            for l in labels:
                v = data[l].get(pose,{}).get(metric)
                parts.append(f"{l}={v:+7.3f}" if isinstance(v,(int,float)) else f"{l}=  —  ")
            print(f"  {metric:22s} {unit:5s}  " + "   ".join(parts))
        row("rms_wc_z", "mm")
        row("lag_ms_wc_z", "ms")
        row("amp_ratio_wc_z_per_deg","mm/°")
        row("peak_wc_z_pos", "mm")
        row("peak_wc_z_neg", "mm")
        row("asym_wc_z", "")
        row("corr_cmd_S", "")
        row("corr_cmd_G", "")
        row("corr_cmd_wc_z", "")

if __name__=="__main__":
    if len(sys.argv)<2: print("usage: {baseline|b1|b2|b3|compare}"); sys.exit(1)
    mode = sys.argv[1]
    if mode in ("baseline","b1","b2","b3"): asyncio.run(main_run(mode))
    elif mode=="compare": main_compare()
    else: print(f"unknown: {mode}"); sys.exit(1)
