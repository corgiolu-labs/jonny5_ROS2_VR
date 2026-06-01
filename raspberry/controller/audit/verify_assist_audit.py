#!/usr/bin/env python3
"""
verify_assist_audit.py — comprehensive ASSIST audit.

Fills gaps not covered by existing scripts:
  1. ESTESO pose (arm extended forward, not the folded UP/DOWN extremes)
  2. Hold-stability (5 s identity, check servo drift)
  3. Post-B1 step responses at CENTRAL (yaw ±15, pitch ±10)
  4. Mid-frequency osc at each pose (pitch ±10 @ 0.8 Hz — faster than 0.4 Hz)

All with the CURRENT LIVE config (B1 applied: headMotionGainPitch=2.0,
_REACH_SCALE_CAP=2.4, Zero-at-HOME present). Captures cmd vs FK vs IMU.
"""
import asyncio, json, math, ssl, time, csv, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"

POSES = [
    ("CENTRAL",   [90,  90,  90, 90, 90, 90]),
    ("ALL_UP",    [90, 145,  40, 90, 90, 90]),
    ("ALL_DOWN",  [90,  40, 140, 90, 90, 90]),
    ("ESTESO",    [90,  70, 115, 90, 90, 90]),  # arm extended forward (reach_xy > central)
]

def qI(): return (1.0, 0.0, 0.0, 0.0)
def q_yaw(d):   x,y,z,w = R.from_euler("Z", d, degrees=True).as_quat(); return (w,x,y,z)
def q_pitch(d): x,y,z,w = R.from_euler("Y", d, degrees=True).as_quat(); return (w,x,y,z)

state = {"q":qI(),"cmd_yaw":0.0,"cmd_pit":0.0,"grip":0,"run":True,"phase":"idle"}

async def inj(ws):
    hb=0
    while state["run"]:
        w,x,y,z=state["q"]; hb=(hb+1)&0xFFFF
        g=1 if state["grip"] else 0; btn=0x0002 if state["grip"] else 0
        await ws.send(json.dumps({"mode":5,"quat_w":w,"quat_x":x,"quat_y":y,"quat_z":z,
            "grip":g,"buttons_left":btn,"buttons_right":btn,"heartbeat":hb,
            "joy_x":0,"joy_y":0,"pitch":0,"yaw":0,"intensity":255}))
        await asyncio.sleep(1/60)

async def cap(ws, sink, r_wb_inv, r_mount_inv, r_home_inv):
    t0=time.monotonic()
    while state["run"]:
        try: raw=await asyncio.wait_for(ws.recv(),timeout=0.5)
        except asyncio.TimeoutError: continue
        except: return
        try: m=json.loads(raw)
        except: continue
        if m.get("type")!="telemetry" or "servo_deg_B" not in m: continue
        try:
            v=[float(m[k]) for k in ("servo_deg_B","servo_deg_S","servo_deg_G","servo_deg_Y","servo_deg_P","servo_deg_R")]
            if any(x<5 or x>175 for x in v): continue
        except: continue
        row={"t":round(time.monotonic()-t0,4),"phase":state["phase"],
             "cmd_yaw":state["cmd_yaw"],"cmd_pit":state["cmd_pit"],
             "servo_B":m["servo_deg_B"],"servo_S":m["servo_deg_S"],"servo_G":m["servo_deg_G"],
             "servo_Y":m["servo_deg_Y"],"servo_P":m["servo_deg_P"],"servo_R":m["servo_deg_R"]}
        if m.get("fk_live_valid"):
            row["fk_wc_x"]=m.get("fk_live_wc_x_mm"); row["fk_wc_y"]=m.get("fk_live_wc_y_mm")
            row["fk_wc_z"]=m.get("fk_live_wc_z_mm"); row["fk_z"]=m.get("fk_live_z_mm")
            row["fk_yaw"]=m.get("fk_live_yaw"); row["fk_pit"]=m.get("fk_live_pitch")
        if m.get("imu_valid") is True and m.get("imu_q_w") is not None:
            q=(m["imu_q_w"],m["imu_q_x"],m["imu_q_y"],m["imu_q_z"])
            r_ee=r_home_inv * r_wb_inv * R.from_quat([q[1],q[2],q[3],q[0]]) * r_mount_inv
            ypr=r_ee.as_euler("ZYX",degrees=True)
            row["imu_yaw"]=round(float(ypr[0]),3); row["imu_pit"]=round(float(ypr[1]),3)
            row["imu_rol"]=round(float(ypr[2]),3)
        sink.append(row)

async def wait_sp_done(ws, tmo=25):
    te=time.monotonic()+tmo
    while time.monotonic()<te:
        try: raw=await asyncio.wait_for(ws.recv(),timeout=1.0)
        except asyncio.TimeoutError: continue
        try: m=json.loads(raw)
        except: continue
        if m.get("type")=="setpose_done": return True
    return False

async def run_segment(segname, dur, q_fn, cmd_axis):
    """Drive q_fn(t_rel) → state, keep cmd_yaw/cmd_pit synced."""
    state["phase"]=segname
    t0=time.monotonic()
    while time.monotonic()-t0 < dur:
        t=time.monotonic()-t0
        q = q_fn(t); state["q"] = q
        # Decode the commanded Euler for CSV
        r = R.from_quat([q[1],q[2],q[3],q[0]])
        y,p,r_ = r.as_euler("ZYX",degrees=True)
        state["cmd_yaw"]=float(y); state["cmd_pit"]=float(p)
        await asyncio.sleep(0.02)

async def run_pose(ws, pose_label, pose, r_wb_inv, r_mount_inv, r_home_inv):
    print(f"  → {pose_label:9s} pose={pose}")
    await ws.send(json.dumps({"type":"uart","cmd":f"SETPOSE {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]} 22 RTR5"}))
    await wait_sp_done(ws)
    await asyncio.sleep(0.8)

    state["q"]=qI(); state["grip"]=1; state["phase"]="pre"; state["run"]=True
    sink=[]; tinj=asyncio.create_task(inj(ws))
    await asyncio.sleep(2.0)  # warmup
    tcap=asyncio.create_task(cap(ws, sink, r_wb_inv, r_mount_inv, r_home_inv))

    # Baseline 1s identity
    await run_segment("baseline", 1.0, lambda t: qI(), None)
    # Yaw osc 8° 0.6 Hz × 4s
    await run_segment("yaw_osc_08_06", 4.0, lambda t: q_yaw(8.0*math.sin(2*math.pi*0.6*t)), "yaw")
    # Pause 0.5s
    await run_segment("pause1", 0.5, lambda t: qI(), None)
    # Pitch osc 10° 0.4 Hz × 4s
    await run_segment("pitch_osc_10_04", 4.0, lambda t: q_pitch(10.0*math.sin(2*math.pi*0.4*t)), "pitch")
    # Pause 0.5s
    await run_segment("pause2", 0.5, lambda t: qI(), None)
    # Pitch osc 10° 0.8 Hz × 3s (faster)
    await run_segment("pitch_osc_10_08", 3.0, lambda t: q_pitch(10.0*math.sin(2*math.pi*0.8*t)), "pitch")
    # Pause 0.5s
    await run_segment("pause3", 0.5, lambda t: qI(), None)
    # Hold 3s (grip active, identity) — stability test
    await run_segment("hold", 3.0, lambda t: qI(), None)

    state["grip"]=0; await asyncio.sleep(0.2)
    state["run"]=False
    tinj.cancel(); tcap.cancel()
    for t in (tinj,tcap):
        try: await t
        except (asyncio.CancelledError, Exception): pass
    for r in sink: r["pose_label"]=pose_label
    return sink

def rms(xs):
    if not xs: return 0.0
    m=sum(xs)/len(xs); return math.sqrt(sum((v-m)**2 for v in xs)/len(xs))

def corr(a,b):
    if len(a)<10: return None
    ma=sum(a)/len(a); mb=sum(b)/len(b)
    num=sum((x-ma)*(y-mb) for x,y in zip(a,b))
    da=math.sqrt(sum((x-ma)**2 for x in a)); db=math.sqrt(sum((y-mb)**2 for y in b))
    return num/(da*db) if da*db>1e-9 else None

def best_lag_ms(ts, cmd, sig):
    if len(ts)<30: return None
    cm=sum(cmd)/len(cmd); sm=sum(sig)/len(sig)
    c=[v-cm for v in cmd]; s=[v-sm for v in sig]
    dt=(ts[-1]-ts[0])/max(1,(len(ts)-1))
    ml=int(0.8/max(dt,1e-3)); best=-1e9; bl=0
    for lag in range(0,ml+1):
        num=sum(c[i]*s[i+lag] for i in range(len(s)-lag))
        cc=num/(len(s)-lag)
        if cc>best: best=cc; bl=lag
    return round(bl*dt*1000,0)

def pose_metrics(rows):
    out={}
    def ph(p): return [r for r in rows if r.get("phase")==p]

    # YAW osc analysis
    p = ph("yaw_osc_08_06")
    if len(p) >= 30:
        tb = [r for r in rows if r["phase"]=="baseline" and r.get("servo_B") is not None]
        bs_B = tb[-1]["servo_B"] if tb else p[0]["servo_B"]
        bs_R = tb[-1]["servo_R"] if tb else p[0]["servo_R"]
        ts=[r["t"] for r in p]
        cy=[r["cmd_yaw"] for r in p]
        sb=[r["servo_B"] for r in p]
        sr=[r["servo_R"] for r in p]
        fy=[r.get("fk_yaw") for r in p if r.get("fk_yaw") is not None]
        iy=[r.get("imu_yaw") for r in p if r.get("imu_yaw") is not None]
        cy_fk=[r["cmd_yaw"] for r in p if r.get("fk_yaw") is not None]
        ts_fk=[r["t"] for r in p if r.get("fk_yaw") is not None]
        cy_im=[r["cmd_yaw"] for r in p if r.get("imu_yaw") is not None]
        ts_im=[r["t"] for r in p if r.get("imu_yaw") is not None]

        out["yaw"] = {
            "rms_cmd": rms(cy), "rms_servo_B": rms(sb),
            "rms_fk": rms(fy), "rms_imu": rms(iy),
            "corr_cmd_B": corr(cy, sb), "corr_cmd_fk": corr(cy_fk, fy) if fy else None,
            "corr_cmd_imu": corr(cy_im, iy) if iy else None,
            "lag_ms_B": best_lag_ms(ts, cy, sb),
            "lag_ms_fk": best_lag_ms(ts_fk, cy_fk, fy) if fy else None,
            "lag_ms_imu": best_lag_ms(ts_im, cy_im, iy) if iy else None,
            "amp_ratio_B": rms(sb)/rms(cy) if rms(cy)>1e-6 else None,
            "amp_ratio_fk": rms(fy)/rms(cy) if rms(cy)>1e-6 and fy else None,
            "amp_ratio_imu": rms(iy)/rms(cy) if rms(cy)>1e-6 and iy else None,
            "coupling_R_peak": max(abs(v-bs_R) for v in sr),
        }

    # PITCH 0.4 Hz
    p = ph("pitch_osc_10_04")
    if len(p) >= 30:
        tb = [r for r in rows if r["phase"]=="baseline" and r.get("fk_wc_z") is not None]
        bs_z = tb[-1]["fk_wc_z"] if tb else (p[0].get("fk_wc_z") or 0)
        ts=[r["t"] for r in p]; cp=[r["cmd_pit"] for r in p]
        ss=[r["servo_S"] for r in p]; sg=[r["servo_G"] for r in p]
        sy=[r.get("servo_Y") for r in p]
        zs=[r.get("fk_wc_z") for r in p if r.get("fk_wc_z") is not None]
        ip=[r.get("imu_pit") for r in p if r.get("imu_pit") is not None]
        cp_z=[r["cmd_pit"] for r in p if r.get("fk_wc_z") is not None]
        cp_im=[r["cmd_pit"] for r in p if r.get("imu_pit") is not None]
        ts_z=[r["t"] for r in p if r.get("fk_wc_z") is not None]
        ts_im=[r["t"] for r in p if r.get("imu_pit") is not None]
        out["pitch_04"] = {
            "rms_cmd": rms(cp), "rms_S": rms(ss), "rms_G": rms(sg),
            "rms_wc_z": rms(zs) if zs else None, "rms_imu_pit": rms(ip) if ip else None,
            "corr_cmd_S": corr(cp, ss), "corr_cmd_G": corr(cp, sg),
            "corr_cmd_wc_z": corr(cp_z, zs) if zs else None,
            "corr_cmd_imu": corr(cp_im, ip) if ip else None,
            "lag_ms_wc_z": best_lag_ms(ts_z, cp_z, zs) if zs else None,
            "lag_ms_imu": best_lag_ms(ts_im, cp_im, ip) if ip else None,
            "peak_wc_z_pos": max((v-bs_z for v in zs), default=None) if zs else None,
            "peak_wc_z_neg": min((v-bs_z for v in zs), default=None) if zs else None,
            "coupling_Y_peak": max(abs(v - (p[0]["servo_Y"])) for v in [r["servo_Y"] for r in p]),
            "cooperation_SG": "co-sign" if (corr(cp,ss) or 0) * (corr(cp,sg) or 0) > 0 else "opposite",
        }

    # PITCH 0.8 Hz (faster)
    p = ph("pitch_osc_10_08")
    if len(p) >= 30:
        tb = [r for r in rows if r["phase"]=="baseline" and r.get("fk_wc_z") is not None]
        bs_z = tb[-1]["fk_wc_z"] if tb else (p[0].get("fk_wc_z") or 0)
        cp=[r["cmd_pit"] for r in p]
        ss=[r["servo_S"] for r in p]; sg=[r["servo_G"] for r in p]
        zs=[r.get("fk_wc_z") for r in p if r.get("fk_wc_z") is not None]
        cp_z=[r["cmd_pit"] for r in p if r.get("fk_wc_z") is not None]
        ts_z=[r["t"] for r in p if r.get("fk_wc_z") is not None]
        out["pitch_08"] = {
            "rms_cmd": rms(cp), "rms_S": rms(ss), "rms_G": rms(sg),
            "rms_wc_z": rms(zs) if zs else None,
            "amp_ratio_wc_z": rms(zs)/rms(cp) if zs and rms(cp)>1e-6 else None,
            "lag_ms_wc_z": best_lag_ms(ts_z, cp_z, zs) if zs else None,
            "corr_cmd_wc_z": corr(cp_z, zs) if zs else None,
        }

    # Hold stability
    p = ph("hold")
    if len(p) >= 20:
        for k in ("servo_B","servo_S","servo_G","fk_wc_z"):
            vs=[r.get(k) for r in p if r.get(k) is not None]
            if vs:
                out.setdefault("hold", {})[f"{k}_drift"] = max(vs)-min(vs)
                out["hold"][f"{k}_std"] = rms([v-sum(vs)/len(vs) for v in vs])
    return out

async def main():
    print("="*92); print("ASSIST AUDIT — 4 poses × yaw_osc + pitch_osc + pitch_fast + hold"); print("="*92)

    ctx=ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        calib=json.loads(r.read().decode("utf-8"))
    r_mount=R.from_quat([calib["mount"]["quat_wxyz"][1],calib["mount"]["quat_wxyz"][2],calib["mount"]["quat_wxyz"][3],calib["mount"]["quat_wxyz"][0]])
    r_wb=R.from_quat([calib["world_bias"]["quat_wxyz"][1],calib["world_bias"]["quat_wxyz"][2],calib["world_bias"]["quat_wxyz"][3],calib["world_bias"]["quat_wxyz"][0]])
    r_home=R.identity()
    if calib["home"]["present"]:
        h=calib["home"]["quat_wxyz"]
        r_home=R.from_quat([h[1],h[2],h[3],h[0]])
    r_mount_inv=r_mount.inv(); r_wb_inv=r_wb.inv(); r_home_inv=r_home.inv()
    print(f"Calib: mount={calib['mount']['present']} wb={calib['world_bias']['present']} home={calib['home']['present']}")

    async with websockets.connect(WS, ssl=ctx) as ws:
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"})); await asyncio.sleep(0.5)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        en_ok=False; te=time.monotonic()+30.0
        while time.monotonic()<te:
            try: raw=await asyncio.wait_for(ws.recv(),timeout=1.0)
            except asyncio.TimeoutError: continue
            try: m=json.loads(raw)
            except: continue
            if m.get("type")=="uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok=bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok: return

        all_rows=[]; per_pose={}
        for pl, pose in POSES:
            rows = await run_pose(ws, pl, pose, r_wb_inv, r_mount_inv, r_home_inv)
            all_rows += rows
            per_pose[pl] = pose_metrics(rows)
            print(f"    done ({len(rows)} rows)")

        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)

    with open("/tmp/assist_audit.csv","w",newline="") as f:
        keys=sorted({k for r in all_rows for k in r.keys()})
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(all_rows)
    with open("/tmp/assist_audit.metrics.json","w") as f:
        json.dump(per_pose, f, indent=2, default=str)
    print(f"\nCSV: /tmp/assist_audit.csv ({len(all_rows)} rows)")
    print(f"JSON: /tmp/assist_audit.metrics.json")

    # Print summary
    print("\n" + "="*92)
    print("SUMMARY METRICS (post-B1 live)")
    print("="*92)
    for pl in ("CENTRAL","ALL_UP","ALL_DOWN","ESTESO"):
        m = per_pose.get(pl, {})
        print(f"\n--- {pl} ---")
        y=m.get("yaw",{}); p4=m.get("pitch_04",{}); p8=m.get("pitch_08",{}); h=m.get("hold",{})
        def f(v, w=6, p=2): return f"{v:+{w}.{p}f}" if isinstance(v,(int,float)) else "  —  "
        # YAW
        print(f"  YAW osc ±8° 0.6Hz:  amp_ratio_B={f(y.get('amp_ratio_B'),5,3)}  lag_B={f(y.get('lag_ms_B'),4,0)}ms  " +
              f"corr_cmd_B={f(y.get('corr_cmd_B'),5,3)}  corr_cmd_imu={f(y.get('corr_cmd_imu'),5,3)}  " +
              f"coupling_R={f(y.get('coupling_R_peak'),4,1)}°")
        # PITCH 0.4Hz
        print(f"  PITCH osc ±10° 0.4Hz: amp_wc_z={f(p4.get('rms_wc_z'),5,2)}mm  lag_wc_z={f(p4.get('lag_ms_wc_z'),4,0)}ms  " +
              f"corr_S={f(p4.get('corr_cmd_S'),5,3)} corr_G={f(p4.get('corr_cmd_G'),5,3)}  " +
              f"S/G={p4.get('cooperation_SG','—')}  peak+/-={f(p4.get('peak_wc_z_pos'),4,2)}/{f(p4.get('peak_wc_z_neg'),4,2)}mm")
        # PITCH 0.8Hz (faster)
        print(f"  PITCH osc ±10° 0.8Hz: amp_wc_z={f(p8.get('rms_wc_z'),5,2)}mm  lag_wc_z={f(p8.get('lag_ms_wc_z'),4,0)}ms  " +
              f"corr_cmd_wc_z={f(p8.get('corr_cmd_wc_z'),5,3)}")
        # HOLD stability
        if h:
            print(f"  HOLD 3s stability: B_drift={f(h.get('servo_B_drift'),4,1)}°  S_drift={f(h.get('servo_S_drift'),4,1)}°  " +
                  f"G_drift={f(h.get('servo_G_drift'),4,1)}°  wc_z_drift={f(h.get('fk_wc_z_drift'),5,2)}mm")

asyncio.run(main())
