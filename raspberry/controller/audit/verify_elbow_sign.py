#!/usr/bin/env python3
"""
verify_elbow_sign.py — diagnose the sign of SPALLA, GOMITO, and fk_wc_z
during a pure head-pitch oscillation in ASSIST mode.

Outputs signed correlation coefficients cmd_pitch ↔ {servo_S, servo_G, fk_wc_z}
at three robot pre-poses (central, all-up, all-down). Reveals whether:
  - SPALLA and GOMITO move in the same direction (static split) or opposite
    (Jacobian remap preferring an opposite-sign solution)
  - fk_wc_z actually tracks the intended vertical direction, or partially
    cancels because elbow opposes shoulder.

Read-only: no code change here. Pure observation.
"""
import asyncio, json, math, ssl, time, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"

POSES = [
    ("CENTRAL",   [90,  90,  90,  90,  90, 90]),
    ("ALL UP",    [90, 145,  40,  90,  90, 90]),
    ("ALL DOWN",  [90,  40, 140,  90,  90, 90]),
]

def q_from_pitch(deg):
    x, y, z, w = R.from_euler("Y", deg, degrees=True).as_quat()
    return (w, x, y, z)

def q_identity(): return (1.0, 0.0, 0.0, 0.0)

state = {"q": (1.0, 0.0, 0.0, 0.0), "cmd_pit": 0.0, "grip": 0, "run": True, "phase": "idle"}

async def inj(ws):
    hb = 0; dt = 1.0/60.0
    while state["run"]:
        w, x, y, z = state["q"]
        hb = (hb+1) & 0xFFFF
        g = 1 if state["grip"] else 0
        btn = 0x0002 if state["grip"] else 0
        await ws.send(json.dumps({
            "mode": 5, "quat_w": w, "quat_x": x, "quat_y": y, "quat_z": z,
            "grip": g, "buttons_left": btn, "buttons_right": btn,
            "heartbeat": hb, "joy_x":0,"joy_y":0,"pitch":0,"yaw":0, "intensity": 255,
        }))
        await asyncio.sleep(dt)

async def cap(ws, sink):
    t0 = time.monotonic()
    while state["run"]:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError: continue
        except Exception: return
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
            "servo_B": m["servo_deg_B"], "servo_Y": m["servo_deg_Y"], "servo_P": m["servo_deg_P"], "servo_R": m["servo_deg_R"],
        }
        if m.get("fk_live_valid"):
            row["fk_wc_z"] = m.get("fk_live_wc_z_mm")
            row["fk_wc_x"] = m.get("fk_live_wc_x_mm")
            row["fk_z"] = m.get("fk_live_z_mm")
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

def pearson(xs, ys):
    if len(xs) < 5: return 0.0
    mx = sum(xs)/len(xs); my = sum(ys)/len(ys)
    num = sum((x-mx)*(y-my) for x,y in zip(xs, ys))
    dx = math.sqrt(sum((x-mx)**2 for x in xs))
    dy = math.sqrt(sum((y-my)**2 for y in ys))
    return num / (dx*dy) if (dx*dy) > 1e-9 else 0.0

async def run_pose(ws, label, pose):
    print(f"\n--- {label} pose={pose} ---")
    cmd = f"SETPOSE {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]} 22 RTR5"
    await ws.send(json.dumps({"type":"uart","cmd":cmd}))
    await await_setpose_done(ws)
    await asyncio.sleep(0.8)

    state["q"] = q_identity(); state["grip"] = 1; state["phase"] = "pre"; state["run"] = True
    sink = []
    tinj = asyncio.create_task(inj(ws))
    await asyncio.sleep(2.0)   # warmup: firmware engages mode=5 + telemetry fresh
    tcap = asyncio.create_task(cap(ws, sink))

    # baseline
    state["phase"] = "baseline"
    await asyncio.sleep(0.8)

    # slow pitch osc: ±12° @ 0.4 Hz for 5 s (clean signal, low noise)
    state["phase"] = "osc_pitch"
    t0 = time.monotonic(); T = 5.0
    while time.monotonic() - t0 < T:
        t = time.monotonic() - t0
        a = 12.0 * math.sin(2*math.pi*0.4*t)
        state["q"] = q_from_pitch(a); state["cmd_pit"] = a
        await asyncio.sleep(0.02)

    state["grip"] = 0; await asyncio.sleep(0.2)
    state["run"] = False
    tinj.cancel(); tcap.cancel()
    for t in (tinj, tcap):
        try: await t
        except (asyncio.CancelledError, Exception): pass

    # Analyze osc_pitch only
    xs = [r for r in sink if r["phase"]=="osc_pitch"]
    if len(xs) < 30:
        print(f"  insufficient samples ({len(xs)})")
        return
    cp = [r["cmd_pit"] for r in xs]
    ss = [r["servo_S"] for r in xs]
    sg = [r["servo_G"] for r in xs]
    zs = [r.get("fk_wc_z") for r in xs if r.get("fk_wc_z") is not None]
    cp_z = [r["cmd_pit"] for r in xs if r.get("fk_wc_z") is not None]

    corr_s = pearson(cp, ss)
    corr_g = pearson(cp, sg)
    corr_z = pearson(cp_z, zs) if len(zs) > 10 else None

    # RMS of each signal
    def rms(vs): m=sum(vs)/len(vs); return math.sqrt(sum((v-m)**2 for v in vs)/len(vs))
    rms_cp = rms(cp); rms_s = rms(ss); rms_g = rms(sg)
    rms_z = rms(zs) if len(zs) > 10 else None

    print(f"  signals rms:  cmd_pit={rms_cp:.2f}°   servo_S={rms_s:.2f}°   servo_G={rms_g:.2f}°   fk_wc_z={rms_z:.2f}mm" if rms_z else
          f"  signals rms:  cmd_pit={rms_cp:.2f}°   servo_S={rms_s:.2f}°   servo_G={rms_g:.2f}°   fk_wc_z=—")
    print(f"  correlation cmd_pit↔servo_S = {corr_s:+.3f}   (sign = {'+' if corr_s>0 else '−'})")
    print(f"  correlation cmd_pit↔servo_G = {corr_g:+.3f}   (sign = {'+' if corr_g>0 else '−'})")
    if corr_z is not None:
        print(f"  correlation cmd_pit↔fk_wc_z = {corr_z:+.3f}   (sign = {'+' if corr_z>0 else '−'})")

    # Diagnosis
    same_sign = (corr_s * corr_g) > 0
    print(f"  → SPALLA, GOMITO move in {'SAME' if same_sign else 'OPPOSITE'} direction")
    if corr_z is not None:
        # Note: physical sign of cmd_pit → expected fk_wc_z depends on signPitch
        # and hp_drive=-hp convention. We just report the observed sign.
        if abs(corr_z) < 0.3:
            print(f"  → fk_wc_z tracking LOW: |corr|={abs(corr_z):.2f} → partial cancellation possible")
        else:
            print(f"  → fk_wc_z tracks with |corr|={abs(corr_z):.2f}")

async def main():
    print("=" * 78)
    print("ELBOW SIGN DIAGNOSTIC — pitch osc ±12° @ 0.4 Hz over 3 pre-poses")
    print("=" * 78)

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

        for label, pose in POSES:
            await run_pose(ws, label, pose)

        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)

asyncio.run(main())
