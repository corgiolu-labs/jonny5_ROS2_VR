#!/usr/bin/env python3
"""
verify_imu_stress.py — IMU vs FK alignment stress test at extended poses.

Logica:
  Quando il braccio è esteso o ruotato, un piccolo errore di orientazione
  sull'IMU (es. ~1° bias sul pitch) si amplifica in errore di posizione
  proporzionalmente al raggio del wrist-center dal base origin. Se la
  pipeline `R_ee = R_wb^-1 · R_imu · R_mount^-1` è corretta, Δrot resta
  piccolo e costante, Δpos può crescere leggermente ma non esplode.

Procedura (identica a verify_imu_alignment.py, POSE diverse):
  SAFE → ENABLE → per ogni posa: SETPOSE → attendi SETPOSE_DONE → settle
  → media 40 campioni → Δ = IMU − FK + norme + raggio wc.

Pose: pensate per massimizzare il lever arm rotazionale, restando entro i
limiti virtuali sicuri (base [35,125], spalla [33,148], gomito [27,142],
yaw [45,135], pitch [60,145], roll [52,142]).

Read-only analysis — zero modifiche a firmware/math/calib.
"""
import asyncio, json, math, ssl, time, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS  = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"

# Ordine joints: BASE, SPALLA, GOMITO, YAW, PITCH, ROLL (virtuali, home=90)
POSES = [
    ("HOME reference",      [ 90,  90,  90,  90,  90,  90]),   # baseline di confronto
    ("Arm ext forward",     [ 90,  70, 115,  90,  90,  90]),   # spalla−20, gomito+25 → wc più lontano in X
    ("Arm ext + base rot",  [115,  75, 115,  90,  90,  90]),   # come sopra + base +25° (rotazione yaw robot)
    ("Wrist roll +45°",     [ 90,  90,  90,  90,  90, 135]),   # solo roll estremo
    ("Wrist pitch -30°",    [ 90,  90,  90,  90, 120,  90]),   # solo pitch estremo
    ("Mixed extended",      [110,  80, 110, 100, 105, 105]),   # tutti joint diversi, braccio esteso
    ("Deep extension",      [ 90,  65, 125,  90,  90,  90]),   # spalla verso min, gomito verso max
]

SETTLE_S     = 0.7
SAMPLE_COUNT = 40
MOTION_TMO   = 25.0
TOOL_M       = [0.06, 0.0, 0.0]

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]

def load_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def r_from_quat_wxyz(q): return R.from_quat(q_wxyz_to_xyzw(q))

def wrap_deg(a):
    while a > 180:  a -= 360
    while a < -180: a += 360
    return a

async def drain(ws, dur_s):
    t_end = time.monotonic() + dur_s
    while time.monotonic() < t_end:
        try: await asyncio.wait_for(ws.recv(), timeout=0.05)
        except: return

async def await_setpose_done(ws, tmo):
    t_end = time.monotonic() + tmo
    while time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except: continue
        try: m = json.loads(raw)
        except: continue
        if m.get("type") == "setpose_done": return True
    return False

async def sample_pose(ws, r_wb_inv, r_mount_inv, n):
    fk_x, fk_y, fk_z, fk_yw, fk_pt, fk_rl = [], [], [], [], [], []
    imu_x, imu_y, imu_z, imu_yw, imu_pt, imu_rl = [], [], [], [], [], []
    wc_r = []  # raggio wrist-center per correlazione lever-arm
    collected = 0
    t_end = time.monotonic() + 6.0
    while collected < n and time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except: continue
        try: m = json.loads(raw)
        except: continue
        if m.get("type") != "telemetry": continue
        if m.get("imu_valid") is not True: continue
        if "fk_live_x_mm" not in m or "imu_q_w" not in m: continue

        fk_x.append(m["fk_live_x_mm"]); fk_y.append(m["fk_live_y_mm"]); fk_z.append(m["fk_live_z_mm"])
        fk_yw.append(m["fk_live_yaw"]); fk_pt.append(m["fk_live_pitch"]); fk_rl.append(m["fk_live_roll"])

        wc_m = [m["fk_live_wc_x_mm"]/1000.0, m["fk_live_wc_y_mm"]/1000.0, m["fk_live_wc_z_mm"]/1000.0]
        wc_r.append(math.sqrt(sum(v*v for v in wc_m)) * 1000.0)

        q_imu = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
        r_ee = r_wb_inv * r_from_quat_wxyz(q_imu) * r_mount_inv
        ypr = r_ee.as_euler("ZYX", degrees=True)
        imu_yw.append(ypr[0]); imu_pt.append(ypr[1]); imu_rl.append(ypr[2])
        tip = [wc_m[i] + (r_ee.as_matrix() @ TOOL_M)[i] for i in range(3)]
        imu_x.append(tip[0]*1000); imu_y.append(tip[1]*1000); imu_z.append(tip[2]*1000)
        collected += 1

    if collected == 0: return None
    avg = lambda xs: sum(xs)/len(xs)
    return {
        "n": collected,
        "fk":  {"x":avg(fk_x),"y":avg(fk_y),"z":avg(fk_z),"yaw":avg(fk_yw),"pitch":avg(fk_pt),"roll":avg(fk_rl)},
        "imu": {"x":avg(imu_x),"y":avg(imu_y),"z":avg(imu_z),"yaw":avg(imu_yw),"pitch":avg(imu_pt),"roll":avg(imu_rl)},
        "wc_r_mm": avg(wc_r),
    }

async def main():
    print("=" * 92)
    print("IMU vs FK STRESS TEST — pose estese, lever-arm massimizzato")
    print("=" * 92)

    calib = load_calib()
    r_mount = r_from_quat_wxyz(calib["mount"]["quat_wxyz"])
    r_wb    = r_from_quat_wxyz(calib["world_bias"]["quat_wxyz"])
    r_mount_inv = r_mount.inv(); r_wb_inv = r_wb.inv()

    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"})); await drain(ws, 1.0)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        en_ok = False
        t_end = time.monotonic() + 30.0
        while time.monotonic() < t_end:
            try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except: continue
            try: m = json.loads(raw)
            except: continue
            if m.get("type") == "uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok = bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok: return

        results = []
        for name, angles in POSES:
            b,s,g,y,p,r_ = angles
            cmd = f"SETPOSE {b} {s} {g} {y} {p} {r_} 22 RTR5"
            await ws.send(json.dumps({"type":"uart","cmd":cmd}))
            print(f"[{name}] SETPOSE {angles} — waiting SETPOSE_DONE…")
            if not await await_setpose_done(ws, MOTION_TMO):
                print(f"  WARN: SETPOSE_DONE timeout ({MOTION_TMO}s)")
            await asyncio.sleep(SETTLE_S); await drain(ws, 0.1)
            snap = await sample_pose(ws, r_wb_inv, r_mount_inv, SAMPLE_COUNT)
            if snap is None:
                print(f"  ERR: no valid samples for {name}")
                continue
            results.append((name, angles, snap))

        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await drain(ws, 0.5)

    if not results:
        print("No results."); return

    print("\n" + "=" * 92)
    print(f"Per-pose table (averaged over {SAMPLE_COUNT} samples each)")
    print("=" * 92)
    hdr = f"{'pose':20s} {'wc_r(mm)':>9s} | {'ΔX':>8s} {'ΔY':>8s} {'ΔZ':>8s} | {'ΔYaw':>8s} {'ΔPit':>8s} {'ΔRol':>8s} | {'|Δpos|':>8s} {'|Δrot|':>8s}"
    print(hdr); print("-"*len(hdr))
    dxs, dys, dzs, dyaw, dpitch, droll, dnorm_pos, dnorm_rot, wc_rs = [], [], [], [], [], [], [], [], []
    for name, angles, s in results:
        fk = s["fk"]; im = s["imu"]; wc_r_mm = s["wc_r_mm"]
        dx = im["x"]-fk["x"]; dy = im["y"]-fk["y"]; dz = im["z"]-fk["z"]
        dyw = wrap_deg(im["yaw"]-fk["yaw"])
        dpt = wrap_deg(im["pitch"]-fk["pitch"])
        drl = wrap_deg(im["roll"]-fk["roll"])
        np_ = math.sqrt(dx*dx+dy*dy+dz*dz)
        nr_ = math.sqrt(dyw*dyw+dpt*dpt+drl*drl)
        dxs.append(dx); dys.append(dy); dzs.append(dz)
        dyaw.append(dyw); dpitch.append(dpt); droll.append(drl)
        dnorm_pos.append(np_); dnorm_rot.append(nr_); wc_rs.append(wc_r_mm)
        print(f"{name:20s} {wc_r_mm:9.1f} | {dx:+8.2f} {dy:+8.2f} {dz:+8.2f} | {dyw:+8.2f} {dpt:+8.2f} {drl:+8.2f} | {np_:8.2f} {nr_:8.2f}")

    # Verbose: raw FK/IMU values (appendix for thesis)
    print("\n" + "-" * 92)
    print("Dettaglio FK vs IMU (mm / ° — medie)")
    print("-" * 92)
    for name, angles, s in results:
        fk = s["fk"]; im = s["imu"]
        print(f"[{name}] FK ({fk['x']:+7.2f}, {fk['y']:+7.2f}, {fk['z']:+7.2f}) YPR ({fk['yaw']:+7.2f}, {fk['pitch']:+7.2f}, {fk['roll']:+7.2f})")
        print(f"[{name}] IMU({im['x']:+7.2f}, {im['y']:+7.2f}, {im['z']:+7.2f}) YPR ({im['yaw']:+7.2f}, {im['pitch']:+7.2f}, {im['roll']:+7.2f})")

    # Aggregate
    print("\n" + "=" * 92)
    print("Aggregato")
    print("=" * 92)
    def stat(xs):
        m = sum(xs)/len(xs)
        v = sum((x-m)**2 for x in xs)/max(1,len(xs))
        return m, math.sqrt(v), max(abs(x) for x in xs) if xs else 0.0
    for name, xs in [("ΔX mm",dxs),("ΔY mm",dys),("ΔZ mm",dzs),
                     ("ΔYaw °",dyaw),("ΔPitch °",dpitch),("ΔRoll °",droll),
                     ("|Δpos| mm",dnorm_pos),("|Δrot| °",dnorm_rot)]:
        mu, sd, mx = stat(xs)
        print(f"  {name:12s}  mean={mu:+7.2f}  std={sd:6.3f}  max|.|={mx:7.2f}")

    # Correlazione Δpos con raggio wc (lever-arm)
    print("\nCorrelazione |Δpos| ↔ raggio wc (leva rotazionale):")
    for name, _, s in results:
        r_wc = s["wc_r_mm"]
        idx = next(i for i,(n,_,_) in enumerate(results) if n == name)
        print(f"  {name:20s}  r_wc={r_wc:7.1f} mm  |Δpos|={dnorm_pos[idx]:6.2f} mm  |Δrot|={dnorm_rot[idx]:6.2f}°")
    # Pearson
    n = len(results)
    if n >= 2:
        mx_r = sum(wc_rs)/n; mx_p = sum(dnorm_pos)/n
        num = sum((wc_rs[i]-mx_r)*(dnorm_pos[i]-mx_p) for i in range(n))
        den = math.sqrt(sum((wc_rs[i]-mx_r)**2 for i in range(n)) *
                        sum((dnorm_pos[i]-mx_p)**2 for i in range(n)))
        rho = (num/den) if den > 1e-9 else 0.0
        print(f"\nPearson corr(r_wc, |Δpos|) = {rho:+.3f}   (1.0 = leva domina, 0 = scorrelato, <0 = anticorr)")

    max_pos = max(dnorm_pos) if dnorm_pos else 0.0
    max_rot = max(dnorm_rot) if dnorm_rot else 0.0

    print("\n" + "=" * 92)
    print("Conclusione automatica")
    print("=" * 92)
    print(f"  max |Δpos| = {max_pos:.2f} mm")
    print(f"  max |Δrot| = {max_rot:.2f}°")

    if max_rot < 4.0 and max_pos < 25.0:
        print("  → ALIGNMENT HOLDS UNDER EXTENSION. Δrot bounded, Δpos bounded (few mm/cm).")
    elif max_rot < 4.0 and max_pos >= 25.0:
        print("  → Δrot bounded ma Δpos alto: residuo rotazionale amplificato dal braccio.")
        print("    Non è un bug della pipeline, è l'effetto leva di ~1-2° di bias su 300+ mm.")
    elif max_rot >= 4.0:
        print("  → Δrot pose-dependent (>4° worst-case): calib mount/world_bias da rifare.")

asyncio.run(main())
