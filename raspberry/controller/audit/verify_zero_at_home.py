#!/usr/bin/env python3
"""
verify_zero_at_home.py — prova Zero-at-HOME end-to-end.

1. Clear any existing home ref (start from pipeline "nuda")
2. Baseline: go HOME + 3 stress poses, compute Δ = IMU − FK on each
3. Return to HOME, capture snapshot, compute q_home_ref = q_obs · q_fk_conj
   and POST to /api/imu-home-ref (exactly what the JS button does)
4. Re-run the same 4 poses: compute Δ using the NEW chain
   (q_ee = q_home^-1 · q_wb^-1 · q_imu · q_mount^-1)
5. Print BEFORE vs AFTER table + conclusion

Replica la matematica del frontend senza dipendere dal browser.
"""
import asyncio, json, math, ssl, time, urllib.request
import websockets
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"
POST_URL  = "https://127.0.0.1:8443/api/imu-home-ref"
CLEAR_URL = "https://127.0.0.1:8443/api/imu-home-ref/clear"

POSES = [
    ("HOME",            [ 90,  90,  90,  90,  90,  90]),
    ("Arm ext",         [ 90,  70, 115,  90,  90,  90]),
    ("Base rot +25",    [115,  75, 115,  90,  90,  90]),
    ("Mixed extended",  [110,  80, 110, 100, 105, 105]),
]

SETTLE_S = 0.7
SAMPLE_N = 35
MOTION_TMO = 25.0
TOOL_M = [0.06, 0.0, 0.0]

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]
def q_xyzw_to_wxyz(q): return [q[3], q[0], q[1], q[2]]

def load_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def post_json(url, payload):
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    req = urllib.request.Request(url, method="POST",
            headers={"Content-Type":"application/json"},
            data=json.dumps(payload).encode("utf-8"))
    with urllib.request.urlopen(req, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def post_clear(url):
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    req = urllib.request.Request(url, method="POST")
    with urllib.request.urlopen(req, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def r_from_calib(slot): return R.from_quat(q_wxyz_to_xyzw(slot["quat_wxyz"]))

def wrap_deg(a):
    while a > 180: a -= 360
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

async def sample_pose(ws, r_wb_inv, r_mount_inv, r_home_inv, n):
    fk_x, fk_y, fk_z, fk_yw, fk_pt, fk_rl = [], [], [], [], [], []
    imu_x, imu_y, imu_z, imu_yw, imu_pt, imu_rl = [], [], [], [], [], []
    collected = 0; t_end = time.monotonic() + 6.0
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
        q_imu = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
        r_imu = R.from_quat(q_wxyz_to_xyzw(q_imu))
        r_ee = r_home_inv * r_wb_inv * r_imu * r_mount_inv
        ypr = r_ee.as_euler("ZYX", degrees=True)
        imu_yw.append(ypr[0]); imu_pt.append(ypr[1]); imu_rl.append(ypr[2])
        wc_m = [m["fk_live_wc_x_mm"]/1000, m["fk_live_wc_y_mm"]/1000, m["fk_live_wc_z_mm"]/1000]
        tip = [wc_m[i] + (r_ee.as_matrix() @ TOOL_M)[i] for i in range(3)]
        imu_x.append(tip[0]*1000); imu_y.append(tip[1]*1000); imu_z.append(tip[2]*1000)
        collected += 1
    if collected == 0: return None
    avg = lambda xs: sum(xs)/len(xs)
    return {
        "fk":  {"x":avg(fk_x),"y":avg(fk_y),"z":avg(fk_z),"yaw":avg(fk_yw),"pitch":avg(fk_pt),"roll":avg(fk_rl)},
        "imu": {"x":avg(imu_x),"y":avg(imu_y),"z":avg(imu_z),"yaw":avg(imu_yw),"pitch":avg(imu_pt),"roll":avg(imu_rl)},
    }

async def capture_home_ref(ws, r_wb_inv, r_mount_inv):
    """Replica JS: cattura q_obs · q_fk_conj al HOME."""
    # Drena vecchi frame
    await drain(ws, 0.3)
    # Prendi 5 sample e media il quaternione (quaternion average non triviale:
    # facciamo elemento-wise, poi normalizziamo — OK per variazioni sub-gradi).
    q_obs_list, q_fk_list = [], []
    t_end = time.monotonic() + 3.0; collected = 0
    while collected < 8 and time.monotonic() < t_end:
        try: raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except: continue
        try: m = json.loads(raw)
        except: continue
        if m.get("type") != "telemetry": continue
        if m.get("imu_valid") is not True: continue
        if "fk_live_quat_w" not in m: continue
        q_imu = R.from_quat(q_wxyz_to_xyzw((m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])))
        q_obs = r_wb_inv * q_imu * r_mount_inv
        q_fk  = R.from_quat([m["fk_live_quat_x"], m["fk_live_quat_y"], m["fk_live_quat_z"], m["fk_live_quat_w"]])
        q_obs_list.append(q_obs.as_quat()); q_fk_list.append(q_fk.as_quat())
        collected += 1
    if collected == 0:
        raise RuntimeError("No valid samples for home capture")
    # Media elemento-wise + normalizzazione (va bene per piccole variazioni)
    import numpy as np
    q_obs_avg = np.mean(q_obs_list, axis=0); q_obs_avg /= (np.linalg.norm(q_obs_avg) or 1.0)
    q_fk_avg  = np.mean(q_fk_list, axis=0);  q_fk_avg  /= (np.linalg.norm(q_fk_avg)  or 1.0)
    r_obs = R.from_quat(q_obs_avg); r_fk = R.from_quat(q_fk_avg)
    r_home_ref = r_obs * r_fk.inv()
    q_home_ref = r_home_ref.as_quat()  # xyzw
    q_home_ref_wxyz = q_xyzw_to_wxyz(q_home_ref)
    return q_home_ref_wxyz, collected

async def run_sweep(ws, r_wb_inv, r_mount_inv, r_home_inv, label):
    results = []
    for name, angles in POSES:
        b,s,g,y,p,r_ = angles
        cmd = f"SETPOSE {b} {s} {g} {y} {p} {r_} 22 RTR5"
        await ws.send(json.dumps({"type":"uart","cmd":cmd}))
        if not await await_setpose_done(ws, MOTION_TMO):
            print(f"  [{label}/{name}] SETPOSE_DONE timeout")
        await asyncio.sleep(SETTLE_S); await drain(ws, 0.1)
        snap = await sample_pose(ws, r_wb_inv, r_mount_inv, r_home_inv, SAMPLE_N)
        if snap:
            results.append((name, snap))
    return results

async def main():
    print("=" * 92)
    print("ZERO-AT-HOME end-to-end verification")
    print("=" * 92)

    # 0. Clear any pre-existing home ref so we start from identity
    try:
        res = post_clear(CLEAR_URL); print("clear pre-existing home ref:", res)
    except Exception as e:
        print("clear failed (ok if none):", e)

    calib = load_calib()
    r_mount = r_from_calib(calib["mount"]); r_mount_inv = r_mount.inv()
    r_wb    = r_from_calib(calib["world_bias"]); r_wb_inv = r_wb.inv()
    r_home_id_inv = R.identity()
    print(f"\nPre-zero calib:")
    print(f"  mount     YPR = {tuple(round(v,3) for v in r_mount.as_euler('ZYX', degrees=True))}")
    print(f"  worldBias YPR = {tuple(round(v,3) for v in r_wb.as_euler('ZYX', degrees=True))}")
    print(f"  home present  = {calib['home']['present']}")
    print()

    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"})); await drain(ws, 1.0)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        en_ok = False; t_end = time.monotonic() + 30.0
        while time.monotonic() < t_end:
            try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except: continue
            try: m = json.loads(raw)
            except: continue
            if m.get("type") == "uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok = bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok: return

        # BASELINE: pipeline nuda
        print("\n[BASELINE] pipeline senza home (r_home = identity)")
        results_before = await run_sweep(ws, r_wb_inv, r_mount_inv, r_home_id_inv, "BASELINE")

        # CAPTURE: go HOME and take q_home_ref
        print("\n[CAPTURE] returning to HOME for zero-at-HOME snapshot")
        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 22 RTR5"}))
        await await_setpose_done(ws, MOTION_TMO)
        await asyncio.sleep(SETTLE_S + 0.3)
        q_home_wxyz, nsamp = await capture_home_ref(ws, r_wb_inv, r_mount_inv)
        r_home = R.from_quat(q_wxyz_to_xyzw(q_home_wxyz))
        print(f"  home quat_wxyz = {[round(v,6) for v in q_home_wxyz]}  (avg over {nsamp} samples)")
        print(f"  home YPR deg   = {tuple(round(v,3) for v in r_home.as_euler('ZYX', degrees=True))}")

        # POST to server (same payload shape the JS button uses)
        payload = {
            "quat_wxyz": q_home_wxyz,
            "calibrated_at": "verify_zero_at_home.py AUTOMATED",
            "note": "Captured via verify_zero_at_home.py for E2E proof",
        }
        res = post_json(POST_URL, payload)
        print(f"  POST /api/imu-home-ref: ok={res.get('ok')} path={res.get('path')}")

        # Reload calib (now home.present = true) to prove the GET reflects it
        calib2 = load_calib()
        print(f"  GET confirms: home.present = {calib2['home']['present']}, calibrated_at = {calib2['home']['calibrated_at']}")

        # AFTER sweep: pipeline with home inverse
        r_home_inv = r_home.inv()
        print("\n[AFTER] pipeline con q_home^-1 applicata")
        results_after = await run_sweep(ws, r_wb_inv, r_mount_inv, r_home_inv, "AFTER")

        # Home restore
        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await drain(ws, 0.5)

    def rowify(results):
        out = {}
        for name, s in results:
            fk = s["fk"]; im = s["imu"]
            dx = im["x"]-fk["x"]; dy = im["y"]-fk["y"]; dz = im["z"]-fk["z"]
            dyw = wrap_deg(im["yaw"]-fk["yaw"])
            dpt = wrap_deg(im["pitch"]-fk["pitch"])
            drl = wrap_deg(im["roll"]-fk["roll"])
            np_ = math.sqrt(dx*dx+dy*dy+dz*dz)
            nr_ = math.sqrt(dyw*dyw+dpt*dpt+drl*drl)
            out[name] = (dx, dy, dz, dyw, dpt, drl, np_, nr_)
        return out

    before = rowify(results_before)
    after  = rowify(results_after)

    print("\n" + "=" * 92)
    print("BEFORE vs AFTER (Δ = IMU − FK, mm / °)")
    print("=" * 92)
    hdr = f"{'pose':18s} {'phase':7s} | {'ΔX':>7s} {'ΔY':>7s} {'ΔZ':>7s} | {'ΔYaw':>7s} {'ΔPit':>7s} {'ΔRol':>7s} | {'|Δpos|':>7s} {'|Δrot|':>7s}"
    print(hdr); print("-"*len(hdr))
    for name, _ in POSES:
        if name in before:
            dx, dy, dz, dyw, dpt, drl, np_, nr_ = before[name]
            print(f"{name:18s} BEFORE  | {dx:+7.2f} {dy:+7.2f} {dz:+7.2f} | {dyw:+7.2f} {dpt:+7.2f} {drl:+7.2f} | {np_:7.2f} {nr_:7.2f}")
        if name in after:
            dx, dy, dz, dyw, dpt, drl, np_, nr_ = after[name]
            print(f"{name:18s} AFTER   | {dx:+7.2f} {dy:+7.2f} {dz:+7.2f} | {dyw:+7.2f} {dpt:+7.2f} {drl:+7.2f} | {np_:7.2f} {nr_:7.2f}")
        if name in before and name in after:
            impr_pos = before[name][6] - after[name][6]
            impr_rot = before[name][7] - after[name][7]
            print(f"{name:18s} Δ(B−A)  | {'':>7s} {'':>7s} {'':>7s} | {'':>7s} {'':>7s} {'':>7s} | {impr_pos:+7.2f} {impr_rot:+7.2f}")
        print()

    def norm_stats(d, idx):
        vs = [v[idx] for v in d.values() if d]
        return max(vs) if vs else 0.0, sum(vs)/len(vs) if vs else 0.0

    max_pos_b, mean_pos_b = norm_stats(before, 6)
    max_rot_b, mean_rot_b = norm_stats(before, 7)
    max_pos_a, mean_pos_a = norm_stats(after, 6)
    max_rot_a, mean_rot_a = norm_stats(after, 7)
    print("=" * 92)
    print("Aggregato")
    print("=" * 92)
    print(f"  BEFORE:  max |Δpos|={max_pos_b:.2f} mm  mean={mean_pos_b:.2f}   max |Δrot|={max_rot_b:.2f}°  mean={mean_rot_b:.2f}")
    print(f"  AFTER:   max |Δpos|={max_pos_a:.2f} mm  mean={mean_pos_a:.2f}   max |Δrot|={max_rot_a:.2f}°  mean={mean_rot_a:.2f}")
    print()
    if max_rot_a < 3.0 and max_pos_a < 10.0 and max_rot_b > max_rot_a * 2:
        print("  ✓ ZERO-AT-HOME FIX VALIDATED: residui scesi a soglie sub-thesis.")
    elif max_rot_a < max_rot_b * 0.5:
        print("  ✓ Riduzione significativa, ma residuo ancora presente.")
    else:
        print("  ⚠ Zero-at-HOME non ha prodotto miglioramento atteso — investigare.")

asyncio.run(main())
