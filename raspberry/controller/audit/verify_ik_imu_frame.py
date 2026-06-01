#!/usr/bin/env python3
"""
verify_ik_imu_frame.py — validation of the base-frame IMU compare.

Compares three orientation samples per telemetry frame:
  * OLD: raw IMU quaternion (BNO085 world frame: Z-grav up, X mag-north)
  * NEW: R_ee_from_imu = R_world_bias^-1 · R_imu · R_mount^-1   (base frame)
  * FK : base-frame EE rotation reported by ws_handlers_imu (ground truth)

Also compares tool-tip position:
  * OLD tool = FK wc + R_imu_raw · TOOL_OFFSET
  * NEW tool = FK wc + R_ee_from_imu · TOOL_OFFSET
  * FK  tool = fk_live_{x,y,z}_mm   (ground truth)

If the fix works, NEW orientation is close to FK orientation (within sensor
noise + mechanical imperfections) while OLD differs by the world_bias yaw
(≈ 30°) and mount tilt (~2°). Tool position similarly snaps closer.
"""
import asyncio, json, math, ssl, urllib.request
import websockets

HTTP = "https://127.0.0.1:8443/api/imu-frame-calib"
WS   = "wss://127.0.0.1:8557"
TOOL_OFFSET_M = (0.06, 0.0, 0.0)

def q_mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def q_conj(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    return (w/n, -x/n, -y/n, -z/n)

def q_to_euler_zyx(q):
    """scipy-compat: as_euler('ZYX') → (yaw, pitch, roll) in radians."""
    w, x, y, z = q
    # yaw (Z), pitch (Y), roll (X) — intrinsic ZYX = extrinsic XYZ
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch_s = 2*(w*y - z*x); pitch_s = max(-1.0, min(1.0, pitch_s))
    pitch = math.asin(pitch_s)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return yaw, pitch, roll

def q_to_mat(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    w, x, y, z = w/n, x/n, y/n, z/n
    return [
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ]

def mat_vec(m, v):
    return (m[0][0]*v[0]+m[0][1]*v[1]+m[0][2]*v[2],
            m[1][0]*v[0]+m[1][1]*v[1]+m[1][2]*v[2],
            m[2][0]*v[0]+m[2][1]*v[1]+m[2][2]*v[2])

def rdeg(r): return math.degrees(r)

async def main():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(HTTP, context=ctx, timeout=5) as r:
        calib = json.loads(r.read().decode("utf-8"))
    q_mount = tuple(calib["mount"]["quat_wxyz"])
    q_wb    = tuple(calib["world_bias"]["quat_wxyz"])
    q_mount_inv = q_conj(q_mount); q_wb_inv = q_conj(q_wb)
    print(f"Mount     quat_wxyz = {q_mount}  (present={calib['mount']['present']})")
    print(f"WorldBias quat_wxyz = {q_wb}     (present={calib['world_bias']['present']})")
    y_wb, p_wb, r_wb = q_to_euler_zyx(q_wb)
    y_m, p_m, r_m = q_to_euler_zyx(q_mount)
    print(f"World bias RPY deg   = yaw={rdeg(y_wb):+7.2f}  pitch={rdeg(p_wb):+6.2f}  roll={rdeg(r_wb):+6.2f}")
    print(f"Mount      RPY deg   = yaw={rdeg(y_m):+7.2f}  pitch={rdeg(p_m):+6.2f}  roll={rdeg(r_m):+6.2f}")
    print()

    async with websockets.connect(WS, ssl=ctx) as ws:
        rows = []
        t_end = asyncio.get_event_loop().time() + 10.0
        while asyncio.get_event_loop().time() < t_end and len(rows) < 80:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            try: m = json.loads(raw)
            except Exception: continue
            if m.get("type") != "telemetry": continue
            if m.get("imu_valid") is not True: continue
            if "fk_live_x_mm" not in m or "fk_live_yaw" not in m: continue
            q_imu = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
            q_base = q_mul(q_mul(q_wb_inv, q_imu), q_mount_inv)
            yi, pi, ri = q_to_euler_zyx(q_imu)
            yb, pb, rb = q_to_euler_zyx(q_base)
            yf, pf, rf = math.radians(m["fk_live_yaw"]), math.radians(m["fk_live_pitch"]), math.radians(m["fk_live_roll"])
            wc = (m["fk_live_wc_x_mm"]/1000, m["fk_live_wc_y_mm"]/1000, m["fk_live_wc_z_mm"]/1000)
            fk_tool = (m["fk_live_x_mm"], m["fk_live_y_mm"], m["fk_live_z_mm"])
            old_tool = tuple((wc[i] + mat_vec(q_to_mat(q_imu), TOOL_OFFSET_M)[i]) * 1000 for i in range(3))
            new_tool = tuple((wc[i] + mat_vec(q_to_mat(q_base), TOOL_OFFSET_M)[i]) * 1000 for i in range(3))
            rows.append({
                "old_yaw": rdeg(yi), "old_pitch": rdeg(pi), "old_roll": rdeg(ri),
                "new_yaw": rdeg(yb), "new_pitch": rdeg(pb), "new_roll": rdeg(rb),
                "fk_yaw": rdeg(yf),  "fk_pitch": rdeg(pf),  "fk_roll": rdeg(rf),
                "old_tool": old_tool, "new_tool": new_tool, "fk_tool": fk_tool,
            })
        if not rows:
            print("No valid samples (is IMU publishing, is FK live computed?)"); return

        def avg(key): return sum(r[key] for r in rows)/len(rows)
        def avg_tool(key):
            return tuple(sum(r[key][i] for r in rows)/len(rows) for i in range(3))

        ref = rows[0]
        print(f"Samples collected: {len(rows)}")
        print()
        print("=== Orientation (avg over samples, degrees, ZYX yaw/pitch/roll) ===")
        print(f"{'quantity':18s} {'yaw':>8s} {'pitch':>8s} {'roll':>8s}   vs FK (deg, wrapped)")
        for label, pfx in (("OLD (raw IMU)", "old"), ("NEW (base frame)", "new"), ("FK live (base)", "fk")):
            y, p, r = avg(f"{pfx}_yaw"), avg(f"{pfx}_pitch"), avg(f"{pfx}_roll")
            if pfx == "fk":
                print(f"{label:18s} {y:+8.2f} {p:+8.2f} {r:+8.2f}")
            else:
                dy = ((y - avg("fk_yaw") + 180) % 360) - 180
                dp = ((p - avg("fk_pitch") + 180) % 360) - 180
                dr = ((r - avg("fk_roll") + 180) % 360) - 180
                print(f"{label:18s} {y:+8.2f} {p:+8.2f} {r:+8.2f}   Δ=({dy:+6.1f}, {dp:+6.1f}, {dr:+6.1f})")
        print()
        print("=== Tool position (mm, avg over samples) ===")
        ot = avg_tool("old_tool"); nt = avg_tool("new_tool"); ft = avg_tool("fk_tool")
        print(f"{'quantity':18s} {'x':>9s} {'y':>9s} {'z':>9s}   vs FK (mm)")
        print(f"{'OLD (raw IMU)':18s} {ot[0]:+9.2f} {ot[1]:+9.2f} {ot[2]:+9.2f}   Δ=({ot[0]-ft[0]:+6.1f}, {ot[1]-ft[1]:+6.1f}, {ot[2]-ft[2]:+6.1f})")
        print(f"{'NEW (base frame)':18s} {nt[0]:+9.2f} {nt[1]:+9.2f} {nt[2]:+9.2f}   Δ=({nt[0]-ft[0]:+6.1f}, {nt[1]-ft[1]:+6.1f}, {nt[2]-ft[2]:+6.1f})")
        print(f"{'FK live (base)':18s} {ft[0]:+9.2f} {ft[1]:+9.2f} {ft[2]:+9.2f}")
        print()

        old_err = sum(abs(((avg(f"old_{k}") - avg(f"fk_{k}") + 180) % 360) - 180) for k in ("yaw","pitch","roll"))
        new_err = sum(abs(((avg(f"new_{k}") - avg(f"fk_{k}") + 180) % 360) - 180) for k in ("yaw","pitch","roll"))
        print(f"Sum |Δrpy| vs FK:  OLD = {old_err:.2f}°   NEW = {new_err:.2f}°")
        if new_err < old_err * 0.5 + 5.0:
            print("✓ FRAME FIX VALIDATED — NEW significantly closer to FK than OLD.")
        else:
            print("⚠ NEW not significantly closer — verify calib files / mount sign.")

asyncio.run(main())
