#!/usr/bin/env python3
"""
verify_ik_imu_compare.py — Quantitative before/after for the IK compare window.

Replays the same math the frontend uses on live telemetry frames.
Simulates:
  * OLD: inertial integrator reading imu_accel_* and subtracting gravity
  * NEW: if accBody raw is zero, anchor wc to FK live wc (no integration)

With BNO085 v1 publishing accel=[0,0,0] exactly, the OLD path should drift
Z at ~-0.45 m/s (clamped by IMU_MAX_SPEED_MPS). The NEW path should stay
pinned at the FK wc, so Z(est) ≈ Z(fk) on each sample.
"""
import asyncio, json, ssl, time, math
import websockets

# Constants mirrored from ik.js
IMU_GRAVITY_MPS2 = 9.80665
IMU_ACCEL_DEADBAND_MPS2 = 0.18
IMU_STILL_GYRO_RAD_S = 0.12
IMU_MAX_DT_S = 0.08
IMU_ACTIVE_DAMP = 0.985
IMU_STILL_DAMP = 0.42
IMU_MAX_SPEED_MPS = 0.45
TOOL_OFFSET_M = [0.06, 0.0, 0.0]

def q2m(w, x, y, z):
    n = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    w, x, y, z = w/n, x/n, y/n, z/n
    return [
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ]

def mv(M, v):
    return [M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],
            M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],
            M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]]

async def main():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect('wss://127.0.0.1:8557', ssl=ctx) as ws:
        primed = False
        est_wc_old = None
        est_vel_old = [0.0, 0.0, 0.0]
        last_ts = None
        last_counter = None
        sim_started = None
        samples = []

        print("Simulating OLD vs NEW wc integration over 8 s at current robot pose (fermo)…")
        print(f"{'t':>5s} {'fk_wc_z_mm':>11s} {'OLD_est_z_mm':>13s} {'NEW_est_z_mm':>13s} {'OLD-FK':>10s} {'NEW-FK':>10s}")
        print("-"*76)
        t_end = time.monotonic() + 8.0
        while time.monotonic() < t_end:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            try:
                m = json.loads(raw)
            except Exception:
                continue
            if m.get("type") != "telemetry" or m.get("imu_valid") is not True:
                continue
            if "fk_live_wc_z_mm" not in m or "imu_q_w" not in m:
                continue

            fk_wc_m = [m["fk_live_wc_x_mm"]/1000.0, m["fk_live_wc_y_mm"]/1000.0, m["fk_live_wc_z_mm"]/1000.0]
            qw, qx, qy, qz = m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"]
            acc = [float(m.get("imu_accel_x") or 0), float(m.get("imu_accel_y") or 0), float(m.get("imu_accel_z") or 0)]
            gyro = [float(m.get("imu_gyro_x") or 0), float(m.get("imu_gyro_y") or 0), float(m.get("imu_gyro_z") or 0)]
            counter = m.get("imu_sample_counter")

            now = time.monotonic()
            if sim_started is None:
                sim_started = now

            if not primed:
                est_wc_old = list(fk_wc_m)
                est_vel_old = [0.0, 0.0, 0.0]
                last_ts = now
                last_counter = counter
                primed = True
                continue
            if counter is not None and counter == last_counter:
                continue

            dt = max(0.001, min(IMU_MAX_DT_S, now - last_ts))
            last_ts = now
            last_counter = counter

            # OLD integrator (replica fedele della logica pre-fix)
            rot = q2m(qw, qx, qy, qz)
            accW = mv(rot, acc)
            lin = [accW[0], accW[1], accW[2] - IMU_GRAVITY_MPS2]
            acc_mag = math.sqrt(lin[0]**2 + lin[1]**2 + lin[2]**2)
            gyro_mag = math.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2)
            if acc_mag < IMU_ACCEL_DEADBAND_MPS2:
                lin = [0,0,0]
            damp = IMU_STILL_DAMP if (lin == [0,0,0] and gyro_mag < IMU_STILL_GYRO_RAD_S) else IMU_ACTIVE_DAMP
            nv = [est_vel_old[i]*damp + lin[i]*dt for i in range(3)]
            speed = math.sqrt(nv[0]**2 + nv[1]**2 + nv[2]**2)
            if speed > IMU_MAX_SPEED_MPS:
                k = IMU_MAX_SPEED_MPS/max(speed,1e-6)
                nv = [nv[i]*k for i in range(3)]
            linN = math.sqrt(lin[0]**2 + lin[1]**2 + lin[2]**2)
            if gyro_mag < IMU_STILL_GYRO_RAD_S*0.7 and linN < IMU_ACCEL_DEADBAND_MPS2*0.5:
                nv = [0,0,0]
            est_vel_old = nv
            est_wc_old = [est_wc_old[i] + nv[i]*dt for i in range(3)]

            # NEW logic (anchor to FK wc when accel raw not available)
            accel_raw_available = any(abs(v) > 1e-6 for v in acc)
            est_wc_new = list(fk_wc_m) if not accel_raw_available else list(est_wc_old)

            t_rel = now - sim_started
            samples.append((t_rel, fk_wc_m[2]*1000, est_wc_old[2]*1000, est_wc_new[2]*1000))
            # print a row every ~0.5 s
            if int(t_rel*2) % 1 == 0 and samples and len(samples) % 5 == 0:
                pass

        # Print a dense summary: first sample + every ~1 s
        print("\nSummary (dense, samples at ~100Hz — showing every ~1s):")
        if not samples:
            print("  NO SAMPLES — is the robot publishing IMU+FK telemetry?")
            return
        period = max(1, len(samples)//8)
        for i in range(0, len(samples), period):
            t, fk, old, new = samples[i]
            print(f"{t:5.2f}s  fk_z={fk:8.2f} mm  OLD_z={old:10.2f} mm  NEW_z={new:8.2f} mm  "
                  f"(OLD-FK={old-fk:+8.2f} mm, NEW-FK={new-fk:+6.2f} mm)")
        t, fk, old, new = samples[-1]
        print(f"{t:5.2f}s  fk_z={fk:8.2f} mm  OLD_z={old:10.2f} mm  NEW_z={new:8.2f} mm  "
              f"(OLD-FK={old-fk:+8.2f} mm, NEW-FK={new-fk:+6.2f} mm)   ← FINAL")

        old_drift = samples[-1][2] - samples[0][1]
        new_drift = samples[-1][3] - samples[0][1]
        print("\nVerdict:")
        print(f"  OLD integrator Z drift over {samples[-1][0]:.1f}s at rest: {old_drift:+.2f} mm")
        print(f"  NEW logic       Z drift over {samples[-1][0]:.1f}s at rest: {new_drift:+.2f} mm")
        if abs(new_drift) < 10 and abs(old_drift) > 100:
            print("  ✓ FIX VALIDATED: new path stays pinned to FK wc; old path diverges.")
        else:
            print("  ⚠ Unexpected: inspect raw samples above.")

asyncio.run(main())
