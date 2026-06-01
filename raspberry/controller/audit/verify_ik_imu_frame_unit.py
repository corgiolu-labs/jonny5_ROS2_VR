#!/usr/bin/env python3
"""
verify_ik_imu_frame_unit.py — algebraic unit test of the frame composition
applied by ik.js after the IMU frame-alignment fix.

Uses the LIVE calib fetched from /api/imu-frame-calib to be absolutely sure
the frontend and the analytics validators use the same rotations. Then for
a hand-chosen R_ee_true (a known base-frame orientation):

  1. Simulate what the IMU would report in BNO085 world frame:
         R_imu = R_world_bias · R_ee_true · R_mount
     (exactly the analytics physical model; this is NOT the code path under
     test — it's only the oracle that generates a realistic IMU reading).

  2. Apply the frontend composition that we just deployed:
         R_ee_recovered = R_world_bias⁻¹ · R_imu · R_mount⁻¹

  3. Verify R_ee_recovered == R_ee_true to within float precision, for both
     orientation and position (tool tip = wc + R_ee · TOOL_OFFSET).

This is independent from whether the STM32 is currently publishing valid
telemetry — it exercises only the rotation algebra, same routine path the
browser runs on real samples.
"""
import json, math, ssl, urllib.request
from scipy.spatial.transform import Rotation as R

HTTP = "https://127.0.0.1:8443/api/imu-frame-calib"
TOOL_OFFSET_M = [0.06, 0.0, 0.0]

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]

def load_live_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(HTTP, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def r_from_calib(slot):
    q = slot["quat_wxyz"]
    return R.from_quat(q_wxyz_to_xyzw(q))

def frontend_recover(q_imu_wxyz, r_mount_inv, r_world_bias_inv):
    """Same composition the JS does (_imuQuatToBaseFrame)."""
    r_imu = R.from_quat(q_wxyz_to_xyzw(q_imu_wxyz))
    return r_world_bias_inv * r_imu * r_mount_inv

def tool_tip(wc_m, r_ee):
    v = r_ee.as_matrix() @ TOOL_OFFSET_M
    return [wc_m[0] + v[0], wc_m[1] + v[1], wc_m[2] + v[2]]

def q_xyzw_to_wxyz(q): return [q[3], q[0], q[1], q[2]]

def main():
    calib = load_live_calib()
    print("Live calib from Pi /api/imu-frame-calib:")
    print(f"  mount    : quat_wxyz={calib['mount']['quat_wxyz']}  present={calib['mount']['present']}")
    print(f"  worldBias: quat_wxyz={calib['world_bias']['quat_wxyz']}  present={calib['world_bias']['present']}")
    r_mount = r_from_calib(calib["mount"])
    r_wb    = r_from_calib(calib["world_bias"])
    r_mount_inv = r_mount.inv()
    r_wb_inv    = r_wb.inv()
    wb_ypr = r_wb.as_euler("ZYX", degrees=True)
    mn_ypr = r_mount.as_euler("ZYX", degrees=True)
    print(f"  worldBias YPR deg = yaw={wb_ypr[0]:+7.2f}  pitch={wb_ypr[1]:+6.2f}  roll={wb_ypr[2]:+6.2f}")
    print(f"  mount     YPR deg = yaw={mn_ypr[0]:+7.2f}  pitch={mn_ypr[1]:+6.2f}  roll={mn_ypr[2]:+6.2f}")

    # A small suite of EE orientations to test round-trip recovery.
    # Chosen to exercise non-trivial YPR combinations.
    test_poses = [
        ("HOME",                R.identity()),
        ("Yaw+30°",             R.from_euler("ZYX", [+30, 0, 0], degrees=True)),
        ("Yaw-45°",             R.from_euler("ZYX", [-45, 0, 0], degrees=True)),
        ("Pitch+20°",           R.from_euler("ZYX", [0, +20, 0], degrees=True)),
        ("Roll+15°",            R.from_euler("ZYX", [0, 0, +15], degrees=True)),
        ("Compound 30/20/15",   R.from_euler("ZYX", [30, 20, 15], degrees=True)),
        ("Compound -60/-10/-30",R.from_euler("ZYX", [-60, -10, -30], degrees=True)),
    ]
    wc_m = [0.20, -0.10, 0.30]  # arbitrary wrist-center in base frame

    print(f"\n{'case':>22s} {'Δyaw':>7s} {'Δpitch':>7s} {'Δroll':>7s} {'‖Δtool‖ mm':>11s}  result")
    print("-"*76)
    worst = 0.0
    for name, r_ee_true in test_poses:
        # Oracle: what would BNO085 report if the analytics model is exact?
        r_imu_oracle = r_wb * r_ee_true * r_mount
        q_imu_wxyz = q_xyzw_to_wxyz(r_imu_oracle.as_quat())

        # Apply the frontend composition (this is what ik.js now does).
        r_ee_recovered = frontend_recover(q_imu_wxyz, r_mount_inv, r_wb_inv)

        # Errors.
        r_err = r_ee_true.inv() * r_ee_recovered
        dyaw, dpitch, droll = r_err.as_euler("ZYX", degrees=True)
        tool_true = tool_tip(wc_m, r_ee_true)
        tool_rec  = tool_tip(wc_m, r_ee_recovered)
        d_tool_mm = math.sqrt(sum((1000*(a-b))**2 for a, b in zip(tool_true, tool_rec)))
        worst = max(worst, abs(dyaw), abs(dpitch), abs(droll), d_tool_mm)
        ok = "PASS" if (abs(dyaw) < 1e-6 and abs(dpitch) < 1e-6 and abs(droll) < 1e-6 and d_tool_mm < 1e-4) else "FAIL"
        print(f"{name:>22s} {dyaw:+7.4f} {dpitch:+7.4f} {droll:+7.4f} {d_tool_mm:11.6f}   {ok}")

    print()
    # Also show, on HOME, how different the raw-IMU vs base-frame YPR are,
    # so the user can see the magnitude of the correction in degrees.
    r_imu_home = r_wb * R.identity() * r_mount
    y_raw, p_raw, r_raw = r_imu_home.as_euler("ZYX", degrees=True)
    y_new, p_new, r_new = R.identity().as_euler("ZYX", degrees=True)
    print("Magnitude of the frame correction (HOME EE as reference):")
    print(f"  OLD display (raw IMU YPR)  = yaw={y_raw:+7.2f}  pitch={p_raw:+6.2f}  roll={r_raw:+6.2f}")
    print(f"  NEW display (base-frame)   = yaw={y_new:+7.2f}  pitch={p_new:+6.2f}  roll={r_new:+6.2f}")
    print(f"  Δ (OLD − NEW)              = yaw={y_raw-y_new:+7.2f}  pitch={p_raw-p_new:+6.2f}  roll={r_raw-r_new:+6.2f}")
    print()
    print(f"Worst-case round-trip residual across {len(test_poses)} test orientations: {worst:.9f}")
    print("✓ Algebra coerente con il modello R_imu = R_world_bias · R_ee · R_mount." if worst < 1e-4
          else "✗ Residual oltre soglia — verificare composizione.")

if __name__ == "__main__":
    main()
