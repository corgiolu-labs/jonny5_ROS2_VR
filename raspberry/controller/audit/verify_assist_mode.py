#!/usr/bin/env python3
"""
verify_assist_mode.py — real tests on live robot in ASSIST mode (mode=5).

Strategy
========
- Inject simulated head quaternion via WS intent JSON (same path as Quest),
  using bits that activate firmware deadman (buttons_left=buttons_right=0x0002
  + grip=1 + heartbeat incrementing).
- Concurrently capture telemetry at ~100 Hz (servo_deg_*, fk_live_*, imu_q_*,
  plus mode5_arm target when exposed) into in-memory timelines.
- Run a sequence of canonical test scenarios (step, osc, fast, extended).
- Compute latency / rise time / settling time / overshoot / coupling offline.
- Print a structured summary table + save CSV with raw samples.

Observer model
==============
- servo_deg_*       = physical servo readback (what really happened)
- fk_live_*         = model-predicted tool pose (base frame)
- imu_q_* (+ calib) = IMU-observed wrist orientation in base frame
- mode5_arm (Pi)    = what the assist logic asked the arm joints to do

Read-only: no firmware / math / calib change.
"""
import asyncio, json, math, ssl, sys, time, csv, os, urllib.request
import websockets
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation as R

WS = "wss://127.0.0.1:8557"
CALIB_URL = "https://127.0.0.1:8443/api/imu-frame-calib"
OUT_CSV  = "/tmp/verify_assist_mode.csv"
OUT_SUM  = "/tmp/verify_assist_mode.summary.txt"

INTENT_HZ    = 60
SAMPLE_HZ    = 100
SETTLE_S     = 0.6
MOTION_TMO   = 25.0

# Test phases (each: (name, duration_s, quat_generator_fn(t_rel)))
# quat is xyzw; returned quat_wxyz = (w, x, y, z)
def q_identity():
    return (1.0, 0.0, 0.0, 0.0)

def q_from_yaw_deg(deg):
    r = R.from_euler("Z", deg, degrees=True)
    x, y, z, w = r.as_quat()
    return (w, x, y, z)

def q_from_pitch_deg(deg):
    r = R.from_euler("Y", deg, degrees=True)
    x, y, z, w = r.as_quat()
    return (w, x, y, z)

def q_from_roll_deg(deg):
    r = R.from_euler("X", deg, degrees=True)
    x, y, z, w = r.as_quat()
    return (w, x, y, z)

# ---------------------------------------------------------------------------
# Test scenarios — each is a list of (phase_name, duration_s, quat_fn(t_phase))
# plus an optional pre-pose (joint angles virtual; if set, SETPOSE before test)
# ---------------------------------------------------------------------------
TESTS = [
    {
        "id": 1, "name": "Yaw step +30° hold",
        "pre_pose": [90,90,90,90,90,90],   # HOME
        "phases": [
            ("baseline",  1.0, lambda t: q_identity()),
            ("step_yaw",  4.0, lambda t: q_from_yaw_deg(+30.0)),
        ],
    },
    {
        "id": 2, "name": "Pitch step +20° hold",
        "pre_pose": [90,90,90,90,90,90],
        "phases": [
            ("baseline",   1.0, lambda t: q_identity()),
            ("step_pitch", 4.0, lambda t: q_from_pitch_deg(+20.0)),
        ],
    },
    {
        "id": 3, "name": "Small osc ±8° yaw @ 0.6 Hz",
        "pre_pose": [90,90,90,90,90,90],
        "phases": [
            ("baseline", 0.8, lambda t: q_identity()),
            ("osc_yaw",  6.0, lambda t: q_from_yaw_deg(8.0 * math.sin(2*math.pi*0.6*t))),
        ],
    },
    {
        "id": 4, "name": "Fast large ±25° yaw @ 1.4 Hz",
        "pre_pose": [90,90,90,90,90,90],
        "phases": [
            ("baseline", 0.8, lambda t: q_identity()),
            ("osc_fast", 4.0, lambda t: q_from_yaw_deg(25.0 * math.sin(2*math.pi*1.4*t))),
        ],
    },
    {
        "id": 5, "name": "Extended pose + yaw step",
        "pre_pose": [90, 70, 115, 90, 90, 90],  # spalla-20, gomito+25 → wc forward
        "phases": [
            ("baseline", 1.0, lambda t: q_identity()),
            ("step_yaw", 4.0, lambda t: q_from_yaw_deg(+20.0)),
        ],
    },
    {
        "id": 6, "name": "Wrist-dominated: roll step +25° (visore roll → PITCH robot)",
        "pre_pose": [90,90,90,90,90,90],
        "phases": [
            ("baseline", 0.8, lambda t: q_identity()),
            ("step_roll", 4.0, lambda t: q_from_roll_deg(+25.0)),
        ],
    },
    {
        "id": 7, "name": "Velocity-based probe: yaw ramp 0→30° in 3s, then hold",
        "pre_pose": [90,90,90,90,90,90],
        "phases": [
            ("baseline", 0.8, lambda t: q_identity()),
            # Linear ramp over 3 s: simulates continuous head turn
            ("ramp_yaw", 3.0, lambda t: q_from_yaw_deg(min(30.0, (t/3.0)*30.0))),
            # Hold at +30: head still. A velocity-based controller should STOP moving here.
            ("hold_yaw", 3.0, lambda t: q_from_yaw_deg(30.0)),
        ],
    },
]

# ---------------------------------------------------------------------------
# Helpers: calib + WS intent injector + telemetry capture
# ---------------------------------------------------------------------------
def load_calib():
    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    with urllib.request.urlopen(CALIB_URL, context=ctx, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))

def q_wxyz_to_xyzw(q): return [q[1], q[2], q[3], q[0]]

def r_from_quat_wxyz(q_wxyz): return R.from_quat(q_wxyz_to_xyzw(q_wxyz))

def wrap_deg(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a

# Global shared state
state = {
    "target_quat_wxyz": (1.0, 0.0, 0.0, 0.0),
    "target_yaw_cmd_deg": 0.0,     # convenience: ground-truth commanded yaw
    "target_pitch_cmd_deg": 0.0,
    "target_roll_cmd_deg": 0.0,
    "phase_name": "idle",
    "assist_grip": 0,              # 0 = grip off, 1 = grip on (deadman)
    "injector_run": True,
}

async def intent_injector(ws):
    """Send intents at INTENT_HZ. Headset quat from state['target_quat_wxyz']."""
    dt = 1.0 / INTENT_HZ
    hb = 0
    while state["injector_run"]:
        qw, qx, qy, qz = state["target_quat_wxyz"]
        hb = (hb + 1) & 0xFFFF
        grip = 1 if state["assist_grip"] else 0
        buttons = 0x0002 if state["assist_grip"] else 0x0000  # bit1 = deadman
        msg = {
            "mode": 5,
            "quat_w": float(qw), "quat_x": float(qx), "quat_y": float(qy), "quat_z": float(qz),
            "grip": grip,
            "buttons_left": buttons, "buttons_right": buttons,
            "heartbeat": hb,
            "joy_x": 0.0, "joy_y": 0.0, "pitch": 0.0, "yaw": 0.0,
            "intensity": 255,
        }
        try:
            await ws.send(json.dumps(msg))
        except Exception:
            return
        await asyncio.sleep(dt)

async def telemetry_capture(ws, r_wb_inv, r_mount_inv, r_home_inv, sink):
    """Sink telemetry frames into `sink` list. Keeps the latest of everything."""
    TOOL = [0.06, 0.0, 0.0]
    dt = 1.0 / SAMPLE_HZ
    t0 = time.monotonic()
    while state["injector_run"]:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
        except asyncio.TimeoutError:
            continue
        except Exception:
            return
        try: m = json.loads(raw)
        except Exception: continue
        if m.get("type") != "telemetry": continue
        if "servo_deg_B" not in m: continue
        # Filter stale telemetry frames (servo readback out of valid [5,175]).
        try:
            vals = [float(m.get(k)) for k in ("servo_deg_B","servo_deg_S","servo_deg_G","servo_deg_Y","servo_deg_P","servo_deg_R")]
            if any((v < 5.0 or v > 175.0) for v in vals):
                continue
        except Exception:
            continue
        t_rel = time.monotonic() - t0
        row = {
            "t":       round(t_rel, 4),
            "phase":   state["phase_name"],
            "cmd_yaw": round(state["target_yaw_cmd_deg"], 3),
            "cmd_pit": round(state["target_pitch_cmd_deg"], 3),
            "cmd_rol": round(state["target_roll_cmd_deg"], 3),
            "servo_B": m.get("servo_deg_B"),
            "servo_S": m.get("servo_deg_S"),
            "servo_G": m.get("servo_deg_G"),
            "servo_Y": m.get("servo_deg_Y"),
            "servo_P": m.get("servo_deg_P"),
            "servo_R": m.get("servo_deg_R"),
        }
        if m.get("fk_live_valid"):
            row.update({
                "fk_x":  m.get("fk_live_x_mm"),
                "fk_y":  m.get("fk_live_y_mm"),
                "fk_z":  m.get("fk_live_z_mm"),
                "fk_yaw": m.get("fk_live_yaw"),
                "fk_pit": m.get("fk_live_pitch"),
                "fk_rol": m.get("fk_live_roll"),
            })
        if m.get("imu_valid") is True and m.get("imu_q_w") is not None:
            qi = (m["imu_q_w"], m["imu_q_x"], m["imu_q_y"], m["imu_q_z"])
            r_ee = r_home_inv * r_wb_inv * r_from_quat_wxyz(qi) * r_mount_inv
            ypr = r_ee.as_euler("ZYX", degrees=True)
            row["imu_yaw"]   = round(float(ypr[0]), 3)
            row["imu_pit"]   = round(float(ypr[1]), 3)
            row["imu_rol"]   = round(float(ypr[2]), 3)
        sink.append(row)

async def run_phases(phases):
    """Drive state['target_quat_wxyz'] according to each phase's function.
       Returns the list of (phase_name, t_start, t_end) boundaries in relative
       time from the start of this run. Also updates state['target_*_cmd_deg']
       so the CSV preserves the commanded angle."""
    boundaries = []
    t_start_abs = time.monotonic()
    for pname, dur, qfn in phases:
        state["phase_name"] = pname
        t_phase_start = time.monotonic() - t_start_abs
        t_phase0 = time.monotonic()
        step_dt = 0.02  # 50 Hz quat update
        while True:
            t_phase = time.monotonic() - t_phase0
            if t_phase >= dur: break
            q = qfn(t_phase)
            state["target_quat_wxyz"] = q
            # Derive convenience commanded angles (Euler ZYX) for the CSV
            r = r_from_quat_wxyz(q)
            y, p, r_ = r.as_euler("ZYX", degrees=True)
            state["target_yaw_cmd_deg"]   = float(y)
            state["target_pitch_cmd_deg"] = float(p)
            state["target_roll_cmd_deg"]  = float(r_)
            await asyncio.sleep(step_dt)
        t_phase_end = time.monotonic() - t_start_abs
        boundaries.append((pname, t_phase_start, t_phase_end))
    state["phase_name"] = "idle"
    return boundaries

async def await_setpose_done(ws, tmo):
    t_end = time.monotonic() + tmo
    while time.monotonic() < t_end:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        except Exception:
            return False
        try: m = json.loads(raw)
        except Exception: continue
        if m.get("type") == "setpose_done": return True
    return False

# ---------------------------------------------------------------------------
# Offline metrics
# ---------------------------------------------------------------------------
def compute_step_metrics(rows, phase_name, signal_key, cmd_delta, t_phase_start):
    """For a step response: detect reaction delay, 10-90% rise, settling ±5%, overshoot."""
    xs = [r for r in rows if r.get("phase") == phase_name and r.get(signal_key) is not None]
    if len(xs) < 5:
        return None
    baseline_rows = [r for r in rows if r.get("phase") == "baseline" and r.get(signal_key) is not None]
    if not baseline_rows:
        return None
    baseline = sum(r[signal_key] for r in baseline_rows[-10:]) / min(10, len(baseline_rows))
    t0 = xs[0]["t"]
    vals = [(r["t"] - t0, r[signal_key] - baseline) for r in xs]

    # React delay: first time |delta| exceeds 5% of commanded |cmd_delta|
    threshold = 0.05 * abs(cmd_delta)
    t_react = None
    for t, v in vals:
        if abs(v) >= threshold:
            t_react = t; break

    final_vals = [v for _, v in vals[-int(0.3 * len(vals)):]]
    steady = sum(final_vals) / max(1, len(final_vals))
    if abs(cmd_delta) < 1e-3:
        return {"n": len(vals), "t_react": t_react, "steady": steady}

    # Rise time 10% -> 90% of steady
    signs = 1.0 if steady >= 0 else -1.0
    target10 = 0.1 * steady
    target90 = 0.9 * steady
    t10 = t90 = None
    for t, v in vals:
        if t10 is None and v * signs >= target10 * signs:
            t10 = t
        if t10 is not None and v * signs >= target90 * signs:
            t90 = t; break
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Settling time ±5% of steady
    settle_band = 0.05 * abs(steady) if abs(steady) > 0.01 else 0.5
    t_settle = None
    for i, (t, v) in enumerate(vals):
        window = [vv for tt, vv in vals[i:] if tt - t <= 0.4]
        if not window: continue
        if all(abs(vv - steady) <= settle_band for vv in window):
            t_settle = t; break

    # Overshoot (beyond steady toward same direction as cmd)
    if signs > 0:
        peak = max(v for _, v in vals)
        overshoot = max(0.0, (peak - steady) / max(1e-6, abs(steady)))
    else:
        peak = min(v for _, v in vals)
        overshoot = max(0.0, (steady - peak) / max(1e-6, abs(steady)))

    ss_error = (cmd_delta - steady)

    return {
        "n": len(vals), "baseline": round(baseline, 3),
        "t_react_ms": round((t_react or 0.0) * 1000, 0),
        "t_rise_ms": round((rise or 0.0) * 1000, 0) if rise is not None else None,
        "t_settle_ms": round((t_settle or 0.0) * 1000, 0) if t_settle is not None else None,
        "steady": round(steady, 3),
        "cmd_delta": round(cmd_delta, 3),
        "ss_error": round(ss_error, 3),
        "overshoot_pct": round(overshoot * 100, 1),
    }

def compute_tracking_metrics(rows, phase_name, sig_key, cmd_key):
    """For oscillatory input: tracking lag (cross-corr peak), amplitude ratio."""
    xs = [r for r in rows if r.get("phase") == phase_name and r.get(sig_key) is not None and r.get(cmd_key) is not None]
    if len(xs) < 30:
        return None
    ts  = [r["t"] for r in xs]
    cmd = [r[cmd_key] for r in xs]
    sig = [r[sig_key] for r in xs]
    cm  = sum(cmd)/len(cmd); sm = sum(sig)/len(sig)
    cmd_c = [v - cm for v in cmd]; sig_c = [v - sm for v in sig]
    cmd_rms = math.sqrt(sum(v*v for v in cmd_c)/len(cmd_c)) or 1e-9
    sig_rms = math.sqrt(sum(v*v for v in sig_c)/len(sig_c))
    # Cross-corr peak (sig lags cmd) — search lag up to 500 ms
    dt = (ts[-1] - ts[0]) / max(1, (len(ts)-1))
    max_lag_steps = int(0.5 / max(dt, 1e-3))
    best_lag = 0; best_c = -1e9
    for lag in range(0, max_lag_steps + 1):
        num = 0.0
        for i in range(len(sig_c) - lag):
            num += cmd_c[i] * sig_c[i+lag]
        c = num / (len(sig_c) - lag)
        if c > best_c:
            best_c = c; best_lag = lag
    return {
        "n": len(xs),
        "cmd_rms_deg": round(cmd_rms, 2),
        "sig_rms_deg": round(sig_rms, 2),
        "amp_ratio": round(sig_rms / cmd_rms, 3),
        "lag_ms": round(best_lag * dt * 1000, 0),
    }

def coupling_analysis(rows, phase_name, ignore_key=None):
    """Returns max signed deviation per joint during phase, useful to spot coupling."""
    xs = [r for r in rows if r.get("phase") == phase_name]
    if not xs: return {}
    baseline_rows = [r for r in rows if r.get("phase") == "baseline"]
    out = {}
    for k in ("servo_B", "servo_S", "servo_G", "servo_Y", "servo_P", "servo_R"):
        if k == ignore_key: continue
        base_vals = [r[k] for r in baseline_rows if r.get(k) is not None]
        phase_vals = [r[k] for r in xs if r.get(k) is not None]
        if not base_vals or not phase_vals: continue
        base = sum(base_vals[-10:]) / min(10, len(base_vals))
        dev = max(phase_vals, key=lambda v: abs(v - base)) - base
        out[k] = round(dev, 2)
    return out

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
async def run_test(ws, r_wb_inv, r_mount_inv, r_home_inv, test):
    print(f"\n=== TEST {test['id']} — {test['name']} ===")

    # Pre-pose (if required)
    pp = test.get("pre_pose")
    if pp is not None:
        cmd = f"SETPOSE {pp[0]} {pp[1]} {pp[2]} {pp[3]} {pp[4]} {pp[5]} 25 RTR5"
        await ws.send(json.dumps({"type":"uart","cmd":cmd}))
        print(f"  pre-pose: {pp}")
        await await_setpose_done(ws, MOTION_TMO)
        await asyncio.sleep(SETTLE_S)

    # Reset state
    state["target_quat_wxyz"] = (1.0, 0.0, 0.0, 0.0)
    state["assist_grip"] = 1   # engage deadman
    state["phase_name"]  = "pre"
    state["injector_run"] = True

    # Start background tasks
    sink = []
    injector_task = asyncio.create_task(intent_injector(ws))
    # Pre-warmup: give firmware mode=5 time to engage + telemetry to refresh
    # (we saw stale servo_deg values persisting until ~2.5 s into the first test)
    await asyncio.sleep(2.2)
    capture_task = asyncio.create_task(telemetry_capture(ws, r_wb_inv, r_mount_inv, r_home_inv, sink))
    # Drive the phases
    boundaries = await run_phases(test["phases"])

    # Tear down
    state["assist_grip"] = 0
    await asyncio.sleep(0.2)  # allow a couple of intents with grip=0 to flush
    state["injector_run"] = False
    injector_task.cancel()
    capture_task.cancel()
    for t in (injector_task, capture_task):
        try: await t
        except (asyncio.CancelledError, Exception): pass

    return sink, boundaries

async def main():
    print("=" * 92); print("ASSIST MODE — live measurement sweep"); print("=" * 92)

    calib = load_calib()
    r_mount = r_from_quat_wxyz(calib["mount"]["quat_wxyz"]); r_mount_inv = r_mount.inv()
    r_wb    = r_from_quat_wxyz(calib["world_bias"]["quat_wxyz"]); r_wb_inv = r_wb.inv()
    r_home  = r_from_quat_wxyz(calib["home"]["quat_wxyz"]) if calib["home"]["present"] else R.identity()
    r_home_inv = r_home.inv()
    print(f"Calib: mount={calib['mount']['present']} wb={calib['world_bias']['present']} home={calib['home']['present']}")

    ctx = ssl.create_default_context(); ctx.check_hostname=False; ctx.verify_mode=ssl.CERT_NONE
    async with websockets.connect(WS, ssl=ctx) as ws:
        await ws.send(json.dumps({"type":"uart","cmd":"SAFE"})); await asyncio.sleep(0.5)
        await ws.send(json.dumps({"type":"uart","cmd":"ENABLE"}))
        en_ok = False; tend = time.monotonic() + 30.0
        while time.monotonic() < tend:
            try: raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError: continue
            try: m = json.loads(raw)
            except: continue
            if m.get("type")=="uart_response" and "ENABLE" in str(m.get("cmd","")).upper():
                en_ok = bool(m.get("ok")); break
        print(f"ENABLE ok={en_ok}")
        if not en_ok: return

        summaries = []
        all_rows = []
        for test in TESTS:
            sink, bounds = await run_test(ws, r_wb_inv, r_mount_inv, r_home_inv, test)
            # Persist with test id
            for r in sink:
                r["test_id"] = test["id"]; r["test_name"] = test["name"]
                all_rows.append(r)
            summaries.append(("%d - %s" % (test["id"], test["name"]), sink, bounds, test))

        # Return to HOME
        await ws.send(json.dumps({"type":"uart","cmd":"SETPOSE 90 90 90 90 90 90 25 RTR5"}))
        await asyncio.sleep(0.3)

    # Save CSV
    try:
        keys = sorted({k for r in all_rows for k in r.keys()})
        with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader(); w.writerows(all_rows)
        print(f"\nCSV saved: {OUT_CSV}  ({len(all_rows)} rows)")
    except Exception as e:
        print(f"CSV save failed: {e}")

    # Compute + print summary
    lines = []
    def log(s):
        print(s); lines.append(s)
    log("\n" + "=" * 92)
    log("METRICS PER TEST")
    log("=" * 92)
    for label, sink, bounds, test in summaries:
        log(f"\n--- {label} ---")
        log(f"  samples: {len(sink)}   phases: {[b[0] for b in bounds]}")

        tid = test["id"]
        # Per-test analysis
        if tid == 1 or tid == 5:
            # yaw step: track BASE servo + IMU yaw + FK yaw
            for k, cmd_delta in (("servo_B", None), ("imu_yaw", None), ("fk_yaw", None)):
                m = compute_step_metrics(sink, "step_yaw", k, 30.0 if tid == 1 else 20.0, 0.0)
                if m: log(f"  {k:8s}  react={m.get('t_react_ms')}ms  rise10-90={m.get('t_rise_ms')}ms  settle±5%={m.get('t_settle_ms')}ms  over={m.get('overshoot_pct')}%  steady={m.get('steady')}  ss_err={m.get('ss_error')}")
            cpl = coupling_analysis(sink, "step_yaw", ignore_key="servo_B")
            log(f"  coupling (non-BASE joints, max Δ°): {cpl}")

        elif tid == 2:
            for k in ("servo_S", "servo_G", "fk_z", "imu_pit"):
                m = compute_step_metrics(sink, "step_pitch", k, 20.0, 0.0)
                if m: log(f"  {k:8s}  react={m.get('t_react_ms')}ms  rise10-90={m.get('t_rise_ms')}ms  settle±5%={m.get('t_settle_ms')}ms  over={m.get('overshoot_pct')}%  steady={m.get('steady')}  ss_err={m.get('ss_error')}")
            cpl = coupling_analysis(sink, "step_pitch")
            log(f"  coupling (max Δ° vs baseline, all joints): {cpl}")

        elif tid == 3 or tid == 4:
            phase = "osc_yaw" if tid == 3 else "osc_fast"
            for k in ("servo_B", "imu_yaw", "fk_yaw"):
                m = compute_tracking_metrics(sink, phase, k, "cmd_yaw")
                if m: log(f"  {k:8s}  amp_ratio={m.get('amp_ratio')}  lag={m.get('lag_ms')}ms  sig_rms={m.get('sig_rms_deg')}°  cmd_rms={m.get('cmd_rms_deg')}°")
            cpl = coupling_analysis(sink, phase, ignore_key="servo_B")
            log(f"  coupling: {cpl}")

        elif tid == 6:
            # Roll visore → robot wrist (remap roll_vis→pitch_robot, yaw_vis→roll_robot per head pipeline firmware)
            for k in ("servo_Y", "servo_P", "servo_R", "imu_yaw", "imu_pit", "imu_rol"):
                m = compute_step_metrics(sink, "step_roll", k, 25.0, 0.0)
                if m: log(f"  {k:8s}  react={m.get('t_react_ms')}ms  rise10-90={m.get('t_rise_ms')}ms  settle±5%={m.get('t_settle_ms')}ms  over={m.get('overshoot_pct')}%  steady={m.get('steady')}")
            cpl = coupling_analysis(sink, "step_roll")
            log(f"  coupling (all joints): {cpl}")

        elif tid == 7:
            # Ramp + hold: evidenzia controllo velocity-based sul braccio.
            # Il servo_B si muove durante il ramp, poi si ferma in hold (Δhead=0 → Δbase=0).
            ramp = [r for r in sink if r["phase"] == "ramp_yaw" and r.get("servo_B") is not None]
            hold = [r for r in sink if r["phase"] == "hold_yaw" and r.get("servo_B") is not None]
            if ramp:
                base = ramp[0]["servo_B"]
                ramp_peak = max(ramp, key=lambda r: r["servo_B"])["servo_B"] - base
                ramp_end  = ramp[-1]["servo_B"] - base
                log(f"  ramp_yaw servo_B: start={base:+.1f}  peak_delta={ramp_peak:+.1f}  end_delta={ramp_end:+.1f}")
            if hold:
                base_hold = hold[0]["servo_B"]
                hold_end  = hold[-1]["servo_B"] - base_hold
                hold_peak = max(hold, key=lambda r: abs(r["servo_B"] - base_hold))["servo_B"] - base_hold
                log(f"  hold_yaw servo_B: start={base_hold:+.1f}  end_drift={hold_end:+.1f}  peak_drift={hold_peak:+.1f}")
                log(f"    → se hold_end ≈ 0 → controllo VELOCITY confermato (arm si ferma con head ferma)")
                log(f"    → se hold_end ≠ 0 → drift integrativo o accoppiamento residuo")

    with open(OUT_SUM, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\nSummary saved: {OUT_SUM}")

asyncio.run(main())
