#!/usr/bin/env python3
"""
analyze_pitch_direction.py — diagnostic-only, no robot interaction.
Reads existing /tmp/assist_audit.csv and computes lag-compensated
correlations cmd_pit ↔ fk_wc_z, cmd_pit ↔ imu_pit, imu_pit ↔ fk_wc_z,
cmd_pit ↔ servo_S/G per pose, for the pitch_osc_10_04 phase.
"""
import csv, math, sys

CSV_PATH = "/tmp/assist_audit.csv"

def rms(xs):
    if not xs: return 0.0
    m=sum(xs)/len(xs); return math.sqrt(sum((v-m)**2 for v in xs)/len(xs))

def corr(a,b):
    if len(a)<10: return None
    ma=sum(a)/len(a); mb=sum(b)/len(b)
    num=sum((x-ma)*(y-mb) for x,y in zip(a,b))
    da=math.sqrt(sum((x-ma)**2 for x in a)); db=math.sqrt(sum((y-mb)**2 for y in b))
    return num/(da*db) if da*db>1e-9 else None

def best_lag_corr(a, b, ts, max_lag_s=0.8):
    """Return (best_corr, lag_ms) — lag>0 means b is shifted later (b lags a)."""
    if len(a)<30 or len(a)!=len(b): return (None, 0)
    dt=(ts[-1]-ts[0])/max(1,(len(ts)-1))
    max_lag=int(max_lag_s/max(dt,1e-3))
    ma=sum(a)/len(a); mb=sum(b)/len(b)
    best_c = 0.0; best_lag = 0
    # Lag positive: b lags a (compare a[i] with b[i+lag])
    # Lag negative: a lags b (compare a[i-lag] with b[i], where -lag > 0)
    for lag in range(-max_lag, max_lag+1):
        if lag >= 0:
            # a[0..N-lag-1] vs b[lag..N-1]
            a_sl = a[:len(a)-lag] if lag > 0 else a
            b_sl = b[lag:] if lag > 0 else b
        else:
            # a[-lag..N-1] vs b[0..N+lag-1]
            k = -lag
            a_sl = a[k:]
            b_sl = b[:len(b)-k]
        if len(a_sl) < 10 or len(a_sl) != len(b_sl): continue
        ax = [x-ma for x in a_sl]; bx = [y-mb for y in b_sl]
        num=sum(x*y for x,y in zip(ax, bx))
        da=math.sqrt(sum(x*x for x in ax)); db=math.sqrt(sum(y*y for y in bx))
        if da*db < 1e-9: continue
        c = num/(da*db)
        if abs(c) > abs(best_c):
            best_c = c; best_lag = lag
    return (best_c, round(best_lag*dt*1000, 0))

def main():
    rows = list(csv.DictReader(open(CSV_PATH)))
    print(f"Loaded {len(rows)} rows from {CSV_PATH}")
    print()

    poses = ("CENTRAL","ALL_UP","ALL_DOWN","ESTESO")
    phase = "pitch_osc_10_04"

    print(f"{'='*110}")
    print(f"CORRELATIONS (phase={phase}) — with best-lag search (lag positive: 2nd signal lags 1st)")
    print(f"{'='*110}")
    print(f"{'pose':10s}  {'corr(cmd_pit, fk_wc_z)':>24s}  {'corr(cmd_pit, imu_pit)':>23s}  {'corr(imu_pit, fk_wc_z)':>24s}")
    print(f"{'':10s}  {'value / lag':>24s}  {'value / lag':>23s}  {'value / lag':>24s}")
    print("-"*110)

    for pose in poses:
        xs = [r for r in rows if r.get("pose_label")==pose and r.get("phase")==phase]
        if len(xs) < 30:
            print(f"{pose:10s}  insufficient samples ({len(xs)})")
            continue
        ts = [float(r["t"]) for r in xs]
        cp = [float(r["cmd_pit"]) for r in xs]
        # fk_wc_z might be missing in some rows
        has_fk = [r for r in xs if r.get("fk_wc_z") not in (None,"","None")]
        has_imu= [r for r in xs if r.get("imu_pit") not in (None,"","None")]

        # cmd vs fk_wc_z
        ts_fk = [float(r["t"]) for r in has_fk]
        cp_fk = [float(r["cmd_pit"]) for r in has_fk]
        wc_z  = [float(r["fk_wc_z"]) for r in has_fk]
        c1, lag1 = best_lag_corr(cp_fk, wc_z, ts_fk)

        # cmd vs imu_pit
        ts_im = [float(r["t"]) for r in has_imu]
        cp_im = [float(r["cmd_pit"]) for r in has_imu]
        im_p  = [float(r["imu_pit"]) for r in has_imu]
        c2, lag2 = best_lag_corr(cp_im, im_p, ts_im)

        # imu_pit vs fk_wc_z
        has_both = [r for r in xs if r.get("imu_pit") not in (None,"","None") and r.get("fk_wc_z") not in (None,"","None")]
        ts_both = [float(r["t"]) for r in has_both]
        im_b   = [float(r["imu_pit"]) for r in has_both]
        wc_b   = [float(r["fk_wc_z"]) for r in has_both]
        c3, lag3 = best_lag_corr(im_b, wc_b, ts_both)

        def fmt(c,l):
            if c is None: return "   —   "
            sign = "+" if c > 0 else "−"
            return f"{c:+6.3f} / lag {int(l):+4d}ms"

        print(f"{pose:10s}  {fmt(c1,lag1):>24s}  {fmt(c2,lag2):>23s}  {fmt(c3,lag3):>24s}")

    # Joint-level correlations (cmd_pit ↔ servo_S and servo_G)
    print()
    print(f"{'='*110}")
    print(f"JOINT-LEVEL correlations cmd_pit ↔ servo_S/G  (for kinematic direction)")
    print(f"{'='*110}")
    print(f"{'pose':10s}  {'corr(cmd_pit, servo_S)':>24s}  {'corr(cmd_pit, servo_G)':>24s}")
    print("-"*110)
    for pose in poses:
        xs = [r for r in rows if r.get("pose_label")==pose and r.get("phase")==phase]
        if len(xs) < 30: continue
        ts = [float(r["t"]) for r in xs]
        cp = [float(r["cmd_pit"]) for r in xs]
        ss = [float(r["servo_S"]) for r in xs]
        sg = [float(r["servo_G"]) for r in xs]
        c_s, lag_s = best_lag_corr(cp, ss, ts)
        c_g, lag_g = best_lag_corr(cp, sg, ts)
        def fmt(c,l): return f"{c:+6.3f} / lag {int(l):+4d}ms" if c is not None else "   —   "
        print(f"{pose:10s}  {fmt(c_s,lag_s):>24s}  {fmt(c_g,lag_g):>24s}")

    # Full cycle interpretation
    print()
    print("="*110)
    print("SIGN INTERPRETATION (per pose)")
    print("="*110)
    print("cmd_pit convention: R.from_euler('Y', deg, degrees=True) then decode to ZYX Euler[1]")
    print("  cmd_pit POSITIVE  = simulated head quat rotated by positive Y-axis angle")
    print()
    print("FK wc_z convention: positive Z = UP in robot base frame")
    print()
    for pose in poses:
        xs = [r for r in rows if r.get("pose_label")==pose and r.get("phase")==phase and r.get("fk_wc_z") not in (None,"","None")]
        if len(xs) < 30: continue
        ts = [float(r["t"]) for r in xs]
        cp = [float(r["cmd_pit"]) for r in xs]
        wc = [float(r["fk_wc_z"]) for r in xs]
        c, lag = best_lag_corr(cp, wc, ts)
        if c is None: continue
        if abs(c) < 0.3:
            verdict = "NEGLIGIBLE (CENTRAL dead zone)"
        elif c > 0:
            verdict = "POSITIVE corr → cmd_pit UP means wc_z UP (SAME direction)"
        else:
            verdict = "NEGATIVE corr → cmd_pit UP means wc_z DOWN (OPPOSITE direction)"
        print(f"  {pose:10s}  corr = {c:+.3f} at lag {int(lag):+d}ms  →  {verdict}")

if __name__ == "__main__":
    main()
