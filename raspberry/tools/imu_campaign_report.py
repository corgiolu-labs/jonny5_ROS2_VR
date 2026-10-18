#!/usr/bin/env python3
"""
imu_campaign_report.py - analysis of an ASSIST measurement campaign (head_assist_gimbal campaign logger).

Question: is a closed loop on the wrist IMU worth it? Per sample (30 Hz) the log has the orientation the head asked
for (target_R), the pose the gimbal commanded (cmd_v, virtual deg), the servo positions the firmware is driving
(tel_phys, physical deg: after its rate limits / head pipeline) and the BNO085 quaternion (imu).

    R_cmd = FK(cmd_v)        R_fw = FK(tel_phys)        R_imu ~ W . R_true . M
W (IMU world -> robot base) and M (IMU mounting on the wrist) are fitted on the settled samples (alternating
Procrustes, started from the robot's calibration); the residual E = (W R_fw M)^T R_imu is the orientation the
kinematics does NOT explain: servo dead band / backlash, sag under load, offset errors, IMU noise and yaw drift.
Reported per test (label): RMS / max error per axis (tool frame: roll x, pitch y, yaw z), error vs arm extension,
backlash (error with + / - velocity), lag command -> IMU (cross-correlation), yaw drift (deg/min), and the
command -> firmware difference (rate limits). Output: an HTML report with plots.

Usage: python imu_campaign_report.py campaign.jsonl [--settings j5_settings.json] [--out report.html]
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SETTINGS = HERE.parent / "config_runtime" / "robot" / "j5_settings.json"
# robot calibration (2026-10, /api/imu-frame-calib): R_imu = R_wb . R_ee . R_mount
WB0 = [0.6551074397161296, 0.0, 0.0, -0.755535732066047]
MOUNT0 = [0.9978145291471898, 0.0553353613787577, 0.002751663137535594, 0.03600821508811257]
H0, L1, L2 = 0.112, 0.060, 0.173           # POE geometry (head_assist_gimbal._arm_geom)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def fk(v_deg):
    q = np.radians(np.asarray(v_deg, float) - 90.0)
    return rz(q[0]) @ ry(q[1] + q[2]) @ rz(q[3]) @ ry(q[4]) @ rx(q[5])


def qmat(q):
    w, x, y, z = (float(v) for v in q[:4])
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def proj_so3(m):
    u, _, vt = np.linalg.svd(m)
    d = np.sign(np.linalg.det(u @ vt))
    return u @ np.diag([1, 1, d]) @ vt


def rotvec_deg(r):
    """Small-angle error vector (x, y, z) in degrees of a rotation matrix (log map)."""
    c = max(-1.0, min(1.0, (np.trace(r) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(a))
    return np.degrees(v * a)


def ypr_deg(r):
    return np.degrees([math.atan2(r[1, 0], r[0, 0]), math.asin(max(-1, min(1, -r[2, 0]))), math.atan2(r[2, 1], r[2, 2])])


def ang_deg(a, b):
    return float(np.linalg.norm(rotvec_deg(a.T @ b)))


def fig_png(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    return base64.b64encode(buf.getvalue()).decode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="+")
    ap.add_argument("--settings", default=str(SETTINGS))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    cfg = json.loads(Path(args.settings).read_text())
    off = np.asarray(cfg["offsets"], float)
    dirs = np.asarray(cfg["dirs"], float)

    recs = []
    for path in args.log:
        for line in Path(path).read_text().splitlines():
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("imu") and r.get("tel_phys") and r.get("cmd_v"):
                recs.append(r)
    if len(recs) < 50:
        raise SystemExit(f"too few samples with IMU: {len(recs)}")
    t = np.array([r["t"] for r in recs])
    t -= t[0]
    labels = [r.get("label") or "-" for r in recs]
    v_cmd = np.array([r["cmd_v"] for r in recs])
    v_fw = (np.array([r["tel_phys"] for r in recs]) - off) / dirs + 90.0
    R_cmd = [fk(v) for v in v_cmd]
    R_fw = [fk(v) for v in v_fw]
    R_imu = [qmat(r["imu"]) for r in recs]
    R_tgt = [None if r.get("target_R") is None else np.asarray(r["target_R"]).reshape(3, 3) for r in recs]
    n = len(recs)

    # settled samples: the firmware pose barely moving (the IMU and the servos agree in time)
    rate = np.zeros(n)
    for i in range(1, n):
        dt = max(1e-3, t[i] - t[i - 1])
        rate[i] = ang_deg(R_fw[i - 1], R_fw[i]) / dt
    settled = rate < 3.0
    # fit W, M on the settled samples
    W, M = qmat(WB0), qmat(MOUNT0)
    idx = np.nonzero(settled)[0]
    if len(idx) < 30:
        idx = np.arange(n)
    for _ in range(50):
        M = proj_so3(sum((W @ R_fw[i]).T @ R_imu[i] for i in idx))
        W = proj_so3(sum(R_imu[i] @ M.T @ R_fw[i].T for i in idx))
    E = np.array([rotvec_deg((W @ R_fw[i] @ M).T @ R_imu[i]) for i in range(n)])   # tool frame-ish, deg
    E_cmd_fw = np.array([rotvec_deg(R_cmd[i].T @ R_fw[i]) for i in range(n)])
    E_tgt = np.array([rotvec_deg(R_tgt[i].T @ R_cmd[i]) if R_tgt[i] is not None else [np.nan] * 3 for i in range(n)])
    # gravity-referenced (drift-free) camera elevation / roll: FK (base assumed level) vs IMU
    ypr_fw = np.array([ypr_deg(R) for R in R_fw])
    ypr_imu = np.array([ypr_deg(W.T @ R_imu[i] @ M.T) for i in range(n)])
    # arm extension: wrist-centre reach
    q2 = np.radians(v_fw[:, 1] - 90)
    q3 = np.radians(v_fw[:, 2] - 90)
    reach = L1 * np.sin(q2) + L2 * np.sin(q2 + q3)

    def stats(sel):
        e = E[sel]
        return {"n": int(sel.sum()), "rms": np.sqrt(np.nanmean(e ** 2, axis=0)), "max": np.nanmax(np.abs(e), axis=0),
                "mean": np.nanmean(e, axis=0)}

    rows = []
    for lab in dict.fromkeys(labels):
        sel = np.array([lb == lab for lb in labels])
        s = stats(sel)
        cf = np.sqrt(np.nanmean(E_cmd_fw[sel] ** 2, axis=0))
        rows.append((lab, s, cf))

    # lag command -> IMU on the camera pitch / yaw (base frame), whole log
    def lag_ms(a, b):
        a = a - a.mean()
        b = b - b.mean()
        best, bl = -1e9, 0
        fs = 30.0
        for k in range(0, 16):          # 0..500 ms
            if k >= len(a) - 10:
                break
            c = float(np.dot(a[:len(a) - k], b[k:]) / (np.linalg.norm(a[:len(a) - k]) * np.linalg.norm(b[k:]) + 1e-9))
            if c > best:
                best, bl = c, k
        return bl * 1000.0 / fs, best

    ypr_cmd = np.array([ypr_deg(R) for R in R_cmd])
    lag_p = lag_ms(ypr_cmd[:, 1], ypr_imu[:, 1])
    lag_y = lag_ms(np.unwrap(np.radians(ypr_cmd[:, 0])), np.unwrap(np.radians(ypr_imu[:, 0])))
    lag_fw = lag_ms(ypr_cmd[:, 1], ypr_fw[:, 1])
    # backlash: settled-after-motion error split by the sign of the commanded velocity (pitch, yaw)
    def backlash(axis_ypr, err_axis):
        vel = np.gradient(ypr_cmd[:, axis_ypr], t)
        pos = (vel > 2.0)
        neg = (vel < -2.0)
        if pos.sum() < 10 or neg.sum() < 10:
            return None
        return float(np.mean(E[pos, err_axis]) - np.mean(E[neg, err_axis]))
    bl_pitch = backlash(1, 1)
    bl_yaw = backlash(0, 2)
    # yaw drift: settled yaw error vs time
    drift = None
    if len(idx) > 30:
        p = np.polyfit(t[idx], E[idx, 2], 1)
        drift = p[0] * 60.0
    # sag: pitch error vs reach (settled)
    sag = None
    if len(idx) > 30 and np.ptp(reach[idx]) > 0.03:
        p = np.polyfit(reach[idx], E[idx, 1], 1)
        sag = p[0] * 0.1     # deg per 10 cm of reach

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figs = []
    fig, ax = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    for k, nm in enumerate(("roll (x)", "pitch (y)", "yaw (z)")):
        ax[k].plot(t, E[:, k], lw=0.8, label="IMU - modello (firmware)")
        ax[k].plot(t, E_cmd_fw[:, k], lw=0.8, alpha=0.7, label="firmware - comando")
        ax[k].set_ylabel(nm + " deg")
        ax[k].grid(alpha=0.3)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[2].set_xlabel("s")
    # label bands
    last, t0 = None, 0.0
    for i, lb in enumerate(labels + [None]):
        if lb != last:
            if last is not None:
                for a in ax:
                    a.axvspan(t0, t[i - 1], alpha=0.06, color="C%d" % (hash(last) % 8))
                ax[0].text(t0, ax[0].get_ylim()[1], last, fontsize=7, va="top")
            last, t0 = lb, (t[i] if i < n else 0)
    fig.suptitle("Errore di orientamento nel tempo")
    figs.append(fig_png(fig))
    plt.close(fig)
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    ax[0].plot(t, ypr_cmd[:, 1], lw=0.8, label="comando")
    ax[0].plot(t, ypr_fw[:, 1], lw=0.8, label="firmware")
    ax[0].plot(t, ypr_imu[:, 1], lw=0.8, label="IMU")
    ax[0].set_title("beccheggio camera (deg, + = giù)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[1].scatter(reach[idx] * 1000, E[idx, 1], s=4)
    ax[1].set_xlabel("sbraccio centro polso (mm)")
    ax[1].set_ylabel("errore beccheggio (deg)")
    ax[1].set_title("errore vs posa del braccio")
    ax[1].grid(alpha=0.3)
    figs.append(fig_png(fig))
    plt.close(fig)

    def f3(a):
        return " / ".join(f"{v:.2f}" for v in a)

    trs = "".join(f"<tr><td>{lab}</td><td>{s['n']}</td><td>{f3(s['rms'])}</td><td>{f3(s['max'])}</td>"
                  f"<td>{f3(s['mean'])}</td><td>{f3(cf)}</td></tr>" for lab, s, cf in rows)
    allst = stats(np.ones(n, bool))
    mount_ypr = ypr_deg(M)
    html = f"""<!doctype html><html><head><meta charset="utf-8"><title>Campagna IMU JONNY5</title>
<style>body{{font-family:system-ui,sans-serif;max-width:1100px;margin:24px auto;padding:0 16px;color:#222}}
table{{border-collapse:collapse;font-size:14px}}td,th{{border:1px solid #ccc;padding:4px 8px;text-align:right}}
td:first-child,th:first-child{{text-align:left}}.k{{font-size:15px;line-height:1.6}}img{{max-width:100%}}</style></head><body>
<h1>Campagna di misura IMU · ASSIST</h1>
<p class="k">{n} campioni ({t[-1]:.0f} s), {int(settled.sum())} fermi. Errore = orientamento misurato dall'IMU meno
quello previsto dalla cinematica sulle posizioni servo del firmware (dopo l'allineamento IMU stimato dai dati).
Assi nel riferimento del polso: <b>rollio x / beccheggio y / imbardata z</b>, gradi.</p>
<h2>Sintesi</h2><ul class="k">
<li>Errore complessivo RMS: <b>{f3(allst['rms'])}</b> deg, massimo {f3(allst['max'])}</li>
<li>Ritardo comando → IMU: beccheggio <b>{lag_p[0]:.0f} ms</b> (corr {lag_p[1]:.2f}), imbardata {lag_y[0]:.0f} ms
 (corr {lag_y[1]:.2f}); comando → firmware {lag_fw[0]:.0f} ms</li>
<li>Gioco all'inversione (differenza errore + / -): beccheggio {('%.2f deg' % bl_pitch) if bl_pitch is not None else 'n.d.'},
 imbardata {('%.2f deg' % bl_yaw) if bl_yaw is not None else 'n.d.'}</li>
<li>Cedimento: {('%.2f deg di beccheggio ogni 10 cm di sbraccio' % sag) if sag is not None else 'n.d. (poca variazione di posa)'}</li>
<li>Deriva imbardata IMU: {('%.2f deg/min' % drift) if drift is not None else 'n.d.'}</li>
<li>Montaggio IMU stimato (ypr): {f3(mount_ypr)} deg (calibrazione salvata: 4.1 / 0.1 / 6.4 circa)</li></ul>
<h2>Per prova</h2><table><tr><th>prova</th><th>N</th><th>RMS x/y/z</th><th>max x/y/z</th><th>medio x/y/z</th>
<th>firmware-comando RMS</th></tr>{trs}</table>
<h2>Grafici</h2>{''.join(f'<img src="data:image/png;base64,{g}">' for g in figs)}
</body></html>"""
    out = Path(args.out or (Path(args.log[0]).with_suffix(".html")))
    out.write_text(html, encoding="utf-8")
    print("report:", out)
    print("RMS x/y/z", f3(allst["rms"]), "max", f3(allst["max"]), "lag pitch", lag_p, "backlash", bl_pitch, bl_yaw,
          "sag/10cm", sag, "drift", drift)
    for lab, s, cf in rows:
        print(f"  {lab:20s} n={s['n']:5d} rms {f3(s['rms'])} max {f3(s['max'])} fw-cmd {f3(cf)}")


if __name__ == "__main__":
    main()
