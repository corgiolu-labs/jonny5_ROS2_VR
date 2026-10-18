#!/usr/bin/env python3
"""imu_calib_fit.py - IMU mounting / base tilt / joint offset fit from an IMU campaign log (gravity direction of the
still samples vs FK). Usage: python imu_calib_fit.py campaign.jsonl
Reports, for nested models (mount only, + base tilt, + wrist offsets, all), the train / held-out gravity error and
each parameter with its standard deviation: only parameters that are well determined AND lower the held-out error
are worth applying (2026-10-18: mount + tilt, 9.4 -> 2.8 deg; joint offsets not separable from the mounting)."""
import json, math, sys
import numpy as np
from scipy.optimize import least_squares
sys.path.insert(0, ".")
import imu_campaign_report as r
off = np.array([88, 85, 85, 90, 90, 98.]); dirs = np.array([1, -1, 1, 1, -1, 1.])
recs = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
recs = [x for x in recs if x.get("imu")]
t = np.array([x["t"] for x in recs]); lab = np.array([x["label"] for x in recs])
vfw = (np.array([x["tel_phys"] for x in recs]) - off) / dirs + 90
Ri = [r.qmat(x["imu"]) for x in recs]
n = len(recs)
still = []
for i in range(n):
    j = np.searchsorted(t, t[i] - 0.6)
    if t[i] - t[j] < 0.5 or i - j < 10: continue
    if np.max(np.ptp(vfw[j:i+1], axis=0)) < 1.0 and r.ang_deg(Ri[j], Ri[i]) < 1.0:
        still.append(i)
still = np.array(still)
# gravity (up) in the IMU sensor frame: R_imu^T z_world
g_meas = np.array([Ri[i].T @ np.array([0, 0, 1.0]) for i in still])
V = vfw[still]
def rotv(x):
    a = np.linalg.norm(x)
    if a < 1e-12: return np.eye(3)
    k = x / a; K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3) + math.sin(a)*K + (1-math.cos(a))*K@K
# params: mount rotvec (3), base tilt (2: about x,y), joint offset corrections deg: [S+G as one on G], Y, P, R
names = ["mount_x","mount_y","mount_z","tilt_x","tilt_y","dSG","dY","dP","dR"]
def model(p, V):
    Mt = rotv(p[0:3]); T = rotv(np.array([p[3], p[4], 0.0]))
    out = []
    for v in V:
        vv = v.copy(); vv[2] += p[5]; vv[3] += p[6]; vv[4] += p[7]; vv[5] += p[8]
        R = r.fk(vv)
        # sensor frame up = (R_tool->world . M)^T . T^T z   (R_imu = W R M)
        out.append((R @ Mt).T @ (T.T @ np.array([0, 0, 1.0])))
    return np.array(out)
def res(p, V, g):
    return (model(p, V) - g).reshape(-1)
p0 = np.zeros(9); m0 = r.qmat(r.MOUNT0)
from scipy.spatial.transform import Rotation as Rot
p0[0:3] = Rot.from_matrix(m0).as_rotvec()
def fit(mask_params, V, g):
    free = np.array(mask_params, bool)
    def rr(q):
        p = p0.copy(); p[free] = q
        return res(p, V, g)
    s = least_squares(rr, p0[free])
    p = p0.copy(); p[free] = s.x
    J = s.jac; cov = np.linalg.pinv(J.T @ J) * (np.sum(s.fun**2) / max(1, len(s.fun) - free.sum()))
    std = np.zeros(9); std[free] = np.sqrt(np.diag(cov))
    return p, std, s
def ang_err(p, V, g):
    m = model(p, V)
    return np.degrees(np.arccos(np.clip(np.sum(m * g, 1), -1, 1)))
e0 = ang_err(p0, V, g_meas)
print(f"still samples {len(still)}; saved mount, no corrections: gravity error RMS {np.sqrt((e0**2).mean()):.2f} p95 {np.percentile(e0,95):.2f} deg")
rng = np.random.default_rng(1); perm = rng.permutation(len(still)); tr, te = perm[: int(0.7*len(perm))], perm[int(0.7*len(perm)):]
for nm, mask in (("mount only", [1,1,1,0,0,0,0,0,0]), ("mount+tilt", [1,1,1,1,1,0,0,0,0]),
                 ("mount+tilt+wrist P,R", [1,1,1,1,1,0,0,1,1]), ("all", [1,1,1,1,1,1,1,1,1])):
    p, std, s = fit(mask, V[tr], g_meas[tr])
    etr, ete = ang_err(p, V[tr], g_meas[tr]), ang_err(p, V[te], g_meas[te])
    print(f"{nm:22s} train RMS {np.sqrt((etr**2).mean()):.2f}  TEST RMS {np.sqrt((ete**2).mean()):.2f} p95 {np.percentile(ete,95):.2f}")
    print("   ", ", ".join(f"{names[k]} {np.degrees(p[k]) if k<5 else p[k]:+.2f}±{np.degrees(std[k]) if k<5 else std[k]:.2f}" for k in range(9) if mask[k]))
print("pose ranges (virtual):", [f"{a:.0f}-{b:.0f}" for a, b in zip(V.min(0), V.max(0))])
