"""kin.js (the dashboard's kinematics) against an independent numpy reference: the same POE math as the old
Pi IK solver (raspberry/controller/web_services/ik_solver.py), a numerical Jacobian and the conversions.

    python -m pytest minimal/dashboard -q          (needs node and numpy; skipped without them)
"""
import json
import math
import pathlib
import shutil
import subprocess

import pytest

np = pytest.importorskip("numpy")
HERE = pathlib.Path(__file__).resolve().parent
KIN = json.loads((HERE / "kinematics.json").read_text(encoding="utf-8"))
S, M = np.array(KIN["S"], float), np.array(KIN["M"], float)

POSES = [[90] * 6, [120, 60, 130, 90, 45, 160], [45, 100, 80, 30, 120, 10], [90, 90, 90, 90, 90.5, 90],
         [100, 140, 40, 150, 70, 95], [77.3, 85.1, 101.9, 64.2, 133.7, 12.5]]


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], float)


def exp6(Si, th):          # ik_solver.matrix_exp6 of [S] * th
    w, v = Si[:3] * th, Si[3:] * th
    t = np.linalg.norm(w)
    T = np.eye(4)
    if t < 1e-12:
        T[:3, 3] = v
        return T
    W = skew(w) / t
    T[:3, :3] = np.eye(3) + math.sin(t) * W + (1 - math.cos(t)) * W @ W
    T[:3, 3] = (np.eye(3) * t + (1 - math.cos(t)) * W + (t - math.sin(t)) * W @ W) @ (v / t)
    return T


def fk(q):
    T = np.eye(4)
    for i in range(6):
        T = T @ exp6(S[i], q[i])
    return T @ M


def twist(T, Td):          # spatial twist (w, v) that moves T to Td in unit time, small displacement
    D = Td @ np.linalg.inv(T)
    return np.array([D[2, 1], D[0, 2], D[1, 0], D[0, 3], D[1, 3], D[2, 3]])


def run_node(script):
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    out = subprocess.run([node, "--input-type=module", "-e", script], capture_output=True, text=True,
                         cwd=HERE, check=True)
    return json.loads(out.stdout)


def js_eval(expr_per_pose):
    poses = json.dumps(POSES)
    kin = json.dumps(KIN)
    return run_node(f"""
import * as k from "./kin.js";
const K = {kin}, poses = {poses};
console.log(JSON.stringify(poses.map(v => {{ const q = k.qFromVirtual(v); return {expr_per_pose}; }})));
""")


def test_forward_kinematics_matches_numpy():
    res = js_eval("k.fk(K.S, K.M, q).T")
    for v, T in zip(POSES, res):
        ref = fk([math.radians(a - 90) for a in v])
        assert np.allclose(np.array(T), ref, atol=1e-12), v


def test_home_is_M_and_pose_in_mm_rpy():
    res = js_eval("k.poseFromVirtualDeg(K.S, K.M, v)")
    home = res[0]
    assert (home["x_mm"], home["y_mm"], home["z_mm"]) == pytest.approx((57.0, 0.0, 381.225), abs=1e-9)
    for v, p in zip(POSES, res):
        T = fk([math.radians(a - 90) for a in v])
        R = T[:3, :3]
        assert p["x_mm"] == pytest.approx(T[0, 3] * 1000, abs=1e-9)
        # rebuild R from ZYX angles
        y, pi, r = (math.radians(p[k]) for k in ("yaw_deg", "pitch_deg", "roll_deg"))
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        Ry = np.array([[math.cos(pi), 0, math.sin(pi)], [0, 1, 0], [-math.sin(pi), 0, math.cos(pi)]])
        Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        assert np.allclose(Rz @ Ry @ Rx, R, atol=1e-9), v


def test_space_jacobian_matches_finite_differences():
    res = js_eval("k.spaceJacobian(K.S, q)")
    h = 1e-6
    for v, J in zip(POSES, res):
        q = np.array([math.radians(a - 90) for a in v])
        T = fk(q)
        num = np.column_stack([twist(T, fk(q + h * np.eye(6)[i])) / h for i in range(6)])
        assert np.allclose(np.array(J), num, atol=1e-5), v


def test_manipulability_and_chain():
    res = js_eval("({m: k.manipulability(K.S, q), c: k.chain(K.S, K.M, q), "
                  "d: k.det(k.spaceJacobian(K.S, q))})")
    for v, r in zip(POSES, res):
        q = [math.radians(a - 90) for a in v]
        J = np.column_stack([twist(fk(q), fk(np.array(q) + 1e-6 * np.eye(6)[i])) / 1e-6 for i in range(6)])
        assert r["m"] == pytest.approx(abs(np.linalg.det(J)), rel=1e-3, abs=1e-9)
        assert r["c"]["ee"] == pytest.approx(list(fk(q)[:3, 3]), abs=1e-12)
        assert len(r["c"]["joints"]) == 6
    # HOME: wrist yaw (Z) and roll (X) axes are orthogonal, base and wrist yaw are both vertical
    home = res[0]["c"]["joints"]
    assert home[0]["axis"] == pytest.approx([0, 0, 1]) and home[3]["axis"] == pytest.approx([0, 0, 1])
    assert home[1]["p"][2] == pytest.approx(0.1343)            # shoulder height


def test_virtual_physical_conversions():
    r = run_node("""
import * as k from "./kin.js";
console.log(JSON.stringify([k.physicalToVirtual(100, 88, 1), k.physicalToVirtual(100, 88, -1),
  k.virtualToPhysical(78, 88, -1), k.virtualToPhysical(k.physicalToVirtual(37, 104, -1), 104, -1)]));
""")
    assert r == [102, 78, 100, 37]


def test_draw_points_lie_on_their_axes_and_move_with_the_chain():
    for (wx, wy, wz, vx, vy, vz), p in zip(KIN["S"], KIN["draw_points"]):
        w, v = np.array([wx, wy, wz]), np.array([vx, vy, vz])
        assert np.allclose(np.cross(w, p), -v, atol=1e-9)        # v = -w x p  <=>  p is on the axis
    res = js_eval("k.chain(K.S, K.M, q, K.draw_points).joints.map(j => j.p)")
    for v, pts in zip(POSES, res):
        q = [math.radians(a - 90) for a in v]
        F = np.eye(4)
        for i in range(6):
            assert np.allclose(pts[i], (F @ np.append(KIN["draw_points"][i], 1))[:3], atol=1e-12)
            F = F @ exp6(S[i], q[i])
