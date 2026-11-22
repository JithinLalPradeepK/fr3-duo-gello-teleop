#!/usr/bin/env python3
"""OFFLINE test of isaac/texturing.py (multi-view best-photo texturing): python3 test/texturing_test.py
T1 faces hidden by the arms in view 0 take view 1 (arms lifted there), T2 visible faces keep view 0 (equal score ->
first view wins = the live photo), T3 UVs point at the right pixels (a colour-coded synthetic photo), T4 a wrist view
(closer, depth image) wins where it sees the surface, T5 faces outside every view -> -1."""
import os
import sys

import numpy as np

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'isaac'))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
import texturing  # noqa: E402
from duo_geometry import DuoGeometry, READY  # noqa: E402
from twin_recon_core import RobotFilter  # noqa: E402

R = []


def check(name, ok, info=''):
    R.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}")


def look_at(eye, target):
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, np.cross(z, x), z], 1)
    T[:3, 3] = eye
    return T


def main():
    # table grid z = 0, x 0.3..1.2, y -0.5..0.5 (2 cm triangles)
    xs, ys = np.meshgrid(np.arange(0.3, 1.2, 0.02), np.arange(-0.5, 0.5, 0.02), indexing='ij')
    V = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], 1)
    nx, ny = xs.shape
    F = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            a, b, c, d = i * ny + j, (i + 1) * ny + j, (i + 1) * ny + j + 1, i * ny + j + 1
            F += [(a, b, c), (a, c, d)]
    F = np.array(F)
    K, wh = [700.0, 700.0, 640.0, 360.0], (1280, 720)
    T = look_at(np.array([0.135, 0.0, 0.60]), [0.75, 0.0, 0.0])
    g = DuoGeometry()
    rf = RobotFilter(g)
    from twin_state import load_home
    home = load_home()                                     # the real HOME: grippers hang in front of the head camera
    caps_low = rf.capsules({'left': np.array(home['left']), 'right': np.array(home['right'])}, filter_margin=False)
    caps_up = rf.capsules({'left': np.array(READY), 'right': np.array(READY)}, filter_margin=False)
    v0 = {'T': T, 'K': K, 'wh': wh, 'robot': texturing.robot_depth(T, K, wh, caps_low), 'depth': None}
    v1 = {'T': T, 'K': K, 'wh': wh, 'robot': texturing.robot_depth(T, K, wh, caps_up), 'depth': None}
    best, uv = texturing.choose(V, F, [v0, v1])
    ctr = V[F].mean(1)
    u, v, z = texturing.project(T, K, ctr)
    ui, vi = np.clip(u.astype(int), 0, wh[0] - 1), np.clip(v.astype(int), 0, wh[1] - 1)
    hid0 = v0['robot'][vi, ui] < z - 0.01
    hid1 = v1['robot'][vi, ui] < z - 0.01
    sel = hid0 & ~hid1 & (best >= 0)
    check('T1 faces hidden by the arms in view 0 take view 1', sel.sum() > 20 and np.all(best[sel] == 1),
          f'{int(sel.sum())} faces hidden in view 0 only, {int((best[sel] == 1).sum())} use view 1')
    vis = ~hid0 & (best >= 0)
    check('T2 visible faces keep view 0 (live photo first)', vis.sum() > 100 and np.all(best[vis] == 0),
          f'{int(vis.sum())} faces')
    pu = uv.mean(1)[:, 0] * wh[0]
    pv = (1 - uv.mean(1)[:, 1]) * wh[1]
    m = best >= 0
    err = np.hypot(pu[m] - u[m], pv[m] - v[m])
    check('T3 UVs point at the face in the photo (< 1 px from the projected centre)', np.percentile(err, 99) < 1.0,
          f'p99 {np.percentile(err, 99):.3f} px')
    # T4: a close wrist view above x 0.7..0.9 with its depth image
    Tw = look_at(np.array([0.8, 0.0, 0.35]), [0.8, 0.0, 0.0] + np.array([0.001, 0, 0]))
    Kw, whw = [434.0, 434.0, 424.0, 240.0], (848, 480)
    du, dv = np.meshgrid(np.arange(424), np.arange(240))
    ray = np.stack([(du * 2 - Kw[2]) / Kw[0], (dv * 2 - Kw[3]) / Kw[1], np.ones_like(du, float)], -1) @ Tw[:3, :3].T
    t = -Tw[2, 3] / ray[..., 2]
    depth = (t * 1.0).astype(np.float32)                      # optical z of the table point (ray z component = 1)
    vw = {'T': Tw, 'K': Kw, 'wh': whw, 'depth': (depth, (424, 240)), 'robot': None}
    best2, _ = texturing.choose(V, F, [v1, vw])
    under = (np.abs(ctr[:, 0] - 0.8) < 0.08) & (np.abs(ctr[:, 1]) < 0.08)
    check('T4 the close wrist view wins under itself', np.all(best2[under] == 1), f'{int((best2[under] == 1).sum())}/'
          f'{int(under.sum())} faces')
    far = {'T': look_at(np.array([3.0, 3.0, 0.5]), [3.0, 4.0, 0.5]), 'K': K, 'wh': wh, 'depth': None, 'robot': None}
    best3, _ = texturing.choose(V, F, [far])
    check('T5 faces no view sees -> -1 (vertex colours)', np.all(best3 == -1))
    print(f'\n{sum(R)}/{len(R)} passed')
    return 0 if all(R) else 1


if __name__ == '__main__':
    sys.exit(main())
