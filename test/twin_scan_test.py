#!/usr/bin/env python3
"""Offline test of the 3D scan (scripts/twin_scan.py + scripts/scan_plan.py; no ROS, no robot, no camera).
Synthetic workspace (table + cubes + back wall + floor, textured), the REAL scan plan from the saved home poses,
depth/colour images rendered for both wrist D405s with a known (hidden) camera mount -> fuse() must
  S1 produce a scan plan (>= 5 views)          S2 recover both D405 mounts (< 1 cm, < 1.5 deg)
  S3 fuse a model that lies on the scene (median 3 mm)   S4 remove the robot arms   S5 colours come through
  python3 test/twin_scan_test.py
"""
import math
import os
import sys
import time

import numpy as np

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
import scan_plan  # noqa: E402
import twin_scan as S  # noqa: E402
from duo_geometry import DuoGeometry, rpy_matrix  # noqa: E402
from twin_state import load_home  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

R = []


def check(name, ok, detail=''):
    R.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}{"  - " + detail if detail else ""}', flush=True)


def scene(step=0.004):
    pts, col = [], []

    def plane(x, y, z, c):
        pts.append(np.stack([x.ravel(), y.ravel(), z.ravel()], 1))
        col.append(np.tile(c, (x.size, 1)) if np.ndim(c) == 1 else c.reshape(-1, 3))
    xs, ys = np.meshgrid(np.arange(0.15, 1.05, step), np.arange(-0.75, 0.75, step))
    checker = ((np.floor(xs / 0.05) + np.floor(ys / 0.05)) % 2)[..., None] * np.array([60, 40, 20]) + np.array([150, 150, 160])
    plane(xs, ys, np.full_like(xs, 0.05), checker)                                   # table, z = 5 cm
    for cx, cy, c in ((0.55, 0.15, (220, 60, 40)), (0.55, -0.15, (40, 160, 60)), (0.7, 0.0, (240, 200, 20))):
        a, b = np.meshgrid(np.arange(-0.03, 0.03, step), np.arange(-0.03, 0.03, step))
        plane(cx + a, cy + b, np.full_like(a, 0.11), np.array(c))                    # cube tops
        plane(cx + a, np.full_like(a, cy - 0.03), 0.05 + (b + 0.03), np.array(c))
        plane(cx + a, np.full_like(a, cy + 0.03), 0.05 + (b + 0.03), np.array(c))
        plane(np.full_like(a, cx - 0.03), cy + a, 0.05 + (b + 0.03), np.array(c))
    ys, zs = np.meshgrid(np.arange(-0.75, 0.75, step), np.arange(-0.5, 0.9, step))
    plane(np.full_like(ys, 1.05), ys, zs, np.array([90, 120, 140]))                  # back wall
    xs, ys = np.meshgrid(np.arange(-0.4, 0.15, step), np.arange(-0.75, 0.75, step))
    plane(xs, ys, np.full_like(xs, -0.5), np.array([70, 80, 70]))                    # floor in front of the table edge
    return np.concatenate(pts), np.concatenate(col).astype(np.uint8)


def render(P, C, T_world_cam, K, w=424, h=240):
    fx, fy, cx, cy = K
    Ti = np.linalg.inv(T_world_cam)
    pc = P @ Ti[:3, :3].T + Ti[:3, 3]
    ok = pc[:, 2] > 0.05
    pc, cc = pc[ok], C[ok]
    u = np.round(pc[:, 0] / pc[:, 2] * fx + cx).astype(int)
    v = np.round(pc[:, 1] / pc[:, 2] * fy + cy).astype(int)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z, cc = u[ok], v[ok], pc[ok, 2], cc[ok]
    order = np.argsort(-z)                         # far first, near overwrites
    depth = np.zeros((h, w), np.uint16)
    color = np.zeros((h, w, 3), np.uint8)
    depth[v[order], u[order]] = np.clip(z[order] * 1000, 0, 65535).astype(np.uint16)
    color[v[order], u[order]] = cc[order]
    return depth, color


def main():
    g, home = DuoGeometry(), load_home()
    plan, skipped = scan_plan.make_plan(g, home)
    check('S1 scan plan from the saved home poses (>= 5 views)', plan is not None and len(plan) >= 5,
          f'{len(plan) if plan else 0} moves, skipped {[n for n, _ in skipped]}')
    P, C = scene()
    K = (215.0, 215.0, 212.0, 120.0)
    # hidden truth: camera on the gripper, looking 55 deg off the flange axis (towards flange -x), with some roll
    Xtrue = {}
    for cam, roll in (('d405_1', 0.4), ('d405_2', -0.6)):
        X = np.eye(4)
        X[:3, :3] = rpy_matrix(0.0, math.radians(-55), 0.0) @ rpy_matrix(0, 0, roll)
        X[:3, 3] = (-0.05, 0.01, 0.06)
        Xtrue[cam] = X
    frames = []
    for name, ql, qr in [('home', np.array(home['left']), np.array(home['right']))] + plan:
        f = {'t': 0.0, 'q': {'left': list(ql), 'right': list(qr)}}
        for cam, arm in S.CAMS.items():
            F = g.link_frames(arm, ql if arm == 'left' else qr)[8]
            d, c = render(P, C, F @ Xtrue[cam], K)
            f[cam] = {'depth': d, 'color': c, 'K': list(K)}
        frames.append(f)
    t0 = time.time()
    out = S.fuse({'frames': frames, 'zed': None}, {'cameras': {}}, log=lambda s: print('     ' + s))
    print(f'     fusion took {time.time() - t0:.0f} s')
    for cam in S.CAMS:
        X = out['X'].get(cam)
        if X is None:
            check(f'S2 {cam} mount recovered', False, 'no result')
            continue
        dp = np.linalg.norm(X[:3, 3] - Xtrue[cam][:3, 3])
        dr = math.degrees(math.acos(np.clip((np.trace(X[:3, :3].T @ Xtrue[cam][:3, :3]) - 1) / 2, -1, 1)))
        check(f'S2 {cam} mount recovered (< 1 cm, < 1.5 deg)', dp < 0.01 and dr < 1.5,
              f'{dp * 1000:.1f} mm, {dr:.2f} deg, consistency {out["score"][cam]:.3f}')
    d, _ = cKDTree(P).query(out['xyz'])
    check('S3 fused model lies on the scene (median < 3 mm, 95 % < 10 mm)',
          np.median(d) < 0.003 and np.percentile(d, 95) < 0.01,
          f'median {np.median(d) * 1000:.1f} mm, p95 {np.percentile(d, 95) * 1000:.1f} mm, {len(d)} voxels')
    keep = S.robot_mask(g, out['xyz'].astype(float), home['left'], home['right'], pad=0.0)
    check('S4 no fused points inside the robot arms (at home)', keep.mean() > 0.999, f'{(~keep).sum()} inside')
    r = (out['rgb'] >> 16) & 255
    check('S5 colours carried into the model (red cube visible)', np.sum((r > 200) & ((out['rgb'] >> 8) & 255 < 90)) > 50)
    print(f'===== {"PASS" if all(R) else "FAIL"} ({sum(R)}/{len(R)})')
    sys.exit(0 if all(R) else 1)


if __name__ == '__main__':
    main()
