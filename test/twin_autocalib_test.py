#!/usr/bin/env python3
"""Offline test of scripts/twin_autocalib.py (no ROS, no camera): synthetic ZED view of the twin's robots at the saved
HOME poses (noise + table + clutter, only the camera-facing half of the surface), true camera pose known, start from a
wrong guess -> the registration must recover the camera pose; and a scene WITHOUT the robots must be rejected.
  python3 test/twin_autocalib_test.py
"""
import os
import sys

import numpy as np

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import DuoGeometry  # noqa: E402
from twin_autocalib import mat, matrix_rpy, model_points, register  # noqa: E402
from twin_state import load_home  # noqa: E402

R = []


def check(name, ok, detail=''):
    R.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}{"  - " + detail if detail else ""}')


def links_at(q):
    g = DuoGeometry()
    out = {'mount_link': np.eye(4)}
    for arm in ('left', 'right'):
        fr = g.link_frames(arm, q[arm])
        for i in range(8):
            out[f'{arm}_fr3_link{i}'] = fr[i]
    return out


def synth_view(model, T_true, rng):
    """Points the camera at T_true (world <- cloud frame, x forward like the ZED) would see, in the cloud frame."""
    cam = T_true[:3, 3]
    p = model[rng.random(len(model)) < 0.5] if len(model) else np.zeros((0, 3))
    if len(p):          # crude visibility: keep points on the camera-facing side of the robot's centre
        centre = model.mean(0)
        p = p[((p - centre) @ (cam - centre)) > -0.05]
        p = p + rng.normal(0, 0.003, p.shape)
    xs, ys = np.meshgrid(np.arange(-0.2, 1.0, 0.02), np.arange(-0.7, 0.7, 0.02))
    table = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.02)], 1)
    clutter = rng.uniform([0.3, -0.6, 0.0], [0.9, 0.6, 0.15], (600, 3))
    w = np.concatenate([p, table, clutter])
    Ti = np.linalg.inv(T_true)
    return w @ Ti[:3, :3].T + Ti[:3, 3]


def main():
    rng = np.random.default_rng(1)
    model = model_points(links_at(load_home()))
    check('model points from FR3 + mount collision meshes', 5000 < len(model) < 200000, f'{len(model)} points')
    xyz_true, rpy_true = (1.25, 0.08, 0.80), (0.0, 0.42, 3.10)
    T_true = mat(xyz_true, rpy_true)
    scene = synth_view(model, T_true, rng)
    for label, dxyz, drpy in (('6 cm / 0.10 rad off', (0.06, -0.04, 0.03), (0.0, 0.05, 0.10)),
                              ('15 cm / 0.25 rad off', (-0.12, 0.08, -0.05), (0.03, -0.06, -0.25))):
        T0 = mat(np.add(xyz_true, dxyz), np.add(rpy_true, drpy))
        T, rms, n = register(scene, model, T0)
        dp = np.linalg.norm(T[:3, 3] - T_true[:3, 3])
        dr = np.arccos(np.clip((np.trace(T[:3, :3].T @ T_true[:3, :3]) - 1) / 2, -1, 1))
        check(f'start {label}: camera pose recovered (< 1 cm, < 0.01 rad), tight fit',
              dp < 0.01 and dr < 0.01 and rms < 0.008 and n > 1500,
              f'error {dp * 1000:.1f} mm / {dr:.4f} rad, RMS {rms * 1000:.1f} mm, {n} matched')
    r = matrix_rpy(mat((0, 0, 0), rpy_true)[:3, :3])
    check('matrix_rpy inverts rpy_matrix', np.allclose(r, rpy_true, atol=1e-9), str(np.round(r, 4)))
    # no robots in view: only table + clutter -> must not reach the acceptance thresholds
    empty = synth_view(np.zeros((0, 3)), T_true, rng)
    T, rms, n = register(empty, model, mat(xyz_true, rpy_true))
    check('scene without robots is not accepted (few matches or loose fit)', n < 1500 or rms > 0.012,
          f'{n} matched, RMS {rms * 1000:.1f} mm')
    print(f'===== {"PASS" if all(R) else "FAIL"} ({sum(R)}/{len(R)})')
    sys.exit(0 if all(R) else 1)


if __name__ == '__main__':
    main()
