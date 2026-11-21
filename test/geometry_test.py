#!/usr/bin/env python3
"""Unit test of scripts/duo_geometry.py (numpy only, no ROS, no robot)."""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from duo_geometry import MIRROR, DuoGeometry, seg_seg_dist  # noqa: E402

fails = []


def check(what, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {what}' + (f' - {detail}' if detail else ''), flush=True)
    if not ok:
        fails.append(what)


rng = np.random.default_rng(7)
g = DuoGeometry()
print('== G-1 segment-segment distance vs brute force')
n = 300
p0, p1, q0, q1 = (rng.uniform(-1, 1, (n, 3)) for _ in range(4))
s = np.linspace(0, 1, 400)[:, None]
err = 0.0
for k in range(n):
    A = p0[k] + s * (p1[k] - p0[k])
    B = q0[k] + s * (q1[k] - q0[k])
    brute = np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)).min()
    err = max(err, seg_seg_dist(p0[k:k + 1], p1[k:k + 1], q0[k:k + 1], q1[k:k + 1])[0] - brute)
check('analytic <= brute force + 0 and within 2 mm', -1e-9 <= err <= 2e-3 + 1e-9 or err < 1e-9, f'max excess {err:.2e}')
d_vec = seg_seg_dist(p0, p1, q0, q1)
d_one = np.array([seg_seg_dist(p0[k:k + 1], p1[k:k + 1], q0[k:k + 1], q1[k:k + 1])[0] for k in range(n)])
check('vectorised == one by one', np.abs(d_vec - d_one).max() < 1e-12)
deg = seg_seg_dist(np.zeros((1, 3)), np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([[1.0, 0, 0]]))[0]
check('degenerate (point-point) segments', abs(deg - 1.0) < 1e-12, f'{deg}')

print('== G-2 mirror symmetry of the Duo')
Mi = np.diag([1, -1, 1.0])
worst = 0.0
for _ in range(200):
    q = rng.uniform(-1.5, 1.5, 7)
    pl = g.flange_position('left', q)
    pr = g.flange_position('right', q * MIRROR)
    worst = max(worst, np.linalg.norm(pr - Mi @ pl))
check('flange of (q) left == mirror of flange of (-q1,q2,-q3,q4,-q5,q6,-q7) right', worst < 1e-9, f'{worst:.1e} m')
worst = 0.0
for _ in range(100):
    q = rng.uniform(-1.5, 1.5, 7)
    fl, fr = g.link_frames('left', q), g.link_frames('right', q * MIRROR)
    worst = max(worst, max(np.linalg.norm(fr[i][:3, 3] - Mi @ fl[i][:3, 3]) for i in range(9)))
check('ALL link origins (link0..flange) mirrored exactly', worst < 1e-9, f'{worst:.1e} m')
# note: clearance itself is NOT mirror symmetric - the link shapes are identical hardware, not mirrored hardware

print('== G-3 known poses')
home_r = np.array([-1.8343, -0.6202, 1.5144, -2.2085, 1.1681, 1.2407, -1.3288])     # IP .50 (saved)
home_l = np.array([1.9313, -0.6306, -1.7178, -2.1633, -1.1038, 1.2573, 1.5174])      # IP .51 (saved)
avg_r = (home_r + home_l * MIRROR) / 2
avg_l = avg_r * MIRROR
for name, ql, qr in (('saved home poses (used many times on the real robots)', home_l, home_r),
                     ('mirrored average home poses', avg_l, avg_r)):
    c, info = g.clearance(ql, qr)
    check(f'{name}: capsules do not touch (real collision impossible)', info[2] > 0.005,
          f'tightest {info[0]}/{info[1]} {info[2] * 100:.1f} cm (margin {info[3] * 100:.0f} cm -> clearance {c * 100:.1f} cm)')
check('height difference of the SAVED homes ~5 cm (observed on the robots: "one slightly above the other")',
      0.03 < abs(g.flange_position('left', home_l)[2] - g.flange_position('right', home_r)[2]) < 0.08,
      f'{(g.flange_position("left", home_l)[2] - g.flange_position("right", home_r)[2]) * 100:+.1f} cm')
check('mirrored average homes: equal flange height and mirrored y',
      abs(g.flange_position('left', avg_l)[2] - g.flange_position('right', avg_r)[2]) < 1e-9
      and abs(g.flange_position('left', avg_l)[1] + g.flange_position('right', avg_r)[1]) < 1e-9)
mock = np.array([0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398])
c, info = g.clearance(mock, mock)
check('sim mock pose (both arms same q) is clear', c > 0, f'clearance {c * 100:.1f} cm')
best = None
for _ in range(4000):
    q = rng.uniform([-1, -1, -1, -2.8, -1, 0.7, -1], [1, 1, 1, -1.0, 1, 3.0, 1])
    y = g.flange_position('left', q)[1]
    if best is None or abs(y) < best[0]:
        best = (abs(y), q.copy())
q = best[1]
c, info = g.clearance(q, q * MIRROR)
check('left flange on the symmetry plane -> collision detected', c < 0 and info[2] < 0,
      f'|y| {best[0]:.3f} m, clearance {c * 100:.1f} cm, {info[0]}/{info[1]}')

print('== G-4 path check')
hit = g.path_collision(home_l, home_l, home_r, home_r)
check('static saved home pair (already inside the margin): accepted, not worse than the start', hit is None)
hit = g.path_collision(avg_l, avg_l, avg_r, avg_r)
check('static mirrored home pair: accepted', hit is None)
hit = g.path_collision(avg_l, q, avg_r, q * MIRROR, n=10)
check('path from home to the colliding pose is flagged', hit is not None and hit[0] < 0, str(hit and hit[2:]))
hit = g.path_collision(avg_l, mock, avg_r, mock, n=10)
check('path from home to the sim mock pose is accepted', hit is None, str(hit))
# HOME: target pose inside the 4 cm margin but not touching
hit = g.path_collision(mock, avg_l, mock, avg_r, n=10)
check('START-style check to the (inside-margin) home pose is refused: end pose worse than the start', hit is not None)
hit = g.path_collision(mock, avg_l, mock, avg_r, n=10, end_ok=True)
check('HOME-style check (end_ok) from the sim pose to the mirrored home: accepted', hit is None, str(hit and (hit[0], hit[1][:2], hit[2:])))
hit = g.path_collision(mock, home_l, mock, home_r, n=10, end_ok=True)
check('HOME-style check to the SAVED home pair: accepted', hit is None, str(hit and (hit[0], hit[1][:2], hit[2:])))
hit = g.path_collision(mock, q, mock, q * MIRROR, n=10, end_ok=True)
check('HOME-style check to a pose where the arms touch: refused', hit is not None and hit[1][2] <= 0.005, str(hit and hit[1]))

print('== G-6 HOME planner')
from duo_geometry import READY, plan_home  # noqa: E402
ready = np.array(READY)
ph = plan_home(g, ready, ready, avg_l, avg_r)
check('from the ready pose both arms can go home at once', ph is not None and len(ph) == 1, str(None if ph is None else len(ph)))
# a start pose of the left arm that sits on the symmetry plane next to the right arm's home: direct path crosses the arms
found = None
for seed in range(400):
    r2 = np.random.default_rng(seed)
    ql = r2.uniform([-1, -1, -1, -2.6, -1, 0.9, -1], [1, 1, 1, -1.4, 1, 2.8, 1])
    qr = ql * MIRROR * 0 + r2.uniform([-1, -1, -1, -2.6, -1, 0.9, -1], [1, 1, 1, -1.4, 1, 2.8, 1])
    c, info = g.clearance(ql, qr)
    if c < 0.02 or info[2] < 0.03:
        continue
    if g.path_collision(ql, avg_l, qr, avg_r, n=10, end_ok=True) is not None:
        found = (ql, qr)
        break
check('test setup: found a start pair whose DIRECT simultaneous path to home collides', found is not None)
if found:
    ql, qr = found
    ph = plan_home(g, ql, qr, avg_l, avg_r)
    check('planner finds a collision-free alternative (sequential or via ready) instead of refusing', ph is not None,
          'none' if ph is None else f'{len(ph)} phase(s): ' + ', '.join('both' if (a is not None and b is not None) else ('left' if a is not None else 'right') for a, b in ph))
    if ph is not None:
        cur_l, cur_r, bad = ql.copy(), qr.copy(), []
        for a, b in ph:                                   # re-verify every phase independently
            nl = cur_l if a is None else a
            nr = cur_r if b is None else b
            if g.path_collision(cur_l, nl, cur_r, nr, n=20, end_ok=True) is not None:
                bad.append((a is not None, b is not None))
            cur_l, cur_r = nl, nr
        check('every phase re-verified on a finer grid (n=20)', not bad, str(bad))
        check('the plan ends exactly at both home poses', np.allclose(cur_l, avg_l) and np.allclose(cur_r, avg_r))
ph = plan_home(g, q, q * MIRROR, q, q * MIRROR)            # home pose itself = touching arms
check('a home target where the arms touch cannot be planned (None)', plan_home(g, ready, ready, q, q * MIRROR) is None)
check('forced strategies return the fixed schemes', [len(plan_home(g, ready, ready, avg_l, avg_r, strategy=s_)) for s_ in
      ('simultaneous', 'left-first', 'right-first', 'via-ready')] == [1, 2, 2, 2])

print('== G-5 speed')
t0 = time.perf_counter()
for _ in range(200):
    g.clearance(avg_l, avg_r)
dt = (time.perf_counter() - t0) / 200
check('clearance() fast enough for 200 Hz (< 1.5 ms)', dt < 1.5e-3, f'{dt * 1e3:.2f} ms')
t0 = time.perf_counter()
g.path_collision(avg_l, q, avg_r, q * MIRROR, n=10)
dt = time.perf_counter() - t0
check('path_collision n=10 (121 pairs of poses) < 1 s', dt < 1.0, f'{dt * 1e3:.0f} ms')

print(f'===== {"PASS" if not fails else "FAIL: " + str(fails)}')
sys.exit(1 if fails else 0)
