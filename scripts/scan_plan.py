#!/usr/bin/env python3
"""SCAN plan for the digital twin's 3D reconstruction (numpy only; no ROS; plans, never moves anything).

  python3 scripts/scan_plan.py            print the plan for the saved HOME poses and check it

The robots look around the workspace with their wrist D405s from a set of poses ABOVE the home pose:
  * every pose: elbow within ELBOW_BOX of its home position and not lower, joints within MAX_FROM_HOME of home,
    MAX_STEP per move (IK with a null-space pull to the home configuration)
  * every pose: flange >= home flange + MIN_LIFT (never lower than home -> never into the trays / cubes under the
    grippers), finger tips >= home finger tips, flange within +-XY_BOX of home in x and y, joints >= LIMIT_MARGIN
    inside the usable FR3 range, j4 >= J4_MIN (self-collision zone, as the gate), both arms mirrored (Duo mirror law)
  * every move (straight joint-space ramp = what the gate's target mode executes) is sampled: the same height /
    box rules on the way (except the first lift out of home and the final descent into it, which may not go below
    home), and duo_geometry.path_collision for the two arms moving at once.
plan = [(left_q, right_q), ...] from home through the views back to home; the supervisor runs it like HOME phases.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from duo_geometry import MIRROR, DuoGeometry, rpy_matrix  # noqa: E402

# usable FR3 range = gello_gate.Q_MIN / Q_MAX (URDF limits and where the position-dependent speed limit reaches zero)
Q_MIN = np.array([-2.7426, -1.7841, -2.9007, -3.0368, -2.8065, 0.5521, -3.016])
Q_MAX = np.array([2.7426, 1.7841, 2.9007, -0.1571, 2.8065, 4.5094, 3.016])
ELBOW_BOX, ELBOW_DROP = 0.15, 0.04               # elbow within 15 cm of its home position, at most 4 cm lower
MAX_STEP = np.array([0.9] * 6 + [2.0])           # joint change per move (j7 = wrist spin: moves no link)
MAX_FROM_HOME = np.array([1.2] * 6 + [2.6])
LIMIT_MARGIN, J4_MIN = 0.2, -2.70
MIN_LIFT, XY_BOX = 0.04, 0.16
TIP = np.array([0.0, 0.0, 0.20])         # finger tips in the flange frame (Robotiq 2F-85 + coupling, estimate)
# views (left arm; right arm = mirror): (name, flange offset from home [m] in the mount frame, rotation of the
# flange about world axes [rad] (roll x, pitch y, yaw z) applied to the home orientation)
# The wrists are only ~2 cm apart at home: every view first moves both flanges OUTWARD (left arm: -y), so the wrists
# can turn without meeting.
VIEWS = [
    ('lift + apart',        (0.00, -0.08, 0.10), (0.00, 0.00, 0.00)),
    ('turn camera -',       (0.00, -0.08, 0.10), (0.00, 0.00, -1.20)),
    ('turn camera +',       (0.00, -0.08, 0.10), (0.00, 0.00, 1.20)),
    ('apart',               (0.00, -0.08, 0.10), (0.00, 0.00, 0.00)),
    ('forward, look ahead', (0.10, -0.08, 0.12), (0.00, -0.35, 0.00)),
    ('forward, turned',     (0.10, -0.08, 0.12), (0.00, -0.30, 1.20)),
    ('forward, turned -',   (0.10, -0.08, 0.12), (0.00, -0.30, -1.20)),
    ('outward, look out',   (0.03, -0.12, 0.12), (0.35, 0.00, 0.00)),
    ('back, look down',     (-0.05, -0.08, 0.12), (0.00, 0.30, 0.00)),
    # 2026-11-22 (user request: more angles -> precise wrist-camera mounts + more coverage): HIGH views with real
    # tilts about different axes (hand-eye needs rotations about >= 2 non-parallel axes) and wrist spins
    ('high, apart',         (0.00, -0.10, 0.17), (0.00, 0.00, 0.00)),
    ('high, tilt out',      (0.00, -0.10, 0.17), (0.45, 0.00, 0.00)),
    ('high, tilt in',       (0.00, -0.10, 0.17), (-0.30, 0.00, 0.00)),
    ('high, look ahead',    (0.06, -0.10, 0.17), (0.00, -0.45, 0.00)),
    ('high, ahead turned +', (0.06, -0.10, 0.17), (0.00, -0.40, 0.60)),
    ('high, ahead turned -', (0.06, -0.10, 0.17), (0.00, -0.40, -0.60)),
    ('high, diagonal',      (0.04, -0.10, 0.16), (0.30, -0.30, 0.00)),
    ('high, look back',     (-0.04, -0.10, 0.17), (0.00, 0.35, 0.00)),
    ('high, spin +',        (0.00, -0.10, 0.15), (0.00, 0.00, 1.20)),
    ('high, spin -',        (0.00, -0.10, 0.15), (0.00, 0.00, -1.20)),
    ('forward, close',      (0.12, -0.08, 0.10), (0.00, -0.30, 0.00)),
    ('apart',               (0.00, -0.08, 0.10), (0.00, 0.00, 0.00)),
]


def fk(g, arm, q):
    return g.link_frames(arm, q)[8]


def ik(g, arm, q_seed, T_goal, iters=300, q_rest=None):
    """Damped least squares on the 6D pose error (numeric Jacobian) with a null-space pull towards q_rest (the home
    configuration: the elbow stays where it is). Returns q or None."""
    q = np.array(q_seed, float)
    q_rest = np.array(q_seed if q_rest is None else q_rest, float)
    lo, hi = Q_MIN + LIMIT_MARGIN, Q_MAX - LIMIT_MARGIN
    lo[3] = max(lo[3], J4_MIN)
    for _ in range(iters):
        T = fk(g, arm, q)
        ep = T_goal[:3, 3] - T[:3, 3]
        Re = T_goal[:3, :3] @ T[:3, :3].T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2)))
        if ang < 1e-9:
            er = np.zeros(3)
        else:
            er = ang / (2 * math.sin(ang)) * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        e = np.concatenate([ep, 0.3 * er])
        if np.linalg.norm(ep) < 2e-4 and ang < 2e-3:
            return q
        J = np.zeros((6, 7))
        for j in range(7):
            dq = np.zeros(7)
            dq[j] = 1e-5
            T2 = fk(g, arm, q + dq)
            dR = T2[:3, :3] @ T[:3, :3].T
            J[:3, j] = (T2[:3, 3] - T[:3, 3]) / 1e-5
            J[3:, j] = 0.3 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2e-5
        lam = 0.05
        Jp = J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(6))
        far = np.linalg.norm(ep) > 5e-3 or ang > 2e-2          # null-space pull only while still far: with the
        step = Jp @ e                                            # damped inverse it is not exactly task-free
        if far:
            step = step + (np.eye(7) - Jp @ J) @ (0.3 * (q_rest - q))
        q = np.clip(q + np.clip(step, -0.2, 0.2), lo, hi)
    return None


def pose_ok(g, arm, q, home_T, allow_low=False, home_q=None):
    """(ok, why) for one left/right pose against the height / box / limit rules."""
    q = np.asarray(q, float)
    if np.any(q < Q_MIN + LIMIT_MARGIN - 1e-6) or np.any(q > Q_MAX - LIMIT_MARGIN + 1e-6):
        return False, 'joint limit margin'
    if q[3] < J4_MIN - 1e-6:
        return False, 'j4 self-collision zone'
    T = fk(g, arm, q)
    tip = T[:3, :3] @ TIP + T[:3, 3]
    home_tip = home_T[:3, :3] @ TIP + home_T[:3, 3]
    lift = 0.0 if allow_low else MIN_LIFT
    if T[2, 3] < home_T[2, 3] + lift - 0.005:
        return False, f'flange {T[2, 3] - home_T[2, 3]:+.3f} m vs home'
    if tip[2] < home_tip[2] - 0.005:
        return False, f'finger tips {tip[2] - home_tip[2]:+.3f} m vs home'
    if np.any(np.abs(T[:2, 3] - home_T[:2, 3]) > XY_BOX + 1e-6):
        return False, 'outside the xy box'
    if home_q is not None:
        e, he = g.link_frames(arm, q)[4][:3, 3], g.link_frames(arm, home_q)[4][:3, 3]
        if np.linalg.norm(e - he) > ELBOW_BOX + 1e-6 or e[2] < he[2] - ELBOW_DROP:
            return False, f'elbow moves {np.linalg.norm(e - he) * 100:.0f} cm / {(e[2] - he[2]) * 100:+.0f} cm'
        if np.any(np.abs(q - np.asarray(home_q)) > MAX_FROM_HOME + 1e-6):
            return False, 'too far from the home configuration'
    return True, ''


def path_ok(g, ql0, ql1, qr0, qr1, home, n=20, allow_low=False):
    hT = {a: fk(g, a, home[a]) for a in ('left', 'right')}
    for f in np.linspace(0, 1, n + 1):
        for arm, a0, a1 in (('left', ql0, ql1), ('right', qr0, qr1)):
            q = np.asarray(a0) + f * (np.asarray(a1) - np.asarray(a0))
            ok, why = pose_ok(g, arm, q, hT[arm], allow_low=allow_low, home_q=home[arm])
            if not ok:
                return False, f'{arm} at {f:.2f}: {why}'
    if np.any(np.abs(np.subtract(ql1, ql0)) > MAX_STEP + 1e-6) or np.any(np.abs(np.subtract(qr1, qr0)) > MAX_STEP + 1e-6):
        return False, 'joint change too big for one move'
    hit = g.path_collision(ql0, ql1, qr0, qr1, n=10, end_ok=True)
    if hit is not None:
        return False, f'arms too close: {hit[1][0]}/{hit[1][1]} {hit[1][2] * 100:.1f} cm'
    return True, ''


def make_plan(g, home, views=VIEWS):
    """[(name, left_q, right_q)] home -> views -> home, every pose and move checked. Views that fail are skipped."""
    hl = np.asarray(home['left'], float)
    hT = fk(g, 'left', hl)
    out, prev, skipped = [], (hl, MIRROR * hl), []
    for name, dxyz, drpy in views:
        T = np.eye(4)
        T[:3, :3] = rpy_matrix(*drpy) @ hT[:3, :3]
        T[:3, 3] = hT[:3, 3] + np.array(dxyz)
        ql = ik(g, 'left', prev[0], T, q_rest=hl)
        if ql is None:
            skipped.append((name, 'no IK'))
            continue
        qr = MIRROR * ql
        ok, why = pose_ok(g, 'left', ql, hT, home_q=hl)
        if ok:
            ok, why = pose_ok(g, 'right', qr, fk(g, 'right', home['right']), home_q=home['right'])
        if ok:
            ok, why = path_ok(g, prev[0], ql, prev[1], qr, home, allow_low=not out)
        if not ok:
            skipped.append((name, why))
            continue
        out.append((name, ql, qr))
        prev = (ql, qr)
    ok, why = path_ok(g, prev[0], hl, prev[1], MIRROR * hl, home, allow_low=True)
    if not ok:
        return None, skipped + [('back home', why)]
    out.append(('home', hl, MIRROR * hl))
    return out, skipped


def main():
    from twin_state import load_home
    g, home = DuoGeometry(), load_home()
    if np.max(np.abs(MIRROR * np.asarray(home['left']) - np.asarray(home['right']))) > 0.02:
        print('NOTE: the saved home poses are not mirror images - the right arm uses the mirror of the left home')
        home = dict(home, right=list(MIRROR * np.asarray(home['left'])))
    plan, skipped = make_plan(g, home)
    for name, why in skipped:
        print(f'  skipped view "{name}": {why}')
    if plan is None:
        sys.exit('NO PLAN')
    hT = fk(g, 'left', home['left'])
    for name, ql, _qr in plan:
        T = fk(g, 'left', ql)
        print(f'  {name:22s} flange {np.round(T[:3, 3] - hT[:3, 3], 3)} m vs home   q {np.round(ql, 2)}')
    print(f'{len(plan)} phases')


if __name__ == '__main__':
    main()
