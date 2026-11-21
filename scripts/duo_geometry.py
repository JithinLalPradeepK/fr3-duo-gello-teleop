#!/usr/bin/env python3
"""Dual-arm (FR3 Duo) geometry: forward kinematics of both arms in the shared mount frame and capsule-based
clearance between them. numpy only (no ROS), so it can run inside the gates and in unit tests.

  from duo_geometry import DuoGeometry
  g = DuoGeometry()
  clearance, info = g.clearance(q_left, q_right)       # > 0 safe, < 0 closer than the configured margin
  hit = g.path_collision(qL0, qL1, qR0, qR1)           # None or info of the first colliding pair of path fractions

Configuration: config/duo_geometry.yaml (generated from franka_description) and config/duo_guard.yaml (margins).
Mirror law of the Duo (verified numerically): the two arms are mirror images about the mount's XZ plane, so a
pose q of one arm is mirrored by the other arm at (-q1, q2, -q3, q4, -q5, q6, -q7). Robot namespaces map to mount
slots via `slots` in config/duo_guard.yaml (ours are swapped: namespace left sits in the mount's right slot).
"""
import math
import os

import numpy as np
import yaml

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MIRROR = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose(xyzrpy):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*xyzrpy[3:])
    t[:3, 3] = xyzrpy[:3]
    return t


def seg_seg_dist(p0, p1, q0, q1):
    """Distance between segments p0-p1 and q0-q1; all arrays (n, 3) -> (n,). Ericson, Real-Time Collision Detection."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e = np.einsum('ij,ij->i', d1, d1), np.einsum('ij,ij->i', d2, d2)
    f, c = np.einsum('ij,ij->i', d2, r), np.einsum('ij,ij->i', d1, r)
    b = np.einsum('ij,ij->i', d1, d2)
    den = a * e - b * b
    eps = 1e-12
    s = np.where(den > eps, np.clip((b * f - c * e) / np.where(den > eps, den, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / np.where(e > eps, e, 1.0)
    # t outside [0,1] -> clamp t and recompute s
    t_lo, t_hi = t < 0.0, t > 1.0
    t = np.clip(t, 0.0, 1.0)
    s2 = np.clip((b * t - c) / np.where(a > eps, a, 1.0), 0.0, 1.0)
    s = np.where(t_lo | t_hi, s2, s)
    s = np.where(a > eps, s, 0.0)
    t = np.where(e > eps, t, 0.0)
    return np.linalg.norm((p0 + s[:, None] * d1) - (q0 + t[:, None] * d2), axis=1)


class DuoGeometry:
    def __init__(self, geometry_file=None, guard_file=None):
        geo = yaml.safe_load(open(geometry_file or os.path.join(DEMO, 'config', 'duo_geometry.yaml')))
        cfg = yaml.safe_load(open(guard_file or os.path.join(DEMO, 'config', 'duo_guard.yaml')))
        self.joint_T = [pose(geo['joints'][f'joint{i}']) for i in range(1, 9)]       # joint1..joint8 (flange)
        slots = cfg.get('slots', {'left': 'left', 'right': 'right'})       # robot namespace -> mount slot
        self.base_T = {a: pose(geo['bases'][slots[a]]) for a in ('left', 'right')}
        scale = float(cfg.get('radius_scale', 1.0))
        g = cfg['gripper']
        # capsule table (identical for both arms): (frame index 0..8 where 8 = flange, p0, p1, radius, name)
        caps = []
        for i in range(8):
            for k, (p0, p1, r) in enumerate(geo['capsules'][f'link{i}']):
                caps.append((i, np.array(p0, float), np.array(p1, float), r * scale, f'link{i}'))
        caps.append((8, np.array(g['p0'], float), np.array(g['p1'], float), float(g['radius']) * scale, 'gripper'))
        self.caps = caps
        self.names = [c[4] for c in caps]
        base_links = set(cfg.get('base_links', []))
        m_arm, m_grip = float(cfg['margins']['arm']), float(cfg['margins']['gripper'])
        n = len(caps)
        ii, jj = np.meshgrid(range(n), range(n), indexing='ij')
        keep, margin, radsum = [], [], []
        for i, j in zip(ii.ravel(), jj.ravel()):
            a, b = self.names[i], self.names[j]
            if a in base_links and b in base_links:
                continue
            keep.append((i, j))
            margin.append(m_grip if (a == 'gripper' and b == 'gripper') else m_arm)
            radsum.append(caps[i][3] + caps[j][3])
        self.pairs = np.array(keep)
        self.margin = np.array(margin)
        self.radsum = np.array(radsum)
        self.P0 = np.array([c[1] for c in caps])
        self.P1 = np.array([c[2] for c in caps])
        self.frame = np.array([c[0] for c in caps])
        self.margins = (m_arm, m_grip)

    # ------------------------------------------------------------------ kinematics
    def link_frames(self, arm, q):
        """4x4 poses (mount frame) of link1..link7 and the flange (index 0..7 -> link1..flange); link0 = base."""
        t = self.base_T[arm].copy()
        out = [t.copy()]                                   # frame 0 = link0 (arm base)
        for i in range(7):
            c, s = math.cos(q[i]), math.sin(q[i])
            rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
            t = t @ self.joint_T[i] @ rz
            out.append(t.copy())
        out.append(t @ self.joint_T[7])                    # frame 8 = flange
        return out

    def flange_position(self, arm, q):
        return self.link_frames(arm, q)[8][:3, 3]

    def world_capsules(self, arm, q):
        fr = self.link_frames(arm, q)
        R = np.array([f[:3, :3] for f in fr])[self.frame]
        t = np.array([f[:3, 3] for f in fr])[self.frame]
        a = np.einsum('nij,nj->ni', R, self.P0) + t
        b = np.einsum('nij,nj->ni', R, self.P1) + t
        return a, b

    # ------------------------------------------------------------------ clearance
    def clearance_between(self, caps_l, caps_r):
        """(min clearance, (nameL, nameR, surface distance, margin)) for two world-capsule sets."""
        al, bl = caps_l
        ar, br = caps_r
        i, j = self.pairs[:, 0], self.pairs[:, 1]
        d = seg_seg_dist(al[i], bl[i], ar[j], br[j]) - self.radsum
        c = d - self.margin
        k = int(np.argmin(c))
        return float(c[k]), (self.names[i[k]], self.names[j[k]], float(d[k]), float(self.margin[k]))

    def clearance(self, q_left, q_right):
        return self.clearance_between(self.world_capsules('left', q_left), self.world_capsules('right', q_right))

    def path_collision(self, ql0, ql1, qr0, qr1, n=10, tol=0.005, end_ok=False, touch=0.005):
        """Both arms move on straight joint-space lines (the controller's start ramp) at UNKNOWN relative speed:
        check every pair of path fractions on an n x n grid. A pose that is already inside the margin at the start
        (e.g. the home pose, wrists a few cm apart) is allowed as long as the path never gets worse than the start
        by more than tol. end_ok=True (HOME: the target is a deliberately chosen pose) additionally allows an end pose
        inside the margin - but its capsules must not touch (surface distance > touch) and no pose on the way may be
        worse than the worse of start and end. Returns None if the path is acceptable, else
        (clearance, info, fraction_left, fraction_right) of the worst pair."""
        ql0, ql1, qr0, qr1 = (np.asarray(x, float) for x in (ql0, ql1, qr0, qr1))
        fs = np.linspace(0.0, 1.0, n + 1)
        capl = [self.world_capsules('left', ql0 + f * (ql1 - ql0)) for f in fs]
        capr = [self.world_capsules('right', qr0 + f * (qr1 - qr0)) for f in fs]
        c0, _ = self.clearance_between(capl[0], capr[0])
        c1, info1 = self.clearance_between(capl[-1], capr[-1])
        if end_ok:
            if info1[2] <= touch:                      # the target pose itself would touch the other arm
                return (c1, info1, 1.0, 1.0)
            floor = min(0.0, c0, c1) - tol
        else:
            floor = min(0.0, c0) - tol
        worst = None
        for a, fl in enumerate(fs):
            for b, fr_ in enumerate(fs):
                c, info = self.clearance_between(capl[a], capr[b])
                if c < floor and (worst is None or c < worst[0]):
                    worst = (c, info, float(fl), float(fr_))
        return worst


def describe(info):
    a, b, d, m = info
    return f'left {a} / right {b}: {d * 100:.1f} cm apart, minimum {m * 100:.0f} cm'
