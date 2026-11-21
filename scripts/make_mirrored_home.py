#!/usr/bin/env python3
"""Make the two home poses mirror images of each other (the FR3 Duo is mirror-symmetric about its mount plane).

  python3 scripts/make_mirrored_home.py                       # show what would change (nothing is written)
  python3 scripts/make_mirrored_home.py --reference right     # left becomes the exact mirror of the saved right pose
  python3 scripts/make_mirrored_home.py --write               # write ~/fr3_tools/fr3_duo_home/home_poses.txt (backup first)

Mirror law: q_left = (-q1, q2, -q3, q4, -q5, q6, -q7) * q_right, verified against the Duo kinematics
(scripts/duo_geometry.py). --reference average (default) takes the mean of the right pose and the mirrored left pose,
so each arm moves the least; right / left keeps that arm exactly and mirrors it onto the other.
No robot is contacted; the new pose is used the next time HOME is pressed.
"""
import argparse
import os
import shutil
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from duo_geometry import MIRROR, DuoGeometry  # noqa: E402

DEFAULT_FILE = os.path.expanduser('~/fr3_tools/fr3_duo_home/home_poses.txt')


def read(path):
    head, poses = [], {}
    for line in open(path):
        f = line.split()
        if len(f) == 8 and not f[0].startswith('#'):
            poses[f[0]] = np.array([float(x) for x in f[1:]])
        else:
            head.append(line.rstrip('\n'))
    return head, poses


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--file', default=DEFAULT_FILE)
    ap.add_argument('--left-ip', default='192.168.1.51')
    ap.add_argument('--right-ip', default='192.168.1.50')
    ap.add_argument('--reference', choices=['average', 'right', 'left'], default='average')
    ap.add_argument('--write', action='store_true')
    a = ap.parse_args()
    head, poses = read(a.file)
    ql, qr = poses[a.left_ip], poses[a.right_ip]
    if a.reference == 'average':
        nr = (qr + ql * MIRROR) / 2
        nl = nr * MIRROR
    elif a.reference == 'right':
        nr, nl = qr, qr * MIRROR
    else:
        nl, nr = ql, ql * MIRROR
    g = DuoGeometry()

    def describe(title, l, r):
        c, info = g.clearance(l, r)
        fl, fr = g.flange_position('left', l), g.flange_position('right', r)
        print(f'{title}\n  flange height left {fl[2]:.3f} m, right {fr[2]:.3f} m (diff {1000 * (fl[2] - fr[2]):+.0f} mm); '
              f'flange y left {fl[1]:+.3f} / right {fr[1]:+.3f}; x {fl[0]:.3f} / {fr[0]:.3f}\n'
              f'  closest parts {info[0]}/{info[1]}: {info[2] * 100:.1f} cm apart (guard margin {info[3] * 100:.0f} cm; '
              f'poses inside the margin stay allowed, only getting closer is blocked)')
    describe('SAVED', ql, qr)
    describe(f'NEW ({a.reference})', nl, nr)
    np.set_printoptions(precision=4, suppress=True)
    print(f'left  {a.left_ip}: change {np.abs(nl - ql).max():.3f} rad max\n  old {ql}\n  new {nl}')
    print(f'right {a.right_ip}: change {np.abs(nr - qr).max():.3f} rad max\n  old {qr}\n  new {nr}')
    if not a.write:
        print('\n(nothing written; add --write)')
        return
    bak = f'{a.file}.{time.strftime("%Y%m%d_%H%M%S")}.bak'
    shutil.copy2(a.file, bak)
    with open(a.file, 'w') as f:
        for h in head:
            f.write(h + '\n')
        f.write(f'# {time.strftime("%Y-%m-%d")}: mirrored ({a.reference}) by scripts/make_mirrored_home.py; '
                f'previous values in {os.path.basename(bak)}\n')
        for ip, q in ((a.right_ip, nr), (a.left_ip, nl)):
            f.write(f'{ip} ' + ' '.join(f'{x:.4f}' for x in q) + '\n')
    print(f'\nwritten {a.file} (backup {bak})')


if __name__ == '__main__':
    main()
