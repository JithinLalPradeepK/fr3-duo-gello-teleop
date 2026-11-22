#!/usr/bin/env python3
"""READ-ONLY tracking check: does the digital twin move with the GELLO? (subscribes only, never publishes)

  python3 test/twin_track_check.py [--seconds 40] [--out DIR]

Records, per arm:
  GELLO   /gello_check_<arm>/gello/joint_states  (what the operator does, 25 Hz)
  TWIN    /twin/joint_states                     (measured robot joints the twin draws, 100 Hz)
and /twin/status. Run it while somebody teleoperates (START, GELLOs out of the holders, move them).
Report per arm and joint: GELLO range of motion, scale (slope of twin vs GELLO motion, 1.0 = moves in proportion),
correlation, median / p95 |twin - GELLO| while LIVE, best lag (twin behind GELLO, ms), twin rate / latency.
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARMS = ('left', 'right')


class Rec(Node):
    def __init__(self):
        super().__init__('twin_track_check')
        self.g = {a: [] for a in ARMS}
        self.r = {a: [] for a in ARMS}
        self.status, self.demo = [], []
        for a in ARMS:
            self.create_subscription(JointState, f'/gello_check_{a}/gello/joint_states',
                                     lambda m, a=a: self.g[a].append((time.monotonic(), list(m.position[:7]))),
                                     qos_profile_sensor_data)
        self.create_subscription(JointState, '/twin/joint_states', self.on_twin, 100)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/twin/status', lambda m: self.status.append(json.loads(m.data)), latched)
        self.create_subscription(String, '/demo/status',
                                 lambda m: self.demo.append((time.monotonic(), json.loads(m.data))), latched)

    def on_twin(self, m):
        if m.header.frame_id == 'twin_placeholder' or not m.name:
            return
        for a in ARMS:
            if m.name[0].startswith(f'{a}_fr3_joint'):
                pos = dict(zip(m.name, m.position))
                q = [pos.get(f'{a}_fr3_joint{i}') for i in range(1, 8)]
                if None not in q:
                    self.r[a].append((time.monotonic(), q))


def analyse(arm, g, r, live_windows):
    if len(g) < 20 or len(r) < 50:
        return f'{arm}: not enough data (GELLO {len(g)}, twin {len(r)} samples)', None
    tg, qg = np.array([x[0] for x in g]), np.array([x[1] for x in g])
    tr, qr = np.array([x[0] for x in r]), np.array([x[1] for x in r])
    t0, t1 = max(tg[0], tr[0]), min(tg[-1], tr[-1])
    t = np.arange(t0, t1, 0.01)
    if len(t) < 100:
        return f'{arm}: overlap too short', None
    G = np.stack([np.interp(t, tg, qg[:, j]) for j in range(7)], 1)
    R = np.stack([np.interp(t, tr, qr[:, j]) for j in range(7)], 1)
    live = np.zeros(len(t), bool)
    for a, b in live_windows:
        live |= (t >= a) & (t <= b)
    lines = [f'{arm}: {t1 - t0:.0f} s overlap, LIVE {live.sum() * 0.01:.0f} s']
    rows = []
    for j in range(7):
        rng = np.ptp(G[live, j]) if live.any() else 0.0
        best = (0, 0.0)
        if live.sum() > 200 and rng > 0.05:
            dg = G[:, j] - G[:, j].mean()
            for lag in range(0, 60):                      # twin behind GELLO by lag * 10 ms
                a, b = dg[:len(dg) - lag], R[lag:, j] - R[lag:, j].mean()
                m = live[:len(live) - lag]
                if m.sum() < 100:
                    break
                c = np.corrcoef(a[m], b[m])[0, 1]
                if c > best[1]:
                    best = (lag, c)
            lag = best[0]
            x, y = G[:len(G) - lag, j][live[:len(live) - lag]], R[lag:, j][live[:len(live) - lag]]
            slope = np.polyfit(x - x.mean(), y - y.mean(), 1)[0]
            err = np.abs(R[live, j] - G[live, j])
            rows.append((j + 1, rng, slope, best[1], np.median(err), np.percentile(err, 95), lag * 10))
            lines.append(f'  j{j + 1}: GELLO range {rng:.2f} rad  scale {slope:.3f}  corr {best[1]:.3f}  '
                         f'|twin-GELLO| median {np.median(err):.3f} p95 {np.percentile(err, 95):.3f} rad  lag {lag * 10} ms')
        else:
            lines.append(f'  j{j + 1}: moved {rng:.2f} rad while LIVE (too little to judge)')
    return '\n'.join(lines), rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=40)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rclpy.init()
    n = Rec()
    end = time.monotonic() + a.seconds
    print(f'recording {a.seconds:.0f} s ... (teleoperate now)', flush=True)
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    # LIVE windows from the demo status (gate state LIVE on both arms) - fallback: whole recording
    wins, start = [], None
    for t, s in n.demo:
        live = s.get('state') == 'TELEOP'
        if live and start is None:
            start = t
        elif not live and start is not None:
            wins.append((start, t))
            start = None
    if start is not None:
        wins.append((start, time.monotonic()))
    if not wins:
        print('NOTE: no TELEOP state seen - comparing the whole recording')
        wins = [(0, 1e12)]
    ok = True
    for arm in ARMS:
        text, rows = analyse(arm, n.g[arm], n.r[arm], wins)
        print(text)
        for row in rows or []:
            if row[3] > 0.9 and not (0.9 <= row[2] <= 1.1):
                ok = False
        if a.out:
            os.makedirs(a.out, exist_ok=True)
            with open(os.path.join(a.out, f'track_{arm}.csv'), 'w') as f:
                w = csv.writer(f)
                w.writerow(['t', 'src'] + [f'q{i}' for i in range(1, 8)])
                for t, q in n.g[arm]:
                    w.writerow([f'{t:.3f}', 'gello'] + [f'{x:.4f}' for x in q])
                for t, q in n.r[arm]:
                    w.writerow([f'{t:.3f}', 'twin'] + [f'{x:.4f}' for x in q])
    if n.status:
        s = n.status[-1]
        for arm in ARMS:
            x = s['arms'][arm]
            print(f"twin {arm}: {x['state']} {x['hz']} Hz latency {x['latency_ms']} ms (max {x['latency_max_ms']})")
    print('SCALE CHECK:', 'PASS (moves in proportion)' if ok else 'CHECK the joints above (scale outside 0.9-1.1)')
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
