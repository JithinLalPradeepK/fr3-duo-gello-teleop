#!/usr/bin/env python3
"""Record the GELLO HOLDER pose for one arm (READ ONLY: subscribes, never publishes).

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/record_gello_holder.py --arm left [--file ...] [--seconds 1.0]

Put the GELLO in its holder and leave it still. Needs the GELLO publisher on /gello_check_<arm>
(gello_check.launch.py). Writes the averaged 7 joint values to config/gello_holder_poses.yaml
(other arms in the file are kept). gello_gate.py never moves the robot while the GELLO is within
holder_tol of this pose.
"""
import argparse
import datetime
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_FILE = os.path.expanduser('~/robot_demo/config/gello_holder_poses.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', required=True, choices=['left', 'right'])
    ap.add_argument('--file', default=DEFAULT_FILE)
    ap.add_argument('--seconds', type=float, default=1.0)
    ap.add_argument('--still-tol', type=float, default=0.03)
    a = ap.parse_args()
    rclpy.init()
    node = Node(f'record_gello_holder_{a.arm}')
    samples = []
    node.create_subscription(JointState, f'/gello_check_{a.arm}/gello/joint_states',
                             lambda m: samples.append(list(m.position[:7])), 10)
    end = time.monotonic() + a.seconds + 0.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.try_shutdown()
    if len(samples) < 10:
        sys.exit(f'only {len(samples)} GELLO samples on /gello_check_{a.arm} - is gello_check running?')
    spread = max(max(c) - min(c) for c in zip(*samples))
    if spread > a.still_tol:
        sys.exit(f'GELLO not still (spread {spread:.3f} rad) - leave it resting in the holder and retry')
    pose = [round(sum(c) / len(c), 4) for c in zip(*samples)]

    data = {}
    if os.path.exists(a.file):
        with open(a.file) as f:
            data = yaml.safe_load(f) or {}
    data[a.arm] = pose
    data.setdefault('recorded', {})[a.arm] = datetime.datetime.now().isoformat(timespec='seconds')
    with open(a.file, 'w') as f:
        f.write('# GELLO holder poses (rad, joints 1-7) written by scripts/record_gello_holder.py\n')
        f.write('# gello_gate.py never moves the robot while the GELLO is within holder_tol of these.\n')
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=True)
    print(f'{a.arm} holder pose ({len(samples)} samples, spread {spread:.4f} rad): {pose}')
    print(f'written to {a.file}')


if __name__ == '__main__':
    main()
