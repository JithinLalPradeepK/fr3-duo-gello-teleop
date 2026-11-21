#!/usr/bin/env python3
"""Read-only test recorder: logs every change of the demo state while the operator tests.

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/test_recorder.py [--out ~/robot_demo/logs/test_<ts>/events.log]

Records (wall-clock time, only on change): supervisor state + message, per-arm robot mode / errors / gate /
GELLO / gripper, Desk mode / brakes / FCI / control / busy / last result, gate state topics, pedal connection,
camera fps drops (< 20 fps) and recoveries, and gaps in the status stream. Never calls a service.
"""
import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ARMS = ('left', 'right')


class Recorder(Node):
    def __init__(self, out):
        super().__init__('test_recorder')
        self.f = open(out, 'a', buffering=1)
        self.prev = {}
        self.t_status = time.monotonic()
        self.status_gap = False
        self.cam_low = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/demo/status', self.on_status, latched)
        self.create_subscription(String, '/demo/desk/status', self.on_desk, latched)
        for arm in ARMS:
            self.create_subscription(String, f'/{arm}/gello_gate/state',
                                     lambda m, a=arm: self.change(f'gate_topic.{a}', m.data), latched)
        self.create_timer(1.0, self.watchdog)
        self.write('recorder started')

    def write(self, text):
        line = f'{time.strftime("%H:%M:%S")}.{int(time.time() * 1000) % 1000:03d}  {text}'
        self.f.write(line + '\n')
        print(line, flush=True)

    def change(self, key, value):
        if self.prev.get(key) != value:
            self.prev[key] = value
            self.write(f'{key:22s} {value}')

    def on_status(self, m):
        self.t_status = time.monotonic()
        if self.status_gap:
            self.status_gap = False
            self.write('status stream back')
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        self.change('STATE', f'{s.get("state")}: {s.get("message", "")}')
        for arm in ARMS:
            a = s.get('arms', {}).get(arm, {})
            self.change(f'{arm}.robot', f'{a.get("robot_mode")} errors={a.get("errors")}')
            self.change(f'{arm}.gate', a.get('gate'))
            self.change(f'{arm}.gello', a.get('gello'))
            self.change(f'{arm}.gripper', a.get('gripper'))
        self.change('pedal', s.get('pedal'))
        self.change('cameras_launch', s.get('cameras_launch'))
        for k, fps in (s.get('cameras') or {}).items():
            low = fps < 20
            if low != self.cam_low.get(k, False):
                self.cam_low[k] = low
                self.write(f'camera {k:12s} {"LOW" if low else "ok"} {fps:.1f} fps')

    def on_desk(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        for arm in ARMS:
            a = d.get(arm, {})
            self.change(f'desk.{arm}', f'{a.get("mode")}, brakes {a.get("brakes")}, FCI {a.get("fci")}, '
                                       f'control {a.get("control")}' + (f', ERROR {a["error"]}' if a.get('error') else ''))
        self.change('desk.busy', d.get('busy'))
        self.change('desk.last', d.get('last'))

    def watchdog(self):
        if not self.status_gap and time.monotonic() - self.t_status > 3.0:
            self.status_gap = True
            self.write('NO /demo/status for > 3 s (supervisor not running?)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/robot_demo/logs/test_{time.strftime("%Y%m%d_%H%M%S")}/events.log'))
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    rclpy.init()
    node = Recorder(a.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write('recorder stopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
