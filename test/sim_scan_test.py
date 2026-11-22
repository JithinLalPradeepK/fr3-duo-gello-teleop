#!/usr/bin/env python3
"""SIM test of the supervisor's 3D SCAN (fake hardware, ROS_DOMAIN_ID=77, localhost; no robot, no camera).
Run via test/sim_scan_test.sh. Checks:
  C1 SCAN refused when the arms are not at home     C2 HOME, then SCAN runs every view ("SCAN view i/n") and ends
  C3 the flanges really go up (>= 8 cm) and never below home on the way          C4 ends at home, "scan complete"
  C5 STOP during a scan: robots hold, IDLE, the rest of the scan is dropped, no "scan complete"
"""
import json
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import DuoGeometry  # noqa: E402
from twin_state import load_home  # noqa: E402

R = []


def check(name, ok, detail=''):
    R.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}{"  - " + detail if detail else ""}', flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__('sim_scan_probe')
        lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.s, self.msgs, self.q = {}, [], {}
        self.create_subscription(String, '/demo/status', self.on_status, lat)
        for a in ('left', 'right'):
            self.create_subscription(JointState, f'/{a}/franka/joint_states', lambda m, a=a: self.on_js(a, m), 50)
        self.cli = {}
        self.g = DuoGeometry()
        self.zmin = {a: 9.0 for a in ('left', 'right')}
        self.zmax = {a: -9.0 for a in ('left', 'right')}
        self.track = False

    def on_status(self, m):
        self.s = json.loads(m.data)
        txt = f"{self.s['state']}: {self.s['message']}"
        if not self.msgs or self.msgs[-1] != txt:
            self.msgs.append(txt)

    def on_js(self, arm, m):
        p = dict(zip(m.name, m.position))
        q = [p.get(f'{arm}_fr3_joint{i}') for i in range(1, 8)]
        if None in q:
            return
        self.q[arm] = q
        if self.track:
            z = self.g.link_frames(arm, q)[8][2, 3]
            self.zmin[arm], self.zmax[arm] = min(self.zmin[arm], z), max(self.zmax[arm], z)

    def call(self, name):
        c = self.cli.get(name) or self.cli.setdefault(name, self.create_client(Trigger, f'/demo/{name}'))
        c.wait_for_service(timeout_sec=10)
        f = c.call_async(Trigger.Request())
        t = time.monotonic()
        while not f.done() and time.monotonic() - t < 10:
            time.sleep(0.05)
        r = f.result()
        return (r.success, r.message) if r else (False, 'no answer')

    def wait(self, cond, timeout):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            try:
                if cond():
                    return True
            except (KeyError, TypeError):
                pass
            time.sleep(0.1)
        return False


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        sys.exit('run via test/sim_scan_test.sh')
    logdir = sys.argv[1]
    cfg = yaml.safe_load(open(os.path.join(DEMO, 'config', 'fr3_duo_sim.yaml')))
    assert all(str(c['use_fake_hardware']).lower() == 'true' for c in cfg.values())
    ips = {'LEFT': '0.0.0.1', 'RIGHT': '0.0.0.2'}          # fake IPs (ignored by the fake hardware): own home poses
    for k in cfg:
        cfg[k]['robot_ip'] = ips[k]
    sim_cfg = os.path.join(logdir, 'sim_cfg.yaml')
    yaml.safe_dump(cfg, open(sim_cfg, 'w'))
    home = load_home()
    home_file = os.path.join(logdir, 'home.txt')
    with open(home_file, 'w') as f:
        f.write(f"0.0.0.1 {' '.join(f'{x:.6f}' for x in home['left'])}\n")
        f.write(f"0.0.0.2 {' '.join(f'{x:.6f}' for x in home['right'])}\n")
    holder = os.path.join(logdir, 'holder.yaml')
    yaml.safe_dump({'left': [0.0] * 7, 'right': [0.0] * 7}, open(holder, 'w'))
    sup = subprocess.Popen([sys.executable, os.path.join(DEMO, 'scripts', 'demo_supervisor.py'), '--sim', '--bringup',
                            '--robot-config', sim_cfg, '--no-grippers', '--no-gello', '--no-cameras',
                            '--holder-file', holder, '--home-file', home_file,
                            '--lock-file', os.path.join(logdir, 'home.lock'),
                            '--gate-args', f'--holder-file {holder} --wait-timeout 8'],
                           stdout=open(os.path.join(logdir, 'supervisor.log'), 'w'), stderr=subprocess.STDOUT)
    rclpy.init()
    p = Probe()
    ex = MultiThreadedExecutor()
    ex.add_node(p)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        ok = p.wait(lambda: p.s['state'] == 'IDLE' and len(p.q) == 2, 90)
        check('C0 sim supervisor IDLE', ok, ' -> '.join(p.msgs[-3:]))
        at_home = all(np.max(np.abs(np.subtract(p.q[a], home[a]))) < 0.05 for a in ('left', 'right'))
        if not at_home:
            r = p.call('scan')
            check('C1 SCAN refused when the arms are not at home', not r[0] and 'not at home' in r[1], r[1])
        else:
            check('C1 (arms start at home in this sim - refusal not testable)', True)
        r = p.call('home')
        ok = p.wait(lambda: p.s['state'] == 'IDLE' and 'home' in p.s['message'] and 'HOMING' in ' '.join(p.msgs), 120)
        check('C2 HOME first', ok and r[0], p.s.get('message', ''))
        g = DuoGeometry()
        z_home = {a: g.link_frames(a, home[a])[8][2, 3] for a in ('left', 'right')}
        p.msgs.clear()
        p.track = True
        r = p.call('scan')
        check('C2 SCAN accepted from home', r[0], r[1])
        ok = p.wait(lambda: p.s['state'] == 'IDLE' and p.s['message'].startswith('scan complete'), 400)
        p.track = False
        views = sorted({m.split('SCAN view ')[1].split(':')[0] for m in p.msgs if 'SCAN view' in m})
        n = int(views[0].split('/')[1]) if views else 0
        check('C2 every view visited, in order', ok and len(views) == n and n >= 5, f'{views}')
        lift = {a: p.zmax[a] - z_home[a] for a in ('left', 'right')}
        low = {a: p.zmin[a] - z_home[a] for a in ('left', 'right')}
        check('C3 flanges went up >= 8 cm', all(v >= 0.08 for v in lift.values()),
              ' '.join(f'{a} {v * 100:.1f} cm' for a, v in lift.items()))
        check('C3 never below home (-5 mm)', all(v >= -0.005 for v in low.values()),
              ' '.join(f'{a} {v * 1000:.1f} mm' for a, v in low.items()))
        back = all(np.max(np.abs(np.subtract(p.q[a], home[a]))) < 0.01 for a in ('left', 'right'))
        check('C4 ends at home, "scan complete"', ok and back, p.s.get('message', ''))
        # C5 STOP during a scan
        p.msgs.clear()
        r = p.call('scan')
        ok = p.wait(lambda: 'SCAN view 2/' in p.s.get('message', ''), 120)
        r2 = p.call('stop')
        ok2 = p.wait(lambda: p.s['state'] == 'IDLE', 15)
        time.sleep(4)
        check('C5 STOP during a scan -> IDLE, no further views, no "scan complete"',
              ok and r2[0] and ok2 and p.s['state'] == 'IDLE' and not p.s['message'].startswith('scan complete')
              and p.s.get('scan') is None and not any('SCAN view 3/' in m for m in p.msgs), ' -> '.join(p.msgs[-3:]))
    finally:
        sup.terminate()
        try:
            sup.wait(60)
        except subprocess.TimeoutExpired:
            sup.kill()
        ex.shutdown()
        rclpy.try_shutdown()
    print(f'===== {"PASS" if all(R) else "FAIL"} ({sum(R)}/{len(R)})')
    sys.exit(0 if all(R) else 1)


if __name__ == '__main__':
    main()
