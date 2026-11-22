#!/usr/bin/env python3
"""SIM test of the twin reconstruction (test/sim_recon_test.sh: ROS_DOMAIN_ID=79, localhost only). No robot, no camera.

Starts launch/digital_twin.launch.py (rviz:=false clouds:=false, a TEST copy of the config: map file in the log dir,
d405_1 marked calibrated, ZED looking at the table), test/twin_fake_robots.py --still and test/twin_fake_depth.py, then:
  N1 node up, ZED + left D405 fused through the real throttles / TF chain, d405_2 waiting (no data)
  N2 static mesh on /twin/mesh/static: enough triangles, vertices on the true surfaces (both cameras' TF chains right)
  N3 no vertex inside the robots' capsules
  N4 live object: appears as live voxels at the object, not in the static map at first
  N5 /twin/status (GUI telemetry) lists "mesh"
  N6 layer "mesh" off -> DELETEALL + cameras unsubscribed; on -> mesh back
  N7 save / reset / load services (+ .ply written)
  N8 CPU of the recon process, worker frame time
"""
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
sys.path.insert(0, os.path.join(DEMO, 'test'))
from duo_geometry import DuoGeometry  # noqa: E402
from twin_fake_depth import OBJ  # noqa: E402
from twin_recon_core import RobotFilter, point_segment_dist  # noqa: E402
from twin_recon_test import scene_sdf  # noqa: E402
from twin_state import load_home  # noqa: E402

results = []


def check(name, ok, info=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}", flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__('sim_recon_probe')
        mesh_qos = QoSProfile(depth=17, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.chunks, self.deleteall, self.live, self.status, self.twin = {}, 0, None, None, None
        self.create_subscription(Marker, '/twin/mesh/static', self.on_static, mesh_qos)
        self.create_subscription(Marker, '/twin/mesh/live', lambda m: setattr(self, 'live', m), 5)
        self.create_subscription(String, '/twin/recon/status', lambda m: setattr(self, 'status', json.loads(m.data)),
                                 latched)
        self.create_subscription(String, '/twin/status', lambda m: setattr(self, 'twin', json.loads(m.data)), latched)
        self.layers = self.create_publisher(String, '/twin/layers_cmd', 10)
        self.cli = {k: self.create_client(Trigger, f'/twin/recon/{k}') for k in ('save', 'reset', 'load')}

    def on_static(self, m):
        if m.action == Marker.DELETEALL:
            self.chunks.clear()
            self.deleteall += 1
        else:
            self.chunks[m.id] = np.array([(p.x, p.y, p.z) for p in m.points]).reshape(-1, 3)

    def mesh(self):
        v = [c for c in self.chunks.values() if len(c)]
        return np.concatenate(v) if v else np.zeros((0, 3))

    def spin_for(self, s, until=None):
        t = time.monotonic() + s
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until else True

    def call(self, k):
        if not self.cli[k].wait_for_service(timeout_sec=5):
            return None
        f = self.cli[k].call_async(Trigger.Request())
        self.spin_for(10, lambda: f.done())
        return f.result()


def start(cmd, log, env=None):
    return subprocess.Popen(cmd, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True, env=env)


def cpu_of(pattern):
    try:
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()
        for p in out:
            env = open(f'/proc/{p}/environ', 'rb').read()
            if b'ROS_DOMAIN_ID=79' in env:
                return float(subprocess.run(['ps', '-o', '%cpu=', '-p', p], capture_output=True, text=True).stdout or 0)
    except OSError:
        pass
    return None


def main():
    out = sys.argv[1]
    cfg = yaml.safe_load(open(os.path.join(DEMO, 'config', 'digital_twin.yaml')))
    cfg['cameras']['zed'].update(xyz=[1.35, 0.0, 0.80], rpy=[0.0, 0.6, 3.14159])
    cfg['cameras']['d405_1']['calibrated'] = True
    r = cfg['reconstruction']
    r['map_file'] = os.path.join(out, 'recon_map.npz')
    r['autoload'] = False
    r['cameras']['zed']['stride'] = 2           # fake ZED is 960x540
    r['cameras']['zed']['hz'] = 4.0
    cfg_path = os.path.join(out, 'digital_twin_test.yaml')
    yaml.safe_dump(cfg, open(cfg_path, 'w'))
    launch = [os.path.join(DEMO, 'scripts', 'sigint_default'), 'ros2', 'launch',
              os.path.join(DEMO, 'launch', 'digital_twin.launch.py'), 'rviz:=false', 'clouds:=false', f'config:={cfg_path}']
    procs = [start(launch, os.path.join(out, 'launch.log')),
             start([sys.executable, os.path.join(DEMO, 'test', 'twin_fake_robots.py'), '--still'],
                   os.path.join(out, 'fake_robots.log')),
             start([sys.executable, os.path.join(DEMO, 'test', 'twin_fake_depth.py'), '--config', cfg_path,
                    '--object-after', '22', '--object-until', '1e9'], os.path.join(out, 'fake_depth.log'))]
    rclpy.init()
    p = Probe()
    g = DuoGeometry()
    home = load_home()
    caps = RobotFilter(g).capsules({'left': home['left'], 'right': home['right']})
    try:
        ok = p.spin_for(40, lambda: p.status and p.status['cameras'].get('zed', {}).get('state') == 'ok'
                        and p.status['cameras'].get('d405_1', {}).get('state') == 'ok')
        cams = (p.status or {}).get('cameras', {})
        check('N1 ZED + left D405 fused, right D405 waiting',
              ok and cams.get('d405_2', {}).get('state', '').startswith('waiting'),
              json.dumps({k: v['state'] for k, v in cams.items()}))
        p.spin_for(8)
        v = p.mesh()
        err = scene_sdf(v, caps) if len(v) else np.array([1.0])
        check('N2 static mesh on the true surfaces', len(v) > 3 * 5000 and np.median(err) < 0.003 and
              np.percentile(err, 95) < 0.01, f'{len(v) // 3} triangles in {len(p.chunks)} chunks, median '
              f'{np.median(err) * 1e3:.1f} mm, p95 {np.percentile(err, 95) * 1e3:.1f} mm')
        # wrist-only region: below the left flange (the ZED sees it too, but the D405 adds density) - check the TF chain
        # of the wrist camera separately: the left flange's footprint on the table must be meshed and flat
        fl = g.flange_position('left', np.array(home['left']))
        foot = v[(np.linalg.norm(v[:, :2] - fl[:2], axis=1) < 0.1) & (np.abs(v[:, 2]) < 0.02)]
        check('N2b table under the left wrist camera meshed flat', len(foot) > 100 and np.abs(foot[:, 2]).max() < 0.006,
              f'{len(foot)} vertices, max |z| {np.abs(foot[:, 2]).max() * 1e3 if len(foot) else -1:.1f} mm')
        A, B, R = caps
        dr = (point_segment_dist(v, A, B) - R[None]).min(1) if len(v) else np.array([1.0])
        check('N3 no robot in the static mesh', (dr < -0.005).sum() == 0, f'closest {dr.min() * 1e3:.0f} mm')
        p.spin_for(20, lambda: p.live is not None and len(p.live.points) > 20)
        lp = np.array([(q.x, q.y, q.z) for q in p.live.points]) if p.live else np.zeros((0, 3))
        d_obj = np.abs(np.linalg.norm(lp - np.array(OBJ[0]), axis=1) - OBJ[1]) if len(lp) else np.array([1.0])
        v = p.mesh()
        n_static = int((np.linalg.norm(v - np.array(OBJ[0]), axis=1) < OBJ[1] + 0.01).sum())
        check('N4 live object shown live, not static', len(lp) > 20 and np.percentile(d_obj, 90) < 0.02 and n_static == 0,
              f'{len(lp)} live voxels (p90 {np.percentile(d_obj, 90) * 1e3:.0f} mm), static vertices there {n_static}')
        p.spin_for(3)
        cl = (p.twin or {}).get('clouds', {})
        check('N5 GUI telemetry has the mesh', 'mesh' in cl and cl['mesh'].get('on') and cl['mesh'].get('points', 0) > 0,
              json.dumps(cl.get('mesh')))
        n_del = p.deleteall
        p.layers.publish(String(data=json.dumps({'mesh': False, 'live_objects': False})))
        off = p.spin_for(5, lambda: p.deleteall > n_del and p.status and not p.status['layers']['mesh'])
        p.spin_for(2)
        st_off = {k: c['state'] for k, c in p.status['cameras'].items()}
        p.layers.publish(String(data=json.dumps({'mesh': True, 'live_objects': True})))
        on = p.spin_for(10, lambda: len(p.mesh()) > 3 * 5000)
        check('N6 layer off clears + stops reading cameras, on restores', off and on and
              all(s == 'layer off' for s in st_off.values()), json.dumps(st_off))
        rs = p.call('save')
        saved = rs is not None and rs.success and os.path.exists(r['map_file']) and \
            os.path.exists(r['map_file'].replace('.npz', '.ply'))
        n_before = len(p.mesh())
        rr = p.call('reset')
        # fake cameras keep publishing: right after reset the map is (nearly) empty, then refills
        p.spin_for(1.6)
        n_reset = (p.status or {}).get('triangles', -1)
        rl = p.call('load')
        p.spin_for(2.5)
        n_after = len(p.mesh())
        check('N7 save / reset / load', saved and rr.success and rl.success and n_after > 0.8 * n_before,
              f'{rs.message if rs else None}; after reset {n_reset} triangles; after load {n_after // 3} vs {n_before // 3}')
        # N9 camera pose changed in the config (as a calibration write does) -> map reset, rebuilt at the new pose
        frames0 = (p.status or {}).get('frames', 0)
        c2 = yaml.safe_load(open(cfg_path))
        c2['cameras']['zed']['xyz'] = [c2['cameras']['zed']['xyz'][0] + 0.002] + c2['cameras']['zed']['xyz'][1:]
        yaml.safe_dump(c2, open(cfg_path, 'w'))
        reset = p.spin_for(6, lambda: p.status is not None and p.status.get('frames', 1e9) < frames0)
        p.spin_for(20, lambda: (p.status or {}).get('triangles', 0) > 3000)   # 6 mm: projection cache is rebuilt
        check('N9 camera pose change in the config -> map reset and rebuilt', reset and (p.status or {}).get('triangles', 0) > 3000,
              f"frames {frames0} -> reset; triangles after rebuild {(p.status or {}).get('triangles')}")
        cpu = cpu_of('scripts/twin_recon.py')
        fm = (p.status or {}).get('frame_ms')
        check('N8 recon CPU and frame time', cpu is not None and cpu < 120 and fm is not None and fm < 300,
              f'twin_recon {cpu} % CPU (one core = 100), worker {fm} ms/frame, mesh {(p.status or {}).get("mesh_ms")} ms')
    finally:
        for pr in procs:
            try:
                os.killpg(pr.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for pr in procs:
            try:
                pr.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(pr.pid, signal.SIGKILL)
        p.destroy_node()
        rclpy.try_shutdown()
    print(f'\n{sum(results)}/{len(results)} passed  (logs {out})')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
