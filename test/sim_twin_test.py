#!/usr/bin/env python3
"""SIM test of the digital twin (no robot, no camera, no hardware). Run via test/sim_twin_test.sh (ROS_DOMAIN_ID=78,
localhost only). Starts launch/digital_twin.launch.py (rviz:=false) and test/twin_fake_robots.py and checks:
  W1 twin URDF joint names = the real arm stack's joint names      W2 /twin/joint_states rate + latency per arm
  W3 TF chain world -> fingertips; flange position == duo_geometry FK (same mount geometry as the HOME planner)
  W4 gripper knuckle drives the Robotiq fingers                    W5 load / force overlays (contact = red)
  W6 layers_cmd -> /twin/layers, markers removed, robots layer strips visuals
  W7 camera static TFs + config hot reload                          W8 D405 depth -> coloured cloud, lazy on/off
  W9 ZED organised cloud -> range crop + decimation                 W10 STALE (data stops) / NO DATA (never came)
  W11 GELLO ghost: shown while TELEOP, posed by the GELLO joints, layer switch
"""
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import DuoGeometry  # noqa: E402
from twin_state import load_home  # noqa: E402

RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok)))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}{"  - " + detail if detail else ""}', flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__('twin_test_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.status, self.layers, self.view, self.markers = None, None, None, {}
        self.clouds = {}
        self.create_subscription(String, '/twin/status', lambda m: setattr(self, 'status', json.loads(m.data)), latched)
        self.create_subscription(String, '/twin/layers', lambda m: setattr(self, 'layers', json.loads(m.data)), latched)
        self.create_subscription(String, '/twin/robot_description_view', lambda m: setattr(self, 'view', m.data), latched)
        self.ghost = None
        self.create_subscription(String, '/twin/ghost_description_view', lambda m: setattr(self, 'ghost', m.data), latched)
        self.create_subscription(MarkerArray, '/twin/markers', self.on_markers, 10)
        for k in ('zed', 'd405_1'):
            self.create_subscription(PointCloud2, f'/twin/cloud/{k}', lambda m, k=k: self.clouds.__setitem__(k, m), 2)
        self.cmd = self.create_publisher(String, '/twin/layers_cmd', 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.depth = self.create_publisher(Image, '/camera/d405_1/aligned_depth_to_color/image_raw', 2)
        self.color = self.create_publisher(Image, '/camera/d405_1/color/image_raw', 2)
        self.info = self.create_publisher(CameraInfo, '/camera/d405_1/color/camera_info', 2)
        self.zed = self.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 2)

    def on_markers(self, ma):
        for m in ma.markers:
            if m.action == 2:
                self.markers.pop((m.ns, m.id), None)
            else:
                self.markers[(m.ns, m.id)] = m

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                if cond():
                    return True
            except (KeyError, TypeError, AttributeError):
                pass
        return False

    def lookup(self, a, b):
        try:
            t = self.tf.lookup_transform(a, b, rclpy.time.Time())
            return np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        except Exception:    # noqa: BLE001
            return None

    def fake_d405(self):
        h, w = 120, 160
        stamp = self.get_clock().now().to_msg()
        info = CameraInfo(height=h, width=w)
        info.header.stamp, info.header.frame_id = stamp, 'd405_1_color_optical_frame'
        info.k = [100.0, 0.0, 80.0, 0.0, 100.0, 60.0, 0.0, 0.0, 1.0]
        d = np.full((h, w), 400, np.uint16)          # 0.4 m plane
        d[:, :20] = 0                                # no data
        d[:, -20:] = 1500                            # beyond max range (0.8 m)
        di = Image(height=h, width=w, encoding='16UC1', step=w * 2, data=d.tobytes())
        di.header = info.header
        c = np.zeros((h, w, 3), np.uint8)
        c[..., 0] = 200
        ci = Image(height=h, width=w, encoding='rgb8', step=w * 3, data=c.tobytes())
        ci.header = info.header
        self.info.publish(info)
        self.color.publish(ci)
        self.depth.publish(di)

    def fake_zed(self):
        h, w = 108, 192
        pts = np.zeros((h, w), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        pts['x'] = 1.0                                 # 1 m in front (ZED frames: x forward)
        pts['x'][:, w // 2:] = 6.0                     # right half beyond max range 2.5 m
        pts['x'][:10, :] = np.nan
        m = PointCloud2(height=h, width=w, point_step=16, row_step=16 * w, is_dense=False)
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'zed_left_camera_frame'
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
        m.data = pts.tobytes()
        self.zed.publish(m)


def start(cmd, log):
    return subprocess.Popen(cmd, stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)


def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '78':
        sys.exit('run via test/sim_twin_test.sh (ROS_DOMAIN_ID=78)')
    out = sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp(prefix='simtwin_')
    os.makedirs(out, exist_ok=True)
    cfg_path = os.path.join(out, 'digital_twin.yaml')
    shutil.copy(os.path.join(DEMO, 'config', 'digital_twin.yaml'), cfg_path)
    launch = [os.path.join(DEMO, 'scripts', 'sigint_default'), 'ros2', 'launch', os.path.join(DEMO, 'launch', 'digital_twin.launch.py'), 'rviz:=false',
              f'config:={cfg_path}']
    fake = [sys.executable, os.path.join(DEMO, 'test', 'twin_fake_robots.py')]
    rclpy.init()
    probe = Probe()
    g, home = DuoGeometry(), load_home()
    procs = []
    try:
        # ------------------------------------------------------------------ phase 1: both arms still at HOME
        print('phase 1: both arms (fake, still at HOME)', flush=True)
        procs = [start(launch, os.path.join(out, 'launch1.log')),
                 start(fake + ['--still', '--gello'], os.path.join(out, 'fake1.log'))]
        ok = probe.wait(lambda: probe.status['arms']['left']['state'] == 'live' and
                        probe.status['arms']['right']['state'] == 'live', 25)
        check('W0 twin up, both arms live', ok)
        # W1 joint names
        urdf = subprocess.run(['xacro', os.path.join(DEMO, 'description', 'fr3_duo_twin.urdf.xacro')],
                              capture_output=True, text=True).stdout
        names = set(re.findall(r'<joint name="([^"]+)" type="(?:revolute|continuous)"', urdf))
        want = {f'{a}_fr3_joint{i}' for a in ('left', 'right') for i in range(1, 8)}
        check('W1 URDF has left/right_fr3_joint1..7 (real stack names) + Robotiq knuckles',
              want <= names and {'left_robotiq_85_left_knuckle_joint', 'right_robotiq_85_left_knuckle_joint'} <= names)
        # W2 rate / latency
        probe.spin_for(2.5)
        for arm in ('left', 'right'):
            s = probe.status['arms'][arm]
            check(f'W2 {arm}: /twin/joint_states 70-110 Hz, latency median < 5 ms',
                  70 <= s['hz'] <= 110 and s['latency_ms'] is not None and s['latency_ms'] < 5.0,
                  f"{s['hz']} Hz, {s['latency_ms']} ms (max {s['latency_max_ms']} ms)")
        # W3 TF + FK cross-check
        tip = probe.wait(lambda: probe.lookup('twin/world', 'twin/left_robotiq_85_left_finger_tip_link') is not None, 5)
        check('W3 TF chain twin/world -> twin/left_robotiq_85_left_finger_tip_link', tip)
        for arm in ('left', 'right'):
            p = probe.lookup('twin/mount_link', f'twin/{arm}_fr3_link8')
            fk = g.flange_position(arm, home[arm])
            err = float(np.linalg.norm(p - fk)) if p is not None else 1e9
            check(f'W3 {arm} flange (TF) == duo_geometry FK at HOME (< 1 mm)', err < 1e-3, f'{err * 1000:.2f} mm')
        # W4 gripper moves the fingers
        a = probe.lookup('twin/left_robotiq_85_base_link', 'twin/left_robotiq_85_left_finger_tip_link')
        probe.spin_for(1.5)
        b = probe.lookup('twin/left_robotiq_85_base_link', 'twin/left_robotiq_85_left_finger_tip_link')
        check('W4 Robotiq knuckle from /<arm>/gripper/joint_states moves the finger tip',
              a is not None and b is not None and np.linalg.norm(a - b) > 1e-4 and
              probe.status['arms']['left']['gripper'] is not None,
              f"moved {np.linalg.norm(a - b) * 1000:.1f} mm" if a is not None and b is not None else 'no TF')
        # W5 overlays
        ok = probe.wait(lambda: ('load_left', 3) in probe.markers and ('force_right', 0) in probe.markers and
                        probe.markers[('force_right', 0)].color.r > 0.8 and probe.markers[('force_right', 0)].color.g < 0.3,
                        15)
        check('W5 load sphere on j4 + force arrow, red during contact', ok)
        check('W5 clearance at HOME reported (1.7 cm, link6/link6)',
              probe.status['clearance'] and abs(probe.status['clearance']['m'] - 0.017) < 0.003,
              str(probe.status['clearance']))
        # W6 layers
        probe.cmd.publish(String(data=json.dumps({'load': False, 'robots': False})))
        ok = probe.wait(lambda: probe.layers['load'] is False and probe.layers['robots'] is False, 5)
        probe.spin_for(0.5)
        check('W6 layers_cmd -> /twin/layers (load off, robots off)', ok)
        check('W6 load markers deleted', not any(k[0].startswith('load_') for k in probe.markers))
        arm_vis = re.search(r'<link name="left_fr3_link3">.*?</link>', probe.view or '', re.S)
        check('W6 robots off: view description has no arm visuals, mount kept',
              arm_vis and '<visual' not in arm_vis.group(0) and
              '<visual' in re.search(r'<link name="mount_link">.*?</link>', probe.view, re.S).group(0))
        probe.cmd.publish(String(data=json.dumps({'load': True, 'robots': True})))
        ok = probe.wait(lambda: probe.layers['robots'] and '<visual' in
                        re.search(r'<link name="left_fr3_link3">.*?</link>', probe.view, re.S).group(0), 5)
        check('W6 robots back on: visuals restored', ok)
        # W11 GELLO ghost
        ok = probe.wait(lambda: probe.status['arms']['left']['ghost'] and probe.status['arms']['right']['ghost'], 8)
        check('W11 ghost shown for both arms while TELEOP and GELLO data flows', ok, str({a: probe.status['arms'][a].get(
            'gello_hz') for a in ('left', 'right')}))
        gv = probe.ghost or ''
        check('W11 ghost description: arm visuals yes, mount no',
              '<visual' in (re.search(r'<link name="left_fr3_link3">.*?</link>', gv, re.S) or [''])[0] and
              '<visual' not in (re.search(r'<link name="mount_link">.*?</link>', gv, re.S) or ['<visual'])[0])
        hq = list(home['left'])
        hq[0] += 0.2
        p = probe.lookup('ghost/mount_link', 'ghost/left_fr3_link8')
        err = np.linalg.norm(p - g.flange_position('left', hq)) if p is not None else 1e9
        check('W11 ghost flange == FK of the GELLO pose (GELLO joints in any message order)', err < 1e-3,
              f'{err * 1000:.2f} mm')
        r = probe.lookup('twin/world', 'ghost/world')
        check('W11 ghost world == twin world', r is not None and np.allclose(r, 0))
        probe.cmd.publish(String(data=json.dumps({'gello': False})))
        ok = probe.wait(lambda: not probe.status['arms']['left']['ghost'] and
                        '<visual' not in (re.search(r'<link name="left_fr3_link3">.*?</link>', probe.ghost, re.S)
                                          or ['<visual'])[0], 5)
        check('W11 GELLO layer off -> ghost hidden', ok)
        probe.cmd.publish(String(data=json.dumps({'gello': True})))
        # W7 camera TFs + hot reload
        z = probe.wait(lambda: probe.lookup('twin/world', 'zed_camera_link') is not None, 5)
        p0 = probe.lookup('twin/world', 'zed_camera_link')
        zed_xyz = yaml.safe_load(open(cfg_path))['cameras']['zed']['xyz']
        check('W7 static TF twin/world -> zed_camera_link from config', z and np.allclose(p0, zed_xyz, atol=1e-3),
              str(p0))
        d1 = probe.lookup('twin/left_fr3_link8', 'd405_1_link')
        check('W7 static TF twin/left_fr3_link8 -> d405_1_link', d1 is not None)
        cfg = yaml.safe_load(open(cfg_path))
        cfg['cameras']['zed']['xyz'] = [1.10, 0.20, 0.90]
        time.sleep(1.1)                                  # mtime resolution
        yaml.safe_dump(cfg, open(cfg_path, 'w'))
        ok = probe.wait(lambda: np.allclose(probe.lookup('twin/world', 'zed_camera_link'), [1.10, 0.20, 0.90], atol=1e-3),
                        6)
        check('W7 config change -> camera TF updated without restart', ok)
        # W8 D405 cloud (lazy)
        probe.clouds.clear()
        probe.fake_d405()
        probe.spin_for(1.0)
        check('W8 wrist layer OFF: no D405 cloud computed', 'd405_1' not in probe.clouds or probe.clouds['d405_1'].width == 0)
        probe.cmd.publish(String(data=json.dumps({'wrist_clouds': True})))
        probe.spin_for(1.0)

        def got_d405():
            probe.fake_d405()
            return 'd405_1' in probe.clouds and probe.clouds['d405_1'].width > 0
        ok = probe.wait(got_d405, 8)
        c = probe.clouds.get('d405_1')
        if ok:
            arr = np.frombuffer(bytes(c.data), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            n_exp = (120 // 4) * ((160 - 40) // 4)
            check('W8 D405 depth -> cloud: z = 0.4 m, no-data + far pixels dropped, colour from RGB',
                  len(arr) == n_exp and np.allclose(arr['z'], 0.4, atol=1e-3) and
                  np.all((arr['rgb'] >> 16) & 0xFF == 200) and c.header.frame_id == 'd405_1_color_optical_frame',
                  f'{len(arr)} points (expected {n_exp})')
        else:
            check('W8 D405 depth -> cloud', False, 'no cloud')
        probe.cmd.publish(String(data=json.dumps({'wrist_clouds': False})))
        ok = probe.wait(lambda: probe.clouds['d405_1'].width == 0, 5)
        check('W8 wrist layer OFF again: empty cloud published (RViz clears)', ok)
        # W9 ZED crop (the raw ZED cloud is an optional overlay since the solid mesh: switch it on)
        probe.cmd.publish(String(data=json.dumps({'zed_cloud': True})))
        probe.clouds.pop('zed', None)

        def got_zed():
            probe.fake_zed()
            return 'zed' in probe.clouds and probe.clouds['zed'].width > 0
        ok = probe.wait(got_zed, 8)
        if ok:
            arr = np.frombuffer(bytes(probe.clouds['zed'].data), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
            n_exp = (108 - 10) * 96                     # rows >= 10 (no NaN) x left half (< 2.5 m), every pixel
            check('W9 ZED cloud: NaN + beyond 2.5 m removed', len(arr) == n_exp and np.all(arr['x'] == 1.0),
                  f'{len(arr)} points (expected {n_exp})')
        else:
            check('W9 ZED cloud', False, 'no cloud')
        stop(procs[1])
        stop(procs[0])
        # ------------------------------------------------------------------ phase 2: stale + no data
        print('phase 2: left arm stops after 5 s, right arm never sends', flush=True)
        probe.status = None
        procs = [start(launch, os.path.join(out, 'launch2.log')),
                 start(fake + ['--skip', 'right', '--stop-after', 'left:5'], os.path.join(out, 'fake2.log'))]
        ok = probe.wait(lambda: probe.status['arms']['left']['state'] == 'live', 25)
        check('W10 left live, right NO DATA', ok and probe.status['arms']['right']['state'] == 'no data')
        tf_r = probe.wait(lambda: probe.lookup('twin/mount_link', 'twin/right_fr3_link8') is not None, 5)
        p = probe.lookup('twin/mount_link', 'twin/right_fr3_link8')
        check('W10 right arm drawn at its saved HOME pose (placeholder)',
              tf_r and np.linalg.norm(p - g.flange_position('right', home['right'])) < 1e-3)
        check('W10 OFFLINE label shown for the right arm',
              probe.wait(lambda: probe.markers[('label', 1)].text == 'OFFLINE', 3))
        ok = probe.wait(lambda: probe.status['arms']['left']['state'] == 'stale', 12)
        check('W10 left data stops -> STALE', ok)
        check('W10 STALE label for the left arm', probe.wait(lambda: probe.markers[('label', 0)].text.startswith('STALE'), 3))
        check('W10 no clearance line while an arm is not live', not any(k[0] == 'clearance' for k in probe.markers))
    finally:
        for p in procs:
            stop(p)
        probe.destroy_node()
        rclpy.try_shutdown()
    n_ok = sum(ok for _, ok in RESULTS)
    print(f'\nRESULT: {n_ok}/{len(RESULTS)} passed   (logs: {out})')
    sys.exit(0 if n_ok == len(RESULTS) else 1)


if __name__ == '__main__':
    main()
