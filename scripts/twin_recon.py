#!/usr/bin/env python3
"""DIGITAL TWIN reconstruction node: solid coloured mesh of the workspace + live moving objects. READ-ONLY (subscribes
to cameras, TF and joint states; never commands a robot). Core maths in scripts/twin_recon_core.py.

  python3 scripts/twin_recon.py [--config config/digital_twin.yaml]      (started by launch/digital_twin.launch.py)

IN   per camera (config reconstruction.cameras): depth (C++ throttled copy, launch/digital_twin.launch.py), camera_info,
     compressed colour (exact stamp match); TF twin/world <- depth optical frame AT THE DEPTH STAMP (ZED: static twin TF
     + the wrapper's URDF; D405: twin FR3 FK via robot_state_publisher + calibrated mount); /twin/joint_states (robot
     filter: duo_geometry capsules at the frame's joint positions).
OUT  /twin/mesh/static  visualization_msgs/Marker TRIANGLE_LIST chunks (latched, ns recon_static) - the persistent map
     /twin/mesh/live    Marker CUBE_LIST (ns recon_live) - things in space the map knows to be empty (hands, objects)
     /twin/recon/status std_msgs/String JSON (latched), /twin/cloud_status key "mesh" (GUI telemetry bar)
SRV  /twin/recon/save, /twin/recon/load, /twin/recon/reset (re-reads voxel size / bounds)  std_srvs/Trigger
     /twin/recon/mapping  std_srvs/SetBool (false = freeze the static map; live objects keep updating)
LAYERS  "mesh" (static map) and "live_objects" in /twin/layers. Both off -> camera topics unsubscribed, markers cleared.

ISOLATION: own process, nice +10, numpy/BLAS limited to 1 thread, all heavy work in one worker thread that drops stale
frames (it never queues), so it cannot slow the robot stack or the GUI. Moving wrist cameras are only integrated when
their arm is slow (motion blur / stamp skew) and only if calibrated (config).
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
from collections import deque

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('MKL_NUM_THREADS', '1')

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from tf2_ros import Buffer, TransformException, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import DuoGeometry  # noqa: E402
from twin_recon_core import (TSDF, RobotFilter, backproject, live_points, marker_bytes, shade,  # noqa: E402
                             surface_nets, write_ply)

ARMS = ('left', 'right')
WORLD = 'twin/world'
DEFAULTS = {
    'voxel_m': 0.01, 'bounds_min': [-0.4, -1.0, -0.3], 'bounds_max': [1.4, 1.0, 1.2], 'trunc_vox': 4.0,
    'max_weight': 30.0, 'min_weight': 3.0, 'smooth_iters': 2, 'min_component_tris': 80, 'mesh_period_s': 1.0,
    'robot_pad_m': 0.025, 'chunk_tris': 20000, 'max_chunks': 16, 'map_file': 'scans/recon_map.npz', 'autoload': True,
    'live': {'min_weight': 8.0, 'free_tsdf': 0.3, 'nb_tsdf': 0.0, 'hold_s': 1.5, 'stride': 2},
    'shading': {'light_dir': [0.4, 0.3, 1.0], 'ambient': 0.42, 'diffuse': 0.62, 'sky': 0.08},
    'cameras': {},
}


def merged(cfg):
    r = dict(DEFAULTS)
    r.update(cfg.get('reconstruction') or {})
    r['live'] = dict(DEFAULTS['live'], **((cfg.get('reconstruction') or {}).get('live') or {}))
    r['shading'] = dict(DEFAULTS['shading'], **((cfg.get('reconstruction') or {}).get('shading') or {}))
    return r


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def depth_array(m, s=1):
    """Depth image -> metres, every s-th pixel (subsampled BEFORE copying: the ZED frame is 8 MB)."""
    if m.encoding in ('16UC1', 'mono16'):
        d = np.frombuffer(memoryview(m.data), np.uint16).reshape(m.height, -1)[:, :m.width][::s, ::s].astype(np.float32)
        d *= 0.001
    elif m.encoding == '32FC1':
        d = np.frombuffer(memoryview(m.data), np.float32).reshape(m.height, -1)[:, :m.width][::s, ::s].copy()
    else:
        return None
    d[~np.isfinite(d)] = 0
    return d


class Cam:
    def __init__(self, key, c):
        self.key, self.c = key, c
        self.info = None
        self.depth = None              # newest unprocessed depth msg (older ones are dropped)
        self.colors = deque(maxlen=12)
        self.subs = []
        self.n_in = self.n_done = 0
        self.state, self.t_done = 'waiting for camera', 0.0
        self.live = (np.zeros((0, 3)), np.zeros((0, 3), np.uint8), 0.0)


class TwinRecon(Node):
    def __init__(self, cfg_path):
        super().__init__('twin_recon')
        self.cfg_path = cfg_path
        self.cfg = yaml.safe_load(open(cfg_path)) or {}
        self.r = merged(self.cfg)
        self.lock = threading.Lock()          # volume
        self.geo = DuoGeometry()
        self.rf = RobotFilter(self.geo, self.r['robot_pad_m'])
        self.vol = self.new_volume()
        self.map_file = os.path.join(DEMO, self.r['map_file'])
        if self.r['autoload'] and os.path.exists(self.map_file):
            self.load_map()
        self.mapping = True
        self.layers = {'mesh': True, 'live_objects': True}
        self.layers.update({k: bool(v) for k, v in (self.cfg.get('layers') or {}).items() if k in self.layers})
        self.jb = {a: deque(maxlen=400) for a in ARMS}        # (stamp, q) of /twin/joint_states (100 Hz -> 4 s)
        self.cams = {k: Cam(k, c) for k, c in (self.r['cameras'] or {}).items() if c.get('enabled', True)}
        self.mesh_info = {'triangles': 0, 'ms': 0, 't': 0.0}
        self.n_chunks_pub = 0
        self.dirty = True
        self.int_times = deque(maxlen=50)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        mesh_qos = QoSProfile(depth=int(self.r['max_chunks']) + 1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.static_pub = self.create_publisher(Marker, '/twin/mesh/static', mesh_qos)
        self.live_pub = self.create_publisher(Marker, '/twin/mesh/live', 5)
        self.status_pub = self.create_publisher(String, '/twin/recon/status', latched)
        self.cloud_status_pub = self.create_publisher(String, '/twin/cloud_status', 10)
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tfl = TransformListener(self.tf, self, spin_thread=False)
        self.create_subscription(JointState, '/twin/joint_states', self.on_js, 50)
        self.create_subscription(String, '/twin/layers', self.on_layers, latched)
        self.create_service(Trigger, '/twin/recon/save', self.srv_save)
        self.create_service(Trigger, '/twin/recon/load', self.srv_load)
        self.create_service(Trigger, '/twin/recon/reset', self.srv_reset)
        self.create_service(SetBool, '/twin/recon/mapping', self.srv_mapping)
        self.pending_rec = None
        self.create_subscription(String, '/twin/recon/cmd', self.on_cmd, 5)
        self.clear_markers('recon_static')    # a restarted node must not leave the old node's extra chunks behind
        self.clear_markers('recon_live')
        self.apply_layers()
        self.create_timer(1.0, self.publish_status)
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.work, name='recon', daemon=True)
        self.worker.start()
        self.get_logger().info(f'twin_recon running (read-only): grid {self.vol.shape} at {self.vol.voxel * 1e3:.0f} mm '
                               f'({self.vol.nbytes / 1e6:.0f} MB), cameras {sorted(self.cams)}')

    # ------------------------------------------------------------------ volume management
    def new_volume(self):
        r = self.r
        return TSDF(r['bounds_min'], r['bounds_max'], float(r['voxel_m']), float(r['trunc_vox']), float(r['max_weight']))

    def load_map(self):
        try:
            v = TSDF.load(self.map_file)
        except (OSError, KeyError, ValueError) as e:
            return False, f'load failed: {e}'
        with self.lock:
            self.vol = v
            self.dirty = True
        msg = f'map loaded: {self.map_file} ({v.shape} at {v.voxel * 1e3:.0f} mm)'
        if abs(v.voxel - float(self.r['voxel_m'])) > 1e-9:
            msg += f' - NOTE config voxel_m {self.r["voxel_m"]} differs (reset to use it)'
        self.get_logger().info(msg)
        return True, msg

    def srv_save(self, req, res):
        with self.lock:
            os.makedirs(os.path.dirname(self.map_file), exist_ok=True)
            tmp = os.path.splitext(self.map_file)[0] + '.tmp.npz'
            self.vol.save(tmp)                                   # atomic: readers poll the file's mtime
            os.replace(tmp, self.map_file)
            v, t, c = surface_nets(self.vol, self.r['min_weight'], self.r['smooth_iters'], self.r['min_component_tris'])
        ply = os.path.splitext(self.map_file)[0] + '.ply'
        write_ply(ply, v, t, c)
        res.success, res.message = True, f'saved {self.map_file} + {ply} ({len(t)} triangles)'
        self.get_logger().info(res.message)
        return res

    def srv_load(self, req, res):
        if not os.path.exists(self.map_file):
            res.success, res.message = False, f'no saved map ({self.map_file})'
        else:
            res.success, res.message = self.load_map()
        return res

    def srv_reset(self, req, res):
        try:
            self.cfg = yaml.safe_load(open(self.cfg_path)) or {}
            self.r = merged(self.cfg)
            self.rf.pad = float(self.r['robot_pad_m'])
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warn(f'config: {e} (old settings kept)')
        with self.lock:
            self.vol = self.new_volume()
            self.dirty = True
        res.success, res.message = True, f'map reset: {self.vol.shape} at {self.vol.voxel * 1e3:.0f} mm'
        self.get_logger().info(res.message)
        return res

    def srv_mapping(self, req, res):
        self.mapping = bool(req.data)
        res.success, res.message = True, 'static map ' + ('updating' if self.mapping else 'FROZEN (live objects only)')
        self.get_logger().info(res.message)
        return res

    def on_cmd(self, m):
        """/twin/recon/cmd JSON: {"integrate": "<scans/rec_... dir>"} -> fuse a recorded 3D SCAN (wrist cameras at their
        calibrated mounts, poses from FK of the recorded joint positions) into the map (worker thread)."""
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if d.get('save'):
            self.pending_save = True
        if d.get('integrate'):
            self.pending_rec = d['integrate']
            self.get_logger().info(f'recording queued for integration: {self.pending_rec}')

    def integrate_recording(self, path):
        sys.path.insert(0, os.path.join(DEMO, 'scripts'))
        from duo_geometry import rpy_matrix
        from twin_scan import CAMS, load_recording
        rec = load_recording(path)
        cams = self.cfg.get('cameras') or {}
        opt = np.eye(4)
        opt[:3, :3] = rpy_matrix(-np.pi / 2, 0, -np.pi / 2)
        n = 0
        for cam, arm in CAMS.items():
            c = cams.get(cam) or {}
            if not c.get('calibrated'):
                self.get_logger().info(f'{cam}: not calibrated - recording frames skipped')
                continue
            X = np.eye(4)
            X[:3, :3] = rpy_matrix(*[float(v) for v in c['rpy']])
            X[:3, 3] = [float(v) for v in c['xyz']]
            X = X @ opt
            rc = (self.r['cameras'] or {}).get(cam) or {}
            for f in rec['frames']:
                if cam not in f:
                    continue
                d = f[cam]['depth'].astype(np.float32) * 0.001
                K = tuple(f[cam]['K'])
                T = self.geo.link_frames(arm, np.asarray(f['q'][arm], float))[8] @ X
                pw = backproject(d, K, T)
                valid = d.ravel() > 0
                pw[~valid] = np.nan
                robot = self.rf.mask(pw, self.rf.capsules({a: np.asarray(f['q'][a], float) for a in ARMS}))
                rgb = np.asarray(f[cam]['color'], np.uint8)
                if rgb.shape[:2] != d.shape:
                    rgb = cv2.resize(rgb, (d.shape[1], d.shape[0]), interpolation=cv2.INTER_AREA)
                with self.lock:
                    self.vol.integrate(d, K, T, rgb, free_only=robot.reshape(d.shape), near=float(rc.get('near', 0.12)),
                                       far=float(rc.get('far', 0.8)), weight=float(rc.get('weight', 3.0)))
                    self.dirty = True
                n += 1
        self.get_logger().info(f'recording {path}: {n} wrist frames integrated')
        return n

    # ------------------------------------------------------------------ inputs
    def on_js(self, m):
        if not m.name:
            return
        arm = 'left' if m.name[0].startswith('left_') else 'right' if m.name[0].startswith('right_') else None
        if arm is None:
            return
        pos = dict(zip(m.name, m.position))              # real messages have a SCRAMBLED joint order: by name
        q = [pos.get(f'{arm}_fr3_joint{i}') for i in range(1, 8)]
        if None in q:
            return
        st = stamp_s(m.header.stamp) or time.time()
        self.jb[arm].append((st, np.array(q, float)))

    def q_at(self, arm, t):
        b = list(self.jb[arm])
        if not b:
            return None, 0.0
        ts = np.array([s for s, _ in b])
        i = int(np.argmin(np.abs(ts - t)))
        lo, hi = max(i - 3, 0), min(i + 3, len(b) - 1)
        speed = 0.0
        if hi > lo and ts[hi] > ts[lo]:
            speed = float(np.abs(b[hi][1] - b[lo][1]).max() / (ts[hi] - ts[lo]))
        return b[i][1], speed

    def on_layers(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        new = {k: bool(d.get(k, v)) for k, v in self.layers.items()}
        if new != self.layers:
            self.layers = new
            self.apply_layers()

    def active(self):
        return self.layers['mesh'] or self.layers['live_objects']

    def apply_layers(self):
        for cam in self.cams.values():
            for s in cam.subs:
                self.destroy_subscription(s)
            cam.subs, cam.depth = [], None
            if not self.active():
                cam.state = 'layer off'
                continue
            c, sd = cam.c, qos_profile_sensor_data
            cam.subs = [self.create_subscription(CameraInfo, c['info'], lambda m, cam=cam: setattr(cam, 'info', m), sd),
                        self.create_subscription(Image, c['depth'], lambda m, cam=cam: self.on_depth(cam, m), sd)]
            if c.get('color'):
                cam.subs.append(self.create_subscription(CompressedImage, c['color'],
                                                         lambda m, cam=cam: cam.colors.append(m), sd))
        if not self.layers['mesh']:
            self.clear_markers('recon_static')
        else:
            self.dirty = True
        if not self.layers['live_objects']:
            self.clear_markers('recon_live')
        self.get_logger().info(f'layers: {self.layers}')

    def on_depth(self, cam, m):
        cam.depth = m                     # only the newest frame is kept (the worker never queues)
        cam.n_in += 1

    # ------------------------------------------------------------------ worker
    def work(self):
        try:
            os.nice(10)
        except OSError:
            pass
        last_mesh = 0.0
        while not self.stop.is_set():
            busy = False
            if getattr(self, 'pending_save', False):
                self.pending_save = False
                self.srv_save(None, Trigger.Response())
            if self.pending_rec:
                path, self.pending_rec = self.pending_rec, None
                try:
                    self.rec_info = {'path': path, 'frames': self.integrate_recording(path), 't': time.time()}
                except Exception as e:      # noqa: BLE001
                    self.get_logger().warn(f'integrating {path}: {e}')
                    self.rec_info = {'path': path, 'error': str(e), 't': time.time()}
            for cam in self.cams.values():
                m = cam.depth
                if m is None or not self.active():
                    continue
                if time.time() - stamp_s(m.header.stamp) < 0.04:       # give TF / colour a moment to arrive
                    continue
                cam.depth = None
                busy = True
                try:
                    self.process(cam, m)
                except Exception as e:        # a bad frame must never kill the twin
                    cam.state = f'error: {e}'
                    self.get_logger().warn(f'{cam.key}: {e}', throttle_duration_sec=5.0)
            now = time.monotonic()
            if self.active() and now - last_mesh > float(self.r['mesh_period_s']):
                last_mesh = now
                try:
                    self.publish_mesh()
                    self.publish_live()
                except Exception as e:
                    self.get_logger().warn(f'mesh: {e}', throttle_duration_sec=5.0)
            if not busy:
                time.sleep(0.01)

    def process(self, cam, m):
        c = cam.c
        t0 = time.perf_counter()
        if cam.info is None:
            cam.state = 'no camera_info'
            return
        if c.get('require_calibrated') and not ((self.cfg.get('cameras') or {}).get(cam.key) or {}).get('calibrated'):
            cam.state = 'skipped: mount not calibrated (run 3D SCAN)'
            return
        st = stamp_s(m.header.stamp)
        stamp = Time(seconds=st) if c.get('moving') else Time()
        try:
            T = tf_matrix(self.tf.lookup_transform(WORLD, m.header.frame_id, stamp, Duration(seconds=0.1)))
        except TransformException as e:
            cam.state = f'no TF {WORLD} <- {m.header.frame_id}'
            self.get_logger().info(f'{cam.key}: {e}', throttle_duration_sec=10.0)
            return
        qs, speed = {}, 0.0
        for arm in ARMS:
            qs[arm], sp = self.q_at(arm, st)
            if c.get('arm') == arm:
                speed = sp
        if c.get('moving') and speed > float(c.get('max_joint_speed', 0.4)):
            cam.state = f'paused: arm moving ({speed:.2f} rad/s)'
            return
        s = int(c.get('stride', 1))
        d = depth_array(m, s)
        if d is None:
            cam.state = f'unsupported depth encoding {m.encoding}'
            return
        k = cam.info.k
        sx, sy = m.width / max(cam.info.width, 1), m.height / max(cam.info.height, 1)   # info may be another size
        K = (k[0] * sx / s, k[4] * sy / s, (k[2] * sx + 0.5) / s - 0.5, (k[5] * sy + 0.5) / s - 0.5)
        rgb = self.color_for(cam, st, d.shape)
        pw = backproject(d, K, T)
        valid = d.ravel() > 0
        pw[~valid] = np.nan
        caps = self.rf.capsules(qs)
        # robot mask on a 2x coarser grid (the pad covers the step), nearest upsampled
        h, w = d.shape
        coarse = self.rf.mask(pw.reshape(h, w, 3)[::2, ::2].reshape(-1, 3), caps).reshape((h + 1) // 2, (w + 1) // 2)
        robot = np.repeat(np.repeat(coarse, 2, 0), 2, 1)[:h, :w].ravel() & valid
        near, far = float(c.get('near', 0.1)), float(c.get('far', 2.5))
        with self.lock:
            if self.mapping:
                self.vol.integrate(d, K, T, rgb, free_only=robot.reshape(d.shape), near=near, far=far,
                                   weight=float(c.get('weight', 1.0)))
                self.dirty = True
            if self.layers['live_objects']:
                ls = int(self.r['live'].get('stride', 2))
                sel = (valid & ~robot).reshape(d.shape)[::ls, ::ls].ravel()
                p2 = pw.reshape(d.shape + (3,))[::ls, ::ls].reshape(-1, 3)[sel]
                c2 = (rgb if rgb is not None else np.full(d.shape + (3,), 200, np.uint8))[::ls, ::ls].reshape(-1, 3)[sel]
                lv = self.r['live']
                lp, lc = live_points(self.vol, p2, c2, float(lv['min_weight']), float(lv['free_tsdf']),
                                     float(lv['nb_tsdf']))
                cam.live = (lp, lc, time.monotonic())
        cam.n_done += 1
        cam.t_done = time.monotonic()
        self.int_times.append(time.perf_counter() - t0)
        cam.state = ('ok' + ('' if rgb is not None else ' (no colour)')) if self.active() else 'layer off'

    def color_for(self, cam, st, shape):
        best, bd = None, float(cam.c.get('color_tol_s', 0.06))
        for cm in list(cam.colors):
            dt = abs(stamp_s(cm.header.stamp) - st)
            if dt <= bd:
                best, bd = cm, dt
        if best is None:
            return None
        img = cv2.imdecode(np.frombuffer(bytes(best.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        if img.shape[:2] != shape:
            img = cv2.resize(img, (shape[1], shape[0]), interpolation=cv2.INTER_AREA)
        return img[:, :, ::-1].copy()          # BGR -> RGB

    # ------------------------------------------------------------------ output
    def header_marker(self, ns, i, typ):
        mk = Marker()
        mk.header.frame_id = WORLD
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = ns, i, typ, Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 1.0
        mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
        return mk

    def clear_markers(self, ns):
        mk = Marker()
        mk.header.frame_id, mk.ns, mk.action = WORLD, ns, Marker.DELETEALL
        (self.static_pub if ns == 'recon_static' else self.live_pub).publish(mk)
        if ns == 'recon_static':
            self.n_chunks_pub = 0

    def publish_mesh(self):
        if not self.layers['mesh'] or not self.dirty:
            return
        t0 = time.perf_counter()
        with self.lock:
            caps = self.rf.capsules({a: self.q_at(a, time.time())[0] for a in ARMS})
            self.vol.clear_capsules(*caps)             # nothing static can be where a robot is now
            v, t, c = surface_nets(self.vol, float(self.r['min_weight']), int(self.r['smooth_iters']),
                                   int(self.r['min_component_tris']))
            self.dirty = False
        n_chunk, max_chunks = int(self.r['chunk_tris']), int(self.r['max_chunks'])
        if len(t) > n_chunk * max_chunks:              # keep the biggest surfaces readable: drop the excess evenly
            t = t[np.linspace(0, len(t) - 1, n_chunk * max_chunks).astype(int)]
        exp = self.r.get('export_npz')
        if exp and len(t):
            path = os.path.join(DEMO, exp)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            np.savez(path + '.tmp.npz', points=v.astype(np.float32), faces=t.astype(np.int32),
                     colors=(c.astype(np.float32) / 255.0))
            os.replace(path + '.tmp.npz', path)
        sh = dict(DEFAULTS['shading'], **(self.r.get('shading') or {}))
        c = shade(v, t, c, sh['light_dir'], float(sh['ambient']), float(sh['diffuse']), float(sh['sky']))
        pts = v[t].reshape(-1, 3)
        cols = np.concatenate([c[t].reshape(-1, 3).astype(np.float32) / 255.0,
                               np.ones((len(pts), 1), np.float32)], 1)
        n = (len(t) + n_chunk - 1) // n_chunk
        for i in range(max(n, self.n_chunks_pub)):
            mk = self.header_marker('recon_static', i, Marker.TRIANGLE_LIST)
            a, b = i * n_chunk * 3, (i + 1) * n_chunk * 3
            self.static_pub.publish(marker_bytes(mk, pts[a:b], cols[a:b]))   # empty chunk = deletes an old one
        self.n_chunks_pub = n
        self.mesh_info = {'triangles': int(len(t)), 'ms': round((time.perf_counter() - t0) * 1e3), 't': time.monotonic()}

    def publish_live(self):
        if not self.layers['live_objects']:
            return
        now = time.monotonic()
        P, C = [], []
        for cam in self.cams.values():
            lp, lc, t = cam.live
            if now - t < float(self.r['live']['hold_s']) and len(lp):
                P.append(lp)
                C.append(lc)
        mk = self.header_marker('recon_live', 0, Marker.CUBE_LIST)
        s = self.vol.voxel
        mk.scale.x = mk.scale.y = mk.scale.z = s
        if P:
            P, C = np.concatenate(P), np.concatenate(C)
            cols = np.concatenate([C.astype(np.float32) / 255.0, np.ones((len(C), 1), np.float32)], 1)
        else:
            P, cols = np.zeros((0, 3)), np.zeros((0, 4), np.float32)
        self.live_pub.publish(marker_bytes(mk, P, cols))
        self.live_n = len(P)

    def extrinsics(self, cfg):
        cams = cfg.get('cameras') or {}
        return {k: ((cams.get(k) or {}).get('xyz'), (cams.get(k) or {}).get('rpy'),
                    (cams.get(k) or {}).get('calibrated')) for k in self.cams}

    def watch_config(self):
        """A changed camera pose would fuse new frames at the new pose ON TOP of the old ones (seen 2026-11-22: a wrong
        scan refit of the ZED pose duplicated the trays 25 cm away and baked robot parts in). -> reset the map."""
        try:
            mt = os.path.getmtime(self.cfg_path)
            if mt == getattr(self, 'cfg_mtime', None):
                return
            first = not hasattr(self, 'cfg_mtime')
            self.cfg_mtime = mt
            cfg = yaml.safe_load(open(self.cfg_path)) or {}
        except (OSError, yaml.YAMLError):
            return
        ext = self.extrinsics(cfg)
        if not first and ext != getattr(self, 'ext', ext):
            changed = [k for k in ext if ext[k] != self.ext.get(k)]
            self.cfg = cfg
            with self.lock:
                self.vol = self.new_volume()
                self.dirty = True
            self.get_logger().warn(f'camera extrinsics changed ({", ".join(changed)}) - map RESET (old frames were fused '
                                   'at the old pose)')
        self.cfg, self.ext = cfg, ext
        old = self.r
        self.r = merged(cfg)                      # live: smoothing, specks, live objects, shading, mesh period ...
        for k in ('voxel_m', 'bounds_min', 'bounds_max', 'trunc_vox', 'max_weight'):
            self.r[k] = old[k]                    # ... but the GRID changes only with /twin/recon/reset

    def publish_status(self):
        self.watch_config()
        now = time.monotonic()
        cams = {}
        for k, cam in self.cams.items():
            cams[k] = {'state': cam.state, 'in': cam.n_in, 'fused': cam.n_done,
                       'age_s': round(now - cam.t_done, 1) if cam.t_done else None}
            cam.n_in = cam.n_done = 0
        it = list(self.int_times)
        st = {'layers': self.layers, 'mapping': self.mapping, 'voxel_m': self.vol.voxel, 'grid': list(self.vol.shape),
              'frames': self.vol.frames, 'triangles': self.mesh_info['triangles'], 'mesh_ms': self.mesh_info['ms'],
              'live_voxels': getattr(self, 'live_n', 0), 'frame_ms': round(1e3 * float(np.median(it))) if it else None,
              'cameras': cams, 'map_file': self.map_file, 'recording': getattr(self, 'rec_info', None)}
        self.status_pub.publish(String(data=json.dumps(st)))
        fused = sum(c['fused'] for c in cams.values())
        self.cloud_status_pub.publish(String(data=json.dumps({
            'key': 'mesh', 'on': self.active(), 'hz': float(fused), 'points': self.mesh_info['triangles'],
            'age_s': round(now - self.mesh_info['t'], 2) if self.mesh_info['t'] else None})))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # own SIGINT/SIGTERM handling (rclpy's default handler shuts the context while callbacks still run -> traceback)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TwinRecon(a.config)
    quit_ = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: quit_.set())
    signal.signal(signal.SIGTERM, lambda *_: quit_.set())
    try:
        while rclpy.ok() and not quit_.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop.set()
        node.worker.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
