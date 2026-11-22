#!/usr/bin/env python3
"""ROS side of the PHOTOREAL twin (Isaac Sim, scripts/twin_isaac.py). READ-ONLY: subscribes only, never commands.

  python3 scripts/twin_isaac_bridge.py [--config config/digital_twin.yaml]   (system Python + ROS Humble)

Isaac Sim runs in its own Python 3.12 venv (~/isaacsim_env) and does NOT use ROS: this bridge is the only link, so the
ROS / CUDA / driver setup stays untouched and no DDS interop between ROS distributions is involved.

  isaac/cache/fr3_duo_twin.urdf   the SAME twin URDF as RViz (launch/digital_twin.launch.py twin_urdf: Duo mount, both
                                  FR3s at the duo_geometry base poses, Robotiq 2F-85) with package:// -> absolute paths
  UDP 127.0.0.1:47811             JSON {"t": wall time, "q": {joint name: position}} at --hz (default 60): measured arm
                                  joints from /twin/joint_states (the twin's own 100 Hz copy), gripper knuckle + all
                                  mimic joints resolved here (multiplier / offset from the URDF)
  isaac/cache/mesh_static.npz     reconstruction mesh from /twin/mesh/static (vertices, faces, colours), rewritten when
                                  it changes (atomic rename; Isaac reloads by mtime)
  isaac/cache/mesh_live.npz       live-object voxels from /twin/mesh/live (centres, colours, voxel size), <= 4 Hz
  isaac/cache/zed_photo_<n>.jpg + zed_photo.npz   every --photo-period s: the ZED colour image (JPEG as received),
                                  its intrinsics, its pose in twin/world (TF at the image stamp) and a depth snapshot ->
                                  Isaac projects the photo onto the mesh (sharp texture where the ZED sees the surface)
"""
import argparse
import importlib.util
import json
import os
import re
import socket
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(DEMO, 'isaac', 'cache')
PORT = 47811


def make_urdf(cfg):
    """Twin URDF exactly as RViz gets it, with absolute mesh paths. Returns (urdf text, {mimic: (parent, mult, off)})."""
    spec = importlib.util.spec_from_file_location('digital_twin_launch', os.path.join(DEMO, 'launch',
                                                                                       'digital_twin.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    urdf = mod.twin_urdf(cfg)
    urdf = re.sub(r'package://([^/]+)/', lambda m: get_package_share_directory(m.group(1)) + '/', urdf)
    mimic = {}
    for j in re.finditer(r'<joint\s+name="([^"]+)"[^>]*>(.*?)</joint>', urdf, re.S):
        mm = re.search(r'<mimic\s+([^>]*)/>', j.group(2))
        if mm:
            at = dict(re.findall(r'(\w+)="([^"]*)"', mm.group(1)))
            mimic[j.group(1)] = (at['joint'], float(at.get('multiplier', 1.0)), float(at.get('offset', 0.0)))
    return urdf, mimic


def atomic_savez(path, **arrays):
    tmp = path + '.tmp.npz'
    np.savez(tmp, **arrays)
    os.replace(tmp, path)


class Bridge(Node):
    def __init__(self, a, mimic, cfg):
        super().__init__('twin_isaac_bridge')
        self.a, self.mimic, self.cfg = a, mimic, cfg
        self.q = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.chunks, self.mesh_dirty, self.mesh_t = {}, False, 0.0
        self.live_t = 0.0
        self.create_subscription(JointState, '/twin/joint_states', self.on_js, 50)
        # the static mesh comes as a file written by twin_recon itself (reconstruction.export_npz): no marker parsing
        self.create_subscription(Marker, '/twin/mesh/live', self.on_live, 5)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.photo_n, self.photo_t, self.zinfo, self.zdepth = 0, 0.0, None, None
        self.create_subscription(CameraInfo, '/zed/zed_node/depth/camera_info', lambda m: setattr(self, 'zinfo', m), 5)
        self.create_subscription(Image, '/twin/recon_in/zed/depth', lambda m: setattr(self, 'zdepth', m), 2)
        self.create_subscription(CompressedImage, '/zed/zed_node/rgb/color/rect/image/compressed', self.on_photo, 2)
        self.create_timer(1.0 / a.hz, self.send)
        self.get_logger().info(f'twin_isaac_bridge running (read-only): UDP 127.0.0.1:{PORT}, cache {CACHE}')

    def on_js(self, m):
        for n, p in zip(m.name, m.position):     # by name (real messages have a scrambled joint order)
            self.q[n] = float(p)
        if len(m.velocity) == len(m.name):       # measured joint velocities: Isaac extrapolates to its render time
            if not hasattr(self, 'v'):
                self.v = {}
            for n, v in zip(m.name, m.velocity):
                self.v[n] = float(v)
        if m.header.frame_id != 'twin_placeholder':
            self.stamp = max(getattr(self, 'stamp', 0.0), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def send(self):
        if not self.q:
            return
        q = dict(self.q)
        v = dict(getattr(self, 'v', {}))
        for j, (parent, mult, off) in self.mimic.items():
            if parent in q:
                q[j] = mult * q[parent] + off
            if parent in v:
                v[j] = mult * v[parent]
        try:
            self.sock.sendto(json.dumps({'t': time.time(), 'stamp': getattr(self, 'stamp', None), 'q': q,
                                         'v': v}).encode(),
                             ('127.0.0.1', PORT))
        except OSError:
            pass

    @staticmethod
    def arrays(m):
        n = len(m.points)
        p = np.array([(v.x, v.y, v.z) for v in m.points], np.float32).reshape(n, 3)
        c = np.array([(v.r, v.g, v.b) for v in m.colors], np.float32).reshape(-1, 3)
        if len(c) != n:
            c = np.full((n, 3), 0.7, np.float32)
        return p, c

    def on_static(self, m):
        if m.action == Marker.DELETEALL:
            self.chunks.clear()
        else:
            self.chunks[m.id] = self.arrays(m)
        self.mesh_dirty = True

    def write_mesh(self):
        if not self.mesh_dirty or time.monotonic() - self.mesh_t < self.a.mesh_period:
            return
        self.mesh_dirty, self.mesh_t = False, time.monotonic()
        ps = [p for p, _ in self.chunks.values() if len(p)]
        cs = [c for p, c in self.chunks.values() if len(p)]
        p = np.concatenate(ps) if ps else np.zeros((0, 3), np.float32)
        c = np.concatenate(cs) if cs else np.zeros((0, 3), np.float32)
        # triangle soup -> indexed mesh (shared vertices, so Isaac can smooth-shade)
        key = np.round(p / 1e-5).astype(np.int64)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        v = np.zeros((len(uniq), 3), np.float32)
        col = np.zeros((len(uniq), 3), np.float32)
        v[inv.ravel()] = p
        col[inv.ravel()] = c
        f = inv.reshape(-1, 3).astype(np.int32)
        atomic_savez(os.path.join(CACHE, 'mesh_static.npz'), points=v, faces=f, colors=self.unshade(v, f, col))

    def unshade(self, v, f, col):
        """twin_recon bakes soft lighting into the colours for RViz (unlit markers); Isaac lights the scene itself, so
        divide the same deterministic factor out again (same mesh -> same vertex normals, same config)."""
        if not len(f):
            return col
        sys.path.insert(0, os.path.join(DEMO, 'scripts'))
        from twin_recon import DEFAULTS
        from twin_recon_core import vertex_normals
        sh = dict(DEFAULTS['shading'], **((self.cfg.get('reconstruction') or {}).get('shading') or {}))
        n = vertex_normals(v.astype(np.float64), f)
        L = np.asarray(sh['light_dir'], float)
        L /= np.linalg.norm(L)
        fac = float(sh['ambient']) + float(sh['diffuse']) * np.clip(n @ L, 0, 1) + float(sh['sky']) * n[:, 2]
        return np.clip(col / np.maximum(fac, 0.2)[:, None], 0, 1).astype(np.float32)

    def on_photo(self, m):
        now = time.monotonic()
        if now - self.photo_t < self.a.photo_period or self.zinfo is None or self.zdepth is None:
            return
        d = self.zdepth
        if abs((d.header.stamp.sec + d.header.stamp.nanosec * 1e-9) - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)) > 0.5:
            return
        try:
            t = self.tf.lookup_transform('twin/world', m.header.frame_id, rclpy.time.Time())
        except TransformException:
            return
        self.photo_t = now
        q, p = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        T = np.eye(4)
        T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        T[:3, 3] = (p.x, p.y, p.z)
        dep = np.frombuffer(bytes(d.data), np.float32).reshape(d.height, -1)[:, :d.width][::4, ::4].copy()
        k = self.zinfo.k
        self.photo_n += 1
        name = f'zed_photo_{time.time_ns() // 1000000}.jpg'     # unique across restarts (Isaac caches by file name)
        with open(os.path.join(CACHE, name + '.tmp'), 'wb') as f:
            f.write(bytes(m.data))
        os.replace(os.path.join(CACHE, name + '.tmp'), os.path.join(CACHE, name))
        atomic_savez(os.path.join(CACHE, 'zed_photo.npz'), image=name, K=np.array([k[0], k[4], k[2], k[5]]),
                     info_wh=np.array([self.zinfo.width, self.zinfo.height]), T=T, depth=dep,
                     depth_wh=np.array([d.width, d.height]))
        olds = sorted((f for f in os.listdir(CACHE) if f.startswith('zed_photo_') and f.endswith('.jpg') and f != name),
                      key=lambda f: os.path.getmtime(os.path.join(CACHE, f)))
        for old in olds[:-2]:                  # keep the current + 2 previous (Isaac may still be loading one)
            os.remove(os.path.join(CACHE, old))

    def on_live(self, m):
        now = time.monotonic()
        if now - self.live_t < 0.25:
            return
        self.live_t = now
        p, c = self.arrays(m)
        atomic_savez(os.path.join(CACHE, 'mesh_live.npz'), centers=p, colors=c, size=np.float32(m.scale.x or 0.01))


# ------------------------------------------------------------------------------------------------ browser UI
WEB = os.path.join(DEMO, 'web', 'twin_ui.html')
COMMANDS = {'scan': '/demo/scan', 'home': '/demo/home', 'stop': '/demo/stop', 'recon_reset': '/twin/recon/reset',
            'gripper_left_open': '/demo/gripper/left/open', 'gripper_left_close': '/demo/gripper/left/close',
            'gripper_right_open': '/demo/gripper/right/open', 'gripper_right_close': '/demo/gripper/right/close'}


class WebUI:
    """Status + commands for twin_ui.html. Commands go to the SAME supervisor services as the GUI buttons and the
    pedals (supervisor gating applies: SCAN only from home, HOME only when teleop is stopped, STOP always)."""

    def __init__(self, node, bind, port, stream_port):
        self.node, self.stream_port = node, stream_port
        self.status = {}
        lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        for key, topic in (('demo', '/demo/status'), ('scan', '/twin/scan_status'), ('recon', '/twin/recon/status'),
                           ('twin', '/twin/status')):
            node.create_subscription(String, topic, lambda m, k=key: self.on_status(k, m), lat)
        self.clients = {k: node.create_client(Trigger, v) for k, v in COMMANDS.items()}
        ui = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def send(self, code, body, ctype='application/json'):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self.send(200, open(WEB, 'rb').read(), 'text/html; charset=utf-8')
                elif self.path.startswith('/board.pdf'):
                    self.send(200, open(os.path.join(DEMO, 'docs', 'charuco_board_A4.pdf'), 'rb').read(), 'application/pdf')
                elif self.path.startswith('/api/status'):
                    self.send(200, json.dumps(ui.snapshot()).encode())
                else:
                    self.send(404, b'{}')

            def do_POST(self):
                if not self.path.startswith('/api/cmd'):
                    return self.send(404, b'{}')
                try:
                    cmd = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}').get('cmd')
                except ValueError:
                    return self.send(400, b'{}')
                self.send(200, json.dumps(ui.call(cmd)).encode())

        self.srv = ThreadingHTTPServer((bind, port), H)
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        node.get_logger().info(f'browser UI: http://{bind}:{port}/')

    def on_status(self, key, m):
        try:
            self.status[key] = json.loads(m.data)
        except ValueError:
            pass

    def snapshot(self):
        out = dict(self.status)
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{self.stream_port}/info', timeout=0.3) as r:
                out['stream'] = json.loads(r.read())
        except (OSError, ValueError):
            out['stream'] = None
        return out

    def call(self, cmd):
        cli = self.clients.get(cmd)
        if cli is None:
            return {'ok': False, 'message': f'unknown command {cmd}'}
        if not cli.service_is_ready():
            return {'ok': False, 'message': f'{COMMANDS[cmd]} not available (demo not running?)'}
        ev, box = threading.Event(), {}
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: (box.setdefault('r', f.result()), ev.set()))
        if not ev.wait(10.0):
            return {'ok': False, 'message': 'no answer within 10 s'}
        r = box['r']
        self.node.get_logger().info(f'browser UI: {cmd} -> {r.success} {r.message}')
        return {'ok': bool(r.success), 'message': r.message}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    ap.add_argument('--hz', type=float, default=100.0, help='joint updates to Isaac (= the twin joint state rate)')
    ap.add_argument('--mesh-period', type=float, default=2.0, help='min seconds between mesh file rewrites')
    ap.add_argument('--bind', default='127.0.0.1', help='127.0.0.1 = this computer only (the UI can move the robots)')
    ap.add_argument('--ui-port', type=int, default=8211)
    ap.add_argument('--stream-port', type=int, default=8212)
    ap.add_argument('--photo-period', type=float, default=4.0, help='seconds between ZED photo texture updates')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    os.makedirs(CACHE, exist_ok=True)
    cfg = yaml.safe_load(open(a.config)) or {}
    urdf, mimic = make_urdf(cfg)
    with open(os.path.join(CACHE, 'fr3_duo_twin.urdf'), 'w') as f:
        f.write(urdf)
    json.dump({'mimic': mimic}, open(os.path.join(CACHE, 'joints.json'), 'w'))
    rclpy.init()
    node = Bridge(a, mimic, cfg)
    WebUI(node, a.bind, a.ui_port, a.stream_port)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
