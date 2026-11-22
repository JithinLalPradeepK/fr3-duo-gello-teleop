#!/usr/bin/env python3
"""Point clouds for the DIGITAL TWIN (read-only, numpy only, no new dependencies).

  python3 scripts/twin_cloud.py --source zed       ZED point cloud  -> cropped + decimated -> /twin/cloud/zed
  python3 scripts/twin_cloud.py --source d405_1    D405 aligned depth + colour -> coloured cloud -> /twin/cloud/d405_1
  python3 scripts/twin_cloud.py --source map       ZED spatial-mapping fused cloud -> /twin/cloud/map (relay, 1-2 Hz)
  python3 scripts/twin_cloud.py --source scan      saved 3D scan (scans/latest.npz, scripts/twin_scan.py) -> /twin/cloud/scan
                                                   (latched, re-published when a new scan is saved)

LAZY like depth_view.py: the camera topics are only subscribed while the matching layer is ON in /twin/layers (latched
JSON from the twin display), so the ZED does not compute a point cloud and the D405 depth is not read while nobody looks.
Switching a layer off publishes one empty cloud (RViz would otherwise keep showing the last one).
The output keeps the camera's frame_id; scripts/twin_state.py places the camera in the twin with a static TF
(config/digital_twin.yaml -> cameras).
Status (1 Hz): /twin/cloud_status  std_msgs/String JSON {"key", "on", "hz", "points", "age_s"}.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYER = {'zed': 'zed_cloud', 'd405_1': 'wrist_clouds', 'd405_2': 'wrist_clouds', 'map': 'map', 'scan': 'scan'}
SCAN_FILE = os.path.join(DEMO, 'scans', 'latest.npz')
OUT_FIELDS = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
NP_TYPE = {PointField.FLOAT32: '<f4', PointField.FLOAT64: '<f8', PointField.UINT32: '<u4', PointField.INT32: '<i4',
           PointField.UINT8: 'u1', PointField.UINT16: '<u2', PointField.INT16: '<i2', PointField.INT8: 'i1'}


def make_cloud(stamp, frame, xyz, rgb_f32):
    """xyz (n,3) float32, rgb_f32 (n,) float32 (packed 0x00RRGGBB) -> PointCloud2 (unorganised)."""
    n = len(xyz)
    buf = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
    if n:
        buf['x'], buf['y'], buf['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        buf['rgb'] = rgb_f32
    m = PointCloud2()
    m.header.stamp, m.header.frame_id = stamp, frame
    m.height, m.width = 1, n
    m.fields, m.is_bigendian, m.point_step = OUT_FIELDS, False, 16
    m.row_step, m.is_dense = 16 * n, True
    m.data = buf.tobytes()
    return m


def pack_rgb(r, g, b):
    return ((r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)).view(np.float32)


def cloud_view(msg):
    """Structured numpy view of a PointCloud2 (any field layout), shape (height, width)."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.count == 1 and f.datatype in NP_TYPE:
            names.append(f.name)
            formats.append(NP_TYPE[f.datatype])
            offsets.append(f.offset)
    dt = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})
    arr = np.frombuffer(memoryview(msg.data), dtype=dt, count=msg.width * msg.height)
    return arr.reshape(msg.height, msg.width)


def reduce_zed(msg, stride, max_range):
    """ZED cloud (organised, x/y/z/rgb or rgba) -> every stride-th point within max_range [m] (camera frame)."""
    a = cloud_view(msg)
    if a.ndim == 2 and a.shape[0] > 1:
        a = a[::stride, ::stride]
    a = a.ravel()
    xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float32, copy=False)
    d2 = np.einsum('ij,ij->i', xyz, xyz)
    keep = np.isfinite(d2) & (d2 < max_range * max_range) & (d2 > 1e-6)
    col = 'rgb' if 'rgb' in a.dtype.names else ('rgba' if 'rgba' in a.dtype.names else None)
    rgb = a[col][keep].astype(np.float32, copy=False) if col else np.full(int(keep.sum()), pack_rgb(
        np.array([200]), np.array([210]), np.array([215]))[0], np.float32)
    if col and a.dtype[col] != np.float32:              # rgba as uint32 -> view as float32 (rviz RGB8 reads the bits)
        rgb = a[col][keep].astype(np.uint32).view(np.float32)
    return xyz[keep], rgb


def deproject(depth, info, stride, max_range, color=None):
    """Aligned depth image (16UC1 mm or 32FC1 m) + CameraInfo (+ rgb8/bgr8 colour of the same size) -> (xyz, rgb)."""
    if depth.encoding in ('16UC1', 'mono16'):
        d = np.frombuffer(memoryview(depth.data), np.uint16).reshape(depth.height, -1)[:, :depth.width]
        z = d[::stride, ::stride].astype(np.float32) * 0.001
    elif depth.encoding == '32FC1':
        d = np.frombuffer(memoryview(depth.data), np.float32).reshape(depth.height, -1)[:, :depth.width]
        z = d[::stride, ::stride]
    else:
        return None
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if fx <= 0 or fy <= 0:
        return None
    vs, us = np.mgrid[0:depth.height:stride, 0:depth.width:stride]
    keep = np.isfinite(z) & (z > 0.03) & (z < max_range)
    z, us, vs = z[keep], us[keep], vs[keep]
    xyz = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1).astype(np.float32)
    if color is not None and color.height == depth.height and color.width == depth.width and \
            color.encoding in ('rgb8', 'bgr8'):
        c = np.frombuffer(memoryview(color.data), np.uint8).reshape(color.height, -1)[:, :color.width * 3]
        c = c.reshape(color.height, color.width, 3)[vs, us]
        r, g, b = (c[:, 0], c[:, 1], c[:, 2]) if color.encoding == 'rgb8' else (c[:, 2], c[:, 1], c[:, 0])
        rgb = pack_rgb(r, g, b)
    else:                                       # no colour: shade by distance (Franka Grey -> off-white)
        t = np.clip(z / max_range, 0, 1)
        rgb = pack_rgb((212 - 100 * t).astype(np.uint8), (219 - 104 * t).astype(np.uint8),
                       (220 - 92 * t).astype(np.uint8))
    return xyz, rgb


class TwinCloud(Node):
    def __init__(self, a, cfg):
        super().__init__(f'twin_cloud_{a.source}')
        self.a, self.key = a, a.source
        cams = cfg.get('cameras', {})
        rates = cfg.get('rates', {})
        self.layer = LAYER[self.key]
        self.on = bool(cfg.get('layers', {}).get(self.layer, False)) if a.force is None else a.force
        self.max_range = float(cams.get('zed' if self.key == 'map' else self.key, {}).get('max_range_m', 2.5))
        hz = a.hz or float(rates.get('zed_cloud_hz' if self.key in ('zed', 'map') else 'd405_cloud_hz', 6.0))
        if self.key == 'map':
            hz = min(hz, 2.0)
        self.period = 1.0 / max(hz, 0.1)
        rel = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        if self.key == 'scan':           # the saved 3D scan: latched, so RViz (re)starts with it
            rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_mtime = 0.0
        self.pub = self.create_publisher(PointCloud2, f'/twin/cloud/{self.key}', rel)
        self.stat_pub = self.create_publisher(String, '/twin/cloud_status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/twin/layers', self.on_layers, latched)
        self.subs, self.info, self.color = [], None, None
        self.last_t, self.count, self.points, self.out_t, self.hz = 0.0, 0, 0, 0.0, 0.0
        self.win_t = time.monotonic()
        self.apply()
        self.create_timer(1.0, self.status)
        if self.key == 'scan':
            self.create_timer(2.0, self.check_scan)

    # -- layer switching (lazy subscriptions)
    def on_layers(self, m):
        try:
            on = bool(json.loads(m.data).get(self.layer, self.on))
        except (ValueError, AttributeError):
            return
        if self.a.force is None and on != self.on:
            self.on = on
            self.apply()

    def apply(self):
        for s in self.subs:
            self.destroy_subscription(s)
        self.subs, self.info, self.color = [], None, None
        if not self.on:
            self.pub.publish(make_cloud(self.get_clock().now().to_msg(), 'twin/world',
                                        np.zeros((0, 3), np.float32), np.zeros(0, np.float32)))
            self.get_logger().info(f'{self.key}: layer OFF (camera not read)')
            return
        sd = qos_profile_sensor_data
        if self.key == 'scan':
            self.scan_mtime = 0.0
            self.check_scan()
            return
        if self.key == 'zed':
            self.subs.append(self.create_subscription(PointCloud2, self.a.input or '/zed/zed_node/point_cloud/cloud_registered',
                                                      self.on_zed, sd))
        elif self.key == 'map':
            self.subs.append(self.create_subscription(PointCloud2, self.a.input or '/zed/zed_node/mapping/fused_cloud',
                                                      self.on_map, sd))
        else:
            ns = f'/camera/{self.key}'
            self.subs += [
                self.create_subscription(CameraInfo, f'{ns}/color/camera_info', self.on_info, sd),
                self.create_subscription(Image, f'{ns}/color/image_raw', self.on_color, sd),
                self.create_subscription(Image, self.a.input or f'{ns}/aligned_depth_to_color/image_raw', self.on_depth, sd)]
        self.get_logger().info(f'{self.key}: layer ON')

    def check_scan(self):
        """Publish scans/latest.npz (fused 3D model in twin/world) whenever it changes; nothing if there is none."""
        if not self.on:
            return
        try:
            mt = os.path.getmtime(SCAN_FILE)
        except OSError:
            return
        if mt == self.scan_mtime:
            return
        try:
            z = np.load(SCAN_FILE)
            xyz, rgb = z['xyz'].astype(np.float32), z['rgb'].astype(np.uint32).view(np.float32)
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().warn(f'scan file: {e}')
            return
        self.scan_mtime = mt
        self.pub.publish(make_cloud(self.get_clock().now().to_msg(), 'twin/world', xyz, rgb))
        self.count += 1
        self.points, self.out_t = len(xyz), time.monotonic()
        self.get_logger().info(f'3D scan published: {len(xyz)} points ({SCAN_FILE})')

    def due(self):
        now = time.monotonic()
        if now - self.last_t < self.period * 0.9:
            return False
        self.last_t = now
        return True

    def emit(self, header, xyz, rgb):
        self.pub.publish(make_cloud(header.stamp, header.frame_id, xyz, rgb))
        self.count += 1
        self.points, self.out_t = len(xyz), time.monotonic()

    # -- sources
    def on_zed(self, m):
        if self.due():
            xyz, rgb = reduce_zed(m, self.a.stride, self.max_range)
            self.emit(m.header, xyz, rgb)

    def on_map(self, m):
        if self.due():
            self.pub.publish(m)
            self.count += 1
            self.points, self.out_t = m.width * m.height, time.monotonic()

    def on_info(self, m):
        self.info = m

    def on_color(self, m):
        self.color = m

    def on_depth(self, m):
        if self.info is None or not self.due():
            return
        r = deproject(m, self.info, self.a.stride, self.max_range, self.color)
        if r is not None:
            self.emit(m.header, *r)

    def status(self):
        now = time.monotonic()
        dt, self.win_t = now - self.win_t, now
        self.hz, self.count = self.count / max(dt, 1e-3), 0
        self.stat_pub.publish(String(data=json.dumps({
            'key': self.key, 'on': self.on, 'hz': round(self.hz, 1), 'points': self.points,
            'age_s': round(now - self.out_t, 2) if self.out_t else None})))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True, choices=sorted(LAYER))
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    ap.add_argument('--input', default='', help='override the input topic (tests)')
    ap.add_argument('--stride', type=int, default=0, help='use every n-th pixel (default zed 1, d405 4)')
    ap.add_argument('--hz', type=float, default=0.0, help='max output rate (default from the config)')
    ap.add_argument('--force', choices=('on', 'off'), help='ignore /twin/layers (tests)')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    a.stride = a.stride or (1 if a.source == 'zed' else 4)      # the ZED cloud is already small (measured)
    a.force = None if a.force is None else a.force == 'on'
    with open(a.config) as f:
        cfg = yaml.safe_load(f) or {}
    rclpy.init()
    node = TwinCloud(a, cfg)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
