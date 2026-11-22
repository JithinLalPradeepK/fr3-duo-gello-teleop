#!/usr/bin/env python3
"""FAKE ZED + D405 depth/colour for the reconstruction sim test (isolated ROS domain only; no camera, no robot).

Scene = test/twin_recon_test.py (table z = 0, box, sphere, both FR3s at their saved HOME as capsules). Rendered from the
ZED pose of --config (cameras.zed) and from the LEFT wrist D405 (FK at HOME @ cameras.d405_1 mount), published like the
real drivers: depth (ZED 32FC1 m, D405 16UC1 mm) + camera_info + compressed JPEG colour with the SAME stamp, and the
drivers' own static TF  zed_camera_link -> zed_left_camera_optical_frame,  d405_1_link -> d405_1_color_optical_frame.
--replay snap.npz: loop a captured real ZED snapshot (read-only capture) instead of the synthetic ZED.
--object-after / --object-until: a 5 cm sphere (blue) appears in free space for that window (live-object test).
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import StaticTransformBroadcaster

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
sys.path.insert(0, os.path.join(DEMO, 'test'))
from duo_geometry import DuoGeometry, rpy_matrix  # noqa: E402
from twin_recon_core import RobotFilter  # noqa: E402
from twin_recon_test import render  # noqa: E402
from twin_state import load_home  # noqa: E402

OPT = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])        # optical (z fwd, x right, y down) in body (x fwd, z up)
OBJ = ((0.90, -0.30, 0.20), 0.05)


def pose(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


class FakeDepth(Node):
    def __init__(self, a):
        super().__init__('twin_fake_depth')
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
            raise SystemExit('refusing to run on the default ROS domain (fake data would mix with the real cameras)')
        self.a = a
        cfg = yaml.safe_load(open(a.config))
        g = DuoGeometry()
        home = load_home()
        self.caps = RobotFilter(g).capsules({'left': home['left'], 'right': home['right']}, filter_margin=False)
        To = np.eye(4)
        To[:3, :3] = OPT
        z = cfg['cameras']['zed']
        d = cfg['cameras']['d405_1']
        self.cams = {
            'zed': dict(T=pose(z['xyz'], z['rpy']) @ To, wh=(960, 540), K=(520.0, 520.0, 479.5, 269.5),
                        ns='/zed/zed_node', depth='/zed/zed_node/depth/depth_registered',
                        info='/zed/zed_node/depth/camera_info', color='/zed/zed_node/rgb/color/rect/image/compressed',
                        frame='zed_left_camera_optical_frame', parent='zed_camera_link', mm=False),
            'd405_1': dict(T=g.link_frames('left', np.array(home['left']))[8] @ pose(d['xyz'], d['rpy']) @ To,
                           wh=(424, 240), K=(215.0, 215.0, 211.5, 119.5),
                           depth='/camera/d405_1/aligned_depth_to_color/image_raw',
                           info='/camera/d405_1/aligned_depth_to_color/camera_info',
                           color='/camera/d405_1/color/image_raw/compressed',
                           frame='d405_1_color_optical_frame', parent='d405_1_link', mm=True)}
        self.rng = np.random.default_rng(3)
        if a.replay:                       # real ZED snapshot (depth m, rgb, K) instead of the synthetic ZED; no D405
            z = np.load(a.replay)
            c = self.cams['zed']
            c['replay'] = [(d, rgb, np.zeros(d.shape, bool)) for d, rgb in zip(z['depth'], z['rgb'])]
            c['wh'] = (z['depth'].shape[2], z['depth'].shape[1])
            c['K'] = tuple(float(v) for v in z['K'])
            c['frame'] = str(z['frame'])
            del self.cams['d405_1']
        tfs = []
        for k, c in self.cams.items():
            if 'replay' not in c:
                c['base'] = render(c['T'], c['K'], c['wh'], self.caps)
                c['obj'] = render(c['T'], c['K'], c['wh'], self.caps, extra=[OBJ])
            c['pub_d'] = self.create_publisher(Image, c['depth'], qos_profile_sensor_data)
            c['pub_i'] = self.create_publisher(CameraInfo, c['info'], qos_profile_sensor_data)
            c['pub_c'] = self.create_publisher(CompressedImage, c['color'], qos_profile_sensor_data)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = c['parent'], c['frame']
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = \
                -0.5, 0.5, -0.5, 0.5                   # body -> optical (same as the real drivers)
            tfs.append(t)
        self.stf = StaticTransformBroadcaster(self)
        self.stf.sendTransform(tfs)
        self.t0 = time.monotonic()
        self.create_timer(1.0 / a.hz, self.tick)
        self.get_logger().info('fake ZED + left D405 publishing (scene of test/twin_recon_test.py)')

    def tick(self):
        el = time.monotonic() - self.t0
        obj = self.a.object_after <= el < self.a.object_until
        st = self.get_clock().now().to_msg()
        for k, c in self.cams.items():
            if 'replay' in c:
                self.k = getattr(self, 'k', -1) + 1
                d, rgb, _ = c['replay'][self.k % len(c['replay'])]
                d = np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0)
            else:
                depth, rgb, _ = c['obj'] if obj else c['base']
                d = depth + self.rng.normal(0, 0.002, depth.shape).astype(np.float32) * (depth > 0)
            w, h = c['wh']
            im = Image()
            im.header.stamp, im.header.frame_id = st, c['frame']
            im.height, im.width = h, w
            if c['mm']:
                im.encoding, im.step = '16UC1', 2 * w
                im.data = np.clip(d * 1000, 0, 65535).astype(np.uint16).tobytes()
            else:
                im.encoding, im.step = '32FC1', 4 * w
                im.data = np.where(d > 0, d, np.nan).astype(np.float32).tobytes()
            ci = CameraInfo()
            ci.header = im.header
            ci.height, ci.width = h, w
            fx, fy, cx, cy = c['K']
            ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            cm = CompressedImage()
            cm.header, cm.format = im.header, 'jpeg'
            cm.data = cv2.imencode('.jpg', rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
            c['pub_c'].publish(cm)
            c['pub_i'].publish(ci)
            c['pub_d'].publish(im)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--object-after', type=float, default=1e9)
    ap.add_argument('--object-until', type=float, default=1e9)
    ap.add_argument('--replay', help='npz snapshot of the real ZED (depth, rgb, K, frame) to replay instead')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    n = FakeDepth(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
