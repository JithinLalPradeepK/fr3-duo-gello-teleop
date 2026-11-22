#!/usr/bin/env python3
"""FAKE ZED point cloud for the digital-twin tests (isolated ROS domain only; no camera).
A synthetic table top (z = 0, in front of the robots) with a 10 cm cube on it, built in the twin's world frame and
expressed in the ZED frame using the extrinsics of config/digital_twin.yaml - so if the TF chain is right, the cloud
lands exactly on the floor grid in front of the robots. Also publishes the ZED wrapper's own static TF
zed_camera_link -> zed_left_camera_frame (identity here) like the real wrapper does.
"""
import os
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import rpy_matrix  # noqa: E402


def scene():
    xs, ys = np.meshgrid(np.arange(0.25, 1.05, 0.008), np.arange(-0.6, 0.6, 0.008))
    table = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], 1)
    g = np.arange(-0.05, 0.05, 0.006)
    a, b = np.meshgrid(g, g)
    a, b = a.ravel(), b.ravel()
    c = np.array([0.6, 0.0, 0.05])
    faces = [np.stack([a, b, np.full_like(a, 0.05)], 1), np.stack([a, np.full_like(a, -0.05), b], 1),
             np.stack([a, np.full_like(a, 0.05), b], 1), np.stack([np.full_like(a, 0.05), a, b], 1)]
    cube = np.concatenate(faces) + c
    col_t = np.full(len(table), (150 << 16) | (130 << 8) | 105, np.uint32)          # wood-ish
    col_c = np.full(len(cube), (230 << 16) | (149 << 8) | 0, np.uint32)            # Franka amber cube
    return np.concatenate([table, cube]), np.concatenate([col_t, col_c])


class FakeCloud(Node):
    def __init__(self):
        super().__init__('twin_fake_cloud')
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
            raise SystemExit('refusing: isolated ROS_DOMAIN_ID only')
        cam = yaml.safe_load(open(os.path.join(DEMO, 'config', 'digital_twin.yaml')))['cameras']['zed']
        R, t = rpy_matrix(*cam['rpy']), np.array(cam['xyz'], float)
        pw, col = scene()
        pc = (pw - t) @ R                     # world -> camera frame (R^T (p - t))
        self.data = np.zeros(len(pc), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        self.data['x'], self.data['y'], self.data['z'], self.data['rgb'] = pc[:, 0], pc[:, 1], pc[:, 2], col
        self.pub = self.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 2)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = 'zed_camera_link', 'zed_left_camera_frame'
        st.transform.rotation.w = 1.0
        self.stf = StaticTransformBroadcaster(self)
        self.stf.sendTransform(st)
        self.create_timer(0.1, self.tick)

    def tick(self):
        m = PointCloud2(height=1, width=len(self.data), point_step=16, row_step=16 * len(self.data), is_dense=True)
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'zed_left_camera_frame'
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
        m.data = self.data.tobytes()
        self.pub.publish(m)


def main():
    rclpy.init()
    n = FakeCloud()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
