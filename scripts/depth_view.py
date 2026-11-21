#!/usr/bin/env python3
"""Depth camera -> colourised JPEG for the GUI (sensor_msgs/CompressedImage that QImage can decode).

  python3 depth_view.py --input /camera/d405_1/depth/image_rect_raw --output /camera/d405_1/depth_view/compressed \
                        --min 0.10 --max 1.00 [--fps 12] [--width 480]

Input: a raw depth image, 16UC1 in millimetres (RealSense) or 32FC1 in metres (ZED). Output: JPEG, near = red,
far = blue (turbo colour map), no-data pixels black. The picture is shrunk BEFORE colouring (a 1920x1080 ZED frame is
8 MB), the output rate is capped (--fps) and the node is LAZY: it only subscribes to the camera while somebody is
subscribed to the output, so an unused depth view costs nothing. Rotation is not done here (the GUI does it).
launch/cameras.launch.py starts one node per camera.
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def depth_to_jpeg(msg, dmin, dmax, width, quality=80):
    """Image message (16UC1 mm or 32FC1 m) -> JPEG bytes, or None if the encoding is not supported."""
    if msg.encoding == '16UC1':
        a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        scale = 0.001
    elif msg.encoding == '32FC1':
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        scale = 1.0
    else:
        return None
    if msg.width > width:                        # shrink first: nearest neighbour never invents depth values
        h = max(1, int(round(msg.height * width / msg.width)))
        a = cv2.resize(a, (width, h), interpolation=cv2.INTER_NEAREST)
    d = a.astype(np.float32) * scale
    valid = np.isfinite(d) & (d > 0.0)
    norm = np.clip((d - dmin) / max(dmax - dmin, 1e-6), 0.0, 1.0)
    # turbo ends in dark red / dark violet: use only its bright middle 84 % so near is a clear red and far a clear blue
    img = cv2.applyColorMap(((0.08 + 0.84 * (1.0 - norm)) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid] = 0
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


class DepthView(Node):
    def __init__(self, a):
        super().__init__('depth_view_' + a.output.strip('/').split('/')[-3 if a.output.endswith('/compressed') else -1]
                         .replace('-', '_'))
        self.a = a
        self.pub = self.create_publisher(CompressedImage, a.output, qos_profile_sensor_data)
        self.sub = None
        self.t_last = 0.0
        self.n = 0
        self.create_timer(1.0, self.check_subscribers)

    def check_subscribers(self):
        wanted = self.pub.get_subscription_count() > 0
        if wanted and self.sub is None:
            self.sub = self.create_subscription(Image, self.a.input, self.on_depth, qos_profile_sensor_data)
            self.get_logger().info(f'viewer connected: reading {self.a.input}')
        elif not wanted and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info('no viewer: stopped reading the camera')

    def on_depth(self, msg):
        now = time.monotonic()
        if now - self.t_last < 1.0 / self.a.fps:
            return
        self.t_last = now
        data = depth_to_jpeg(msg, self.a.min, self.a.max, self.a.width, self.a.quality)
        if data is None:
            self.get_logger().error(f'unsupported depth encoding {msg.encoding}', throttle_duration_sec=30)
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = data
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--input', required=True, help='raw depth topic (16UC1 mm or 32FC1 m)')
    ap.add_argument('--output', required=True, help='CompressedImage topic to publish')
    ap.add_argument('--min', type=float, default=0.10, help='nearest depth that is coloured [m] (red)')
    ap.add_argument('--max', type=float, default=2.00, help='farthest depth that is coloured [m] (blue)')
    ap.add_argument('--fps', type=float, default=12.0, help='maximum output rate')
    ap.add_argument('--width', type=int, default=480, help='output width in pixels')
    ap.add_argument('--quality', type=int, default=80, help='JPEG quality')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = DepthView(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
