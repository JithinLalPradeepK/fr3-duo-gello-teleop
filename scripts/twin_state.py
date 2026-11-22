#!/usr/bin/env python3
"""DIGITAL TWIN state node - READ-ONLY: it never commands a robot, a gripper or a controller.

  python3 scripts/twin_state.py --ros-args -p robot_description:="$(xacro description/fr3_duo_twin.urdf.xacro ...)"
  (started by launch/digital_twin.launch.py)

What it does
  * joints     The twin's robot_state_publisher (namespace /twin, frame prefix "twin/") listens on /twin/joint_states, which
               two C++ topic_tools throttles feed straight from the REAL robots' /<arm>/franka/joint_states (1 kHz -> 100 Hz;
               measured joint positions, not GELLO commands). This node only adds the Robotiq knuckle joints (renamed
               <arm>_robotiq_85_left_knuckle_joint) from /twin/raw/<arm>_gripper, and measures rate / latency / staleness.
               An arm that never sent data is drawn at its saved HOME pose and labelled NO DATA; an arm whose data stops
               keeps its last pose and is labelled STALE.
  * overlays   /twin/markers (MarkerArray): joint load spheres from tau_ext_hat_filtered, external force arrow from
               O_F_ext_hat_K at the end effector (red while the robot reports contact), inter-arm clearance line + distance
               from scripts/duo_geometry.py (the same capsules as the HOME planner), warning labels.
               Source of load / force: /<arm>/robot_state_10hz (the supervisor's C++ throttle of the 1 kHz robot state).
  * layers     owns the layer state: requests on /twin/layers_cmd (JSON, e.g. {"zed_cloud": false}), the state is published
               latched on /twin/layers (twin_cloud.py nodes follow it). "robots" off = robot model without visuals.
  * cameras    static TFs twin/<parent> -> camera root frame from config/digital_twin.yaml (re-read when the file changes);
               map layer: enables ZED spatial mapping and anchors the "map" frame to the calibrated ZED pose.
  * ghost      GELLO GHOST: /gello_check_<arm>/gello/joint_states + gripper target -> /twin_ghost/joint_states (second
               robot_state_publisher, frame prefix ghost/); its description /twin/ghost_description_view shows an arm
               only while teleop starts / runs and that GELLO sends data (layer "gello").
  * status     /twin/status (std_msgs/String JSON, 2 Hz) for the twin display in demo_gui.py.
"""
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float32, String
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from duo_geometry import READY, DuoGeometry, rpy_matrix, seg_seg_dist  # noqa: E402

try:
    from franka_msgs.msg import FrankaRobotState
except ImportError:                  # sourced without the franka workspace: no load / force overlays
    FrankaRobotState = None

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARMS = ('left', 'right')
ARM_IP = {'left': '192.168.1.51', 'right': '192.168.1.50'}
HOME_FILE = os.path.expanduser('~/fr3_tools/fr3_duo_home/home_poses.txt')
PREFIX = 'twin/'
GRIP = 'robotiq_85_left_knuckle_joint'
PLACEHOLDER = 'twin_placeholder'          # frame_id of the HOME-pose stand-in messages (ignored by our own statistics)
MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX', 5: 'USER_STOPPED', 6: 'AUTOMATIC_ERROR_RECOVERY'}
# Franka palette (scripts/franka_theme.py) for the overlays
MINT, AMBER, RED = (0.459, 0.761, 0.545), (0.902, 0.584, 0.0), (0.827, 0.184, 0.184)
OFF_WHITE, TEAL = (0.831, 0.859, 0.863), (0.341, 0.718, 0.784)
DEFAULT_LAYERS = {'robots': True, 'gello': True, 'load': True, 'force': True, 'clearance': True, 'zed_cloud': False,
                  'wrist_clouds': False, 'map': False, 'scan': False,
                  'mesh': True, 'live_objects': True}           # scripts/twin_recon.py (solid reconstruction)
TELEOP_STATES = ('TELEOP_STARTING', 'TELEOP')
GHOST = 'ghost/'


def color(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def heat(t):
    """0 -> mint, 0.5 -> amber, 1 -> red."""
    t = min(max(t, 0.0), 1.0)
    a, b, u = (MINT, AMBER, t * 2) if t < 0.5 else (AMBER, RED, t * 2 - 1)
    return tuple(x + (y - x) * u for x, y in zip(a, b))


def load_home():
    """Saved HOME poses per arm (same file as the homing tool / supervisor), READY as fallback."""
    out = {a: list(READY) for a in ARMS}
    try:
        for line in open(HOME_FILE):
            p = line.split()
            if len(p) == 8 and not line.startswith('#'):
                for arm, ip in ARM_IP.items():
                    if p[0] == ip:
                        out[arm] = [float(x) for x in p[1:]]
    except OSError:
        pass
    return out


def closest_points(p0, p1, q0, q1):
    """Closest points of segments p0-p1 and q0-q1 (Ericson)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return p0, q0
    if a <= 1e-12:
        return p0, q0 + np.clip(f / e, 0, 1) * d2
    c = d1 @ r
    if e <= 1e-12:
        return p0 + np.clip(-c / a, 0, 1) * d1, q0
    b = d1 @ d2
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, np.clip(-c / a, 0, 1)
    elif t > 1:
        t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return p0 + s * d1, q0 + t * d2


def strip_visuals(urdf, keep=('mount_link', 'cover_link')):
    """Robot description without <visual> on every link except `keep` (a tuple of names, or a predicate)."""
    root = ET.fromstring(urdf)
    keeps = keep if callable(keep) else (lambda n: n in keep)
    for link in root.findall('link'):
        if keeps(link.get('name')):
            continue
        for v in link.findall('visual'):
            link.remove(v)
    return ET.tostring(root, encoding='unicode')


class ArmData:
    def __init__(self):
        self.q, self.t, self.stamp_lat = None, 0.0, deque(maxlen=400)   # (arrival time, latency s)
        self.arrivals = deque(maxlen=400)
        self.ever = False
        self.rs, self.rs_t = None, 0.0
        self.grip, self.grip_t = None, 0.0


class TwinState(Node):
    def __init__(self, cfg_path):
        super().__init__('twin_state')
        self.cfg_path, self.cfg_mtime = cfg_path, 0.0
        self.cfg = {}
        self.load_cfg()
        self.declare_parameter('robot_state_topic', '/{arm}/robot_state_10hz')
        self.urdf = ''                  # from the twin robot_state_publisher's latched /twin/robot_description
        self.geo = DuoGeometry()
        self.home = load_home()
        self.arm = {a: ArmData() for a in ARMS}
        self.layers = dict(DEFAULT_LAYERS)
        self.layers.update({k: bool(v) for k, v in (self.cfg.get('layers') or {}).items() if k in DEFAULT_LAYERS})
        self.clouds = {}
        self.clear = None
        self.shown = set()
        self.zed_pose = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.js_pub = self.create_publisher(JointState, '/twin/joint_states', 20)
        self.view_pub = self.create_publisher(String, '/twin/robot_description_view', latched)
        self.ghost_view_pub = self.create_publisher(String, '/twin/ghost_description_view', latched)
        self.ghost_pub = self.create_publisher(JointState, '/twin_ghost/joint_states', 20)
        self.gello_t = {a: 0.0 for a in ARMS}
        self.gello_n = {a: deque(maxlen=100) for a in ARMS}
        self.demo_state, self.ghost_shown = 'OFFLINE', None
        self.layers_pub = self.create_publisher(String, '/twin/layers', latched)
        self.status_pub = self.create_publisher(String, '/twin/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/twin/markers', 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dyn_tf = TransformBroadcaster(self)
        self.create_subscription(JointState, '/twin/joint_states', self.on_joints, 50)
        self.create_subscription(String, '/twin/robot_description', self.on_description, latched)
        self.create_subscription(String, '/twin/layers_cmd', self.on_layers_cmd, 10)
        self.create_subscription(String, '/twin/cloud_status', self.on_cloud_status, 10)
        self.scan = None
        self.create_subscription(String, '/twin/scan_status', lambda m: setattr(self, 'scan', json.loads(m.data)), latched)
        rs_topic = self.get_parameter('robot_state_topic').value
        self.create_subscription(String, '/demo/status', self.on_demo, latched)
        for a in ARMS:
            self.create_subscription(JointState, f'/gello_check_{a}/gello/joint_states',
                                     lambda m, a=a: self.on_gello(a, m), qos_profile_sensor_data)
            self.create_subscription(Float32, f'/gello_check_{a}/gripper/gripper_client/target_gripper_width_percent',
                                     lambda m, a=a: self.on_gello_grip(a, m), 10)
            self.create_subscription(JointState, f'/twin/raw/{a}_gripper', lambda m, a=a: self.on_gripper(a, m),
                                     qos_profile_sensor_data)
            if FrankaRobotState is not None:
                self.create_subscription(FrankaRobotState, rs_topic.format(arm=a), lambda m, a=a: self.on_rs(a, m),
                                         qos_profile_sensor_data)
        self.map_cli = self.create_client(SetBool, '/zed/zed_node/enable_mapping')
        self.pose_sub = None

        self.publish_view()
        self.publish_layers()
        self.send_static()
        hz = float(self.cfg.get('rates', {}).get('overlay_hz', 20.0))
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(0.5, self.slow)
        self.create_timer(0.2, self.placeholders)
        self.get_logger().info('twin_state running (read-only)')

    # ------------------------------------------------------------------ config
    def load_cfg(self):
        try:
            mt = os.path.getmtime(self.cfg_path)
            if mt == self.cfg_mtime:
                return False
            with open(self.cfg_path) as f:
                self.cfg = yaml.safe_load(f) or {}
            self.cfg_mtime = mt
            return True
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warn(f'config {self.cfg_path}: {e}')
            return False

    def c(self, section, key, default):
        return float((self.cfg.get(section) or {}).get(key, default))

    def send_static(self):
        out = []
        for key, cam in (self.cfg.get('cameras') or {}).items():
            try:
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = PREFIX + cam['parent']
                t.child_frame_id = cam['child']
                x, y, z = (float(v) for v in cam['xyz'])
                q = rpy_to_quat(*(float(v) for v in cam['rpy']))
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
                out.append(t)
                if cam.get('optical_frame'):    # RealSense driver publishes no link -> optical TF (measured 2026-11-22)
                    o = TransformStamped()
                    o.header.stamp = t.header.stamp
                    o.header.frame_id, o.child_frame_id = cam['child'], cam['optical_frame']
                    (o.transform.rotation.x, o.transform.rotation.y, o.transform.rotation.z,
                     o.transform.rotation.w) = (-0.5, 0.5, -0.5, 0.5)        # body (x fwd, z up) -> optical (z fwd)
                    out.append(o)
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().warn(f'camera {key}: bad extrinsics ({e})')
        g = TransformStamped()                # the ghost model lives in the same world
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id, g.child_frame_id = PREFIX + 'world', GHOST + 'world'
        g.transform.rotation.w = 1.0
        out.append(g)
        self.static_tf.sendTransform(out)

    # ------------------------------------------------------------------ inputs
    def on_joints(self, m):
        if m.header.frame_id == PLACEHOLDER or not m.name:
            return
        arm = 'left' if m.name[0].startswith('left_fr3_joint') else 'right' if m.name[0].startswith('right_fr3_joint') \
            else None
        if arm is None:
            return
        d = self.arm[arm]
        pos = dict(zip(m.name, m.position))
        q = [pos.get(f'{arm}_fr3_joint{i}') for i in range(1, 8)]
        if None in q:
            return
        now = time.monotonic()
        d.q, d.t, d.ever = q, now, True
        d.arrivals.append(now)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st > 0:
            d.stamp_lat.append((now, time.time() - st))

    def on_gripper(self, arm, m):
        if GRIP not in m.name:
            return
        v = m.position[m.name.index(GRIP)]
        d = self.arm[arm]
        d.grip, d.grip_t = v, time.monotonic()
        out = JointState()
        out.header.stamp = m.header.stamp
        out.name, out.position = [f'{arm}_{GRIP}'], [v]
        self.js_pub.publish(out)

    def on_description(self, m):
        if m.data and m.data != self.urdf:
            self.urdf = m.data
            self.publish_view()
            self.ghost_shown = None
            self.update_ghost()

    # ------------------------------------------------------------------ GELLO ghost
    def on_demo(self, m):
        try:
            st = json.loads(m.data).get('state', 'OFFLINE')
        except ValueError:
            return
        if st != self.demo_state:
            self.demo_state = st
            self.update_ghost()

    def on_gello(self, arm, m):
        """GELLO joints (names fr3_joint1..7, any order) -> ghost joints <arm>_fr3_jointN."""
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [f'{arm}_{n}' for n in m.name if n.startswith('fr3_joint')]
        out.position = [p for n, p in zip(m.name, m.position) if n.startswith('fr3_joint')]
        if len(out.name) != 7:
            return
        self.ghost_pub.publish(out)
        now = time.monotonic()
        was = now - self.gello_t[arm] < 0.5
        self.gello_t[arm] = now
        self.gello_n[arm].append(now)
        if not was:
            self.update_ghost()

    def on_gello_grip(self, arm, m):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [f'{arm}_{GRIP}']
        out.position = [min(0.79, max(0.0, 1.0 - float(m.data)))]      # width 1 = open (0 rad), knuckle = 1 - width
        self.ghost_pub.publish(out)

    def update_ghost(self):
        """Ghost visible per arm only while teleop starts / runs and that GELLO sends data (in the holder the GELLO pose
        has nothing to do with the robot)."""
        now = time.monotonic()
        arms = tuple(a for a in ARMS if self.layers.get('gello') and self.demo_state in TELEOP_STATES
                     and now - self.gello_t[a] < 0.5)
        if arms == self.ghost_shown or not self.urdf:
            return
        self.ghost_shown = arms
        self.ghost_view_pub.publish(String(data=strip_visuals(self.urdf, lambda n: any(n.startswith(a + '_') for a in arms))))

    def on_rs(self, arm, m):
        self.arm[arm].rs, self.arm[arm].rs_t = m, time.monotonic()

    def on_cloud_status(self, m):
        try:
            s = json.loads(m.data)
            self.clouds[s['key']] = dict(s, t=time.monotonic())
        except (ValueError, KeyError, TypeError):
            pass

    def on_layers_cmd(self, m):
        try:
            req = json.loads(m.data)
        except ValueError:
            return
        changed = {k: bool(v) for k, v in req.items() if k in self.layers and bool(v) != self.layers[k]}
        if not changed:
            return
        self.layers.update(changed)
        self.get_logger().info(f'layers: {changed}')
        if 'robots' in changed:
            self.publish_view()
        if 'gello' in changed:
            self.update_ghost()
        if 'map' in changed:
            self.set_mapping(changed['map'])
        self.publish_layers()
        self.tick()

    def publish_layers(self):
        self.layers_pub.publish(String(data=json.dumps(self.layers)))

    def publish_view(self):
        if not self.urdf:
            return
        self.view_pub.publish(String(data=self.urdf if self.layers['robots'] else strip_visuals(self.urdf)))

    # ------------------------------------------------------------------ mapping (optional)
    def set_mapping(self, on):
        if on and self.pose_sub is None:
            topic = (self.cfg.get('mapping') or {}).get('pose_topic', '/zed/zed_node/pose')
            self.pose_sub = self.create_subscription(PoseStamped, topic, self.on_zed_pose, qos_profile_sensor_data)
        elif not on and self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
            self.pose_sub, self.zed_pose = None, None
        if self.map_cli.service_is_ready():
            self.map_cli.call_async(SetBool.Request(data=bool(on)))
        else:
            self.get_logger().warn('ZED enable_mapping service not available (cameras not running?)')

    def on_zed_pose(self, m):
        """Anchor the ZED "map" frame so that the camera sits at its calibrated pose: world->map = world->cam * inv(map->cam)."""
        cam = (self.cfg.get('cameras') or {}).get('zed')
        if not cam:
            return
        p, o = m.pose.position, m.pose.orientation
        t_map_cam = np.eye(4)
        t_map_cam[:3, :3] = quat_matrix(o.x, o.y, o.z, o.w)
        t_map_cam[:3, 3] = (p.x, p.y, p.z)
        t_w_cam = np.eye(4)
        t_w_cam[:3, :3] = rpy_matrix(*(float(v) for v in cam['rpy']))
        t_w_cam[:3, 3] = [float(v) for v in cam['xyz']]
        t = t_w_cam @ np.linalg.inv(t_map_cam)
        ts = TransformStamped()
        ts.header.stamp = m.header.stamp
        ts.header.frame_id, ts.child_frame_id = PREFIX + cam['parent'], m.header.frame_id or 'map'
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = (float(v) for v in t[:3, 3])
        (ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z,
         ts.transform.rotation.w) = matrix_quat(t[:3, :3])
        self.dyn_tf.sendTransform(ts)
        self.zed_pose = time.monotonic()

    # ------------------------------------------------------------------ periodic
    def arm_state(self, arm, now):
        d = self.arm[arm]
        if not d.ever:
            return 'no data'
        return 'stale' if now - d.t > self.c('stale', 'joint_states_s', 0.25) else 'live'

    def placeholders(self):
        """An arm without live data is drawn at its HOME pose (never connected) or its last pose (stale), so the twin is
        always complete; the label says which. Placeholder messages carry frame_id twin_placeholder."""
        now = time.monotonic()
        for arm in ARMS:
            st = self.arm_state(arm, now)
            d = self.arm[arm]
            if st == 'live':
                continue
            q = d.q if d.q is not None else self.home[arm]
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = PLACEHOLDER
            m.name = [f'{arm}_fr3_joint{i}' for i in range(1, 8)]
            m.position = [float(x) for x in q]
            if d.grip is None or now - d.grip_t > 1.0:
                m.name.append(f'{arm}_{GRIP}')
                m.position.append(float(d.grip) if d.grip is not None else 0.0)
            self.js_pub.publish(m)

    def slow(self):
        self.update_ghost()                   # GELLO data stopped -> hide that arm's ghost
        if self.load_cfg():
            self.get_logger().info('config changed: camera extrinsics / thresholds reloaded')
            self.send_static()
        self.publish_status()

    def tick(self):
        now = time.monotonic()
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        want = set()

        def add(mk):
            mk.header.stamp = stamp
            mk.frame_locked = True
            mk.action = Marker.ADD
            ma.markers.append(mk)
            want.add((mk.ns, mk.id))

        rs_max = self.c('stale', 'robot_state_s', 1.0)
        for ai, arm in enumerate(ARMS):
            d = self.arm[arm]
            st = self.arm_state(arm, now)
            rs = d.rs if d.rs is not None and now - d.rs_t < rs_max and st == 'live' else None
            # ---- joint load spheres
            if self.layers['load'] and rs is not None:
                lo, full = self.c('overlays', 'load_min_nm', 1.5), self.c('overlays', 'load_full_nm', 12.0)
                for j, tau in enumerate(list(rs.tau_ext_hat_filtered.effort)[:7]):
                    if abs(tau) < lo:
                        continue
                    t = (abs(tau) - lo) / max(full - lo, 1e-3)
                    mk = Marker(ns=f'load_{arm}', id=j, type=Marker.SPHERE)
                    mk.header.frame_id = f'{PREFIX}{arm}_fr3_link{j + 1}'
                    s = 0.085 + 0.05 * min(t, 1.0)
                    mk.scale.x = mk.scale.y = mk.scale.z = s
                    mk.pose.orientation.w = 1.0
                    mk.color = color(heat(t), 0.55)
                    add(mk)
            # ---- external force arrow at the end effector
            if self.layers['force'] and rs is not None:
                f = rs.o_f_ext_hat_k.wrench.force
                fn = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
                if fn >= self.c('overlays', 'force_min_n', 6.0):
                    ci = rs.collision_indicators
                    contact = any((ci.is_cartesian_linear_contact.x, ci.is_cartesian_linear_contact.y,
                                   ci.is_cartesian_linear_contact.z, *ci.is_joint_contact))
                    p = rs.o_t_ee.pose.position
                    k = self.c('overlays', 'force_scale_m_per_n', 0.012)
                    mk = Marker(ns=f'force_{arm}', id=0, type=Marker.ARROW)
                    mk.header.frame_id = f'{PREFIX}{arm}_fr3_link0'
                    mk.points = [Point(x=p.x, y=p.y, z=p.z), Point(x=p.x + f.x * k, y=p.y + f.y * k, z=p.z + f.z * k)]
                    mk.scale.x, mk.scale.y, mk.scale.z = 0.012, 0.03, 0.045
                    mk.pose.orientation.w = 1.0
                    mk.color = color(RED if contact else AMBER, 0.95)
                    add(mk)
                    tx = Marker(ns=f'force_{arm}', id=1, type=Marker.TEXT_VIEW_FACING, text=f'{fn:.0f}N')
                    tx.header.frame_id = mk.header.frame_id
                    tx.pose.position.x, tx.pose.position.y = p.x + f.x * k, p.y + f.y * k
                    tx.pose.position.z = p.z + f.z * k + 0.05
                    tx.pose.orientation.w = 1.0
                    tx.scale.z = 0.045
                    tx.color = color(OFF_WHITE)
                    add(tx)
            # ---- warning label (no data / stale / robot stopped)
            mode = MODES.get(d.rs.robot_mode, '') if d.rs is not None and now - d.rs_t < 2.0 else ''
            text = {'no data': 'OFFLINE', 'stale': 'STALE'}.get(st)        # NO spaces: RViz2 Humble draws them far too wide
            if not text and mode in ('REFLEX', 'USER_STOPPED'):
                text = {'REFLEX': 'REFLEX STOP', 'USER_STOPPED': 'E-STOP'}[mode]
            if text:
                mk = Marker(ns='label', id=ai, type=Marker.TEXT_VIEW_FACING, text=text)
                mk.header.frame_id = f'{PREFIX}{arm}_fr3_link0'
                mk.pose.position.z = 1.05
                mk.pose.orientation.w = 1.0
                mk.scale.z = 0.06
                mk.color = color(AMBER if st != 'live' else RED)
                add(mk)
        # ---- inter-arm clearance (duo_geometry capsules, mount frame = twin/mount_link)
        self.clear = None
        ql, qr = self.arm['left'].q or self.home['left'], self.arm['right'].q or self.home['right']
        try:
            self.clear = self.clearance(ql, qr)
        except Exception as e:      # noqa: BLE001 - geometry must never take the twin down
            self.get_logger().warn(f'clearance: {e}', throttle_duration_sec=10.0)
        both_live = all(self.arm_state(a, now) == 'live' for a in ARMS)     # never draw a distance of stand-in poses
        if self.layers['clearance'] and both_live and self.clear and \
                self.clear['m'] < self.c('overlays', 'clearance_warn_m', 0.10):
            cl = self.clear
            col = {'ok': TEAL, 'warn': AMBER, 'bad': RED}[cl['level']]
            a, b = cl['pa'], cl['pb']
            mk = Marker(ns='clearance', id=0, type=Marker.LINE_LIST)
            mk.header.frame_id = PREFIX + 'mount_link'
            mk.points = [Point(x=a[0], y=a[1], z=a[2]), Point(x=b[0], y=b[1], z=b[2])]
            mk.scale.x = 0.006
            mk.pose.orientation.w = 1.0
            mk.color = color(col, 0.95)
            add(mk)
            for i, p in enumerate((a, b)):
                s = Marker(ns='clearance', id=1 + i, type=Marker.SPHERE)
                s.header.frame_id = mk.header.frame_id
                s.pose.position.x, s.pose.position.y, s.pose.position.z = (float(v) for v in p)
                s.pose.orientation.w = 1.0
                s.scale.x = s.scale.y = s.scale.z = 0.018
                s.color = color(col, 0.95)
                add(s)
            tx = Marker(ns='clearance', id=3, type=Marker.TEXT_VIEW_FACING, text=f'{max(cl["m"], 0) * 100:.1f}cm')
            tx.header.frame_id = mk.header.frame_id
            mid = (a + b) / 2
            tx.pose.position.x, tx.pose.position.y, tx.pose.position.z = float(mid[0]), float(mid[1]), float(mid[2]) + 0.06
            tx.pose.orientation.w = 1.0
            tx.scale.z = 0.045
            tx.color = color(col)
            add(tx)
        for ns, i in self.shown - want:
            mk = Marker(ns=ns, id=i, action=Marker.DELETE)
            mk.header.stamp = stamp
            ma.markers.append(mk)
        self.shown = want
        if ma.markers:
            self.marker_pub.publish(ma)

    def clearance(self, ql, qr):
        g = self.geo
        al, bl = g.world_capsules('left', np.asarray(ql, float))
        ar, br = g.world_capsules('right', np.asarray(qr, float))
        c, info = g.clearance_between((al, bl), (ar, br))
        # the pair behind the minimum: recompute its closest points for drawing
        i, j = g.pairs[:, 0], g.pairs[:, 1]
        dist = seg_seg_dist(al[i], bl[i], ar[j], br[j]) - g.radsum
        k = int(np.argmin(dist - g.margin))
        ii, jj = int(i[k]), int(j[k])
        pa, pb = closest_points(al[ii], bl[ii], ar[jj], br[jj])
        u = pb - pa
        n = float(np.linalg.norm(u))
        u = u / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        pa_s, pb_s = pa + u * g.caps[ii][3], pb - u * g.caps[jj][3]       # capsule surface points
        surf, margin = info[2], info[3]
        level = 'bad' if surf < 0.01 else 'warn' if surf < margin else 'ok'
        return {'m': float(surf), 'margin_m': float(margin), 'pair': f'{info[0]} / {info[1]}', 'level': level,
                'pa': pa_s, 'pb': pb_s}

    def publish_status(self):
        now = time.monotonic()
        arms = {}
        for arm in ARMS:
            d = self.arm[arm]
            arr = [t for t in d.arrivals if now - t < 1.0]
            lat = [l for t, l in d.stamp_lat if now - t < 1.0]
            rs = d.rs if d.rs is not None and now - d.rs_t < 2.0 else None
            s = {'state': self.arm_state(arm, now), 'hz': len(arr),
                 'latency_ms': round(1000 * sorted(lat)[len(lat) // 2], 1) if lat else None,
                 'latency_max_ms': round(1000 * max(lat), 1) if lat else None,
                 'age_s': round(now - d.t, 2) if d.ever else None,
                 'gripper': round(d.grip, 3) if d.grip is not None and now - d.grip_t < 1.0 else None,
                 'gello_hz': len([t for t in self.gello_n[arm] if now - t < 1.0]),
                 'ghost': arm in (self.ghost_shown or ())}
            if rs is not None:
                f = rs.o_f_ext_hat_k.wrench.force
                tau = [abs(x) for x in list(rs.tau_ext_hat_filtered.effort)[:7]]
                ci = rs.collision_indicators
                s.update({'mode': MODES.get(rs.robot_mode, str(rs.robot_mode)),
                          'f_ext_n': round(math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z), 1),
                          'tau_max_nm': round(max(tau), 1) if tau else None,
                          'tau_joint': (tau.index(max(tau)) + 1) if tau else None,
                          'contact': bool(any((ci.is_cartesian_linear_contact.x, ci.is_cartesian_linear_contact.y,
                                               ci.is_cartesian_linear_contact.z, *ci.is_joint_contact)))})
            arms[arm] = s
        clouds = {k: {kk: v.get(kk) for kk in ('on', 'hz', 'points', 'age_s')}
                  for k, v in self.clouds.items() if now - v['t'] < 3.0}
        cl = self.clear
        st = {'arms': arms, 'layers': self.layers, 'clouds': clouds,
              'clearance': ({'m': round(cl['m'], 4), 'margin_m': cl['margin_m'], 'pair': cl['pair'], 'level': cl['level'],
                             'from_home_pose': not (self.arm['left'].ever and self.arm['right'].ever)} if cl else None),
              'calibrated': {k: bool(v.get('calibrated')) for k, v in (self.cfg.get('cameras') or {}).items()},
              'map_anchor': self.zed_pose is not None and now - self.zed_pose < 2.0, 'scan': self.scan, 't': time.time()}
        self.status_pub.publish(String(data=json.dumps(st)))


def rpy_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = TwinState(a.config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
