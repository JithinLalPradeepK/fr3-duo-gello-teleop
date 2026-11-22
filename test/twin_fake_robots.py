#!/usr/bin/env python3
"""FAKE robot data for the digital-twin tests (publishes only - no robot, no controller, no hardware).

Run ONLY on an isolated ROS domain (test/sim_twin_test.sh sets ROS_DOMAIN_ID=78, localhost only).
  /<arm>/franka/joint_states      JointState, --hz (default 500), slow sine motion around the saved HOME poses
  /<arm>/gripper/joint_states     JointState robotiq_85_left_knuckle_joint, 100 Hz, opens / closes
  /<arm>/robot_state_10hz         franka_msgs/FrankaRobotState, 10 Hz: tau_ext rising on j4, external force with
                                  contact flag on the right arm (so the overlays can be seen)
--stop-after ARM:SECONDS  stop publishing that arm's joint states after SECONDS (stale-data test)
--approach                move both wrists towards each other (clearance overlay test)
--gello                   also fake the GELLOs (/gello_check_<arm>/gello/joint_states = robot pose + 0.2 rad on j1,
                          gripper target) and a latched /demo/status in TELEOP (ghost test)
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
from twin_state import load_home  # noqa: E402

try:
    from franka_msgs.msg import FrankaRobotState
except ImportError:
    FrankaRobotState = None

ARMS = ('left', 'right')
GELLO_OFFSET = 0.2          # fake GELLO j1 = robot j1 + this (ghost visibly apart from the robot)
ORDER = {'left': [0, 2, 4, 1, 5, 6, 3], 'right': [2, 3, 4, 6, 1, 0, 5]}     # measured on the real robots


class Fake(Node):
    def __init__(self, a):
        super().__init__('twin_fake_robots')
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
            raise SystemExit('refusing: run on an isolated ROS_DOMAIN_ID (test/sim_twin_test.sh)')
        self.a, self.t0 = a, time.monotonic()
        self.home = load_home()
        self.js = {arm: self.create_publisher(JointState, f'/{arm}/franka/joint_states', 10) for arm in ARMS}
        self.gr = {arm: self.create_publisher(JointState, f'/{arm}/gripper/joint_states', 10) for arm in ARMS}
        self.rs = {arm: self.create_publisher(FrankaRobotState, f'/{arm}/robot_state_10hz', 10)
                   for arm in ARMS} if FrankaRobotState else {}
        self.stop = {}
        for s in a.stop_after:
            arm, sec = s.split(':')
            self.stop[arm] = float(sec)
        if a.gello:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Float32, String
            self.gl = {arm: self.create_publisher(JointState, f'/gello_check_{arm}/gello/joint_states', 10) for arm in ARMS}
            self.gg = {arm: self.create_publisher(Float32, f'/gello_check_{arm}/gripper/gripper_client/'
                                                  'target_gripper_width_percent', 10) for arm in ARMS}
            lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.st = self.create_publisher(String, '/demo/status', lat)
            self.st.publish(String(data='{"state": "TELEOP", "message": "fake", "arms": {}}'))
            self.Float32 = Float32
            self.create_timer(0.04, self.gellos)
        self.create_timer(1.0 / a.hz, self.joints)
        self.create_timer(0.01, self.grippers)
        self.create_timer(0.1, self.states)

    def q(self, arm, t):
        q = list(self.home[arm])
        if self.a.still:
            return q
        s = 1.0 if arm == 'left' else -1.0
        q[0] += s * 0.25 * math.sin(0.5 * t)
        q[1] += 0.15 * math.sin(0.7 * t)
        q[3] += 0.2 * math.sin(0.4 * t)
        if self.a.approach:                 # bring the wrists together (j1 inwards)
            q[0] -= s * 0.35 * (0.5 - 0.5 * math.cos(0.3 * t))
        return q

    def joints(self):
        t = time.monotonic() - self.t0
        stamp = self.get_clock().now().to_msg()
        for arm in ARMS:
            if (arm in self.stop and t > self.stop[arm]) or arm in self.a.skip:
                continue
            m = JointState()
            m.header.stamp = stamp
            # SCRAMBLED order, as the real joint_state_broadcaster publishes it (seen 2026-11-21 on both FR3s)
            order = ORDER[arm]
            q = self.q(arm, t)
            m.name = [f'{arm}_fr3_joint{i + 1}' for i in order]
            m.position = [q[i] for i in order]
            m.velocity = [0.0] * 7
            m.effort = [0.0] * 7
            self.js[arm].publish(m)

    def gellos(self):
        t = time.monotonic() - self.t0
        for arm in ARMS:
            q = self.q(arm, t)
            q[0] += GELLO_OFFSET
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = [f'fr3_joint{i}' for i in (2, 1, 3, 4, 5, 6, 7)]          # order does not matter
            m.position = [q[i - 1] for i in (2, 1, 3, 4, 5, 6, 7)]
            self.gl[arm].publish(m)
            self.gg[arm].publish(self.Float32(data=0.5))

    def grippers(self):
        t = time.monotonic() - self.t0
        for arm in ARMS:
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = ['robotiq_85_left_knuckle_joint']
            m.position = [0.4 + 0.39 * math.sin(0.8 * t + (0 if arm == 'left' else 1.5))]
            self.gr[arm].publish(m)

    def states(self):
        if not self.rs:
            return
        t = time.monotonic() - self.t0
        for arm in ARMS:
            m = FrankaRobotState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.robot_mode = 2
            eff = [0.5, 1.0, 0.4, 2.0 + 9.0 * (0.5 + 0.5 * math.sin(0.6 * t)), 0.3, 1.0, 0.2]
            m.tau_ext_hat_filtered.effort = eff
            m.measured_joint_state.position = self.q(arm, t)
            f = 4.0 + (14.0 if arm == 'right' else 3.0) * (0.5 + 0.5 * math.sin(0.5 * t))
            m.o_f_ext_hat_k.wrench.force.x, m.o_f_ext_hat_k.wrench.force.z = 0.3 * f, -f
            m.o_t_ee.pose.position.x, m.o_t_ee.pose.position.y, m.o_t_ee.pose.position.z = 0.45, 0.0, 0.35
            m.o_t_ee.pose.orientation.w = 1.0
            if arm == 'right' and f > 12:
                m.collision_indicators.is_cartesian_linear_contact.z = 1.0
            self.rs[arm].publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hz', type=float, default=500.0)
    ap.add_argument('--stop-after', action='append', default=[], metavar='ARM:SECONDS')
    ap.add_argument('--approach', action='store_true')
    ap.add_argument('--still', action='store_true', help='hold the HOME poses exactly (FK cross-check)')
    ap.add_argument('--gello', action='store_true', help='also fake GELLOs + demo status TELEOP (ghost test)')
    ap.add_argument('--skip', action='append', default=[], help='never publish joint states of this arm')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    n = Fake(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
