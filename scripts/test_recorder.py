#!/usr/bin/env python3
"""Read-only test recorder: logs every change of the demo state while the operator tests.

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/test_recorder.py [--out ~/robot_demo/logs/test_<ts>/events.log]

Records (wall-clock time, only on change): supervisor state + message, per-arm robot mode / errors / gate /
GELLO / gripper, Desk mode / brakes / FCI / control / busy / last result, gate state topics, pedal connection,
camera fps drops (< 20 fps) and recoveries (only while cameras run), and gaps in the status stream.
Never calls a service.

Motion data (S2, to tell contact from command steps when a reflex happens):
  motion_<arm>.csv  one row per robot state sample (10 Hz throttle): |F_ext|, F_ext xyz, max |tau_ext|,
                    max |dq|, max |command - q| (command = what the gate relays to the controller),
                    contact / collision flags, control_command_success_rate, q1..q7, pos_margin (smallest
                    distance of a joint to the usable FR3 range), vel_margin (smallest |FR3 speed limit at q| - |dq|;
                    near 0 = close to a joint_velocity_violation) and the joint it belongs to.
  command steps     every relayed command sample (25 Hz in LIVE) is compared with the previous one;
                    the largest step per window goes into the CSV row.
  events.log        "PEAK" lines when |F_ext| > --force-warn or a command step > --step-warn, and a
                    "LAST 3 s" summary per arm when its robot state stops or the robot goes to REFLEX.
"""
import collections
import math
import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from franka_msgs.msg import FrankaRobotState
from gello_gate import Q_MAX, Q_MIN, fr3_vel_limit       # FR3 usable range + position-dependent speed limits
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARMS = ('left', 'right')


class Recorder(Node):
    def __init__(self, out, force_warn, step_warn):
        super().__init__('test_recorder')
        self.f = open(out, 'a', buffering=1)
        self.force_warn, self.step_warn = force_warn, step_warn
        self.cameras_running = False
        self.motion = {}
        for arm in ARMS:
            csv = open(os.path.join(os.path.dirname(out), f'motion_{arm}.csv'), 'a', buffering=1)
            if csv.tell() == 0:
                csv.write('wall_time,robot_mode,f_ext_n,fx,fy,fz,tau_ext_max_nm,dq_max_rads,'
                          'cmd_err_max_rad,cmd_step_max_rad,contact,collision,cmd_success_rate,'
                          + ','.join(f'q{i}' for i in range(1, 8)) + ',pos_margin_rad,pos_margin_joint,'
                          'vel_margin_rads,vel_margin_joint\n')
            self.motion[arm] = {'csv': csv, 'cmd': None, 'cmd_t': 0.0, 'step_max': 0.0, 't_state': None, 'mode': None,
                                'hist': collections.deque(maxlen=40), 'peak_t': 0.0}
            self.create_subscription(FrankaRobotState, f'/{arm}/robot_state_10hz',
                                     lambda m, a=arm: self.on_robot_state(a, m), 10)
            self.create_subscription(JointState, f'/{arm}/gello/joint_states',
                                     lambda m, a=arm: self.on_cmd(a, m), 50)
        self.prev = {}
        self.t_status = time.monotonic()
        self.status_gap = False
        self.cam_low = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/demo/status', self.on_status, latched)
        self.create_subscription(String, '/demo/desk/status', self.on_desk, latched)
        for arm in ARMS:
            self.create_subscription(String, f'/{arm}/gello_gate/state',
                                     lambda m, a=arm: self.change(f'gate_topic.{a}', m.data), latched)
        self.create_timer(1.0, self.watchdog)
        self.write('recorder started')

    def write(self, text):
        line = f'{time.strftime("%H:%M:%S")}.{int(time.time() * 1000) % 1000:03d}  {text}'
        self.f.write(line + '\n')
        print(line, flush=True)

    def change(self, key, value):
        if self.prev.get(key) != value:
            self.prev[key] = value
            self.write(f'{key:22s} {value}')

    def on_status(self, m):
        self.t_status = time.monotonic()
        if self.status_gap:
            self.status_gap = False
            self.write('status stream back')
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        self.change('STATE', f'{s.get("state")}: {s.get("message", "")}')
        for arm in ARMS:
            a = s.get('arms', {}).get(arm, {})
            self.change(f'{arm}.robot', f'{a.get("robot_mode")} errors={a.get("errors")}')
            self.change(f'{arm}.gate', a.get('gate'))
            self.change(f'{arm}.gello', a.get('gello'))
            self.change(f'{arm}.gripper', a.get('gripper'))
        self.change('pedal', s.get('pedal'))
        self.change('cameras_launch', s.get('cameras_launch'))
        self.cameras_running = s.get('cameras_launch') == 'running'
        for k, fps in (s.get('cameras') or {}).items() if self.cameras_running else ():
            low = fps < 20
            if low != self.cam_low.get(k, False):
                self.cam_low[k] = low
                self.write(f'camera {k:12s} {"LOW" if low else "ok"} {fps:.1f} fps')

    def on_desk(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        for arm in ARMS:
            a = d.get(arm, {})
            self.change(f'desk.{arm}', f'{a.get("mode")}, brakes {a.get("brakes")}, FCI {a.get("fci")}, '
                                       f'control {a.get("control")}' + (f', ERROR {a["error"]}' if a.get('error') else ''))
        self.change('desk.busy', d.get('busy'))
        self.change('desk.last', d.get('last'))

    # -------------------------------------------------------------- motion data (S2)
    def on_cmd(self, arm, m):
        mo = self.motion[arm]
        q = list(m.position[:7])
        if len(q) < 7:
            return
        if mo['cmd'] is not None:
            step = max(abs(a - b) for a, b in zip(q, mo['cmd']))
            if time.monotonic() - mo['cmd_t'] < 0.2:          # consecutive samples only
                mo['step_max'] = max(mo['step_max'], step)
                if step > self.step_warn:
                    self.peak(arm, f'command step {step:.3f} rad')
        mo['cmd'], mo['cmd_t'] = q, time.monotonic()

    def on_robot_state(self, arm, m):
        mo = self.motion[arm]
        mo['t_state'] = time.monotonic()
        w = m.o_f_ext_hat_k.wrench.force
        f = math.sqrt(w.x ** 2 + w.y ** 2 + w.z ** 2)
        tau = max((abs(x) for x in m.tau_ext_hat_filtered.effort), default=0.0)
        q = list(m.measured_joint_state.position[:7])
        dq = max((abs(x) for x in m.measured_joint_state.velocity), default=0.0)
        err = (max(abs(a - b) for a, b in zip(mo['cmd'], q))
               if mo['cmd'] is not None and len(q) == 7 and time.monotonic() - mo['cmd_t'] < 0.2 else 0.0)
        ci = m.collision_indicators
        contact = int(any((ci.is_cartesian_linear_contact.x, ci.is_cartesian_linear_contact.y,
                           ci.is_cartesian_linear_contact.z, *ci.is_joint_contact)))
        collision = int(any((ci.is_cartesian_linear_collision.x, ci.is_cartesian_linear_collision.y,
                             ci.is_cartesian_linear_collision.z, *ci.is_joint_collision)))
        mode = {1: 'IDLE', 2: 'MOVE', 4: 'REFLEX', 5: 'USER_STOPPED'}.get(m.robot_mode, str(m.robot_mode))
        vel = list(m.measured_joint_state.velocity[:7])
        if len(q) == 7 and len(vel) == 7:
            pm = min((min(x - lo, hi - x), j) for j, (x, lo, hi) in enumerate(zip(q, Q_MIN, Q_MAX)))
            vm = min((fr3_vel_limit(j, x, 1 if v >= 0 else -1) - abs(v), j) for j, (x, v) in enumerate(zip(q, vel)))
            extra = ','.join(f'{x:.4f}' for x in q) + f',{pm[0]:.4f},j{pm[1] + 1},{vm[0]:.3f},j{vm[1] + 1}'
            if pm[0] < 0.05:
                self.peak(arm, f'j{pm[1] + 1} {pm[0]:.3f} rad from the usable FR3 range')
            if vm[0] < 0.2:
                self.peak(arm, f'j{vm[1] + 1} only {vm[0]:.2f} rad/s below its position-dependent speed limit')
        else:
            extra = ',' * 10
        step, mo['step_max'] = mo['step_max'], 0.0
        row = dict(f=f, tau=tau, dq=dq, err=err, step=step, contact=contact, collision=collision)
        mo['hist'].append((time.monotonic(), row))
        mo['csv'].write(f'{time.time():.3f},{mode},{f:.2f},{w.x:.2f},{w.y:.2f},{w.z:.2f},{tau:.2f},{dq:.3f},'
                        f'{err:.4f},{step:.4f},{contact},{collision},{m.control_command_success_rate:.3f},{extra}\n')
        if f > self.force_warn:
            self.peak(arm, f'|F_ext| {f:.1f} N (tau_ext {tau:.1f} Nm, dq {dq:.2f} rad/s, cmd err {err:.3f} rad)')
        if mode == 'REFLEX' and mo['mode'] != 'REFLEX':
            self.summary(arm, 'robot went to REFLEX')
        mo['mode'] = mode

    def peak(self, arm, text):
        mo = self.motion[arm]
        if time.monotonic() - mo['peak_t'] >= 0.5:          # at most 2 PEAK lines per second and arm
            mo['peak_t'] = time.monotonic()
            self.write(f'PEAK {arm:5s}             {text}')

    def summary(self, arm, why):
        mo = self.motion[arm]
        rows = [r for t, r in mo['hist'] if mo['t_state'] - t <= 3.0]
        if rows:
            mx = {k: max(r[k] for r in rows) for k in rows[0]}
            self.write(f'LAST 3 s {arm:5s}         ({why}) max |F_ext| {mx["f"]:.1f} N, tau_ext {mx["tau"]:.1f} Nm, '
                       f'dq {mx["dq"]:.2f} rad/s, cmd err {mx["err"]:.3f} rad, cmd step {mx["step"]:.3f} rad, '
                       f'contact {"yes" if mx["contact"] else "no"}, collision {"yes" if mx["collision"] else "no"}')
        mo['hist'].clear()

    def watchdog(self):
        if not self.status_gap and time.monotonic() - self.t_status > 3.0:
            self.status_gap = True
            self.write('NO /demo/status for > 3 s (supervisor not running?)')
        for arm, mo in self.motion.items():
            if mo['t_state'] is not None and mo['hist'] and time.monotonic() - mo['t_state'] > 1.0:
                self.summary(arm, 'robot state stopped')
                mo['cmd'] = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/robot_demo/logs/test_{time.strftime("%Y%m%d_%H%M%S")}/events.log'))
    ap.add_argument('--force-warn', type=float, default=15.0, help='PEAK line above this |F_ext| [N]')
    ap.add_argument('--step-warn', type=float, default=0.05, help='PEAK line above this command step [rad]')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    rclpy.init()
    node = Recorder(a.out, a.force_warn, a.step_warn)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write('recorder stopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
