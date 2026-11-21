#!/usr/bin/env python3
"""START GATE for GELLO teleop of ONE FR3 arm (run one process per arm).

Why: franka_fr3_arm_controllers' joint_impedance_controller fixes its motion-generator start time
in on_activate(). If GELLO data reaches it later, the ramp is already "over" and the robot steps to
the GELLO pose at full stiffness. Also, its ramp flags are only initialised at construction, so a
second activation of the same instance skips the ramp entirely.

Topics/services (arm = left|right):
  in : /gello_check_<arm>/gello/joint_states                       (GELLO, controllers don't hear it)
       /gello_check_<arm>/gripper/gripper_client/target_gripper_width_percent
       /<arm>/franka/joint_states                                  (robot, joint_state_broadcaster)
  out: /<arm>/gello/joint_states                                   (what the controller follows)
       /<arm>/gripper/gripper_client/target_gripper_width_percent (only in LIVE)
       /<arm>/gello_gate/state  std_msgs/String, transient local   ("STATE: detail")
  srv: /<arm>/controller_manager/{list,switch,unload,load,configure}_controller

States:
  WAIT_STILL  GELLO OUT OF ITS HOLDER and still (< still_tol over still_time) and robot data fresh,
              else REFUSED after wait_timeout. While the GELLO is in its holder (every joint within
              holder_tol of the recorded holder pose, config/gello_holder_poses.yaml) the robot never moves.
  -> if the snapshot is outside the FR3 joint limits (minus limit_margin): REFUSED
  -> (optional) if any |gello - robot| > max_start_diff: REFUSED (default: off)
  -> snapshot GELLO; relay snapshot (re-stamped, relay_hz) BEFORE activation;
     fresh controller instance (unload/load/configure), wait until DDS matched its subscription;
     activate (STRICT)
  MOVING      controller ramps to snapshot (speed factor 0.2); at least min_moving_time;
              FAULT after move_timeout
  HOLD        robot within converge_tol of snapshot; wait for live GELLO within live_tol of snapshot
  LIVE        relay live GELLO (+ gripper). Jump guard: steps > jump_tol not relayed, 3 in a row = FAULT
TARGET MODE (HOME): --target-joints q1,..,q7 - no GELLO, no holder check, no LIVE: the same fresh-instance +
  snapshot-relay + ramp path drives the arm to the given pose; once converged it holds for target_hold s,
  deactivates (robot holds) and exits 0 with state DONE.
Exclusive output: the gate refuses to start if anything else publishes /<arm>/gello/joint_states
  (a second gate, or a GELLO publisher wired straight to the controller), and FAULTs if one appears later.
Any FAULT, stale data (> stale_timeout), external deactivation, SIGINT or SIGTERM:
  relay the robot's current pose while deactivating the controller (robot holds), then exit.
Exit codes: 0 stopped by signal, 2 REFUSED, 3 FAULT, 4 DEACTIVATION FAILED, 5 startup error.
"""
import argparse
import collections
import math
import os
import signal
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (ConfigureController, ListControllers, LoadController,
                                         SwitchController, UnloadController)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import yaml

CONTROLLER = 'joint_impedance_controller'
GELLO_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
EXIT_STOPPED, EXIT_REFUSED, EXIT_FAULT, EXIT_DEACT_FAILED, EXIT_STARTUP = 0, 2, 3, 4, 5
# FR3 joint limits from franka_description 1.3.0 robots/fr3/joint_limits.yaml
FR3_Q_MIN = [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508]
FR3_Q_MAX = [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]
DEFAULT_HOLDER_FILE = os.path.expanduser('~/robot_demo/config/gello_holder_poses.yaml')


def load_holder_pose(path, arm):
    """Recorded GELLO holder pose for this arm (7 floats) or None."""
    try:
        with open(path) as f:
            q = (yaml.safe_load(f) or {}).get(arm)
    except OSError:
        return None
    return [float(x) for x in q] if q and len(q) == 7 else None


class GateError(Exception):
    def __init__(self, code, msg):
        super().__init__(msg)
        self.code = code


class StopRequested(Exception):
    pass


def maxabs(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def fmt(q):
    return '[' + ', '.join(f'{x:+.3f}' for x in q) + ']'


class GelloGate(Node):
    def __init__(self, a):
        super().__init__('gello_gate', namespace=a.arm)
        self.a = a
        self.stop_requested = False
        self.stopping = False           # in deactivate(): signals no longer interrupt calls
        self.activated = False          # we activated the controller and have not deactivated it
        self.state = ''
        self.relay_target = None        # re-stamped & published at relay_hz when set
        self.next_relay = 0.0
        self.gello = None               # (t_monotonic, [7])
        self.gello_hist = collections.deque()
        self.robot = None               # (t_monotonic, [7])
        self.robot_idx = None
        self.last_relayed = None
        self.jumps = 0
        self.holder = None if a.no_holder_check else load_holder_pose(a.holder_file, a.arm)
        self.target = a.target_joints     # None = normal GELLO teleop
        self.done = False

        cm = f'/{a.arm}/controller_manager'
        self.cli = {
            'list': self.create_client(ListControllers, f'{cm}/list_controllers'),
            'switch': self.create_client(SwitchController, f'{cm}/switch_controller'),
            'unload': self.create_client(UnloadController, f'{cm}/unload_controller'),
            'load': self.create_client(LoadController, f'{cm}/load_controller'),
            'configure': self.create_client(ConfigureController, f'{cm}/configure_controller'),
        }
        self.pub_cmd = self.create_publisher(JointState, f'/{a.arm}/gello/joint_states', 10)
        self.pub_grip = self.create_publisher(
            Float32, f'/{a.arm}/gripper/gripper_client/target_gripper_width_percent', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_state = self.create_publisher(String, f'/{a.arm}/gello_gate/state', latched)
        self.create_subscription(JointState, f'/gello_check_{a.arm}/gello/joint_states',
                                 self.on_gello, 10)
        self.create_subscription(Float32, f'/gello_check_{a.arm}/gripper/gripper_client/'
                                 'target_gripper_width_percent', self.on_gripper, 10)
        self.create_subscription(JointState, f'/{a.arm}/franka/joint_states', self.on_robot, 10)
        # strong ref: Node.executor is a weakref property (assigning it lost the executor)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)

    # ------------------------------------------------------------------ helpers
    def set_state(self, state, detail=''):
        if state != self.state or detail:
            self.get_logger().info(f'{state}{": " + detail if detail else ""}')
        self.state = state
        self.pub_state.publish(String(data=f'{state}: {detail}' if detail else state))

    def publish_cmd(self, q, stamp=None):
        msg = JointState()
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = 'fr3_link0'
        msg.name = GELLO_NAMES
        msg.position = list(q)
        self.pub_cmd.publish(msg)
        self.last_relayed = list(q)

    def relay_tick(self):
        if self.relay_target is not None and time.monotonic() >= self.next_relay:
            self.next_relay = time.monotonic() + 1.0 / self.a.relay_hz
            self.publish_cmd(self.relay_target)

    def spin_for(self, seconds):
        """Spin callbacks and keep relaying; raises StopRequested on signal."""
        end = time.monotonic() + seconds
        while True:
            self._exec.spin_once(timeout_sec=0.002)
            self.relay_tick()
            if self.stop_requested and not self.stopping:
                raise StopRequested()
            if time.monotonic() >= end:
                return

    def call(self, name, req, timeout=5.0):
        cli = self.cli[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            raise GateError(EXIT_FAULT, f'service {cli.srv_name} not available')
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done():
            self._exec.spin_once(timeout_sec=0.002)
            self.relay_tick()
            if self.stop_requested and not self.stopping:
                raise StopRequested()
            if time.monotonic() > end:
                raise GateError(EXIT_FAULT, f'service {cli.srv_name} timed out')
        return fut.result()

    def controller_states(self):
        res = self.call('list', ListControllers.Request())
        return {c.name: c.state for c in res.controller}

    def switch(self, activate=(), deactivate=(), strict=True):
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT if strict else \
            SwitchController.Request.BEST_EFFORT
        req.activate_asap = False
        req.timeout = Duration(sec=3)
        return self.call('switch', req, timeout=5.0).ok

    def wait_subs(self, cond, what, timeout=3.0):
        end = time.monotonic() + timeout
        while not cond(self.pub_cmd.get_subscription_count()):
            if time.monotonic() > end:
                raise GateError(EXIT_FAULT, f'timeout waiting for {what} '
                                f'({self.pub_cmd.get_subscription_count()} subscribers)')
            self.spin_for(0.01)

    def fresh(self, sample, now=None):
        return sample is not None and (now or time.monotonic()) - sample[0] < self.a.stale_timeout

    # ------------------------------------------------------------------ callbacks
    def on_robot(self, msg):
        if self.robot_idx is None:
            idx = []
            for jn in GELLO_NAMES:
                hits = [i for i, n in enumerate(msg.name) if n.endswith(jn)]
                if len(hits) != 1:
                    return
                idx.append(hits[0])
            self.robot_idx = idx
            self.get_logger().info(f'robot joints: {[msg.name[i] for i in idx]}')
        self.robot = (time.monotonic(), [msg.position[i] for i in self.robot_idx])

    def on_gello(self, msg):
        if len(msg.position) < 7:
            return
        now = time.monotonic()
        q = list(msg.position[:7])
        self.gello = (now, q)
        self.gello_hist.append((now, q))
        while self.gello_hist and now - self.gello_hist[0][0] > self.a.still_time + 0.2:
            self.gello_hist.popleft()
        if self.state == 'LIVE' and self.activated:
            step = maxabs(q, self.last_relayed) if self.last_relayed else 0.0
            if step > self.a.jump_tol:
                self.jumps += 1
                self.get_logger().warn(f'GELLO jump {step:.3f} rad not relayed ({self.jumps}/3)')
                return
            self.jumps = 0
            self.publish_cmd(q)   # fresh stamp: staleness is checked by us (stale_timeout)

    def on_gripper(self, msg):
        if self.state == 'LIVE' and self.activated:
            self.pub_grip.publish(msg)

    def gello_still(self):
        if not self.fresh(self.gello) or len(self.gello_hist) < 3:
            return False
        t_last, q_last = self.gello_hist[-1]
        window = [q for t, q in self.gello_hist if t_last - t <= self.a.still_time]
        span = t_last - next(t for t, q in self.gello_hist if t_last - t <= self.a.still_time)
        if span < 0.9 * self.a.still_time:
            return False
        return all(maxabs(q, q_last) < self.a.still_tol for q in window)

    def in_holder(self, q):
        return self.holder is not None and maxabs(q, self.holder) <= self.a.holder_tol

    def other_publishers(self):
        """Publishers on the controller's command topic besides this gate."""
        return self.count_publishers(self.pub_cmd.topic_name) - 1

    def check_streams(self):
        if self.other_publishers() > 0:
            raise GateError(EXIT_FAULT, f'another publisher appeared on {self.pub_cmd.topic_name}')
        now = time.monotonic()
        if self.target is None and not self.fresh(self.gello, now):
            raise GateError(EXIT_FAULT, 'GELLO data lost')
        if not self.fresh(self.robot, now):
            raise GateError(EXIT_FAULT, 'robot joint states lost')

    # ------------------------------------------------------------------ main sequence
    def run(self):
        a = self.a
        self.set_state('WAIT_STILL', 'waiting for controller manager')
        for name in ('list', 'switch', 'unload', 'load', 'configure'):
            while not self.cli[name].wait_for_service(timeout_sec=0.1):
                self.spin_for(0.0)
        states = self.controller_states()
        if states.get(CONTROLLER) != 'inactive' or states.get('joint_state_broadcaster') != 'active':
            raise GateError(EXIT_FAULT, f'expected {CONTROLLER}=inactive and '
                            f'joint_state_broadcaster=active, got {states}')

        self.spin_for(1.0)   # DDS discovery, so a second publisher on our output topic is visible
        if self.other_publishers() > 0:
            raise GateError(EXIT_REFUSED, f'{self.other_publishers()} other publisher(s) on '
                            f'{self.pub_cmd.topic_name} (another gate, or GELLO wired straight to the '
                            'controller?) - not starting')
        if self.target is not None:
            return self.run_target()
        if self.holder is None and not a.no_holder_check:
            raise GateError(EXIT_REFUSED, f'no GELLO holder pose for "{a.arm}" in {a.holder_file} - '
                            f'run scripts/record_gello_holder.py --arm {a.arm} with the GELLO in its holder')

        # -- WAIT_STILL: GELLO out of its holder and still
        self.set_state('WAIT_STILL', 'take GELLO out of its holder and hold it still')
        t0 = time.monotonic()
        last_hint = ''
        while True:
            if self.other_publishers() > 0:
                raise GateError(EXIT_REFUSED, f'another publisher appeared on {self.pub_cmd.topic_name}')
            fresh_g = self.fresh(self.gello)
            holder = fresh_g and self.in_holder(self.gello[1])
            if fresh_g and not holder and self.gello_still() and self.fresh(self.robot):
                break
            hint = 'GELLO in holder - robot will not move' if holder else 'hold GELLO still'
            if hint != last_hint:
                last_hint = hint
                self.set_state('WAIT_STILL', hint)
            if time.monotonic() - t0 > a.wait_timeout:
                why = 'no GELLO data' if not fresh_g else \
                    'GELLO stayed in its holder' if holder else \
                    'no robot data' if not self.fresh(self.robot) else 'GELLO not still'
                raise GateError(EXIT_REFUSED, f'{why} within {a.wait_timeout:.0f} s')
            self.spin_for(0.01)

        snapshot = list(self.gello[1])
        robot_q = list(self.robot[1])
        diff = [abs(g - r) for g, r in zip(snapshot, robot_q)]
        self.get_logger().info(f'GELLO {fmt(snapshot)}\n          robot {fmt(robot_q)}')
        self.check_limits(snapshot, 'GELLO pose')
        if max(diff) > a.max_start_diff:
            bad = ', '.join(f'j{i + 1} {d:.2f}' for i, d in enumerate(diff) if d > a.max_start_diff)
            raise GateError(EXIT_REFUSED, f'GELLO too far from robot (> {a.max_start_diff} rad): '
                            f'{bad}. Match GELLO to the robot and START again.')

        self.set_state('ACTIVATING', f'max diff {max(diff):.3f} rad')
        self.activate_and_move(snapshot, 'GELLO snapshot')

        # -- HOLD
        self.set_state('HOLD', f'robot at snapshot; move GELLO within {a.live_tol} rad to go LIVE')
        last_hint = time.monotonic()
        while maxabs(self.gello[1], snapshot) > a.live_tol:
            self.check_streams()
            if time.monotonic() - last_hint > 2.0:
                last_hint = time.monotonic()
                d = [g - s for g, s in zip(self.gello[1], snapshot)]
                far = ', '.join(f'j{i + 1} {x:+.2f}' for i, x in enumerate(d) if abs(x) > a.live_tol)
                self.set_state('HOLD', f'GELLO off snapshot: {far}')
            self.spin_for(0.01)
        self.run_live(snapshot)

    def check_limits(self, q, what):
        a = self.a
        out = [f'j{i + 1} {x:+.3f} not in [{lo + a.limit_margin:+.3f}, {hi - a.limit_margin:+.3f}]'
               for i, (x, lo, hi) in enumerate(zip(q, FR3_Q_MIN, FR3_Q_MAX))
               if not lo + a.limit_margin <= x <= hi - a.limit_margin]
        if out:
            raise GateError(EXIT_REFUSED, f'{what} outside FR3 joint limits: ' + ', '.join(out))

    def run_target(self):
        """HOME / target mode: ramp to a fixed pose, hold briefly, then deactivate (in main)."""
        a = self.a
        target = list(self.target)
        self.check_limits(target, 'target pose')
        self.set_state('WAIT_STILL', 'waiting for robot data')
        t0 = time.monotonic()
        while not self.fresh(self.robot):
            if time.monotonic() - t0 > a.wait_timeout:
                raise GateError(EXIT_REFUSED, f'no robot data within {a.wait_timeout:.0f} s')
            self.spin_for(0.01)
        diff = maxabs(target, self.robot[1])
        self.get_logger().info(f'target {fmt(target)}\n          robot  {fmt(self.robot[1])}')
        self.set_state('ACTIVATING', f'target mode, max diff {diff:.3f} rad')
        self.activate_and_move(target, 'target pose')
        self.set_state('HOLD', f'at target pose, holding {a.target_hold:.1f} s')
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.target_hold:
            self.check_streams()
            self.spin_for(0.01)
        self.done = True              # main() deactivates -> robot holds at the target

    def activate_and_move(self, snapshot, what):
        """Relay snapshot BEFORE activation, fresh controller instance, activate, ramp until converged."""
        a = self.a
        self.relay_target = snapshot
        self.spin_for(0.2)
        n_subs = self.pub_cmd.get_subscription_count()   # includes the old controller instance
        self.call('unload', UnloadController.Request(name=CONTROLLER))
        self.wait_subs(lambda n: n < n_subs, 'old controller subscription to disappear')
        if not self.call('load', LoadController.Request(name=CONTROLLER)).ok:
            raise GateError(EXIT_FAULT, 'load_controller failed')
        if not self.call('configure', ConfigureController.Request(name=CONTROLLER)).ok:
            raise GateError(EXIT_FAULT, 'configure_controller failed')
        if self.controller_states().get(CONTROLLER) != 'inactive':
            raise GateError(EXIT_FAULT, 'reloaded controller is not inactive')
        # The new instance subscribes in on_configure: make sure DDS has matched it before activating,
        # otherwise it misses the snapshot and initialises its ramp on later (live) data.
        self.wait_subs(lambda n: n >= n_subs, 'new controller subscription to match')
        self.spin_for(0.2)
        self.check_streams()
        self.activated = True           # set first: if the call times out we still deactivate
        if not self.switch(activate=[CONTROLLER]):
            raise GateError(EXIT_FAULT, 'switch_controller (activate) refused')

        # -- MOVING
        self.set_state('MOVING', f'ramping to {what} (speed factor 0.2)')
        t0 = time.monotonic()
        while (time.monotonic() - t0 < a.min_moving_time
               or maxabs(self.robot[1], snapshot) > a.converge_tol):
            self.check_streams()
            if time.monotonic() - t0 > a.move_timeout:
                raise GateError(EXIT_FAULT, f'robot did not reach {what} within {a.move_timeout:.0f} s '
                                f'(err {maxabs(self.robot[1], snapshot):.3f} rad)')
            self.spin_for(0.01)

    def run_live(self, snapshot):
        # -- LIVE (on_gello relays from here on)
        self.last_relayed = list(snapshot)
        self.relay_target = None
        self.set_state('LIVE')
        next_check = time.monotonic() + 1.0
        while True:
            self.check_streams()
            if self.jumps >= 3:
                raise GateError(EXIT_FAULT, 'GELLO jumped 3 times in a row')
            if time.monotonic() > next_check:
                next_check = time.monotonic() + 1.0
                st = self.controller_states().get(CONTROLLER)
                if st != 'active':
                    self.activated = False
                    raise GateError(EXIT_FAULT, f'{CONTROLLER} is "{st}" (deactivated externally?)')
            self.spin_for(0.01)

    def deactivate(self):
        """Hold the robot where it is and deactivate the controller. Returns True if done/needed-not."""
        if not self.activated:
            return True
        self.stopping = True
        self.set_state('STOPPING', 'deactivating controller')
        if self.fresh(self.robot):
            self.relay_target = list(self.robot[1])
        elif self.last_relayed is not None:
            self.relay_target = list(self.last_relayed)
        for attempt in range(3):
            try:
                ok = self.switch(deactivate=[CONTROLLER], strict=False)
                st = self.controller_states().get(CONTROLLER)
                if ok and st != 'active':
                    self.activated = False
                    self.relay_target = None
                    self.get_logger().info(f'{CONTROLLER} deactivated ({st}); robot holds')
                    return True
            except (GateError, StopRequested) as e:
                self.get_logger().error(f'deactivate attempt {attempt + 1}: {e}')
        return False


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--arm', required=True, choices=['left', 'right'])
    p.add_argument('--still-tol', type=float, default=0.03)
    p.add_argument('--still-time', type=float, default=1.0,
                   help='GELLO must be still this long before the snapshot (avoids snapshots mid-lift)')
    p.add_argument('--max-start-diff', type=float, default=math.inf,
                   help='optional extra limit on |gello - robot| per joint (default: off)')
    p.add_argument('--holder-file', default=DEFAULT_HOLDER_FILE)
    p.add_argument('--holder-tol', type=float, default=0.25,
                   help='GELLO counts as "in holder" if every joint is within this of the holder pose')
    p.add_argument('--no-holder-check', action='store_true', help='sim tests only')
    p.add_argument('--limit-margin', type=float, default=0.1,
                   help='refuse snapshots closer than this to an FR3 joint limit')
    p.add_argument('--converge-tol', type=float, default=0.05)
    p.add_argument('--live-tol', type=float, default=0.08)
    p.add_argument('--wait-timeout', type=float, default=30.0)
    p.add_argument('--move-timeout', type=float, default=30.0)
    p.add_argument('--min-moving-time', type=float, default=0.5,
                   help='keep relaying the snapshot at least this long after activation')
    p.add_argument('--stale-timeout', type=float, default=0.3)
    p.add_argument('--jump-tol', type=float, default=0.3)
    p.add_argument('--relay-hz', type=float, default=100.0)
    p.add_argument('--target-joints', type=lambda s: [float(x) for x in s.split(',')], default=None,
                   help='TARGET MODE (HOME): q1,..,q7 in rad; no GELLO, no LIVE')
    p.add_argument('--target-hold', type=float, default=1.0,
                   help='target mode: hold at the target this long before deactivating')
    a = p.parse_args(argv)
    if a.target_joints is not None and len(a.target_joints) != 7:
        p.error('--target-joints needs 7 values')
    return a


def main():
    args = parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # Own signal handling: rclpy's default handler shuts the context down before we can deactivate.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    gate = GelloGate(args)

    def on_signal(signum, _frame):
        if not gate.stop_requested:
            gate.get_logger().warn(f'{signal.Signals(signum).name} received - stopping')
        gate.stop_requested = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    code = EXIT_STOPPED
    try:
        gate.run()
    except StopRequested:
        code = EXIT_STOPPED
    except GateError as e:
        code = e.code
        gate.set_state('REFUSED' if code == EXIT_REFUSED else 'FAULT', str(e))
    except Exception as e:  # noqa: BLE001 - anything unexpected must still deactivate
        code = EXIT_FAULT
        gate.set_state('FAULT', f'unexpected error: {e!r}')
    finally:
        if not gate.deactivate():
            code = EXIT_DEACT_FAILED
            gate.set_state('FAULT', f'COULD NOT DEACTIVATE {CONTROLLER} - use the E-stop / stop the '
                           'arm launch')
            gate.get_logger().fatal('DEACTIVATION FAILED')
        elif code == EXIT_STOPPED:
            gate.set_state('DONE' if gate.done else 'STOPPED')
        try:
            gate._exec.spin_once(timeout_sec=0.05)   # flush last state message
        except Exception:  # noqa: BLE001
            pass
        gate.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
