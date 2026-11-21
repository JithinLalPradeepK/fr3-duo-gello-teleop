#!/usr/bin/env python3
"""Scenario driver for the gello_gate SIM test. Run via test/sim_gate_test.sh (sets domain + safety).

  single --arm left --cm-log <arms launch log>  : scenarios against an already running sim arm launch
  duo    --sim-config <cfg>                      : runs scripts/teleop_duo.sh itself (sim config, --no-gello)

Fake GELLO data is published on /gello_check_<arm>/...; robot pose comes from the mock hardware.
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

DEMO = os.path.expanduser('~/robot_demo')
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
GATE = os.path.join(DEMO, 'scripts', 'gello_gate.py')
TELEOP = os.path.join(DEMO, 'scripts', 'teleop_duo.sh')
NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
CTRL = 'joint_impedance_controller'
# mock hardware start pose (franka_description initial_position); sim holder pose = this with j4 - 0.6
MOCK_BASE = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
SIM_HOLDER = [q - 0.6 if i == 3 else q for i, q in enumerate(MOCK_BASE)]
HOLDER_FILE = None   # set per run (in logdir)


def write_sim_holder(logdir):
    global HOLDER_FILE
    HOLDER_FILE = os.path.join(logdir, 'sim_holder_poses.yaml')
    with open(HOLDER_FILE, 'w') as f:
        yaml.safe_dump({'left': SIM_HOLDER, 'right': SIM_HOLDER}, f)
    return HOLDER_FILE


def safety_check(config_path=None):
    if os.environ.get('ROS_DOMAIN_ID') != '77' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        sys.exit('REFUSING: needs ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1 (use sim_gate_test.sh)')
    if config_path:
        cfg = yaml.safe_load(open(config_path))
        if any(str(c.get('use_fake_hardware')).lower() != 'true' for c in cfg.values()):
            sys.exit(f'REFUSING: {config_path} has an arm without use_fake_hardware "true"')


class Arm:
    def __init__(self, node, arm):
        self.arm = arm
        self.robot = None
        self.gello_fn = None            # t -> [7] or None (silent)
        self.t0 = time.monotonic()
        self.relayed = []               # (t, positions)
        self.grip_relayed = 0
        self.states = []                # (t, text)
        self.lock = threading.Lock()
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(JointState, f'/{arm}/franka/joint_states', self.on_robot, 10)
        node.create_subscription(JointState, f'/{arm}/gello/joint_states', self.on_relay, 100)
        node.create_subscription(Float32, f'/{arm}/gripper/gripper_client/target_gripper_width_percent',
                                 self.on_grip, 10)
        node.create_subscription(String, f'/{arm}/gello_gate/state', self.on_state, latched)
        self.pub = node.create_publisher(JointState, f'/gello_check_{arm}/gello/joint_states', 10)
        self.pub_grip = node.create_publisher(
            Float32, f'/gello_check_{arm}/gripper/gripper_client/target_gripper_width_percent', 10)
        self.list_cli = node.create_client(ListControllers, f'/{arm}/controller_manager/list_controllers')
        self.switch_cli = node.create_client(SwitchController,
                                             f'/{arm}/controller_manager/switch_controller')
        self.node = node
        node.create_timer(0.04, self.publish_gello)   # 25 Hz like the real GELLO

    def on_robot(self, msg):
        q = []
        for jn in NAMES:
            q.append(next(p for n, p in zip(msg.name, msg.position) if n.endswith(jn)))
        self.robot = q

    def on_relay(self, msg):
        with self.lock:
            self.relayed.append((time.monotonic(), list(msg.position)))

    def on_grip(self, _msg):
        self.grip_relayed += 1

    def on_state(self, msg):
        with self.lock:
            self.states.append((time.monotonic(), msg.data))

    def publish_gello(self):
        if self.gello_fn is None:
            return
        q = self.gello_fn(time.monotonic() - self.t0)
        if q is None:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = NAMES
        msg.position = list(q)
        self.pub.publish(msg)
        self.pub_grip.publish(Float32(data=0.5))

    def set_gello(self, fn):
        self.t0 = time.monotonic()
        self.gello_fn = fn

    def reset_records(self):
        with self.lock:
            self.relayed.clear()
            self.states.clear()
        self.grip_relayed = 0

    def _call(self, cli, req, timeout=5.0):
        assert cli.wait_for_service(timeout_sec=timeout), f'{cli.srv_name} unavailable'
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done():
            assert time.monotonic() < end, f'{cli.srv_name} timeout'
            time.sleep(0.01)
        return fut.result()

    def ctrl_state(self):
        res = self._call(self.list_cli, ListControllers.Request())
        return {c.name: c.state for c in res.controller}.get(CTRL)

    def deactivate_externally(self):
        req = SwitchController.Request(deactivate_controllers=[CTRL],
                                       strictness=SwitchController.Request.BEST_EFFORT)
        return self._call(self.switch_cli, req).ok

    def state_names(self):
        with self.lock:
            return [s.split(':')[0] for _, s in self.states]

    def wait_state(self, name, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if name in self.state_names():
                return True
            time.sleep(0.02)
        return False

    def state_time(self, name):
        with self.lock:
            return next((t for t, s in self.states if s.split(':')[0] == name), None)


class Results:
    def __init__(self):
        self.rows = []

    def check(self, scenario, what, ok, detail=''):
        self.rows.append((scenario, what, bool(ok), detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {what}{' - ' + detail if detail else ''}", flush=True)

    def summary(self):
        failed = [r for r in self.rows if not r[2]]
        print(f'\n===== {len(self.rows) - len(failed)}/{len(self.rows)} checks passed', flush=True)
        for s, w, _, d in failed:
            print(f'  FAIL {s}: {w} {d}')
        return 0 if not failed else 1


def start_gate(arm, logdir, tag, extra=()):
    if not no_gate_running():
        print(f'  [WARN] a gate is still running before {tag}', flush=True)
    log = open(os.path.join(logdir, f'gate_{tag}.log'), 'w')
    extra = list(extra)
    if '--holder-file' not in extra:
        extra += ['--holder-file', HOLDER_FILE]
    p = subprocess.Popen([sys.executable, GATE, '--arm', arm, *extra], stdout=log,
                         stderr=subprocess.STDOUT)
    return p


def wait_exit(p, timeout):
    """Exit code, or None on timeout - then the process is stopped (never leave a gate behind)."""
    try:
        return p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
        return None


GATE_PATTERN = r'^(/usr/bin/)?python3 \S*gello_gate\.py'


def no_gate_running():
    return subprocess.run(['pgrep', '-f', GATE_PATTERN], stdout=subprocess.DEVNULL).returncode != 0


def q_goal_lines(cm_log):
    if not cm_log or not os.path.exists(cm_log):
        return []
    pat = re.compile(r'q_goal of motion generator: \[([^\]]*)\]')
    out = []
    for line in open(cm_log, errors='replace'):
        m = pat.search(line)
        if m:
            out.append([float(x) for x in m.group(1).split(',')])
    return out


def check_ramp(a, R, sc, n_goal, snap):
    """Exactly one new controller ramp, initialised on the snapshot (not on later live data)."""
    time.sleep(1.0)
    goals = q_goal_lines(a.cm_log)[n_goal:]
    R.check(sc, 'exactly one new controller ramp', len(goals) == 1, f'{len(goals)} new q_goal lines')
    if goals and snap:
        err = max(abs(x - y) for x, y in zip(goals[0], snap))
        R.check(sc, 'controller ramp goal == snapshot', err < 1e-5, f'max diff {err:.6f}')


def first_relayed(arm):
    with arm.lock:
        return list(arm.relayed[0][1]) if arm.relayed else None


def analyse_prelive(arm, R, sc, snapshot_hint=None):
    """Snapshot relay before LIVE: constant values, small gaps, started before activation."""
    t_act, t_live = arm.state_time('ACTIVATING'), arm.state_time('LIVE')
    t_mov = arm.state_time('MOVING')
    with arm.lock:
        pre = [(t, q) for t, q in arm.relayed if t_live is None or t < t_live]
    R.check(sc, 'snapshot relayed before activation',
            t_mov is not None and len([1 for t, _ in pre if t < t_mov]) >= 10,
            f'{len([1 for t, _ in pre if t_mov and t < t_mov])} msgs before MOVING')
    if len(pre) >= 2:
        snap = pre[0][1]
        const = all(max(abs(a - b) for a, b in zip(q, snap)) < 1e-9 for _, q in pre)
        R.check(sc, 'pre-LIVE relay is the frozen snapshot', const)
        gaps = [b[0] - a[0] for a, b in zip(pre, pre[1:])]
        R.check(sc, 'pre-LIVE relay gap <= 25 ms', max(gaps) <= 0.025,
                f'max gap {max(gaps) * 1000:.1f} ms, mean {sum(gaps) / len(gaps) * 1000:.1f} ms')
        return snap
    R.check(sc, 'pre-LIVE relay messages present', False)
    return None


class FakeOther:
    """Stands in for the OTHER arm of the duo in single-gate tests: its robot joint states, its command topic
    (what its gate relays) and its gate state - all under test control."""

    def __init__(self, node, arm):
        self.arm, self.on, self.q, self.cmd = arm, False, None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.p_robot = node.create_publisher(JointState, f'/{arm}/franka/joint_states', 10)
        self.p_cmd = node.create_publisher(JointState, f'/{arm}/gello/joint_states', 10)
        self.p_state = node.create_publisher(String, f'/{arm}/gello_gate/state', latched)
        self.names = [f'{arm}_fr3_joint{i}' for i in range(1, 8)]
        node.create_timer(0.02, self.tick)

    def pose(self, q):
        self.q, self.cmd = list(q), list(q)

    def state(self, text):
        self.p_state.publish(String(data=text))

    def tick(self):
        if self.on and self.q is not None:
            self.p_robot.publish(JointState(name=self.names, position=self.q))
            self.p_cmd.publish(JointState(name=self.names, position=self.cmd))


def run_single(a, logdir):
    safety_check()
    rclpy.init()
    node = Node('sim_gate_driver')
    arm = Arm(node, a.arm)
    other_arm = 'right' if a.arm == 'left' else 'left'
    fake = FakeOther(node, other_arm)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    R = Results()

    end = time.monotonic() + 30
    while arm.robot is None and time.monotonic() < end:
        time.sleep(0.1)
    if arm.robot is None:
        sys.exit('no /%s/franka/joint_states from sim' % a.arm)
    base = list(arm.robot)
    print(f'mock robot pose: {[round(x, 3) for x in base]}', flush=True)
    if max(abs(x - y) for x, y in zip(base, MOCK_BASE)) > 1e-3:
        sys.exit('mock pose differs from MOCK_BASE - update test')
    write_sim_holder(logdir)

    def offset(j, d):
        q = list(base)
        q[j] += d
        return q

    def to_live(sc, tag, extra=()):
        arm.reset_records()
        arm.set_gello(lambda t: base)
        p = start_gate(a.arm, logdir, tag, extra)
        ok = arm.wait_state('LIVE', 15)
        R.check(sc, 'reaches LIVE', ok, ' -> '.join(dict.fromkeys(arm.state_names())))
        if ok:
            R.check(sc, 'controller active in LIVE', arm.ctrl_state() == 'active')
        return p, ok

    # ---- S1 + S2: two START cycles (fresh controller instance each time), SIGINT then SIGTERM
    for sc, sig in (('S1 match, SIGINT', signal.SIGINT), ('S2 second START, SIGTERM', signal.SIGTERM)):
        print(f'\n== {sc}', flush=True)
        R.check(sc, 'controller inactive before start', arm.ctrl_state() == 'inactive')
        n_goal = len(q_goal_lines(a.cm_log))
        p, ok = to_live(sc, sc.split()[0])
        snap = analyse_prelive(arm, R, sc)
        if ok:
            n_rel = len(arm.relayed)
            arm.set_gello(lambda t: [b + (0.05 * math.sin(2 * math.pi * t / 4) if i == 0 else 0)
                                     for i, b in enumerate(base)])
            time.sleep(3)
            with arm.lock:
                live = arm.relayed[n_rel:]
            dev = max((abs(q[0] - base[0]) for _, q in live), default=0)
            R.check(sc, 'live GELLO motion relayed', dev > 0.03 and len(live) > 50,
                    f'{len(live)} msgs, max j1 dev {dev:.3f}')
            R.check(sc, 'gripper relayed in LIVE', arm.grip_relayed > 10, f'{arm.grip_relayed} msgs')
        p.send_signal(sig)
        rc = wait_exit(p, 15)
        R.check(sc, 'gate exit code 0', rc == 0, f'rc={rc}')
        R.check(sc, 'controller inactive after stop', arm.ctrl_state() == 'inactive')
        R.check(sc, 'final state STOPPED', 'STOPPED' in arm.state_names())
        check_ramp(a, R, sc, n_goal, snap)

    # ---- S3 / S3b: GELLO in holder (exact / within holder_tol) -> robot never moves, REFUSED
    for sc, pose in (('S3 GELLO in holder', SIM_HOLDER),
                     ('S3b GELLO in holder +0.2 j7', [q + 0.2 if i == 6 else q
                                                      for i, q in enumerate(SIM_HOLDER)])):
        print(f'\n== {sc}', flush=True)
        arm.reset_records()
        arm.set_gello(lambda t, pose=pose: pose)
        p = start_gate(a.arm, logdir, sc.split()[0], ['--wait-timeout', '4'])
        seen_active = False
        while p.poll() is None:
            seen_active |= arm.ctrl_state() == 'active'
            time.sleep(0.1)
        with arm.lock:
            texts = [t for _, t in arm.states]
        R.check(sc, 'gate exit code 2 (REFUSED)', p.returncode == 2, f'rc={p.returncode}')
        R.check(sc, 'status said GELLO in holder', any('in holder' in t for t in texts))
        R.check(sc, 'reason: stayed in its holder', any('stayed in its holder' in t for t in texts),
                texts[-1] if texts else '')
        R.check(sc, 'controller never active', not seen_active and arm.ctrl_state() == 'inactive')
        R.check(sc, 'nothing relayed to controller topic', len(arm.relayed) == 0,
                f'{len(arm.relayed)} msgs')

    # ---- S4: GELLO never still -> REFUSED after wait timeout
    sc = 'S4 GELLO moving'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: offset(0, 0.1 * math.sin(2 * math.pi * t)))
    p = start_gate(a.arm, logdir, 'S4', ['--wait-timeout', '4'])
    rc = wait_exit(p, 15)
    with arm.lock:
        last = arm.states[-1][1] if arm.states else ''
    R.check(sc, 'gate exit code 2 (REFUSED)', rc == 2, f'rc={rc}')
    R.check(sc, 'reason: not still', 'not still' in last, last)
    R.check(sc, 'controller inactive, nothing relayed',
            arm.ctrl_state() == 'inactive' and len(arm.relayed) == 0)

    # ---- S5: mock robot cannot move -> MOVING timeout -> FAULT, deactivated
    sc = 'S5 MOVING timeout'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: offset(0, 0.5))
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S5', ['--move-timeout', '3'])
    
    went_active = arm.wait_state('MOVING', 10) and arm.ctrl_state() == 'active'
    R.check(sc, 'controller activated for the ramp', went_active)
    rc = wait_exit(p, 15)
    with arm.lock:
        last = [s for _, s in arm.states if s.startswith('FAULT')]
    R.check(sc, 'gate exit code 3 (FAULT)', rc == 3, f'rc={rc} {last[-1] if last else ""}')
    R.check(sc, 'controller inactive after fault', arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, first_relayed(arm))

    # ---- S6: GELLO stream lost in LIVE
    sc = 'S6 GELLO lost in LIVE'
    print(f'\n== {sc}', flush=True)
    n_goal = len(q_goal_lines(a.cm_log))
    p, ok = to_live(sc, 'S6')
    
    t_cut = time.monotonic()
    arm.set_gello(lambda t: None)
    rc = wait_exit(p, 10)
    t_fault = arm.state_time('FAULT') or arm.state_time('STOPPING')
    R.check(sc, 'gate exit code 3 (FAULT)', rc == 3, f'rc={rc}')
    R.check(sc, 'reacted within 0.45 s (before controller 0.5 s self-shutdown)',
            t_fault is not None and t_fault - t_cut < 0.45,
            f'{(t_fault - t_cut) * 1000:.0f} ms' if t_fault else 'no FAULT state')
    try:
        st = arm.ctrl_state()
        R.check(sc, 'controller_manager alive and controller inactive', st == 'inactive', st)
    except AssertionError as e:
        R.check(sc, 'controller_manager alive and controller inactive', False, str(e))
    check_ramp(a, R, sc, n_goal, first_relayed(arm))

    # ---- S7: jump guard
    sc = 'S7 GELLO jump in LIVE'
    print(f'\n== {sc}', flush=True)
    n_goal = len(q_goal_lines(a.cm_log))
    p, ok = to_live(sc, 'S7')
    
    arm.set_gello(lambda t: offset(0, 0.5))
    rc = wait_exit(p, 10)
    with arm.lock:
        jumped = [q for _, q in arm.relayed if abs(q[0] - base[0]) > 0.3]
    R.check(sc, 'gate exit code 3 (FAULT)', rc == 3, f'rc={rc}')
    R.check(sc, 'jump never relayed', not jumped, f'{len(jumped)} jump msgs relayed')
    R.check(sc, 'controller inactive', arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, first_relayed(arm))

    # ---- S8: external deactivation (e.g. future supervisor STOP)
    sc = 'S8 external deactivation'
    print(f'\n== {sc}', flush=True)
    n_goal = len(q_goal_lines(a.cm_log))
    p, ok = to_live(sc, 'S8')
    
    arm.deactivate_externally()
    rc = wait_exit(p, 10)
    R.check(sc, 'gate notices and exits 3', rc == 3, f'rc={rc}')
    R.check(sc, 'controller inactive', arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, first_relayed(arm))

    # ---- S9: GELLO near an FR3 joint limit -> gate keeps waiting with a hint, REFUSED only at wait timeout
    sc = 'S9 joint limit (waits, then REFUSED at timeout)'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: offset(5, -1.1))     # j6 0.471 < 0.4398 + 0.1, far from holder
    p = start_gate(a.arm, logdir, 'S9', ['--wait-timeout', '4'])
    rc = wait_exit(p, 15)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED)', rc == 2, f'rc={rc}')
    R.check(sc, 'hint: move GELLO j6 away from its joint limit',
            any('move GELLO j6 away from its joint limit' in t for t in texts), str(texts[-3:]))
    R.check(sc, 'reason: stayed near the joint limit', any('near the joint limit of j6' in t for t in texts),
            texts[-1] if texts else '')
    R.check(sc, 'controller inactive, nothing relayed',
            arm.ctrl_state() == 'inactive' and len(arm.relayed) == 0)

    # ---- S9b: near the limit for 2.5 s, then moved away -> START continues (no new START needed)
    sc = 'S9b joint limit, then moved away -> LIVE'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: offset(5, -1.1) if t < 2.5 else base)
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S9b', ['--wait-timeout', '15'])
    ok = arm.wait_state('LIVE', 20)
    R.check(sc, 'reaches LIVE', ok, ' -> '.join(dict.fromkeys(arm.state_names())))
    with arm.lock:
        early = [t for t, _ in arm.relayed if t - arm.t0 < 2.5]
    R.check(sc, 'nothing relayed while near the limit', not early, f'{len(early)} msgs')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, base)

    # ---- S10: far pose (1.2 rad, out of holder, within limits) is now ACCEPTED -> ramp to it
    sc = 'S10 far pose accepted'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    far = offset(0, 1.2)
    arm.set_gello(lambda t: far)
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S10', ['--move-timeout', '3'])
    went_active = arm.wait_state('MOVING', 10) and arm.ctrl_state() == 'active'
    R.check(sc, 'controller activated for the ramp', went_active)
    rc = wait_exit(p, 15)     # mock cannot move -> MOVING timeout
    R.check(sc, 'gate exit 3 after MOVING timeout (mock cannot move)', rc == 3, f'rc={rc}')
    R.check(sc, 'controller inactive', arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, far)

    # ---- S11: no holder pose recorded -> REFUSED (fail safe)
    sc = 'S11 no holder pose'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: base)
    p = start_gate(a.arm, logdir, 'S11', ['--holder-file', os.path.join(logdir, 'does_not_exist.yaml')])
    rc = wait_exit(p, 15)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED)', rc == 2, f'rc={rc}')
    R.check(sc, 'reason: no holder pose', any('no GELLO holder pose' in t for t in texts))
    R.check(sc, 'controller inactive, nothing relayed',
            arm.ctrl_state() == 'inactive' and len(arm.relayed) == 0)

    # ---- S13: rogue publisher on the controller topic before START -> REFUSED
    sc = 'S13 rogue publisher before start'
    print(f'\n== {sc}', flush=True)
    rogue = node.create_publisher(JointState, f'/{a.arm}/gello/joint_states', 10)
    time.sleep(1.0)
    arm.reset_records()
    arm.set_gello(lambda t: base)
    p = start_gate(a.arm, logdir, 'S13')
    seen_active = False
    while p.poll() is None:
        seen_active |= arm.ctrl_state() == 'active'
        time.sleep(0.1)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED)', p.returncode == 2, f'rc={p.returncode}')
    R.check(sc, 'reason: other publisher', any('other publisher' in t for t in texts),
            texts[-1] if texts else '')
    R.check(sc, 'controller never active', not seen_active and arm.ctrl_state() == 'inactive')
    node.destroy_publisher(rogue)
    time.sleep(1.5)

    # ---- S14: rogue publisher appears while LIVE -> FAULT, deactivated
    sc = 'S14 rogue publisher in LIVE'
    print(f'\n== {sc}', flush=True)
    n_goal = len(q_goal_lines(a.cm_log))
    p, ok = to_live(sc, 'S14')
    rogue = node.create_publisher(JointState, f'/{a.arm}/gello/joint_states', 10)
    rc = wait_exit(p, 10)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 3 (FAULT)', rc == 3, f'rc={rc}')
    R.check(sc, 'reason: publisher appeared', any('publisher appeared' in t for t in texts))
    R.check(sc, 'controller inactive', arm.ctrl_state() == 'inactive')
    node.destroy_publisher(rogue)
    time.sleep(1.5)
    check_ramp(a, R, sc, n_goal, base)

    # ---- S12: real workflow: START with GELLO in holder, then taken out and held -> LIVE
    sc = 'S12 holder -> taken out -> LIVE'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    t_out = 3.0
    arm.set_gello(lambda t: SIM_HOLDER if t < t_out else base)
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S12')
    ok = arm.wait_state('LIVE', 20)
    t_mov = arm.state_time('MOVING')
    R.check(sc, 'reaches LIVE', ok, ' -> '.join(dict.fromkeys(arm.state_names())))
    with arm.lock:
        early = [t for t, _ in arm.relayed if t - arm.t0 < t_out]
    R.check(sc, 'nothing relayed while GELLO in holder', not early, f'{len(early)} msgs')
    R.check(sc, 'activation only after taken out + still >= 0.5 s',
            t_mov is not None and t_mov - arm.t0 >= t_out + 0.5,
            f'MOVING at {t_mov - arm.t0:.2f} s' if t_mov else 'no MOVING')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, base)

    # ---- S15: LIVE smoothing: 25 Hz GELLO samples -> ~200 Hz command, no big steps, speed capped
    sc = 'S15 LIVE smoothing + speed cap'
    print(f'\n== {sc}', flush=True)
    p, ok = to_live(sc, 'S15')
    if ok:
        time.sleep(0.5)
        n_rel = len(arm.relayed)
        arm.set_gello(lambda t: offset(0, 0.15 * math.sin(2 * math.pi * t)))   # 1 Hz, peak 0.94 rad/s
        time.sleep(3)
        with arm.lock:
            live = arm.relayed[n_rel:]
        rate = (len(live) - 1) / (live[-1][0] - live[0][0]) if len(live) > 1 else 0
        steps = [abs(b[1][0] - a_[1][0]) for a_, b in zip(live, live[1:])]
        R.check(sc, 'command rate >= 150 Hz (GELLO 25 Hz)', rate >= 150, f'{rate:.0f} Hz')
        R.check(sc, 'largest command step <= 0.012 rad (raw GELLO ~0.038)', max(steps) <= 0.012,
                f'max step {max(steps):.4f} rad')
        amp = max(abs(q[0] - base[0]) for _, q in live)
        R.check(sc, 'motion still followed (amplitude > 0.12 of 0.15)', amp > 0.12, f'{amp:.3f}')
        # speed cap: 0.25 rad step (below jump_tol 0.3) -> command ramps at <= 2 rad/s
        n_rel = len(arm.relayed)
        arm.set_gello(lambda t: offset(0, 0.25))
        time.sleep(1.0)
        with arm.lock:
            seg = arm.relayed[n_rel:]
        vel = max((abs(b[1][0] - a_[1][0]) / max(b[0] - a_[0], 1e-3) for a_, b in zip(seg, seg[1:])
                   if b[0] - a_[0] > 0.003), default=0)
        t_reach = next((t - seg[0][0] for t, q in seg if abs(q[0] - (base[0] + 0.25)) < 1e-3), None)
        R.check(sc, '0.25 rad GELLO step: command speed <= 2.4 rad/s (cap 2.0 + timing jitter)',
                vel <= 2.4, f'max {vel:.2f} rad/s')
        R.check(sc, '... and reaches the target within 0.4 s', t_reach is not None and t_reach < 0.4,
                f'{t_reach}')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive')

    # ---- S9c: j1 at 2.70 is inside the URDF limits (+-2.90) but where the FR3 speed limit is ~0 (2.743) -> waits
    sc = 'S9c position-dependent speed limit zone (j1 2.70)'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: offset(0, 2.70))
    p = start_gate(a.arm, logdir, 'S9c', ['--wait-timeout', '4'])
    rc = wait_exit(p, 15)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED), hint names j1', rc == 2 and any('j1 away from its joint limit' in t
                                                                          for t in texts), f'rc={rc} {texts[-2:]}')
    R.check(sc, 'controller inactive, nothing relayed', arm.ctrl_state() == 'inactive' and len(arm.relayed) == 0)

    # ---- S17: LIVE soft limit j4 -2.70 (self-collision zone seen on hardware at -2.83)
    sc = 'S17 LIVE soft limit j4 -2.70'
    print(f'\n== {sc}', flush=True)
    p, ok = to_live(sc, 'S17')
    if ok:
        n_rel = len(arm.relayed)
        arm.set_gello(lambda t: offset(3, -min(0.85, 0.3 * t)))   # GELLO j4 down to -3.206 (past every limit)
        time.sleep(4.5)
        with arm.lock:
            live = arm.relayed[n_rel:]
        j4 = [q[3] for _, q in live]
        R.check(sc, 'j4 command never below -2.70', min(j4) >= -2.70 - 1e-6, f'min {min(j4):.4f}')
        R.check(sc, 'j4 command reaches the soft limit', min(j4) < -2.69, f'min {min(j4):.4f}')
        steps = [abs(b[1][3] - a_[1][3]) for a_, b in zip(live, live[1:])]
        R.check(sc, 'no step > 0.012 rad while clamping', max(steps) <= 0.012, f'max step {max(steps):.4f}')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive')

    # ---- S17b: soft limit off -> usable-range clamp (-3.037 + 0.1) with slow-down on the FR3 speed curve
    sc = 'S17b LIVE clamp at the usable j4 range (--live-j4-min -9)'
    print(f'\n== {sc}', flush=True)
    p, ok = to_live(sc, 'S17b', ['--live-j4-min', '-9'])
    if ok:
        n_rel = len(arm.relayed)
        arm.set_gello(lambda t: offset(3, -min(0.85, 0.3 * t)))
        time.sleep(4.5)
        with arm.lock:
            live = arm.relayed[n_rel:]
        j4 = [q[3] for _, q in live]
        R.check(sc, 'j4 command never below -2.937 (zero-speed zone -3.037 + 0.1)', min(j4) >= -2.937 - 1e-6,
                f'min {min(j4):.4f}')
        R.check(sc, 'j4 command reaches the clamp', min(j4) < -2.93, f'min {min(j4):.4f}')
        v = [(b[1][3] - a_[1][3]) / (b[0] - a_[0]) for a_, b in zip(live, live[1:]) if b[0] - a_[0] > 0.003]
        near = [abs(x) for (t, q), x in zip(live[1:], v) if q[3] < -2.90]
        R.check(sc, 'speed near the clamp <= 50 % of the FR3 limit there (<= 0.4 rad/s)',
                max(near, default=0) <= 0.4, f'max {max(near, default=0):.3f} rad/s')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive')

    # ---- S17c (unit): a snapshot beyond the soft limit (j4 -2.85, inside the usable range) is pulled in at the
    #      capped speed, never snapped. The mock robot cannot move, so LiveSmoother is exercised directly.
    sc = 'S17c snapshot beyond the soft limit is pulled in smoothly (unit)'
    print(f'\n== {sc}', flush=True)
    import importlib.util
    spec = importlib.util.spec_from_file_location('gello_gate_mod', GATE)
    gg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gg)
    q0 = list(MOCK_BASE)
    q0[3] = -2.85
    sm = gg.LiveSmoother(q0, 2.0, 0.0, soft_min={3: -2.70})
    sm.sample(q0, 0.0)
    ys, t = [], 0.0
    for _ in range(400):                      # 2 s at 200 Hz, GELLO held at -2.85 the whole time
        t += 0.005
        if round(t / 0.04) * 0.04 - t < 0.0025 and abs(round(t / 0.04) * 0.04 - t) < 0.0025:
            sm.sample(q0, t)
        ys.append(sm.step(t)[3])
    steps = [abs(b - a_) for a_, b in zip([q0[3]] + ys, ys)]
    R.check(sc, 'ends at the soft limit -2.70', abs(ys[-1] + 2.70) < 1e-3, f'{ys[-1]:.4f}')
    R.check(sc, 'no jump (<= 0.01 rad per 5 ms)', max(steps) <= 0.01, f'max step {max(steps):.4f}')
    R.check(sc, 'pull-in speed <= 50 % of the FR3 limit upward at -2.85 (min(2.62, 4.35) / 2 = 1.31 rad/s)',
            max(steps) / 0.005 <= 1.35, f'{max(steps) / 0.005:.2f} rad/s')

    # ---- duo collision guard (single gate + the other arm faked) ---------------------------------------------
    import numpy as np
    from duo_geometry import MIRROR, DuoGeometry
    geo = DuoGeometry()
    q_c = [0.755, 0.1, 0.576, -2.146, -0.956, 1.606, 0.085]      # left pose whose mirror image the right arm takes
    q_o = list(np.array(q_c) * MIRROR)                           # (|y| = 0: the two arms would meet in the middle)
    c0, _ = geo.clearance(base, q_o)
    c1, _ = geo.clearance(q_c, q_o)
    assert a.arm == 'left' and c0 > 0.10 and c1 < -0.05, (c0, c1)   # test setup precondition
    guard = ['--other-arm', other_arm, '--wait-timeout', '6']

    # ---- S18: path would collide -> REFUSED before any motion, with the reason
    sc = 'S18 guard: ramp would collide with the other arm -> REFUSED'
    print(f'\n== {sc}', flush=True)
    fake.pose(q_o)
    fake.on = True
    fake.state('LIVE')
    arm.reset_records()
    arm.set_gello(lambda t: q_c)
    p = start_gate(a.arm, logdir, 'S18', guard)
    seen_active = False
    while p.poll() is None:
        seen_active |= arm.ctrl_state() == 'active'
        time.sleep(0.1)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED)', p.returncode == 2, f'rc={p.returncode} {texts[-1:]}')
    R.check(sc, 'went through BARRIER', any(t.startswith('BARRIER') for t in texts), str(texts[-4:]))
    R.check(sc, 'reason says the arms would collide and names the links',
            any('would collide' in t and 'link' in t for t in texts), texts[-1] if texts else '')
    R.check(sc, 'controller never active (robot never moved)', not seen_active and arm.ctrl_state() == 'inactive')

    # ---- S19: other arm not ready -> waits at the barrier, REFUSED at timeout (nothing moves)
    sc = 'S19 guard: other arm gate not ready -> REFUSED at the barrier timeout'
    print(f'\n== {sc}', flush=True)
    fake.state('WAIT_STILL: waiting for controller manager')
    arm.reset_records()
    arm.set_gello(lambda t: base)
    p = start_gate(a.arm, logdir, 'S19', ['--other-arm', other_arm, '--wait-timeout', '4'])
    seen_active = False
    while p.poll() is None:
        seen_active |= arm.ctrl_state() == 'active'
        time.sleep(0.1)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED)', p.returncode == 2, f'rc={p.returncode}')
    R.check(sc, 'reason: other arm not ready', any('is not ready within' in t for t in texts), texts[-1] if texts else '')
    R.check(sc, 'controller never active', not seen_active and arm.ctrl_state() == 'inactive')
    fake.state('LIVE')

    # ---- S20: LIVE limiter: GELLO drives the arm into the other arm -> command stops short, then follows back
    sc = 'S20 guard: LIVE command limited before the arms touch'
    print(f'\n== {sc}', flush=True)
    p, ok = to_live(sc, 'S20', guard)
    if ok:
        n_rel = len(arm.relayed)
        arm.set_gello(lambda t: [b + min(1.0, t / 3.0) * (c - b) for b, c in zip(base, q_c)])
        time.sleep(5.0)
        with arm.lock:
            live = arm.relayed[n_rel:]
            texts = [t for _, t in arm.states]
        cl = [geo.clearance(q, q_o)[0] for _, q in live[::5]]
        R.check(sc, 'raw GELLO target would collide (precondition)', geo.clearance(q_c, q_o)[0] < -0.05)
        R.check(sc, 'command never closer than the margin (clearance >= -1 cm)', min(cl) >= -0.01,
                f'min clearance {min(cl) * 100:.1f} cm over {len(cl)} samples')
        last = live[-1][1]
        R.check(sc, 'command stopped short of the GELLO target', max(abs(x - y) for x, y in zip(last, q_c)) > 0.15,
                f'max joint diff to target {max(abs(x - y) for x, y in zip(last, q_c)):.3f} rad')
        R.check(sc, 'it did move toward the target first', max(abs(x - y) for x, y in zip(last, base)) > 0.2,
                f'max joint diff from start {max(abs(x - y) for x, y in zip(last, base)):.3f} rad')
        R.check(sc, 'state said "LIVE: limited by the collision guard"',
                any('limited by the collision guard' in t for t in texts), str(texts[-3:]))
        steps = [max(abs(x - y) for x, y in zip(b[1], a_[1])) for a_, b in zip(live, live[1:])]
        R.check(sc, 'no command step > 0.012 rad while limiting', max(steps) <= 0.012, f'max step {max(steps):.4f}')
        arm.set_gello(lambda t: [c + min(1.0, t / 3.0) * (b - c) for b, c in zip(base, q_c)])   # smoothly back
        time.sleep(4.0)
        with arm.lock:
            back = arm.relayed[-1][1]
        R.check(sc, 'command follows the GELLO back (moving away is never limited)',
                max(abs(x - y) for x, y in zip(back, base)) < 0.03,
                f'max diff to start {max(abs(x - y) for x, y in zip(back, base)):.3f} rad')
        # ---- S21 (same LIVE session): the other arm's data disappears -> FAULT, controller deactivated
        fake.on = False
        rc = wait_exit(p, 10)
        with arm.lock:
            texts = [t for _, t in arm.states]
        R.check(sc + ' / S21', 'other arm data lost -> gate exit 3 (FAULT)', rc == 3, f'rc={rc}')
        R.check(sc + ' / S21', 'reason names the other arm', any('joint states of the right arm lost' in t for t in texts),
                texts[-1] if texts else '')
        R.check(sc + ' / S21', 'controller inactive (robot holds)', arm.ctrl_state() == 'inactive')
    else:
        p.send_signal(signal.SIGINT)
        wait_exit(p, 15)
    fake.on = False

    # ---- standby (pre-warm while the pedal is held) ----------------------------------------------------------
    latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
    pub_go = node.create_publisher(String, '/demo/gate_go', latched_qos)

    sc = 'S22 standby: ARMED, nothing moves until the right GO, then fast LIVE'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: base)                        # GELLO already out of the holder and still
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S22', ['--standby', '--go-token', 'tok22'])
    R.check(sc, 'reaches ARMED', arm.wait_state('ARMED', 20), ' -> '.join(dict.fromkeys(arm.state_names())))
    time.sleep(3.0)
    with arm.lock:
        names = [t.split(':')[0] for _, t in arm.states]
    R.check(sc, 'after 3 s still ARMED: no ACTIVATING / MOVING / LIVE', names[-1] == 'ARMED'
            and not {'ACTIVATING', 'MOVING', 'LIVE', 'WAIT_STILL'} & set(names[names.index('ARMED'):]), str(names[-4:]))
    R.check(sc, 'controller inactive and NOTHING relayed while armed', arm.ctrl_state() == 'inactive'
            and len(arm.relayed) == 0, f'{len(arm.relayed)} msgs')
    pub_go.publish(String(data='go:wrong'))
    time.sleep(1.0)
    R.check(sc, 'a GO with the wrong token is ignored', arm.state_names()[-1] == 'ARMED', arm.state_names()[-1])
    t_go = time.monotonic()
    pub_go.publish(String(data='go:tok22'))
    ok = arm.wait_state('LIVE', 10)
    dt = (arm.state_time('LIVE') or time.monotonic()) - t_go
    R.check(sc, 'GO -> LIVE', ok, ' -> '.join(dict.fromkeys(arm.state_names())))
    R.check(sc, 'GO -> LIVE in < 2.2 s (non-standby start takes ~3.3 s)', ok and dt < 2.2, f'{dt:.2f} s')
    R.check(sc, 'controller active in LIVE', arm.ctrl_state() == 'active')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive', f'rc={rc}')
    check_ramp(a, R, sc, n_goal, base)          # exactly one ramp, initialised on the snapshot (fresh instance)

    sc = 'S23 standby timeout: leaves quietly, nothing was done'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: base)
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S23', ['--standby', '--go-token', 'tok23', '--standby-timeout', '3'])
    rc = wait_exit(p, 40)
    R.check(sc, 'gate exit code 0', rc == 0, f'rc={rc}')
    R.check(sc, 'controller inactive, nothing relayed, no ramp', arm.ctrl_state() == 'inactive'
            and len(arm.relayed) == 0 and len(q_goal_lines(a.cm_log)) == n_goal)
    R.check(sc, 'final state STOPPED', arm.state_names()[-1] == 'STOPPED', arm.state_names()[-1])

    sc = 'S24 GO that arrived BEFORE the gate started (latched) is still honoured'
    print(f'\n== {sc}', flush=True)
    pub_go.publish(String(data='go:tok24'))
    time.sleep(0.5)
    arm.reset_records()
    arm.set_gello(lambda t: base)
    p = start_gate(a.arm, logdir, 'S24', ['--standby', '--go-token', 'tok24'])
    R.check(sc, 'reaches LIVE without a second GO', arm.wait_state('LIVE', 20), ' -> '.join(dict.fromkeys(arm.state_names())))
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive', f'rc={rc}')

    # ---- S25: GELLO moved while the robot was ramping (0.3 rad) -> LIVE at once with a slow catch-up (no HOLD wait)
    sc = 'S25 catch-up: GELLO off the snapshot by 0.3 rad goes LIVE at once'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: base)
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S25')
    moving = arm.wait_state('MOVING', 15)
    off = offset(0, 0.3)
    arm.set_gello(lambda t: off)                     # operator starts moving right after the pedal, then holds still
    t_move = time.monotonic()
    ok = arm.wait_state('LIVE', 8)
    R.check(sc, 'LIVE without waiting for the GELLO to come back', moving and ok,
            ' -> '.join(dict.fromkeys(arm.state_names())))
    R.check(sc, 'no HOLD "GELLO off snapshot" wait', not any('GELLO off snapshot' in t for _, t in arm.states))
    time.sleep(3.0)
    with arm.lock:
        live = [(t, q) for t, q in arm.relayed if arm.state_time('LIVE') and t >= arm.state_time('LIVE')]
        texts = [t for _, t in arm.states]
    R.check(sc, 'state said "catching up with the GELLO" and then plain LIVE',
            any('catching up' in t for t in texts) and texts[-1] == 'LIVE', str(texts[-3:]))
    if live:
        first = live[0][1]
        R.check(sc, 'first LIVE command is still the snapshot (no jump at the handover)',
                max(abs(x - y) for x, y in zip(first, base)) < 0.03,
                f'{max(abs(x - y) for x, y in zip(first, base)):.3f} rad')
        v = [abs(b[1][0] - a_[1][0]) / max(b[0] - a_[0], 1e-3) for a_, b in zip(live, live[1:])
             if b[0] - a_[0] > 0.003 and abs(b[1][0] - off[0]) > 0.05]
        R.check(sc, 'catch-up speed <= 0.6 rad/s (cap 0.5)', v and max(v) <= 0.6, f'max {max(v) if v else 0:.2f} rad/s')
        last = live[-1][1]
        R.check(sc, 'command reached the GELLO', max(abs(x - y) for x, y in zip(last, off)) < 0.03,
                f'{max(abs(x - y) for x, y in zip(last, off)):.3f} rad')
        steps = [max(abs(x - y) for x, y in zip(b[1], a_[1])) for a_, b in zip(live, live[1:])]
        R.check(sc, 'no command step > 0.012 rad', max(steps) <= 0.012, f'{max(steps):.4f}')
    else:
        R.check(sc, 'LIVE commands relayed', False)
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive', f'rc={rc}')
    check_ramp(a, R, sc, n_goal, base)

    # ---- S16: HOLD resync: GELLO moved during the ramp and held still elsewhere -> second ramp to it
    sc = 'S16 HOLD resync'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: base)
    n_goal = len(q_goal_lines(a.cm_log))
    p = start_gate(a.arm, logdir, 'S16', ['--move-timeout', '3'])
    moving = arm.wait_state('MOVING', 15)
    moved = offset(0, 0.8)                           # beyond --catchup-max (0.6): no catch-up, RESYNC instead
    arm.set_gello(lambda t: moved)                   # operator moves the GELLO during the ramp, holds it there
    R.check(sc, 'HOLD (GELLO off snapshot)', moving and arm.wait_state('HOLD', 10),
            ' -> '.join(dict.fromkeys(arm.state_names())))
    R.check(sc, 'RESYNC after GELLO held still', arm.wait_state('RESYNC', 8),
            ' -> '.join(dict.fromkeys(arm.state_names())))
    rc = wait_exit(p, 20)                            # mock cannot move -> second ramp times out
    R.check(sc, 'gate exit 3 (mock cannot reach new pose), controller inactive',
            rc == 3 and arm.ctrl_state() == 'inactive', f'rc={rc}')
    time.sleep(1.0)
    goals = q_goal_lines(a.cm_log)[n_goal:]
    R.check(sc, 'two controller ramps: snapshot, then the new GELLO pose',
            len(goals) == 2 and max(abs(x - y) for x, y in zip(goals[0], base)) < 1e-5
            and max(abs(x - y) for x, y in zip(goals[1], moved)) < 1e-5, str([[round(x, 3) for x in g] for g in goals]))

    arm.set_gello(None)
    code = R.summary()
    ex.shutdown()
    spin_thread.join(timeout=3)
    node.destroy_node()
    rclpy.try_shutdown()
    return code


def run_duo(a, logdir):
    safety_check(a.sim_config)
    write_sim_holder(logdir)
    rclpy.init()
    node = Node('sim_gate_driver_duo')
    arms = {n: Arm(node, n) for n in ('left', 'right')}
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    R = Results()

    def teleop(tag):
        log = open(os.path.join(logdir, f'teleop_{tag}.log'), 'w')
        return subprocess.Popen([TELEOP, '--robot-config', a.sim_config, '--no-gello', '--no-grippers', '--gate-args',
                                 f'--holder-file {HOLDER_FILE} --wait-timeout 10'],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_robots(timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if all(x.robot is not None for x in arms.values()):
                return True
            time.sleep(0.2)
        return False

    # ---- D1: both arms LIVE, Ctrl-C to the script
    sc = 'D1 duo LIVE, SIGINT'
    print(f'\n== {sc}', flush=True)
    for x in arms.values():
        x.robot = None
        x.reset_records()
        x.set_gello(lambda t, x=x: list(x.robot) if x.robot else None)   # follows mock pose
    p = teleop('D1')
    up = wait_robots()
    R.check(sc, 'both sim arms up', up)
    if not up:
        os.killpg(p.pid, signal.SIGINT)
        wait_exit(p, 60)
        return R.summary()
    for n, x in arms.items():
        base = list(x.robot)
        x.set_gello(lambda t, b=base: b)
        R.check(sc, f'{n} reaches LIVE', x.wait_state('LIVE', 90),
                ' -> '.join(dict.fromkeys(x.state_names())))
        R.check(sc, f'{n} controller active', x.ctrl_state() == 'active')
    os.killpg(p.pid, signal.SIGINT)   # like Ctrl-C in the terminal (whole process group)
    rc = wait_exit(p, 60)
    R.check(sc, 'teleop_duo.sh exits', rc is not None, f'rc={rc}')
    for n, x in arms.items():
        R.check(sc, f'{n} gate STOPPED (controller deactivated)', 'STOPPED' in x.state_names(),
                ' -> '.join(dict.fromkeys(x.state_names())))
    time.sleep(2)
    R.check(sc, 'no ros2_control_node / gate left',
            subprocess.run(['pgrep', '-f', r'^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py']).returncode != 0)

    # ---- D2: right refused -> left must be stopped too
    sc = 'D2 right in holder REFUSED stops left'
    print(f'\n== {sc}', flush=True)
    for x in arms.values():
        x.robot = None
        x.reset_records()
        x.set_gello(None)
    p = teleop('D2')
    up = wait_robots()
    R.check(sc, 'both sim arms up', up)
    if not up:
        os.killpg(p.pid, signal.SIGINT)
        wait_exit(p, 60)
        return R.summary()
    left_base = list(arms['left'].robot)
    arms['left'].set_gello(lambda t: left_base)
    arms['right'].set_gello(lambda t: SIM_HOLDER)     # right GELLO stays in its holder
    # collision guard barrier: the left arm must WAIT for the right one (it used to go LIVE alone)
    R.check(sc, 'left waits at the BARRIER for the right arm', arms['left'].wait_state('BARRIER', 60),
            ' -> '.join(dict.fromkeys(arms['left'].state_names())))
    rc = wait_exit(p, 90)
    R.check(sc, 'teleop_duo.sh exits with 2 (REFUSED)', rc == 2, f'rc={rc}')
    R.check(sc, 'right REFUSED', 'REFUSED' in arms['right'].state_names())
    left_states = list(dict.fromkeys(arms['left'].state_names()))
    R.check(sc, 'left NEVER moved (no MOVING / LIVE) and ended REFUSED or STOPPED',
            not {'MOVING', 'LIVE'} & set(left_states) and ({'REFUSED', 'STOPPED'} & set(left_states)),
            ' -> '.join(left_states))
    time.sleep(2)
    R.check(sc, 'no ros2_control_node / gate left',
            subprocess.run(['pgrep', '-f', r'^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py']).returncode != 0)

    for x in arms.values():
        x.set_gello(None)
    code = R.summary()
    ex.shutdown()
    spin_thread.join(timeout=3)
    node.destroy_node()
    rclpy.try_shutdown()
    return code


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='mode', required=True)
    s = sub.add_parser('single')
    s.add_argument('--arm', default='left')
    s.add_argument('--cm-log', required=True)
    s.add_argument('--logdir', required=True)
    d = sub.add_parser('duo')
    d.add_argument('--sim-config', required=True)
    d.add_argument('--logdir', required=True)
    a = ap.parse_args()
    sys.exit(run_single(a, a.logdir) if a.mode == 'single' else run_duo(a, a.logdir))


if __name__ == '__main__':
    main()
