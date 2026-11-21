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


def run_single(a, logdir):
    safety_check()
    rclpy.init()
    node = Node('sim_gate_driver')
    arm = Arm(node, a.arm)
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

    # ---- S9: snapshot outside FR3 joint limits (j4 -3.056 < -3.077 + 0.1) -> REFUSED
    sc = 'S9 joint limit'
    print(f'\n== {sc}', flush=True)
    arm.reset_records()
    arm.set_gello(lambda t: offset(5, -1.1))     # j6 0.471 < 0.4398 + 0.1, far from holder
    p = start_gate(a.arm, logdir, 'S9')
    rc = wait_exit(p, 15)
    with arm.lock:
        texts = [t for _, t in arm.states]
    R.check(sc, 'gate exit code 2 (REFUSED)', rc == 2, f'rc={rc}')
    R.check(sc, 'reason: joint limits', any('joint limits' in t for t in texts), texts[-1] if texts else '')
    R.check(sc, 'controller inactive, nothing relayed',
            arm.ctrl_state() == 'inactive' and len(arm.relayed) == 0)

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
    R.check(sc, 'activation only after taken out + still >= 1 s',
            t_mov is not None and t_mov - arm.t0 >= t_out + 1.0,
            f'MOVING at {t_mov - arm.t0:.2f} s' if t_mov else 'no MOVING')
    p.send_signal(signal.SIGINT)
    rc = wait_exit(p, 15)
    R.check(sc, 'gate exit code 0, controller inactive', rc == 0 and arm.ctrl_state() == 'inactive')
    check_ramp(a, R, sc, n_goal, base)

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
    R.check(sc, 'left reaches LIVE', arms['left'].wait_state('LIVE', 90))
    arms['right'].set_gello(lambda t: SIM_HOLDER)     # right GELLO stays in its holder
    rc = wait_exit(p, 90)
    R.check(sc, 'teleop_duo.sh exits with 2 (REFUSED)', rc == 2, f'rc={rc}')
    R.check(sc, 'right REFUSED', 'REFUSED' in arms['right'].state_names())
    R.check(sc, 'left STOPPED (deactivated)', 'STOPPED' in arms['left'].state_names(),
            ' -> '.join(dict.fromkeys(arms['left'].state_names())))
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
