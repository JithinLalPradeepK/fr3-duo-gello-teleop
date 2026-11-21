#!/usr/bin/env python3
"""SIM test of demo_supervisor (fake hardware, ROS_DOMAIN_ID=77). Run via test/sim_supervisor_test.sh.

Starts the supervisor itself (--sim, no grippers/GELLO/cameras), provides fake GELLO data, calls the
/demo services like the GUI/pedal would and checks /demo/status + the real controller states.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from rclpy.action import ActionServer
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_gate_driver import MOCK_BASE, SIM_HOLDER, Arm, Results, safety_check  # noqa: E402

DEMO = os.path.expanduser('~/robot_demo')
SUP = os.path.join(DEMO, 'scripts', 'demo_supervisor.py')
CM_PATTERN = r'^/opt/ros/humble/lib/controller_manager/ros2_control_node'
GATE_PATTERN = r'^(/usr/bin/)?python3 \S*gello_gate\.py'


def running(pattern):
    return subprocess.run(['pgrep', '-f', pattern], stdout=subprocess.DEVNULL).returncode == 0


def sim_pids(pattern):
    """PIDs matching pattern that belong to this sim (ROS_DOMAIN_ID=77) - never a real stack."""
    pids = []
    for pid in subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split():
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                if b'ROS_DOMAIN_ID=77\0' in f.read():
                    pids.append(int(pid))
        except OSError:
            pass
    return pids


def fake_gripper_cm(arm):
    """A process that looks like a Robotiq gripper controller manager (__ns:=/<arm>/gripper), like on the real
    machine. It must NOT count as an arm process (hardware bug 2026-11-21: restart refused, gates not ended)."""
    name = f'/opt/ros/humble/lib/controller_manager/ros2_control_node --ros-args -r __ns:=/{arm}/gripper'
    return subprocess.Popen(['bash', '-c', f'exec -a "{name}" sleep 600'])


class FakeFranka:
    """Stands in for franka_robot_state_broadcaster (robot_mode on /<arm>/robot_state_10hz) and the Franka
    error_recovery action servers, which the fake-hardware sim does not have."""

    def __init__(self, node):
        self.mode = None                      # None = not publishing, 1 = IDLE, 4 = REFLEX
        self.goals = {'left': 0, 'right': 0}
        self.pub = {a: node.create_publisher(FrankaRobotState, f'/{a}/robot_state_10hz', 5) for a in self.goals}
        self.srv = [ActionServer(node, ErrorRecovery, f'/{a}/action_server/error_recovery',
                                 lambda gh, a=a: self.execute(a, gh)) for a in self.goals]
        node.create_timer(0.1, self.tick)

    def execute(self, arm, gh):
        self.goals[arm] += 1
        gh.succeed()
        return ErrorRecovery.Result()

    def tick(self):
        if self.mode is not None:
            for p in self.pub.values():
                p.publish(FrankaRobotState(robot_mode=self.mode))


class Client:
    def __init__(self, node):
        self.node = node
        self.status = {}
        self.history = []
        self.lock = threading.Lock()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String, '/demo/status', self.on_status, latched)
        self.cli = {}

    def on_status(self, m):
        st = json.loads(m.data)
        with self.lock:
            self.status = st
            if not self.history or self.history[-1] != st['state']:
                self.history.append(st['state'])

    def state(self):
        with self.lock:
            return self.status.get('state'), self.status.get('message', '')

    def call(self, name, timeout=5.0):
        if name not in self.cli:
            self.cli[name] = self.node.create_client(Trigger, f'/demo/{name}')
        cli = self.cli[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            return None, 'service not available'
        fut = cli.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        r = fut.result() if fut.done() else None
        return (r.success, r.message) if r else (None, 'timeout')

    def wait_state(self, want, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state()[0] == want:
                return True
            time.sleep(0.05)
        return False


def main():
    logdir = sys.argv[1]
    sim_cfg = sys.argv[2]
    safety_check(sim_cfg)
    holder = os.path.join(logdir, 'sim_holder_poses.yaml')
    with open(holder, 'w') as f:
        yaml.safe_dump({'left': SIM_HOLDER, 'right': SIM_HOLDER}, f)
    home = os.path.join(logdir, 'sim_home_poses.txt')
    with open(home, 'w') as f:                       # sim arms both use robot_ip 0.0.0.0
        # j1 written as -0.000000: a pose starting with '-' must work (real right-arm bug 2026-11-20)
        f.write('0.0.0.0 -0.000000 ' + ' '.join(f'{q:.6f}' for q in MOCK_BASE[1:]) + '\n')

    rclpy.init()
    node = Node('sim_supervisor_test')
    arms = {a: Arm(node, a) for a in ('left', 'right')}
    fake = FakeFranka(node)
    c = Client(node)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    R = Results()

    def start_sup(cfg=sim_cfg, home_file=home, extra=()):
        log = open(os.path.join(logdir, 'supervisor.log'), 'a')
        return subprocess.Popen([sys.executable, SUP, '--sim', '--bringup', *extra, '--robot-config', cfg,
                                 '--no-grippers', '--no-gello', '--no-cameras', '--holder-file', holder,
                                 '--home-file', home_file, '--lock-file', os.path.join(logdir, 'home.lock'),
                                 '--gate-args', f'--holder-file {holder} --wait-timeout 8'],
                                stdout=log, stderr=subprocess.STDOUT)

    def ctrl(arm):
        try:
            return arms[arm].ctrl_state()
        except AssertionError as e:
            return f'n/a ({e})'

    def gello(pose_l, pose_r):
        arms['left'].set_gello(lambda t: pose_l)
        arms['right'].set_gello(lambda t: pose_r)

    sup = start_sup()
    gello(SIM_HOLDER, SIM_HOLDER)                    # both GELLOs in their holders

    sc = 'T1 bringup'
    print(f'\n== {sc}', flush=True)
    R.check(sc, 'reaches IDLE', c.wait_state('IDLE', 90), ' -> '.join(c.history))
    R.check(sc, 'both controllers inactive', ctrl('left') == ctrl('right') == 'inactive')

    sc = 'T2 STOP / refusals in IDLE'
    print(f'\n== {sc}', flush=True)
    ok, msg = c.call('stop')
    R.check(sc, 'STOP accepted as no-op', ok is True, msg)
    ok, msg = c.call('bringup')
    R.check(sc, 'second BRINGUP refused', ok is False, msg)
    ok, msg = c.call('gripper/left/open')
    R.check(sc, 'gripper refused (grippers off)', ok is False, msg)

    sc = 'T3 HOME'
    print(f'\n== {sc}', flush=True)
    ok, msg = c.call('home')
    R.check(sc, 'HOME accepted', ok is True, msg)
    R.check(sc, 'HOMING', c.wait_state('HOMING', 5))
    R.check(sc, 'back to IDLE', c.wait_state('IDLE', 40), c.state()[1])
    R.check(sc, 'message: both arms at home', 'both arms at home' in c.state()[1], c.state()[1])
    R.check(sc, 'controllers inactive after HOME', ctrl('left') == ctrl('right') == 'inactive')

    sc = 'T4 START (holder -> out) -> TELEOP -> STOP'
    print(f'\n== {sc}', flush=True)
    ok, msg = c.call('start')
    R.check(sc, 'START accepted', ok is True, msg)
    R.check(sc, 'TELEOP_STARTING', c.wait_state('TELEOP_STARTING', 5))
    time.sleep(3)
    R.check(sc, 'GELLO in holder: still starting, controllers inactive',
            c.state()[0] == 'TELEOP_STARTING' and ctrl('left') == ctrl('right') == 'inactive', c.state()[1])
    gello(MOCK_BASE, MOCK_BASE)                      # take both out
    R.check(sc, 'TELEOP', c.wait_state('TELEOP', 15), c.state()[1])
    R.check(sc, 'both controllers active', ctrl('left') == ctrl('right') == 'active')
    for cmd in ('home', 'start', 'shutdown', 'bringup'):
        ok, msg = c.call(cmd)
        R.check(sc, f'{cmd.upper()} refused during TELEOP', ok is False, msg)
    ok, msg = c.call('stop')
    R.check(sc, 'STOP accepted', ok is True, msg)
    R.check(sc, 'back to IDLE', c.wait_state('IDLE', 20), c.state()[1])
    R.check(sc, 'controllers inactive after STOP', ctrl('left') == ctrl('right') == 'inactive')
    R.check(sc, 'no gate left', not running(GATE_PATTERN))

    sc = 'T5 right GELLO stays in holder'
    print(f'\n== {sc}', flush=True)
    gello(MOCK_BASE, SIM_HOLDER)
    ok, msg = c.call('start')
    R.check(sc, 'START accepted', ok is True, msg)
    R.check(sc, 'ends in IDLE', c.wait_state('TELEOP_STARTING', 5) and c.wait_state('IDLE', 30), c.state()[1])
    R.check(sc, 'message names the refusal', 'refused' in c.state()[1], c.state()[1])
    R.check(sc, 'controllers inactive', ctrl('left') == ctrl('right') == 'inactive')
    R.check(sc, 'no gate left', not running(GATE_PATTERN))

    sc = 'T6 SHUTDOWN and BRINGUP again'
    print(f'\n== {sc}', flush=True)
    gello(SIM_HOLDER, SIM_HOLDER)
    ok, msg = c.call('shutdown')
    R.check(sc, 'SHUTDOWN accepted', ok is True, msg)
    R.check(sc, 'OFFLINE', c.wait_state('OFFLINE', 40), c.state()[1])
    time.sleep(2)
    R.check(sc, 'no ros2_control_node left', not running(CM_PATTERN))
    ok, msg = c.call('bringup')
    R.check(sc, 'BRINGUP accepted', ok is True, msg)
    R.check(sc, 'IDLE again', c.wait_state('IDLE', 90), c.state()[1])

    sc = 'T6b arm stack crash (SIGABRT right ros2_control_node, like the 2026-11-20 reflex) during TELEOP'
    print(f'\n== {sc}', flush=True)
    gello(MOCK_BASE, MOCK_BASE)
    c.call('start')
    R.check(sc, 'TELEOP', c.wait_state('TELEOP', 20), c.state()[1])
    fakes = [fake_gripper_cm(a) for a in ('left', 'right')]
    time.sleep(0.5)
    pids = sim_pids(CM_PATTERN + r' --ros-args -r __ns:=/right ')
    R.check(sc, 'found exactly one sim right ros2_control_node', len(pids) == 1, str(pids))
    t_crash = time.monotonic()
    for pid in pids:
        os.kill(pid, signal.SIGABRT)
    R.check(sc, 'ERROR', c.wait_state('ERROR', 30), c.state()[1])
    msg0 = c.state()[1]
    ok, msg = c.call('stop')                         # operator presses STOP in ERROR (O8)
    R.check(sc, 'STOP in ERROR: accepted, says already stopped', ok is True and 'already stopped' in msg, msg)
    time.sleep(3)
    msg = c.state()[1]
    R.check(sc, 'still ERROR, message not replaced by "component stopped"',
            c.state()[0] == 'ERROR' and 'component stopped' not in msg, f'{c.history[-3:]} {msg}')
    R.check(sc, 'message names right arm + ERROR RECOVERY',
            'right: arm process died' in msg and 'ERROR RECOVERY' in msg, f'{msg} (first: {msg0})')
    end = time.monotonic() + 30
    while time.monotonic() < end and running(GATE_PATTERN):
        time.sleep(0.1)
    R.check(sc, 'gates gone within 8 s of the crash (no 15 s retries, O9)', time.monotonic() - t_crash < 8,
            f'{time.monotonic() - t_crash:.1f} s')
    time.sleep(1)
    msg = c.state()[1]
    R.check(sc, 'gates gone, no false "use E-stop" alarm', not running(GATE_PATTERN) and 'E-stop,' not in msg
            and 'could not be deactivated' not in msg, msg)
    fake.mode = 4                                    # robots still report REFLEX after the arm restart
    ok, msg = c.call('recover')
    R.check(sc, 'ERROR RECOVERY accepted: restarts the arm stack', ok is True and 'restarting' in msg, msg)
    end = time.monotonic() + 90
    while time.monotonic() < end and not all(fake.goals.values()):
        time.sleep(0.1)
    R.check(sc, 'robot still in REFLEX after the restart -> Franka error recovery runs by itself (one goal per arm)',
            fake.goals == {'left': 1, 'right': 1}, str(fake.goals))
    fake.mode = 1                                    # the recovery cleared the reflex
    R.check(sc, 'IDLE again after the automatic recovery (arm stack restarted, gripper CMs ignored)',
            c.wait_state('IDLE', 30) and 'recovered' in c.state()[1], f'{c.history[-5:]} {c.state()[1]}')
    time.sleep(3)
    R.check(sc, 'recovery ran only once', fake.goals == {'left': 1, 'right': 1}, str(fake.goals))
    for f in fakes:
        f.kill()
        f.wait()
    R.check(sc, 'went through RECOVERING and STARTING_UP',
            'RECOVERING' in c.history and 'STARTING_UP' in c.history[c.history.index('RECOVERING'):], str(c.history))
    R.check(sc, 'both controllers inactive after restart', ctrl('left') == ctrl('right') == 'inactive')
    c.call('start')
    R.check(sc, 'START after restart -> TELEOP', c.wait_state('TELEOP', 20), c.state()[1])
    c.call('stop')
    R.check(sc, 'STOP -> IDLE, controllers inactive',
            c.wait_state('IDLE', 20) and ctrl('left') == ctrl('right') == 'inactive', c.state()[1])
    ok, msg = c.call('recover')
    R.check(sc, 'RECOVER in IDLE without crash: normal path (refused in sim)', ok is False and 'sim' in msg, msg)

    sc = 'T6c crash, then SHUTDOWN at once (gates may still be exiting, O9)'
    print(f'\n== {sc}', flush=True)
    gello(MOCK_BASE, MOCK_BASE)
    c.call('start')
    R.check(sc, 'TELEOP', c.wait_state('TELEOP', 20), c.state()[1])
    fakes = [fake_gripper_cm(a) for a in ('left', 'right')]
    time.sleep(0.5)
    pids = sim_pids(CM_PATTERN + r' --ros-args -r __ns:=/left ')
    R.check(sc, 'found exactly one sim left ros2_control_node', len(pids) == 1, str(pids))
    for pid in pids:
        os.kill(pid, signal.SIGABRT)
    R.check(sc, 'ERROR', c.wait_state('ERROR', 30), c.state()[1])
    ok, msg = c.call('shutdown')
    R.check(sc, 'SHUTDOWN accepted in ERROR', ok is True, msg)
    R.check(sc, 'OFFLINE', c.wait_state('OFFLINE', 60), c.state()[1])
    for f in fakes:
        f.kill()
        f.wait()
    time.sleep(5)
    R.check(sc, 'stays OFFLINE (no late ERROR from exiting gates)', c.state()[0] == 'OFFLINE',
            f'{c.history[-4:]} {c.state()[1]}')
    R.check(sc, 'no gate / ros2_control_node left', not running(GATE_PATTERN) and not running(CM_PATTERN))
    ok, msg = c.call('bringup')
    R.check(sc, 'BRINGUP accepted', ok is True, msg)
    R.check(sc, 'IDLE again', c.wait_state('IDLE', 90), c.state()[1])

    # ---- pedal long press: pre-warmed (standby) gates, GO, cancel ----------------------------------------------
    pub_hold = node.create_publisher(String, '/demo/pedal/hold', 10)

    def hold(pedal, seconds, progress_end=0.9):
        """Publish /demo/pedal/hold at 10 Hz like demo_pedal.py does while a pedal is held."""
        n = int(seconds * 10)
        for k in range(n):
            pub_hold.publish(String(data=json.dumps({'pedal': pedal, 'progress': round(progress_end * k / max(n, 1), 2),
                                                     'hold_s': 3.0, 'fired': False})))
            time.sleep(0.1)

    def release():
        pub_hold.publish(String(data=json.dumps({'pedal': None, 'progress': 0.0, 'hold_s': 3.0, 'fired': False})))

    def arming():
        return c.status.get('arms', {}), c.status.get('arming')

    sc = 'T6d START pedal held -> standby gates -> START (GO) -> fast TELEOP'
    print(f'\n== {sc}', flush=True)
    gello(MOCK_BASE, MOCK_BASE)                       # GELLOs out of their holders and still
    time.sleep(1.5)
    hold('start', 2.0)
    R.check(sc, 'status says arming: start', c.status.get('arming') == 'start', str(c.status.get('arming')))
    R.check(sc, 'state stays IDLE (nothing is starting yet)', c.state()[0] == 'IDLE', str(c.state()))
    end = time.monotonic() + 15
    while time.monotonic() < end and not all(c.status.get('arms', {}).get(a_, {}).get('gate', '').startswith('ARMED')
                                             for a_ in ('left', 'right')):
        hold('start', 0.5)
    R.check(sc, 'both gates ARMED', all(c.status.get('arms', {}).get(a_, {}).get('gate', '').startswith('ARMED')
                                        for a_ in ('left', 'right')),
            str({a_: c.status.get('arms', {}).get(a_, {}).get('gate') for a_ in ('left', 'right')}))
    R.check(sc, 'robots untouched while armed: controllers inactive', ctrl('left') == ctrl('right') == 'inactive')
    t0 = time.monotonic()
    ok, msg = c.call('start')
    R.check(sc, 'START accepted', ok is True, msg)
    reached = c.wait_state('TELEOP', 20)
    dt = time.monotonic() - t0
    R.check(sc, 'TELEOP', reached, c.state()[1])
    R.check(sc, 'START (after the long press) -> TELEOP in < 3 s', reached and dt < 3.0, f'{dt:.2f} s')
    R.check(sc, 'arming cleared', c.status.get('arming') is None, str(c.status.get('arming')))
    R.check(sc, 'controllers active', ctrl('left') == ctrl('right') == 'active')
    c.call('stop')
    R.check(sc, 'STOP -> IDLE', c.wait_state('IDLE', 20), c.state()[1])
    R.check(sc, 'controllers inactive', ctrl('left') == ctrl('right') == 'inactive')

    sc = 'T6e pedal released early -> standby gates are cancelled, nothing moved'
    print(f'\n== {sc}', flush=True)
    time.sleep(1.0)
    hold('start', 2.5)
    R.check(sc, 'gates running while held', running(GATE_PATTERN) and c.status.get('arming') == 'start')
    release()
    end = time.monotonic() + 8
    while time.monotonic() < end and running(GATE_PATTERN):
        time.sleep(0.2)
    R.check(sc, 'gates gone after the release', not running(GATE_PATTERN))
    time.sleep(1.0)
    R.check(sc, 'IDLE, arming cleared, message not an error', c.state()[0] == 'IDLE' and c.status.get('arming') is None,
            str((c.state(), c.status.get('arming'))))
    R.check(sc, 'controllers inactive', ctrl('left') == ctrl('right') == 'inactive')
    hold('start', 1.0)
    R.check(sc, 'a new hold afterwards pre-warms again', c.status.get('arming') == 'start' or running(GATE_PATTERN))
    release()
    end = time.monotonic() + 8
    while time.monotonic() < end and running(GATE_PATTERN):
        time.sleep(0.2)
    R.check(sc, 'and cancels again', not running(GATE_PATTERN))

    def teleop_now():
        gello(MOCK_BASE, MOCK_BASE)
        c.call('start')
        return c.wait_state('TELEOP', 25)

    def wait_reset_done(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            st, m = c.state()
            if st == 'IDLE' and 'reset complete' in m:
                return True
            time.sleep(0.2)
        return False

    def crash_arm(name):
        pids = sim_pids(CM_PATTERN + rf' --ros-args -r __ns:=/{name} ')
        for pid in pids:
            os.kill(pid, signal.SIGABRT)
        return len(pids) == 1

    sc = 'T6f pedal 2 (stop_home) in TELEOP: STOP at once, then HOME by itself'
    print(f'\n== {sc}', flush=True)
    time.sleep(1.0)
    R.check(sc, 'TELEOP', teleop_now(), c.state()[1])
    t0 = time.monotonic()
    ok, msg = c.call('stop_home')
    R.check(sc, 'accepted: stopping, then HOME', ok is True and 'then HOME' in msg, msg)
    R.check(sc, 'robots stopped within 3 s (STOP is not delayed by the HOME that follows)',
            c.wait_state('IDLE', 5) and time.monotonic() - t0 < 3.0, f'{time.monotonic() - t0:.2f} s')
    R.check(sc, 'controllers inactive right after the stop', ctrl('left') == ctrl('right') == 'inactive')
    R.check(sc, 'HOMING starts by itself', c.wait_state('HOMING', 8), c.state()[1])
    R.check(sc, 'back to IDLE: both arms at home', c.wait_state('IDLE', 60) and 'both arms at home' in c.state()[1],
            c.state()[1])

    sc = 'T6g pedal 2 during HOMING only aborts (no new HOME); in IDLE it does nothing'
    print(f'\n== {sc}', flush=True)
    gello(SIM_HOLDER, SIM_HOLDER)
    time.sleep(1.0)
    c.call('home')
    R.check(sc, 'HOMING', c.wait_state('HOMING', 10), c.state()[1])
    n_home = c.history.count('HOMING')
    ok, msg = c.call('stop_home')
    R.check(sc, 'accepted: HOME aborted', ok is True and 'aborted' in msg, msg)
    R.check(sc, 'IDLE again', c.wait_state('IDLE', 30), c.state()[1])
    time.sleep(4.0)
    R.check(sc, 'no new HOMING afterwards', c.state()[0] == 'IDLE' and c.history.count('HOMING') == n_home,
            f'{c.history[-4:]}')
    ok, msg = c.call('stop_home')
    time.sleep(3.0)
    R.check(sc, 'in IDLE: accepted as a no-op, nothing moves', ok is True and c.state()[0] == 'IDLE'
            and ctrl('left') == ctrl('right') == 'inactive', f'{msg} / {c.state()}')

    sc = 'T6h pedal 3 (reset) in TELEOP: stop, HOME, ready again'
    print(f'\n== {sc}', flush=True)
    time.sleep(1.0)
    R.check(sc, 'TELEOP', teleop_now(), c.state()[1])
    ok, msg = c.call('reset')
    R.check(sc, 'reset accepted', ok is True, msg)
    R.check(sc, 'ends IDLE: reset complete - ready', wait_reset_done(90), c.state()[1])
    R.check(sc, 'went through STOPPING and HOMING', 'STOPPING' in c.history and 'HOMING' in c.history[-8:], str(c.history[-8:]))
    R.check(sc, 'status.reset cleared, controllers inactive', c.status.get('reset') is None
            and ctrl('left') == ctrl('right') == 'inactive', str(c.status.get('reset')))

    sc = 'T6i pedal 3 (reset) after an arm crash: recover (restart arms), HOME, ready again'
    print(f'\n== {sc}', flush=True)
    time.sleep(1.0)
    R.check(sc, 'TELEOP', teleop_now(), c.state()[1])
    fake.mode = 1
    R.check(sc, 'crashed the left arm process', crash_arm('left'))
    R.check(sc, 'ERROR', c.wait_state('ERROR', 30), c.state()[1])
    gello(SIM_HOLDER, SIM_HOLDER)
    ok, msg = c.call('reset')
    R.check(sc, 'reset accepted', ok is True, msg)
    R.check(sc, 'reset complete - ready (arms restarted, error cleared, at home)', wait_reset_done(150),
            f'{c.state()}')
    R.check(sc, 'passed RECOVERING / STARTING_UP / HOMING', {'RECOVERING', 'STARTING_UP', 'HOMING'} <= set(c.history[-14:]),
            str(c.history[-14:]))
    R.check(sc, 'controllers inactive, no gate left', ctrl('left') == ctrl('right') == 'inactive' and not running(GATE_PATTERN))

    sc = 'T6j reset refused while an emergency stop is pressed; STOP aborts a running reset'
    print(f'\n== {sc}', flush=True)
    fake.mode = 5                                    # USER_STOPPED on both arms
    time.sleep(1.0)
    ok, msg = c.call('reset')
    R.check(sc, 'reset refused: emergency stop still pressed', ok is False and 'emergency stop' in msg, msg)
    fake.mode = 1
    time.sleep(1.0)
    R.check(sc, 'TELEOP', teleop_now(), c.state()[1])
    ok, msg = c.call('reset')
    R.check(sc, 'reset accepted', ok is True, msg)
    time.sleep(0.3)
    ok, msg = c.call('stop')
    R.check(sc, 'STOP accepted', ok is True, msg)
    R.check(sc, 'IDLE', c.wait_state('IDLE', 20), c.state()[1])
    n_home = c.history.count('HOMING')
    time.sleep(5.0)
    R.check(sc, 'the reset was aborted: no HOME followed', c.state()[0] == 'IDLE' and c.history.count('HOMING') == n_home
            and c.status.get('reset') is None, f'{c.history[-4:]}')

    sc = 'T7 supervisor killed (SIGKILL) during TELEOP'
    print(f'\n== {sc}', flush=True)
    gello(MOCK_BASE, MOCK_BASE)
    c.call('start')
    R.check(sc, 'TELEOP', c.wait_state('TELEOP', 20), c.state()[1])
    R.check(sc, 'both controllers active', ctrl('left') == ctrl('right') == 'active')
    sup.send_signal(signal.SIGKILL)
    sup.wait()
    end = time.monotonic() + 10
    states = {}
    while time.monotonic() < end:
        states = {a: ctrl(a) for a in arms}
        if all(s != 'active' for s in states.values()):
            break
        time.sleep(0.1)
    R.check(sc, 'controllers no longer active (gates got SIGTERM)', all(s != 'active' for s in states.values()),
            str(states))
    end = time.monotonic() + 40
    while time.monotonic() < end and (running(CM_PATTERN) or running(GATE_PATTERN)):
        time.sleep(0.5)
    R.check(sc, 'whole stack shut down (launches got SIGINT)',
            not running(CM_PATTERN) and not running(GATE_PATTERN))

    sc = 'T8 HOME: the arm that arrives first must not stop the other (O10)'
    print(f'\n== {sc}', flush=True)
    # mock robots cannot move: left home = mock pose (DONE after the 1 s hold), right home 0.3 rad away on j1
    # (never reached). Old supervisor: left DONE -> right stopped at once. Fixed: right keeps going.
    with open(sim_cfg) as f:
        cfg = yaml.safe_load(f)
    for v in cfg.values():
        if isinstance(v, dict) and v.get('namespace') == 'right':
            v['robot_ip'] = '0.0.0.1'
    cfg8 = os.path.join(logdir, 'sim_cfg_t8.yaml')
    with open(cfg8, 'w') as f:
        yaml.safe_dump(cfg, f)
    safety_check(cfg8)
    home8 = os.path.join(logdir, 'sim_home_poses_t8.txt')
    with open(home8, 'w') as f:
        f.write('0.0.0.0 ' + ' '.join(f'{q:.6f}' for q in MOCK_BASE) + '\n')
        f.write('0.0.0.1 ' + ' '.join(f'{q + 0.3 if i == 0 else q:.6f}' for i, q in enumerate(MOCK_BASE)) + '\n')
    gello(SIM_HOLDER, SIM_HOLDER)
    c.history.clear()
    sup = start_sup(cfg8, home8)
    R.check(sc, 'bringup IDLE', c.wait_state('IDLE', 90), c.state()[1])
    ok, msg = c.call('home')
    R.check(sc, 'HOME accepted', ok is True, msg)
    end = time.monotonic() + 20
    while time.monotonic() < end and not c.status.get('arms', {}).get('left', {}).get('gate', '').startswith('DONE'):
        time.sleep(0.05)
    R.check(sc, 'left reached home (DONE)', time.monotonic() < end, str(c.status.get('arms', {}).get('left')))
    time.sleep(3)
    rg = c.status.get('arms', {}).get('right', {}).get('gate', '')
    R.check(sc, 'right still moving 3 s later (not stopped by left)',
            c.state()[0] == 'HOMING' and rg.startswith('MOVING') and ctrl('right') == 'active', f'{c.state()} {rg}')
    ok, msg = c.call('stop')
    R.check(sc, 'STOP -> IDLE', ok is True and c.wait_state('IDLE', 20), c.state()[1])
    R.check(sc, 'final message: left DONE, right stopped by STOP',
            'left: stopped - DONE' in c.state()[1] and 'right: stopped - STOPPED' in c.state()[1], c.state()[1])
    R.check(sc, 'controllers inactive', ctrl('left') == ctrl('right') == 'inactive')
    sup.send_signal(signal.SIGINT)
    try:
        sup.wait(timeout=60)
    except subprocess.TimeoutExpired:
        sup.kill()
    R.check(sc, 'supervisor exited, stack shut down', not running(CM_PATTERN) and not running(GATE_PATTERN))

    sc = 'T9 HOME in two phases (--home-strategy left-first): one arm after the other'
    print(f'\n== {sc}', flush=True)
    gello(SIM_HOLDER, SIM_HOLDER)
    c.history.clear()
    sup = start_sup(sim_cfg, home, extra=['--home-strategy', 'left-first'])
    R.check(sc, 'bringup IDLE', c.wait_state('IDLE', 90), c.state()[1])
    ok, msg = c.call('home')
    R.check(sc, 'HOME accepted', ok is True, msg)
    seen, end = [], time.monotonic() + 80
    while time.monotonic() < end:
        st, m = c.state()
        if m and (not seen or seen[-1] != m):
            seen.append(m)
        if st == 'IDLE' and 'both arms at home' in m:
            break
        time.sleep(0.05)
    steps = [m for m in seen if m.startswith('step ')]
    R.check(sc, 'status shows step 1/2 and step 2/2', any(m.startswith('step 1/2') for m in steps)
            and any(m.startswith('step 2/2') for m in steps), str(steps[:2]) + ' ... ' + str(steps[-1:]))
    R.check(sc, 'phase 1 ran the LEFT gate only, phase 2 the RIGHT gate only',
            any(m.startswith('step 1/2') and 'left:' in m and 'right:' not in m for m in steps)
            and any(m.startswith('step 2/2') and 'right:' in m and 'left:' not in m for m in steps), str(steps[:1]) + str(steps[-1:]))
    R.check(sc, 'ends IDLE: both arms at home', c.state()[0] == 'IDLE' and 'both arms at home' in c.state()[1], c.state()[1])
    R.check(sc, 'controllers inactive', ctrl('left') == ctrl('right') == 'inactive')
    sup.send_signal(signal.SIGINT)
    try:
        sup.wait(timeout=60)
    except subprocess.TimeoutExpired:
        sup.kill()
    R.check(sc, 'supervisor exited, stack shut down', not running(CM_PATTERN) and not running(GATE_PATTERN))

    for a in arms.values():
        a.set_gello(None)
    code = R.summary()
    ex.shutdown()
    spin.join(timeout=3)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
