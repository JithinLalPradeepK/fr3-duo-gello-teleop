#!/usr/bin/env python3
"""demo_supervisor - owns the demo logic; GUI and foot pedal are thin clients.

Design: docs/supervisor_design.md (decisions 2026-11-20: persistent bringup, HOME via gate target
mode, JSON status, pedals 1 START / 2 STOP / 3 HOME, always both arms).

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/demo_supervisor.py [--bringup]          # real robots
  python3 ~/robot_demo/scripts/demo_supervisor.py --sim --robot-config ~/robot_demo/config/fr3_duo_sim.yaml

Services (std_srvs/Trigger, answer at once: accepted / refused + reason; progress in /demo/status):
  /demo/start /demo/stop /demo/home /demo/recover        (pedal + GUI)
  /demo/bringup /demo/shutdown                           (GUI)
  /demo/gripper/{left,right}/{open,close}                (GUI, refused during teleop)
Status: /demo/status std_msgs/String (JSON), 2 Hz + on change, transient local.

Cameras are not safety relevant: if the camera launch stops, the status shows cameras "stopped" and the
arms stay usable (no ERROR). Restart them with SHUTDOWN + BRINGUP when convenient.
Process model: everything (bringup launches, gates) is started from the MAIN thread with
PR_SET_PDEATHSIG, so if the supervisor dies, gates get SIGTERM (-> deactivate, robots hold) and the
launches get SIGINT (clean shutdown). Nothing that can move a robot is ever restarted automatically.
"""
import argparse
import ctypes
import fcntl
import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

DEMO = os.path.expanduser('~/robot_demo')
SIGINT_DEFAULT = os.path.join(DEMO, 'scripts', 'sigint_default')
GATE = os.path.join(DEMO, 'scripts', 'gello_gate.py')
ARMS = ('left', 'right')
CTRL = 'joint_impedance_controller'
ROBOT_MODES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING', 4: 'REFLEX', 5: 'USER_STOPPED',
               6: 'AUTOMATIC_ERROR_RECOVERY'}
CAMERAS = {'zed': '/zed/zed_node/rgb/color/rect/image/compressed',
           'd405_1': '/camera/d405_1/color/image_raw/compressed',
           'd405_2': '/camera/d405_2/color/image_raw/compressed'}
ERROR_WORDS = {   # franka_msgs/Errors field -> plain words (unknown fields are shown as-is)
    'joint_position_limits_violation': 'joint limit reached',
    'cartesian_position_limits_violation': 'Cartesian limit reached',
    'self_collision_avoidance_violation': 'self-collision avoidance',
    'joint_velocity_violation': 'joint too fast',
    'cartesian_velocity_violation': 'Cartesian velocity too high',
    'force_control_safety_violation': 'force control safety',
    'joint_reflex': 'collision / joint torque reflex',
    'cartesian_reflex': 'collision / Cartesian force reflex',
    'max_goal_pose_deviation_violation': 'too far from goal pose',
    'max_path_pose_deviation_violation': 'too far from path',
    'cartesian_velocity_profile_safety_violation': 'Cartesian velocity profile',
    'joint_position_motion_generator_start_pose_invalid': 'joint motion start pose invalid',
    'joint_motion_generator_position_limits_violation': 'motion generator joint limit',
    'joint_motion_generator_velocity_limits_violation': 'motion generator velocity limit',
    'joint_motion_generator_velocity_discontinuity': 'velocity jump (discontinuity)',
    'joint_motion_generator_acceleration_discontinuity': 'acceleration jump (discontinuity)',
    'communication_constraints_violation': 'communication too slow (packet loss / realtime)',
    'power_limit_violation': 'power limit',
    'controller_torque_discontinuity': 'torque jump (discontinuity)',
    'start_elbow_sign_inconsistent': 'elbow sign inconsistent',
    'joint_p2p_insufficient_torque_for_planning': 'insufficient torque for planning',
    'tau_j_range_violation': 'joint torque out of range',
    'instability_detected': 'instability detected',
    'joint_move_in_wrong_direction': 'joint moved in wrong direction',
    'base_acceleration_initialization_timeout': 'base acceleration init timeout',
    'base_acceleration_invalid_reading': 'base acceleration invalid',
}

_libc = ctypes.CDLL('libc.so.6', use_errno=True)
PR_SET_PDEATHSIG = 1


def _child_setup(sig):
    def f():
        os.setsid()                                     # own process group: terminal Ctrl-C can't reach it
        _libc.prctl(PR_SET_PDEATHSIG, int(sig))         # die with the supervisor (main thread forks)
    return f


# the ARM controller managers only: each Robotiq gripper runs its own ros2_control_node (__ns:=/<arm>/gripper),
# which must not count as "arm processes still running" (bug seen on hardware 2026-11-21)
ARM_CM_PATTERN = r'^/opt/ros/humble/lib/controller_manager/ros2_control_node --ros-args -r __ns:=/(left|right) '


def crash_causes(text):
    """Plain words for dead ros2_control_nodes in an arms.log text, FIRST failure first ('' if none died).
    Later failures are marked: a communication violation / bare process death right after another arm failed
    is a consequence (the launch shut that arm down while its controller was active)."""
    node_arm = dict(re.findall(r'\[ros2_control_node-(\d+)\]: process has died .*?__ns:=/(\w+)', text))
    if not node_arm:
        return ''
    first = {}                                  # node -> (time, libfranka message), first error per node
    for n, t, what in re.findall(r'\[ros2_control_node-(\d+)\] \[ERROR\] \[(\d+\.\d+)\] '
                                 r'\[FrankaHardwareInterface\]: libfranka: (.*)', text):
        first.setdefault(n, (float(t), what))
    for n, what in re.findall(r'\[ros2_control_node-(\d+)\]\s+what\(\):\s+(.*)', text):
        first.setdefault(n, (math.inf, what.replace('libfranka: ', '')))

    def words(what):
        m = re.search(r'aborted by reflex! \[(.*?)\]', what)
        return f'motion aborted by robot reflex ({m.group(1).replace(chr(34), "")})' if m else what.strip()
    order = sorted(node_arm, key=lambda n: (first.get(n, (math.inf + 1,))[0], int(n)))
    t0 = first.get(order[0], (math.inf,))[0]
    out = []
    for i, n in enumerate(order):
        arm = node_arm[n]
        if n not in first:
            out.append(f'{arm}: arm process died' + (' (as a consequence)' if i else ''))
            continue
        t, what = first[n]
        w = words(what)
        later = f' {t - t0:.2f} s later' if math.isfinite(t) and math.isfinite(t0) else ''
        if i == 0:
            out.append(f'{arm}: {w}')
        elif 'communication_constraints_violation' in what:
            out.append(f'{arm}: stopped as a consequence ({w}{later})')
        else:
            out.append(f'{arm}: also {w}{later}')
    return '; '.join(out)


class Proc:
    """A child process with its own log file."""

    def __init__(self, name, cmd, logdir, death_sig, env=None):
        self.name = name
        self.log_path = os.path.join(logdir, f'{name}.log')
        self.log = open(self.log_path, 'a')
        self.log_offset = self.log.tell()       # only text after this belongs to this run
        self.p = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT, env=env,
                                  preexec_fn=_child_setup(death_sig))
        self.t_start = time.monotonic()
        self.t_signal = None
        self.sent = set()                       # each signal is sent once (pedal bounce -> double STOP)

    def alive(self):
        return self.p.poll() is None

    def rc(self):
        return self.p.poll()

    def signal(self, sig):
        if self.alive() and sig not in self.sent:
            self.sent.add(sig)
            self.p.send_signal(sig)
            if self.t_signal is None:
                self.t_signal = time.monotonic()


class Supervisor(Node):
    def __init__(self, a):
        super().__init__('demo_supervisor', namespace='demo')
        self.a = a
        self.lock = threading.RLock()
        self.cmdq = queue.Queue()
        self.state, self.message, self.since = 'OFFLINE', 'not brought up', time.time()
        self.last_event = ''
        self.error = None                      # plain-words reason while in ERROR
        self.arms_crash_reported = False       # arm stack (ros2_control_node) ended unexpectedly
        self.bringup = {}                      # name -> Proc
        self.gates = {}                        # arm -> Proc
        self.gate_mode = None                  # 'teleop' | 'home'
        self.gate_state = {arm: '' for arm in ARMS}
        self.robot = {arm: {'mode': None, 'errors': [], 't': 0.0, 'n': 0} for arm in ARMS}
        self.gello = {arm: {'q': None, 't': 0.0} for arm in ARMS}
        self.cam = {k: {'n': 0, 'fps': 0.0} for k in CAMERAS}
        self.pedal_t = 0.0
        self.lock_fd = None
        self.t_bringup = 0.0
        self.cameras_reported = False
        self.logdir = None
        self.robot_cfg = self.load_robot_config()
        self.holder = self.load_yaml(a.holder_file)
        self.home = self.load_home_poses()

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, '/demo/status', latched)
        for cmd in ('start', 'stop', 'home', 'recover', 'bringup', 'shutdown'):
            self.create_service(Trigger, f'/demo/{cmd}',
                                lambda req, res, c=cmd: self.on_service(c, res), callback_group=cb)
        self.pub_grip = {}
        for arm in ARMS:
            self.pub_grip[arm] = self.create_publisher(
                Float32, f'/{arm}/gripper/gripper_client/target_gripper_width_percent', 10)
            for act in ('open', 'close'):
                self.create_service(Trigger, f'/demo/gripper/{arm}/{act}',
                                    lambda req, res, arm=arm, act=act: self.on_gripper(arm, act, res),
                                    callback_group=cb)
            # FrankaRobotState is 1 kHz; a C++ topic_tools throttle (started at bringup) passes 10 Hz.
            # (2 x 1 kHz in Python cost ~70% of a core.)
            self.create_subscription(FrankaRobotState, f'/{arm}/robot_state_10hz',
                                     lambda m, arm=arm: self.on_robot_state(arm, m), 5)
            self.create_subscription(JointState, f'/gello_check_{arm}/gello/joint_states',
                                     lambda m, arm=arm: self.on_gello(arm, m), 10)
            self.create_subscription(String, f'/{arm}/gello_gate/state',
                                     lambda m, arm=arm: self.on_gate_state(arm, m), latched)
        for key, topic in CAMERAS.items():
            self.create_subscription(CompressedImage, topic, lambda m, k=key: self.on_cam(k),
                                     qos_profile_sensor_data, raw=True)     # fps only: no decoding
        self.create_subscription(String, '/demo/pedal/status', self.on_pedal, 10)
        self.desk, self.desk_t = {}, 0.0                 # desk_agent status (FCI / brakes / mode per arm)
        self.create_subscription(String, '/demo/desk/status', self.on_desk, latched)
        self.list_cli = {arm: self.create_client(ListControllers, f'/{arm}/controller_manager/list_controllers',
                                                 callback_group=cb) for arm in ARMS}
        self.grip_cli = {arm: ActionClient(self, GripperCommand,
                                           f'/{arm}/gripper/robotiq_gripper_controller/gripper_cmd',
                                           callback_group=cb) for arm in ARMS}
        self.recover_cli = {arm: ActionClient(self, ErrorRecovery, f'/{arm}/action_server/error_recovery',
                                              callback_group=cb) for arm in ARMS}
        self.ctrl_cache = {arm: {} for arm in ARMS}

    # ------------------------------------------------------------------ config
    @staticmethod
    def load_yaml(path):
        try:
            with open(path) as f:
                return yaml.safe_load(f) or {}
        except OSError:
            return {}

    def load_robot_config(self):
        path = self.a.robot_config or os.path.join(
            subprocess.run(['ros2', 'pkg', 'prefix', 'franka_fr3_arm_controllers'], capture_output=True,
                           text=True).stdout.strip(),
            'share', 'franka_fr3_arm_controllers', 'config', 'example_fr3_duo_config.yaml')
        cfg = self.load_yaml(path)
        out = {c['namespace']: c for c in cfg.values() if isinstance(c, dict) and 'namespace' in c}
        if set(ARMS) - set(out):
            sys.exit(f'robot config {path} must define namespaces {ARMS}')
        fake = [str(out[a]['use_fake_hardware']).lower() == 'true' for a in ARMS]
        if self.a.sim and not all(fake):
            sys.exit('--sim needs a config with use_fake_hardware "true" for both arms')
        if not self.a.sim and any(fake):
            sys.exit('config has fake hardware - use --sim')
        self.robot_cfg_path = path
        return out

    def load_home_poses(self):
        poses = {}
        try:
            for line in open(self.a.home_file):
                f = line.split()
                if len(f) == 8 and not f[0].startswith('#'):
                    poses[f[0]] = [float(x) for x in f[1:]]
        except OSError:
            pass
        return {arm: poses.get(str(self.robot_cfg[arm]['robot_ip'])) for arm in ARMS}

    # ------------------------------------------------------------------ callbacks (executor thread)
    def on_robot_state(self, arm, msg):
        r = self.robot[arm]
        now = time.monotonic()
        r['n'] += 1
        errs = []
        for field in msg.current_errors.get_fields_and_field_types():
            if getattr(msg.current_errors, field):
                errs.append(ERROR_WORDS.get(field, field.replace('_', ' ')))
        with self.lock:
            r.update(mode=ROBOT_MODES.get(msg.robot_mode, str(msg.robot_mode)), errors=errs, t=now)

    def on_gello(self, arm, m):
        with self.lock:
            self.gello[arm] = {'q': list(m.position[:7]), 't': time.monotonic()}

    def on_gate_state(self, arm, m):
        with self.lock:
            self.gate_state[arm] = m.data

    def on_cam(self, key):
        self.cam[key]['n'] += 1

    def on_pedal(self, _m):
        self.pedal_t = time.monotonic()

    def on_desk(self, m):
        try:
            self.desk, self.desk_t = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def desk_not_ready(self):
        """Reason BRINGUP cannot work, from desk_agent's status ('' if fine or desk_agent not running)."""
        if self.a.sim or time.monotonic() - self.desk_t > 3.0:
            return ''
        if self.desk.get('busy'):
            return f'Desk {self.desk["busy"]} in progress'
        off = [arm for arm in ARMS if self.desk.get(arm, {}).get('fci') is False]
        return f'FCI is off on {" and ".join(off)} - press PREPARE ROBOTS first' if off else ''

    def on_service(self, cmd, res):
        ok, msg = self.request(cmd)
        res.success, res.message = ok, msg
        return res

    def on_gripper(self, arm, act, res):
        with self.lock:
            if self.state in ('TELEOP_STARTING', 'TELEOP'):
                res.success, res.message = False, 'refused: GELLO owns the grippers during teleop'
            elif not self.a.grippers or 'gripper_' + arm not in self.bringup:
                res.success, res.message = False, 'refused: grippers not running'
            else:
                self.pub_grip[arm].publish(Float32(data=1.0 if act == 'open' else 0.0))
                self.event(f'gripper {arm} {act}')
                res.success, res.message = True, f'{arm} gripper {act}'
        return res

    # ------------------------------------------------------------------ command acceptance
    def request(self, cmd):
        """Validate against the current state; queue for the main thread. Never blocks."""
        with self.lock:
            s = self.state
            if cmd == 'stop':                                   # never refused
                self.cmdq.put('stop')
                if s == 'ERROR':
                    return True, 'robots already stopped (ERROR) - see the message for what to do'
                return True, 'stopping' if self.gates else 'nothing to stop (robots already holding)'
            if cmd == 'start':
                if s != 'IDLE':
                    return False, f'refused: START needs IDLE (now {s})'
                bad = self.robot_not_ready()
                if bad:
                    return False, f'refused: {bad}'
            elif cmd == 'home':
                if s != 'IDLE':
                    return False, f'refused: HOME needs IDLE (now {s}); never during teleop'
                missing = [arm for arm in ARMS if not self.home[arm]]
                if missing:
                    return False, f'refused: no home pose for {missing} in {self.a.home_file}'
                bad = self.robot_not_ready()
                if bad:
                    return False, f'refused: {bad}'
            elif cmd == 'recover':
                if s not in ('ERROR', 'IDLE'):
                    return False, f'refused: RECOVER needs ERROR or IDLE (now {s})'
                if self.arms_crash_reported:            # arm stack gone -> restart it (nothing moves)
                    bad = self.desk_not_ready()
                    if bad:
                        return False, f'refused: {bad}'
                    self.cmdq.put(cmd)
                    return True, 'restarting the arm stack (controllers inactive, robots do not move)'
                if self.a.sim:
                    return False, 'refused: no error recovery in sim'
            elif cmd == 'bringup':
                if s != 'OFFLINE':
                    return False, f'refused: already {s}'
                bad = self.desk_not_ready()
                if bad:
                    return False, f'refused: {bad}'
            elif cmd == 'shutdown':
                if s not in ('IDLE', 'ERROR', 'STARTING_UP'):
                    return False, f'refused: SHUTDOWN needs IDLE/ERROR/STARTING_UP (now {s}); STOP first'
            self.cmdq.put(cmd)
            return True, f'{cmd} accepted'

    def robot_not_ready(self):
        if self.a.sim:
            return ''
        now = time.monotonic()
        for arm in ARMS:
            r = self.robot[arm]
            if now - r['t'] > 1.0:
                return f'no robot state from {arm}'
            if r['mode'] != 'IDLE':
                return f'{arm} robot mode is {r["mode"]} (needs IDLE)'
        return ''

    # ------------------------------------------------------------------ helpers (main thread)
    def event(self, text):
        self.last_event = text
        self.get_logger().info(text)

    def set_state(self, state, message=''):
        with self.lock:
            if state != self.state or message != self.message:
                self.get_logger().info(f'{state}: {message}')
            if state != self.state:
                self.since = time.time()
            self.state, self.message = state, message
        self.publish_status()

    def launch(self, name, args):
        self.bringup[name] = Proc(name, [SIGINT_DEFAULT, 'ros2', 'launch', *args], self.logdir, signal.SIGINT)

    def controllers_ready(self, arm):
        cli = self.list_cli[arm]
        if not cli.service_is_ready():
            return False
        fut = cli.call_async(ListControllers.Request())
        end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        if not fut.done() or fut.result() is None:
            return False
        st = {c.name: c.state for c in fut.result().controller}
        self.ctrl_cache[arm] = st
        return st.get(CTRL) == 'inactive' and st.get('joint_state_broadcaster') == 'active'

    def gello_status(self, arm):
        g = self.gello[arm]
        if g['q'] is None or time.monotonic() - g['t'] > 0.5:
            return 'no data'
        h = self.holder.get(arm)
        if not h:
            return 'no holder pose'
        return 'in holder' if max(abs(x - y) for x, y in zip(g['q'], h)) <= self.a.holder_tol \
            else 'out of holder'

    def acquire_lock(self):
        fd = open(self.a.lock_file, 'w')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            fd.close()
            return False
        self.lock_fd = fd
        return True

    def release_lock(self):
        if self.lock_fd:
            self.lock_fd.close()
            self.lock_fd = None

    # ------------------------------------------------------------------ commands (main thread)
    def do_bringup(self):
        pat = r'^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py|^\S*fr3_duo_home( |$)'
        if subprocess.run(['pgrep', '-f', pat], stdout=subprocess.DEVNULL).returncode == 0:
            return self.set_state('OFFLINE', 'bringup refused: ros2_control_node / gate / homing already running')
        if not self.acquire_lock():
            return self.set_state('OFFLINE', f'bringup refused: {self.a.lock_file} held (homing running?)')
        self.logdir = os.path.join(DEMO, 'logs', 'demo_' + time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(self.logdir, exist_ok=True)
        self.error = None
        self.arms_crash_reported = False
        self.set_state('STARTING_UP', 'starting arms (controllers inactive)')
        self.start_arms()
        if self.a.grippers:
            gcfg = self.load_yaml(os.path.join(
                subprocess.run(['ros2', 'pkg', 'prefix', 'franka_gripper_manager'], capture_output=True,
                               text=True).stdout.strip(),
                'share', 'franka_gripper_manager', 'config', 'example_fr3_duo_config_robotiq.yaml'))
            for arm in ARMS:
                port = next((v['com_port'] for v in gcfg.values() if v.get('namespace') == f'{arm}/gripper'), None)
                if not port or not os.path.exists(port):
                    self.stop_bringup()
                    return self.set_state('OFFLINE', f'bringup failed: gripper {arm} port {port} missing')
                self.launch('gripper_' + arm, ['franka_gripper_manager', 'robotiq.launch.py',
                                               f'namespace:={arm}/gripper', f'com_port:={port}'])
        if self.a.gello:
            self.launch('gello', [os.path.join(DEMO, 'launch', 'gello_check.launch.py'), 'arms:=left,right'])
        if self.a.cameras:
            self.launch('cameras', [os.path.join(DEMO, 'launch', 'cameras.launch.py')])
        self.t_bringup = time.monotonic()
        self.cameras_reported = False
        self.event('bringup started (grippers do their activation open/close)' if self.a.grippers
                   else 'bringup started')

    def start_arms(self):
        """Arm stack (both ros2_control_nodes, impedance controller loaded INACTIVE) + robot state throttles."""
        args = [os.path.join(DEMO, 'launch', 'arms_gated.launch.py'), 'arms:=left,right']
        if self.a.robot_config:
            args.append(f'robot_config:={self.a.robot_config}')
        self.launch('arms', args)
        if not self.a.sim:      # sim has no franka_robot_state_broadcaster
            thr = os.path.join(subprocess.run(['ros2', 'pkg', 'prefix', 'topic_tools'], capture_output=True,
                                              text=True).stdout.strip(), 'lib', 'topic_tools', 'throttle')
            for arm in ARMS:
                self.bringup['throttle_' + arm] = Proc(
                    'throttle_' + arm, [thr, 'messages', f'/{arm}/franka_robot_state_broadcaster/robot_state',
                                        '10.0', f'/{arm}/robot_state_10hz'], self.logdir, signal.SIGINT)

    def restart_arms(self):
        """ERROR RECOVERY after an arm stack crash (reflex / E-stop): restart only the arm stack. Grippers, GELLO
        and cameras keep running. Controllers come up inactive -> the robots do not move."""
        self.set_state('RECOVERING', 'restarting the arm stack (grippers, GELLO, cameras keep running)')
        self.end_gates()
        self.stop_procs(['throttle_left', 'throttle_right', 'arms'])
        if subprocess.run(['pgrep', '-f', ARM_CM_PATTERN], stdout=subprocess.DEVNULL).returncode == 0:
            return self.set_state('ERROR', 'restart failed: old arm processes still running - SHUT DOWN, then BRING UP')
        self.error = None
        self.arms_crash_reported = False
        self.start_arms()
        self.t_bringup = time.monotonic()
        self.event('arm stack restarted')
        self.set_state('STARTING_UP', 'restarting arms (controllers inactive)')

    def check_bringup_ready(self):
        missing = []
        for arm in ARMS:
            if not self.controllers_ready(arm):
                missing.append(f'{arm} controllers')
            if not self.a.sim and time.monotonic() - self.robot[arm]['t'] > 1.0:
                missing.append(f'{arm} robot state')
            if self.a.grippers and not self.grip_cli[arm].server_is_ready():
                missing.append(f'{arm} gripper')
            if self.a.gello and self.gello_status(arm) == 'no data':
                missing.append(f'{arm} GELLO')
        return missing

    def arm_crash_cause(self):
        """Plain words for why a ros2_control_node died, from this run's arms.log ('' if none died)."""
        arms = self.bringup.get('arms')
        if arms is None:
            return ''
        try:
            with open(arms.log_path, errors='replace') as f:
                f.seek(arms.log_offset)
                return crash_causes(f.read())
        except OSError:
            return ''

    def report_arm_crash(self):
        """Arm stack ended: the Franka has stopped both robots (no ROS control any more)."""
        self.arms_crash_reported = True
        self.stop_gates()
        cause = self.arm_crash_cause() or 'arm stack stopped'
        self.error = (f'{cause} - robots stopped by the Franka. Check the robots (release E-stop), '
                      f'then press ERROR RECOVERY (logs: {self.logdir})')
        self.set_state('ERROR', self.error)

    def kill_orphan_gates(self):
        """Arm launch gone and no ros2_control_node left: a gate has nothing left to deactivate (it would only
        retry ~15 s against a missing controller manager), so end it at once."""
        arms = self.bringup.get('arms')
        if not self.gates or not (arms is None or not arms.alive()):
            return
        alive = [arm for arm, p in self.gates.items() if p.alive()]
        if alive and subprocess.run(['pgrep', '-f', ARM_CM_PATTERN], stdout=subprocess.DEVNULL).returncode != 0:
            for arm in alive:
                self.gates[arm].p.kill()
            self.event(f'gates {", ".join(alive)} ended: arm stack gone, nothing left to deactivate')

    def end_gates(self):
        """Signal the gates and wait until they are gone (deactivate -> robots hold); KILL after 20 s."""
        if not self.gates:
            return
        self.stop_gates()
        end = time.monotonic() + 20
        while any(p.alive() for p in self.gates.values()) and time.monotonic() < end:
            self.kill_orphan_gates()
            time.sleep(0.1)
        for arm, p in self.gates.items():
            if p.alive():
                self.get_logger().error(f'gate {arm} still alive 20 s after signal - KILL')
                p.p.kill()
        self.gates.clear()
        self.gate_mode = None

    def stop_gates(self, sig=signal.SIGINT):
        for p in self.gates.values():
            p.signal(sig)

    def stop_bringup(self):
        """Gates must already be gone. Stops launches in reverse order, then releases the lock."""
        self.stop_procs(['cameras', 'gello', 'gripper_left', 'gripper_right', 'throttle_left', 'throttle_right',
                         'arms'])
        self.bringup.clear()
        self.release_lock()

    def stop_procs(self, names):
        """SIGINT each (in order), escalate to SIGTERM / KILL; removes them from the bringup."""
        for name in names:
            p = self.bringup.pop(name, None)
            if p and p.alive():
                p.signal(signal.SIGINT)
                end = time.monotonic() + (20 if name == 'arms' else 10)
                while p.alive() and time.monotonic() < end:
                    time.sleep(0.1)
                if p.alive():
                    self.get_logger().error(f'{name} did not stop on SIGINT - SIGTERM')
                    p.signal(signal.SIGTERM)
                    time.sleep(3)
                    if p.alive():
                        p.p.kill()

    def start_gates(self, mode):
        self.gate_mode = mode
        with self.lock:
            self.gate_state = {arm: '' for arm in ARMS}
        for arm in ARMS:
            # --wait-timeout first: --gate-args (sim tests) may override it
            cmd = [sys.executable, GATE, '--arm', arm, '--wait-timeout', str(self.a.start_timeout),
                   *self.a.gate_args.split()]
            if mode == 'home':
                # '=' form: a pose starting with a negative value must not be parsed as an option
                cmd += ['--target-joints=' + ','.join(f'{q:.5f}' for q in self.home[arm]),
                        '--move-timeout', '60']
            self.gates[arm] = Proc(f'gate_{mode}_{arm}_{time.strftime("%H%M%S")}', cmd, self.logdir,
                                   signal.SIGTERM)

    def do_recover(self):
        if self.arms_crash_reported:
            return self.restart_arms()
        self.set_state('RECOVERING', 'Franka automatic error recovery on both arms')
        futs = {}
        for arm in ARMS:
            if not self.recover_cli[arm].wait_for_server(timeout_sec=2.0):
                return self.set_state('ERROR', f'recover failed: {arm} error_recovery action not available')
            futs[arm] = self.recover_cli[arm].send_goal_async(ErrorRecovery.Goal())
        end = time.monotonic() + 15
        results = {}
        while time.monotonic() < end and len(results) < len(ARMS):
            for arm, f in futs.items():
                if arm in results or not f.done():
                    continue
                gh = f.result()
                if gh is None or not gh.accepted:
                    results[arm] = False
                    continue
                futs[arm] = gh.get_result_async()
                futs[arm].add_done_callback(lambda rf, arm=arm: results.__setitem__(arm, rf.result() is not None))
            time.sleep(0.05)
        time.sleep(1.0)                                     # let robot_mode update
        bad = self.robot_not_ready()
        if len(results) < len(ARMS) or not all(results.values()) or bad:
            return self.set_state('ERROR', f'recover not complete: {bad or results}')
        self.error = None
        self.set_state('IDLE', 'recovered')

    def handle_cmd(self, cmd):
        self.event(f'command: {cmd}')
        if cmd == 'stop':
            if self.gates:
                self.stop_gates()
                if self.state != 'ERROR':                   # keep the ERROR message (what happened, what to do)
                    self.set_state('STOPPING', 'deactivating controllers (robots hold)')
            return
        if cmd == 'bringup':
            return self.do_bringup()
        if cmd == 'shutdown':
            if self.gates:
                self.set_state('SHUTTING_DOWN', 'waiting for the gates to exit (robots hold)')
                self.end_gates()
            self.set_state('SHUTTING_DOWN', 'stopping cameras, GELLO, grippers, arms')
            self.stop_bringup()
            self.error = None
            return self.set_state('OFFLINE', 'shut down')
        if cmd == 'recover':
            return self.do_recover()
        with self.lock:                                     # state may have changed since accepted
            if self.state != 'IDLE':
                return self.event(f'{cmd} dropped: state changed to {self.state}')
        if cmd == 'start':
            self.start_gates('teleop')
            self.set_state('TELEOP_STARTING', 'take the GELLOs out of their holders and hold them still')
        elif cmd == 'home':
            self.start_gates('home')
            self.set_state('HOMING', 'both arms ramping to the home pose')

    # ------------------------------------------------------------------ monitoring (main thread)
    def monitor(self):
        s = self.state
        # bringup components
        if s not in ('OFFLINE', 'SHUTTING_DOWN'):
            dead = [n for n, p in self.bringup.items() if not p.alive() and n != 'cameras']
            cam = self.bringup.get('cameras')
            if cam and not cam.alive() and not self.cameras_reported:
                self.cameras_reported = True
                self.event(f'cameras stopped (rc {cam.rc()}, see {self.logdir}/cameras.log) - arms unaffected')
            if 'arms' in dead and not self.arms_crash_reported:
                return self.report_arm_crash()        # also replaces an earlier ERROR message
            if 'arms' in dead:
                self.kill_orphan_gates()
            if dead and self.error is None:
                self.stop_gates()
                self.error = f'component stopped: {", ".join(dead)} (see {self.logdir}) - SHUTDOWN and BRINGUP'
                return self.set_state('ERROR', self.error)
        if s == 'STARTING_UP':
            missing = self.check_bringup_ready()
            if not missing:
                return self.set_state('IDLE', 'ready')
            if time.monotonic() - self.t_bringup > self.a.bringup_timeout:
                self.set_state('SHUTTING_DOWN', f'bringup timeout, missing: {missing}')
                self.stop_bringup()
                return self.set_state('OFFLINE', f'bringup failed, missing: {missing}')
            if self.message != f'waiting for: {", ".join(missing)}':
                self.set_state('STARTING_UP', f'waiting for: {", ".join(missing)}')
            return
        # robot errors (E-stop, reflex)
        if not self.a.sim and s not in ('OFFLINE', 'STARTING_UP', 'SHUTTING_DOWN', 'RECOVERING'):
            for arm in ARMS:
                r = self.robot[arm]
                if r['mode'] in ('REFLEX', 'USER_STOPPED') and s != 'ERROR':
                    self.stop_gates()
                    words = f'{arm}: {r["mode"]}' + (f' ({", ".join(r["errors"])})' if r['errors'] else '')
                    self.error = words + ' - release E-stop / clear, then press ERROR RECOVERY'
                    return self.set_state('ERROR', self.error)
        # gates
        if self.gates:
            ended = {arm: p.rc() for arm, p in self.gates.items() if not p.alive()}
            if ended and len(ended) < len(self.gates):
                # one ended -> stop the other, except in HOME when it simply arrived (exit 0, never signalled)
                arrived = self.gate_mode == 'home' and all(
                    rc == 0 and self.gates[a].t_signal is None for a, rc in ended.items())
                if not arrived:
                    self.stop_gates()
            for arm, p in self.gates.items():
                if p.alive() and p.t_signal and time.monotonic() - p.t_signal > 15:
                    self.get_logger().error(f'gate {arm} still alive 15 s after signal - SIGTERM/KILL')
                    p.signal(signal.SIGTERM)
                    if time.monotonic() - p.t_signal > 20:
                        p.p.kill()
            if len(ended) == len(self.gates):
                self.finish_gates(ended)
                return
            if s == 'TELEOP_STARTING' and all(self.gate_state[a].startswith('LIVE') for a in ARMS):
                return self.set_state('TELEOP', 'both arms LIVE')
            if s in ('TELEOP_STARTING', 'HOMING'):
                txt = '; '.join(f'{a}: {self.gate_state[a] or "starting"}' for a in ARMS)
                if txt != self.message:
                    self.set_state(s, txt)

    def finish_gates(self, codes):
        mode = self.gate_mode
        words = {0: 'stopped', 2: 'refused', 3: 'fault', 4: 'DEACTIVATION FAILED', 5: 'startup error'}
        detail = '; '.join(f'{a}: {words.get(c, c)} - {self.gate_state[a]}' if self.gate_state[a]
                           else f'{a}: stopped before it started' if self.gates[a].t_signal and c in (0, -2, -15)
                           else f'{a}: exited before starting (rc {c}, see {self.logdir})'
                           for a, c in codes.items())
        self.gates.clear()
        self.gate_mode = None
        if not self.arms_crash_reported and self.arm_crash_cause():
            self.report_arm_crash()                   # gate saw the dead controller manager first
        if any(c == 4 for c in codes.values()) and not self.arms_crash_reported:
            # (with the arm stack gone, exit 4 just means "no controller manager left to ask")
            self.error = 'controller could not be deactivated - use E-stop, SHUTDOWN'
            return self.set_state('ERROR', self.error + f' ({detail})')
        if self.error:
            return self.set_state('ERROR', self.error)
        if mode == 'home' and all(c == 0 and self.gate_state[a].startswith('DONE') for a, c in codes.items()):
            return self.set_state('IDLE', 'both arms at home')
        self.set_state('IDLE', f'{mode} ended: {detail}')

    # ------------------------------------------------------------------ status
    def status_dict(self):
        now = time.monotonic()
        with self.lock:
            arms = {}
            for arm in ARMS:
                r = self.robot[arm]
                fresh = now - r['t'] < 1.0
                arms[arm] = {
                    'connected': bool(self.ctrl_cache[arm]) if 'arms' in self.bringup else False,
                    'robot_mode': r['mode'] if fresh else ('n/a (sim)' if self.a.sim else 'no data'),
                    'errors': r['errors'] if fresh else [],
                    'gate': self.gate_state[arm] if arm in self.gates else 'not running',
                    'gello': self.gello_status(arm),
                    'gripper': ('ready' if self.grip_cli[arm].server_is_ready() else 'not ready')
                    if 'gripper_' + arm in self.bringup else 'off',
                    'home_pose': bool(self.home[arm]),
                }
            cam = self.bringup.get('cameras')
            return {'state': self.state, 'message': self.message, 'since': self.since,
                    'arms': arms, 'cameras': {k: round(v['fps'], 1) for k, v in self.cam.items()},
                    'cameras_launch': 'off' if cam is None else ('running' if cam.alive() else 'stopped'),
                    'pedal': 'connected' if now - self.pedal_t < 3.0 else 'not connected',
                    'last_event': self.last_event, 'sim': self.a.sim, 'logdir': self.logdir}

    def publish_status(self):
        self.pub_status.publish(String(data=json.dumps(self.status_dict())))

    # ------------------------------------------------------------------ main loop
    def run(self, stop_flag):
        self.get_logger().info(f'demo_supervisor running, state {self.state}')   # start_demo.sh waits for this
        if self.a.bringup:
            self.request('bringup')
        last_status = last_cam = time.monotonic()
        while not stop_flag.is_set():
            try:
                cmd = self.cmdq.get(timeout=0.05)
                self.handle_cmd(cmd)
            except queue.Empty:
                pass
            self.monitor()
            now = time.monotonic()
            if now - last_cam >= 2.0:
                for v in self.cam.values():
                    v['fps'], v['n'] = v['n'] / (now - last_cam), 0
                last_cam = now
            if now - last_status >= 0.5:
                last_status = now
                self.publish_status()
        # supervisor exit: stop gates (robots hold), then bringup
        if self.gates:
            self.stop_gates()
            end = time.monotonic() + 15
            while any(p.alive() for p in self.gates.values()) and time.monotonic() < end:
                time.sleep(0.1)
        self.stop_bringup()
        self.set_state('OFFLINE', 'supervisor exited')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--bringup', action='store_true', help='bring up at start')
    ap.add_argument('--sim', action='store_true', help='fake hardware (needs ROS_DOMAIN_ID=77)')
    ap.add_argument('--robot-config', default='')
    ap.add_argument('--no-grippers', dest='grippers', action='store_false')
    ap.add_argument('--no-gello', dest='gello', action='store_false')
    ap.add_argument('--no-cameras', dest='cameras', action='store_false')
    ap.add_argument('--gate-args', default='')
    ap.add_argument('--holder-file', default=os.path.join(DEMO, 'config', 'gello_holder_poses.yaml'))
    ap.add_argument('--holder-tol', type=float, default=0.25)
    ap.add_argument('--home-file', default=os.path.expanduser('~/fr3_tools/fr3_duo_home/home_poses.txt'))
    ap.add_argument('--lock-file', default=os.path.expanduser('~/fr3_tools/.home.lock'))
    ap.add_argument('--bringup-timeout', type=float, default=90.0)
    ap.add_argument('--start-timeout', type=float, default=60.0,
                    help='START: time to lift the GELLOs out of the holders and hold them still (gate default 30 s)')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if a.sim and (os.environ.get('ROS_DOMAIN_ID') != '77' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        sys.exit('--sim needs ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1')
    if a.sim and '--holder-file' not in sys.argv:
        sys.exit('--sim needs an explicit --holder-file (the real holder poses do not match the mock)')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    sup = Supervisor(a)
    stop_flag = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(sup)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        sup.run(stop_flag)
    finally:
        ex.shutdown()
        sup.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
