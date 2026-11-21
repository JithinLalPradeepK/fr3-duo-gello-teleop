#!/usr/bin/env python3
"""Franka Desk agent for the demo: holds the Desk control token of both FR3s and offers PREPARE / PARK.

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/desk_agent.py          (started by start_demo.sh)

Services (std_srvs/Trigger, both arms at once, answered immediately; result in the status):
  /demo/desk/prepare   take control -> unlock brakes -> Execution mode -> FCI on
  /demo/desk/park      take control -> FCI off -> Programming mode -> lock brakes -> release control
  /demo/desk/{unlock, lock, execution, programming, fci_on, fci_off}   one step only (takes control if needed,
                       keeps it; waits until Desk reports the new state)
Both are REFUSED unless the arm stack is down (demo_supervisor OFFLINE / no ros2_control_node running).
Status: /demo/desk/status (JSON, 1 Hz): per arm mode, brakes, fci, control (+ busy / last result).
On exit (SIGINT/SIGTERM): waits until no ros2_control_node runs (max 60 s), then releases control (-> FCI off).
Brakes and operating mode are left as they are (user decision 2026-11-20).
Never forces the token: if Desk in a browser holds it, release it there.
Credentials: GNOME keyring (see franka_desk.py).
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from franka_desk import ROBOTS, Desk  # noqa: E402

CM_PATTERN = r'^/opt/ros/humble/lib/controller_manager/ros2_control_node'
REQUESTED_BY = 'robot_demo'


def stack_running():
    return subprocess.run(['pgrep', '-f', CM_PATTERN], stdout=subprocess.DEVNULL).returncode == 0


def brakes_word(b):
    if not b:
        return '?'
    s = set(b)
    return s.pop().lower() if len(s) == 1 else ', '.join(sorted(x.lower() for x in s))


class ArmDesk:
    """One robot: Desk session + our control token + last status (thread-safe)."""

    def __init__(self, arm, log):
        self.arm, self.log = arm, log
        self.d = Desk(ROBOTS[arm])
        self.lock = threading.Lock()                  # one Desk call sequence at a time
        self.st = {'mode': '?', 'brakes': '?', 'fci': None, 'control': '?', 'error': 'not connected'}
        self.t_status = 0.0

    def ensure_login(self):
        if self.d.auth is None:
            self.d.login()

    def call(self, fn):
        """Desk call with one re-login on an auth error."""
        self.ensure_login()
        try:
            return fn()
        except Exception as e:  # noqa: BLE001
            if '401' in str(e) or '403' in str(e) or 'Unauthorized' in str(e):
                self.d.auth = None
                self.ensure_login()
                return fn()
            raise

    def poll(self):
        try:
            with self.lock:
                raw = self.call(self.d.status)
            tok = raw.get('token') or {}
            if tok.get('id') is not None and tok.get('id') == self.d.control_token_id:
                control = 'demo'
            elif tok:
                control = f'other: {tok.get("ownedBy")}'
                if self.d.control_token:                # we thought we had it
                    self.log(f'{self.arm}: control token taken over by {tok.get("ownedBy")}')
                    self.d.control_token = self.d.control_token_id = None
            else:
                control = 'none'
                if self.d.control_token:
                    self.log(f'{self.arm}: our control token is gone')
                    self.d.control_token = self.d.control_token_id = None
            self.st = {'mode': raw.get('mode') or '?', 'brakes': brakes_word(raw.get('brakes')),
                       'fci': bool(raw.get('fci')), 'control': control, 'error': ''}
            self.t_status = time.monotonic()
        except Exception as e:  # noqa: BLE001
            self.st = dict(self.st, error=f'Desk not reachable: {e}'[:160])
            self.d.auth = None

    def take_control(self):
        if self.d.control_token:
            return
        r = self.call(lambda: self.d.take_control(REQUESTED_BY))
        if r.status_code != 200:
            owner = (self.d.status().get('token') or {}).get('ownedBy', '?')
            raise RuntimeError(f'control token not granted (HTTP {r.status_code}, held by {owner}) - '
                               'release control in Desk (browser) and try again')
        self.log(f'{self.arm}: control token taken (id {self.d.control_token_id})')

    def prepare(self):
        with self.lock:
            self.take_control()
            st = self.call(self.d.status)
            if brakes_word(st.get('brakes')) != 'unlocked':
                self.log(f'{self.arm}: unlocking brakes')
                self.call(self.d.unlock_brakes)
            if st.get('mode') != 'Execution':
                self.log(f'{self.arm}: Execution mode')
                self.call(lambda: self.d.set_mode('execution'))
            if not self.call(self.d.status).get('fci'):
                self.log(f'{self.arm}: FCI on')
                self.call(self.d.fci_on)
            end = time.monotonic() + 5.0           # Desk reports FCI/brakes a moment after the call
            while True:
                st = self.call(self.d.status)
                ok = brakes_word(st.get('brakes')) == 'unlocked' and st.get('mode') == 'Execution' and st.get('fci')
                if ok or time.monotonic() > end:
                    break
                time.sleep(0.3)
            if not ok:
                raise RuntimeError(f'not ready after prepare: mode {st.get("mode")}, '
                                   f'brakes {brakes_word(st.get("brakes"))}, FCI {st.get("fci")}')

    def park(self):
        with self.lock:
            self.take_control()
            st = self.call(self.d.status)
            if st.get('fci'):
                self.log(f'{self.arm}: FCI off')
                self.call(self.d.fci_off)
            if st.get('mode') != 'Programming':
                self.log(f'{self.arm}: Programming mode')
                self.call(lambda: self.d.set_mode('programming'))
            if brakes_word(st.get('brakes')) != 'locked':
                self.log(f'{self.arm}: locking brakes')
                self.call(self.d.lock_brakes)
            self.release()

    SINGLE = {  # name: (log text, call, check on the status dict)
        'unlock': ('unlocking brakes', lambda d: d.unlock_brakes(), lambda st: brakes_word(st.get('brakes')) == 'unlocked'),
        'lock': ('locking brakes', lambda d: d.lock_brakes(), lambda st: brakes_word(st.get('brakes')) == 'locked'),
        'execution': ('Execution mode', lambda d: d.set_mode('execution'), lambda st: st.get('mode') == 'Execution'),
        'programming': ('Programming mode', lambda d: d.set_mode('programming'),
                        lambda st: st.get('mode') == 'Programming'),
        'fci_on': ('FCI on', lambda d: d.fci_on(), lambda st: bool(st.get('fci'))),
        'fci_off': ('FCI off', lambda d: d.fci_off(), lambda st: not st.get('fci')),
    }

    def single(self, name):
        text, fn, check = self.SINGLE[name]
        with self.lock:
            self.take_control()
            if check(self.call(self.d.status)):
                self.log(f'{self.arm}: {text} - already so')
                return
            self.log(f'{self.arm}: {text}')
            self.call(lambda: fn(self.d))
            end = time.monotonic() + 5.0
            while not check(self.call(self.d.status)):
                if time.monotonic() > end:
                    raise RuntimeError(f'{text}: Desk did not confirm within 5 s')
                time.sleep(0.3)

    def release(self):
        if self.d.control_token:
            r = self.d.release_control()
            self.log(f'{self.arm}: control released (HTTP {r.status_code if r is not None else "-"})')


class DeskAgent(Node):
    def __init__(self):
        super().__init__('desk_agent', namespace='demo')
        self.arms = {arm: ArmDesk(arm, self.event) for arm in ROBOTS}
        self.sup_state, self.t_sup = None, 0.0
        self.busy = ''
        self.last = ''
        self.mutex = threading.Lock()
        cg = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, '/demo/desk/status', latched)
        self.create_subscription(String, '/demo/status', self.on_sup, latched, callback_group=cg)
        for name in ('prepare', 'park', *ArmDesk.SINGLE):
            self.create_service(Trigger, f'/demo/desk/{name}',
                                lambda req, res, n=name: self.on_service(n, res), callback_group=cg)

    def event(self, text):
        self.last = f'{time.strftime("%H:%M:%S")} {text}'
        self.get_logger().info(text)

    def on_sup(self, m):
        try:
            self.sup_state = json.loads(m.data).get('state')
            self.t_sup = time.monotonic()
        except ValueError:
            pass

    def on_service(self, name, res):
        with self.mutex:
            if self.busy:
                res.success, res.message = False, f'refused: {self.busy} in progress'
                return res
            sup = self.sup_state if time.monotonic() - self.t_sup < 3.0 else None
            if (sup not in (None, 'OFFLINE')) or stack_running():
                res.success = False
                res.message = (f'refused: arm stack is up (supervisor {sup or "?"}) - SHUT DOWN first'
                               if sup not in (None, 'OFFLINE') else 'refused: a ros2_control_node is running')
                return res
            self.busy = name
        threading.Thread(target=self.run_action, args=(name,), daemon=True).start()
        res.success, res.message = True, f'{name} started (both arms)'
        return res

    def run_action(self, name):
        errors = {}

        def one(arm):
            try:
                if name in ArmDesk.SINGLE:
                    self.arms[arm].single(name)
                else:
                    getattr(self.arms[arm], name)()
            except Exception as e:  # noqa: BLE001
                errors[arm] = str(e)[:200]
        threads = [threading.Thread(target=one, args=(arm,)) for arm in self.arms]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        for a in self.arms.values():
            a.poll()
        if errors:
            self.event(f'{name.upper()} FAILED: ' + '; '.join(f'{a}: {e}' for a, e in errors.items()))
        else:
            self.event(f'{name.upper()} done' + {'prepare': ': robots ready (brakes open, Execution, FCI on) - BRING UP',
                                                   'park': ': brakes locked, Programming, control released'}.get(name, ''))
        with self.mutex:
            self.busy = ''
        self.publish()

    def publish(self):
        st = {arm: dict(a.st) for arm, a in self.arms.items()}
        st['busy'] = self.busy
        st['last'] = self.last
        self.pub.publish(String(data=json.dumps(st)))

    def run(self, stop):
        next_poll = 0.0
        while not stop.is_set():
            if time.monotonic() >= next_poll:
                next_poll = time.monotonic() + 1.0
                if not self.busy:
                    for a in self.arms.values():
                        a.poll()
                self.publish()
            time.sleep(0.05)
        # exit: release control only after the arm stack is down (FCI off under a running stack = robot stop)
        end = time.monotonic() + 60
        while stack_running() and time.monotonic() < end:
            time.sleep(0.5)
        if stack_running():
            self.get_logger().error('arm stack still running after 60 s - releasing control anyway')
        for a in self.arms.values():
            try:
                with a.lock:
                    a.release()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{a.arm}: release failed: {e}')
        self.publish()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DeskAgent()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        node.run(stop)
    finally:
        ex.shutdown(timeout_sec=3.0)
        spin.join(timeout=3.0)                    # (exiting with the spin thread inside rcl aborted the process)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
