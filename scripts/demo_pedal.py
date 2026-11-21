#!/usr/bin/env python3
"""Foot pedal -> demo_supervisor (same /demo services as the GUI buttons).

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/demo_pedal.py

POWBORO FS2020 (PCsensor 3553:b001), programmed with ElfKey as a keyboard (mapping changed 2026-11-21):
  pedal 1 = KEY_A   -> /demo/start      START (3 s hold)
  pedal 2 = KEY_B   -> /demo/stop_home  STOP at once; a stopped teleop is followed by HOME (pressed while HOMING: abort)
  pedal 3 = KEY_F12 -> /demo/reset      RESET: stop, clear errors, HOME, ready again
    F12 is a PULSE key (ElfKey sends press+release within 0.02 s, probe 2026-11-21), so a hold cannot be measured:
    it uses TAP-TO-CONFIRM (tap, then tap again within 5 s). Reprogram pedal 3 in ElfKey to a held letter key (KEY_C is
    already mapped to reset, as a normal 3 s hold) and the double tap is no longer needed.
- The pedal is GRABBED exclusively: its keystrokes never reach other windows or desktop shortcuts.
- LONG PRESS (decision 2026-11-21): START and HOME fire only after the pedal has been held for --hold seconds
  (default 3.0); releasing earlier cancels. STOP stays immediate (--stop-hold 0): it must never be delayed.
  After firing, release the pedal before it can fire again. Logic + tests: scripts/pedal_logic.py,
  test/pedal_logic_test.py.
- Publishes /demo/pedal/hold (JSON, ~10 Hz while a pedal is held; GUI progress bar / supervisor pre-warm):
    {"pedal": "start"|"home"|null, "progress": 0..1, "hold_s": 3.0, "fired": bool}
- Publishes /demo/pedal/status (JSON) at 1 Hz only while the pedal is connected (GUI "Pedal" light).
- Reconnects automatically if the pedal is unplugged.
Needs /etc/udev/rules.d/99-footswitch-event.rules (event* readable by group pcsensorhid).
"""
import argparse
import json
import select
import signal
import sys
import time

import os

import evdev
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pedal_logic import HoldLogic  # noqa: E402

DEVICE = '/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd'
DEFAULT_MAP = 'KEY_A=start,KEY_B=stop_home,KEY_C=reset,KEY_F12=reset'
DEFAULT_PULSE = 'KEY_F12'


class Pedal(Node):
    def __init__(self, a):
        super().__init__('demo_pedal', namespace='demo')
        self.a = a
        self.keymap = {}
        for item in a.map.split(','):
            key, cmd = item.split('=')
            self.keymap[evdev.ecodes.ecodes[key.strip()]] = cmd.strip()
        self.svc = {cmd: self.create_client(Trigger, f'/demo/{cmd}') for cmd in set(self.keymap.values())}
        self.pub = self.create_publisher(String, '/demo/pedal/status', 10)
        self.pub_hold = self.create_publisher(String, '/demo/pedal/hold', 10)
        self.logic = HoldLogic({cmd: (a.stop_hold if cmd in ('stop', 'stop_home') else a.hold)
                                for cmd in set(self.keymap.values())})
        self.down_t = {}                                 # raw key-down times (diagnostics)
        self.pulse_codes = {evdev.ecodes.ecodes[k.strip()] for k in a.pulse_keys.split(',') if k.strip()}
        self.dev = None
        self.last_press = {}
        self.last_event = ''
        self.next_try = 0.0
        self.next_beat = 0.0

    def connect(self):
        try:
            dev = evdev.InputDevice(self.a.device)
            dev.grab()
        except OSError as e:
            self.get_logger().warn(f'pedal not available ({e}) - retrying', throttle_duration_sec=30)
            return
        self.dev = dev
        self.get_logger().info(f'pedal connected and grabbed: {dev.name}')

    def raw_log(self, ev):
        """Every key event (down / up with the time it was held) - to see how each pedal really behaves."""
        now, name = time.monotonic(), self.keymap[ev.code]
        if ev.value == 1:
            self.down_t[ev.code] = now
            self.get_logger().info(f'key {evdev.ecodes.KEY[ev.code]} DOWN ({name})')
        elif ev.value == 0:
            held = now - self.down_t.pop(ev.code, now)
            self.get_logger().info(f'key {evdev.ecodes.KEY[ev.code]} UP   ({name}) held {held:.2f} s')

    def hold_msg(self, cmd, progress, fired=False):
        self.pub_hold.publish(String(data=json.dumps(
            {'pedal': cmd, 'progress': round(progress, 3), 'hold_s': self.a.hold, 'fired': fired})))

    def act(self, acts):
        for a in acts:
            if a[0] == 'fire':
                self.press(a[1])
            elif a[0] == 'progress':
                self.hold_msg(a[1], a[2], a[2] >= 1.0)
            elif a[0] == 'armed':
                self.get_logger().info(f'pedal {a[1].upper()} armed - tap again within {self.logic.confirm_window:.0f} s to confirm')
            elif a[0] == 'cancel':
                self.get_logger().info(f'pedal {a[1].upper()} hold cancelled ({a[2]})')
                self.hold_msg(None, 0.0)
            elif a[0] == 'done':
                self.hold_msg(None, 0.0)

    def disconnect(self, why):
        self.get_logger().warn(f'pedal disconnected: {why}')
        if self.logic.cur:
            self.logic.cur = None
            self.hold_msg(None, 0.0)
        try:
            self.dev.ungrab()
        except OSError:
            pass
        self.dev = None
        self.pub.publish(String(data=json.dumps({'connected': False, 'last': self.last_event})))

    def press(self, cmd):
        now = time.monotonic()
        if cmd != 'stop' and now - self.last_press.get(cmd, 0.0) < self.a.debounce:
            return                                       # STOP is never debounced
        self.last_press[cmd] = now
        self.last_event = f'{cmd} at {time.strftime("%H:%M:%S")}'
        cli = self.svc[cmd]
        if not cli.service_is_ready():
            self.get_logger().error(f'pedal {cmd.upper()}: demo_supervisor not reachable')
            return
        self.get_logger().info(f'pedal {cmd.upper()}')
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f, c=cmd: self.get_logger().info(
            f'  {c}: {"OK" if f.result() and f.result().success else "REFUSED"} - '
            f'{f.result().message if f.result() else "call failed"}'))

    def step(self):
        now = time.monotonic()
        if self.dev is None and now >= self.next_try:
            self.next_try = now + 2.0
            self.connect()
        if self.dev is not None:
            try:
                r, _, _ = select.select([self.dev.fd], [], [], 0.02)
                if r:
                    for ev in self.dev.read():
                        if ev.type == evdev.ecodes.EV_KEY and ev.value in (0, 1, 2) and ev.code in self.keymap:
                            self.raw_log(ev)
                            self.act(self.logic.on_key(self.keymap[ev.code], ev.value, time.monotonic(),
                                                       pulse=ev.code in self.pulse_codes))
                self.act(self.logic.tick(time.monotonic()))
            except OSError as e:
                self.disconnect(e)
            if self.dev is not None and now >= self.next_beat:
                self.next_beat = now + 1.0
                self.pub.publish(String(data=json.dumps({'connected': True, 'last': self.last_event})))
        rclpy.spin_once(self, timeout_sec=0.0 if self.dev is not None else 0.1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--device', default=DEVICE)
    ap.add_argument('--map', default=DEFAULT_MAP, help=f'default {DEFAULT_MAP}')
    ap.add_argument('--debounce', type=float, default=0.3, help='START/HOME only (s)')
    ap.add_argument('--pulse-keys', default=DEFAULT_PULSE,
                    help='keys that only send a quick press-release (no hold possible): tap-to-confirm instead of a hold')
    ap.add_argument('--hold', type=float, default=3.0, help='START/HOME: hold the pedal this long (s)')
    ap.add_argument('--stop-hold', type=float, default=0.0, help='STOP: 0 = immediate (never delay a stop)')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # own handlers: no rclpy traceback on exit
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    node = Pedal(a)
    try:
        while not stop and rclpy.ok():
            node.step()
    finally:
        if node.dev is not None:
            try:
                node.dev.ungrab()
            except OSError:
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
