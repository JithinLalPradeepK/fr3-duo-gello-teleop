#!/usr/bin/env python3
"""Foot pedal -> demo_supervisor (same /demo services as the GUI buttons).

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/demo_pedal.py

POWBORO FS2020 (PCsensor 3553:b001), programmed with ElfKey as a keyboard:
  pedal 1 = KEY_A   -> /demo/start
  pedal 2 = KEY_B   -> /demo/stop
  pedal 3 = KEY_F12 -> /demo/home
- The pedal is GRABBED exclusively: its keystrokes never reach other windows or desktop shortcuts.
- Acts on key-down only (ignores auto-repeat / key-up). STOP is never debounced; START/HOME ignore a
  second press within --debounce s.
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

import evdev
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEVICE = '/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd'
DEFAULT_MAP = 'KEY_A=start,KEY_B=stop,KEY_F12=home'


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

    def disconnect(self, why):
        self.get_logger().warn(f'pedal disconnected: {why}')
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
                        if ev.type == evdev.ecodes.EV_KEY and ev.value == 1 and ev.code in self.keymap:
                            self.press(self.keymap[ev.code])
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
