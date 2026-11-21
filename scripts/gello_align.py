#!/usr/bin/env python3
"""Live GELLO-vs-robot alignment readout (READ ONLY: subscribes, never publishes).

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/gello_align.py --arm left [--tol 0.15]

Needs the arm stack (arms_gated.launch.py) and GELLO on /gello_check_<arm> running.
Move the GELLO until every joint shows OK, then hold it still. Ctrl-C to quit.
If moving a GELLO joint makes its diff move the WRONG way, stop: sign/offset problem.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', required=True, choices=['left', 'right'])
    ap.add_argument('--tol', type=float, default=0.15)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # plain KeyboardInterrupt on Ctrl-C
    node = Node(f'gello_align_{a.arm}')
    st = {'g': None, 'r': None, 'tg': 0.0, 'tr': 0.0}

    def on_gello(m):
        st['g'], st['tg'] = list(m.position[:7]), time.monotonic()

    def on_robot(m):
        st['r'] = [next(p for n, p in zip(m.name, m.position) if n.endswith(f'fr3_joint{i}'))
                   for i in range(1, 8)]
        st['tr'] = time.monotonic()

    node.create_subscription(JointState, f'/gello_check_{a.arm}/gello/joint_states', on_gello, 10)
    node.create_subscription(JointState, f'/{a.arm}/franka/joint_states', on_robot, 10)
    next_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_print:
                continue
            next_print = now + 0.2
            lines = [f'GELLO alignment ({a.arm})  tol {a.tol:.2f} rad   Ctrl-C to quit', '']
            if st['g'] is None or now - st['tg'] > 0.5:
                lines.append('  waiting for GELLO data on /gello_check_%s/gello/joint_states' % a.arm)
            elif st['r'] is None or now - st['tr'] > 0.5:
                lines.append('  waiting for robot data on /%s/franka/joint_states' % a.arm)
            else:
                ok_all = True
                lines.append('  joint   robot    GELLO     diff')
                for i, (g, r) in enumerate(zip(st['g'], st['r'])):
                    d = g - r
                    ok = abs(d) <= a.tol
                    ok_all &= ok
                    hint = 'OK' if ok else ('move GELLO joint  -' if d > 0 else 'move GELLO joint  +')
                    lines.append(f'   j{i + 1}   {r:+7.3f}  {g:+7.3f}  {d:+7.3f}   {hint}')
                worst = max(abs(g - r) for g, r in zip(st['g'], st['r']))
                lines += ['', f'  max |diff| {worst:.3f} rad   ->   '
                          + ('ALL OK - hold GELLO still' if ok_all else 'not aligned yet')]
            sys.stdout.write('\x1b[H\x1b[2J' + '\n'.join(lines) + '\n')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
