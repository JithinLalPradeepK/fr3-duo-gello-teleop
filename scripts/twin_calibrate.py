#!/usr/bin/env python3
"""Hand-eye / extrinsic calibration helper for the DIGITAL TWIN (no ROS, no robot motion - only edits a config file).

  python3 scripts/twin_calibrate.py --camera zed        (or d405_1 / d405_2)

Nudge the camera pose with single keys while watching the twin: scripts/twin_state.py re-reads config/digital_twin.yaml
within ~0.5 s, so the point cloud moves live. Align the cloud with the robot model (e.g. the mount and the arms, which
the camera sees), then press  c  to mark it calibrated.
  x/X y/Y z/Z   move -/+ along the parent frame axes        r/R p/P w/W   roll / pitch / yaw -/+
  [ ]           smaller / bigger steps                      c             toggle calibrated: true/false
  q             quit
Only the xyz / rpy / calibrated lines of that camera are rewritten (comments stay).
"""
import argparse
import os
import re
import sys
import termios
import tty

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STEPS = [(0.001, 0.002), (0.005, 0.01), (0.01, 0.02), (0.05, 0.05)]      # (metres, radians)


def read_block(text, cam):
    """(start, end) character range of `  <cam>:` block inside `cameras:`."""
    m = re.search(r'^cameras:\s*$', text, re.M)
    if not m:
        raise SystemExit('no "cameras:" section in the config')
    b = re.search(rf'^  {re.escape(cam)}:.*$', text[m.end():], re.M)
    if not b:
        raise SystemExit(f'camera "{cam}" not found under cameras:')
    start = m.end() + b.end()
    nxt = re.search(r'^(?:  \S|\S)', text[start + 1:], re.M)
    end = start + 1 + nxt.start() if nxt else len(text)
    return start, end


def get_vec(block, key):
    m = re.search(rf'^\s+{key}:\s*\[([^\]]*)\]', block, re.M)
    return [float(v) for v in m.group(1).split(',')]


def set_line(block, key, value):
    return re.sub(rf'^(\s+{key}:\s*)[^#\n]*?(\s*(#.*)?)$', lambda m: m.group(1) + value + (m.group(2) or ''), block,
                  count=1, flags=re.M)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', required=True)
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    a = ap.parse_args()
    text = open(a.config).read()
    s, e = read_block(text, a.camera)
    xyz, rpy = get_vec(text[s:e], 'xyz'), get_vec(text[s:e], 'rpy')
    cal = 'calibrated: true' in text[s:e]
    step = 1
    print(__doc__.split('\n\n', 2)[2])

    def save():
        nonlocal text
        t = open(a.config).read()
        s2, e2 = read_block(t, a.camera)
        blk = t[s2:e2]
        blk = set_line(blk, 'xyz', '[' + ', '.join(f'{v:.4f}' for v in xyz) + ']')
        blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in rpy) + ']')
        blk = set_line(blk, 'calibrated', 'true' if cal else 'false')
        t = t[:s2] + blk + t[e2:]
        with open(a.config + '.tmp', 'w') as f:
            f.write(t)
        os.replace(a.config + '.tmp', a.config)
        text = t

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    keys = {'x': (0, 0), 'y': (0, 1), 'z': (0, 2), 'r': (1, 0), 'p': (1, 1), 'w': (1, 2)}
    try:
        tty.setcbreak(fd)
        while True:
            dm, dr = STEPS[step]
            sys.stdout.write(f'\r{a.camera}: xyz [{xyz[0]:+.3f} {xyz[1]:+.3f} {xyz[2]:+.3f}] m  '
                             f'rpy [{rpy[0]:+.3f} {rpy[1]:+.3f} {rpy[2]:+.3f}] rad  step {dm * 1000:.0f} mm / '
                             f'{dr:.3f} rad  calibrated: {"yes" if cal else "no "}   ')
            sys.stdout.flush()
            ch = sys.stdin.read(1)
            if ch == 'q':
                break
            if ch == '[':
                step = max(0, step - 1)
            elif ch == ']':
                step = min(len(STEPS) - 1, step + 1)
            elif ch == 'c':
                cal = not cal
                save()
            elif ch.lower() in keys:
                vec, i = keys[ch.lower()]
                sign = 1 if ch.isupper() else -1
                (xyz if vec == 0 else rpy)[i] += sign * (dm if vec == 0 else dr)
                save()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print(f'\nsaved to {a.config}')


if __name__ == '__main__':
    main()
