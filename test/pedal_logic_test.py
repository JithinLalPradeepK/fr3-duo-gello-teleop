#!/usr/bin/env python3
"""Unit test of scripts/pedal_logic.py (simulated time, no pedal, no ROS)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from pedal_logic import HoldLogic  # noqa: E402

fails = []


def check(what, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {what}' + (f' - {detail}' if detail else ''), flush=True)
    if not ok:
        fails.append(what)


def run(events, end, dt=0.02, **kw):
    """events: list of (time, cmd, value). Returns [(time, action)] for fire actions plus all actions."""
    lg = HoldLogic({'start': 3.0, 'home': 3.0, 'stop': 0.0}, **kw)
    out, ev = [], sorted(events)
    t, i = 0.0, 0
    while t <= end + 1e-9:
        while i < len(ev) and ev[i][0] <= t + 1e-9:
            for a in lg.on_key(ev[i][1], ev[i][2], t):
                out.append((round(t, 2), a))
            i += 1
        for a in lg.tick(t):
            out.append((round(t, 2), a))
        t += dt
    return out


def fires(out):
    return [(t, a[1]) for t, a in out if a[0] == 'fire']


print('== P1 START needs a 3 s hold')
out = run([(0.0, 'start', 1), (5.0, 'start', 0)], 6.0)
f = fires(out)
check('fires exactly once', len(f) == 1, str(f))
check('at 3.0 s (not before)', f and 2.99 <= f[0][0] <= 3.05, str(f))
out = run([(0.0, 'start', 1), (2.9, 'start', 0)], 6.0)
check('released at 2.9 s: never fires', not fires(out), str([a for _, a in out if a[0] in ('fire', 'cancel')]))
check('...and reports "released early"', any(a[0] == 'cancel' and 'early' in a[2] for _, a in out))
out = run([(0.0, 'start', 1)], 6.0)
check('held for ever: fires once only (no repeat of the command)', len(fires(out)) == 1)

print('== P2 HOME needs a 3 s hold, short taps are ignored')
out = run([(0.0, 'home', 1), (0.1, 'home', 0), (1.0, 'home', 1), (1.2, 'home', 0)], 6.0)
check('two short taps: nothing fires', not fires(out))
out = run([(0.0, 'home', 1), (3.5, 'home', 0)], 6.0)
check('3.5 s hold fires HOME once', fires(out) == [(fires(out)[0][0], 'home')] and len(fires(out)) == 1)

print('== P3 STOP is immediate')
out = run([(1.0, 'stop', 1), (1.1, 'stop', 0)], 3.0)
f = fires(out)
check('STOP fires at key-down (1.0 s)', f == [(1.0, 'stop')], str(f))
out = run([(1.0, 'stop', 1), (1.3, 'stop', 1), (1.5, 'stop', 1)], 3.0)
check('every STOP press fires (bounce never swallowed)', len(fires(out)) == 3, str(fires(out)))

print('== P4 pressing another pedal during a hold')
out = run([(0.0, 'start', 1), (1.0, 'stop', 1), (1.1, 'stop', 0), (1.5, 'start', 0)], 6.0)
check('STOP during a START hold fires immediately', (1.0, 'stop') in fires(out), str(fires(out)))
check('the START hold continues and is cancelled by its release', not any(c == 'start' for _, c in fires(out)))
out = run([(0.0, 'start', 1), (1.0, 'home', 1), (5.0, 'home', 0), (5.0, 'start', 0)], 6.0)
check('START then HOME pressed: only HOME may fire (it needs its own 3 s from 1.0 s)',
      [c for _, c in fires(out)] == ['home'], str(fires(out)))

print('== P5 auto-repeat as heartbeat (lost key-up)')
ev = [(0.0, 'start', 1)] + [(0.5 + 0.03 * k, 'start', 2) for k in range(40)]     # repeats until 1.7 s, then silence
out = run(ev, 6.0)
check('repeats stop at 1.7 s without key-up: hold cancelled, never fires', not fires(out),
      str([a for _, a in out if a[0] in ('fire', 'cancel')]))
ev = [(0.0, 'start', 1)] + [(0.5 + 0.03 * k, 'start', 2) for k in range(120)]     # repeats until 4.1 s
out = run(ev, 6.0)
check('continuous repeats for >3 s: fires at 3.0 s', fires(out) and 2.99 <= fires(out)[0][0] <= 3.05, str(fires(out)))

print('== P6 release and press again')
out = run([(0.0, 'start', 1), (3.2, 'start', 0), (4.0, 'start', 1), (7.1, 'start', 0)], 8.0)
check('two separate 3 s holds fire twice', len(fires(out)) == 2, str(fires(out)))

print('== P7 progress reports')
out = run([(0.0, 'start', 1), (5.0, 'start', 0)], 6.0)
prog = [a[2] for _, a in out if a[0] == 'progress']
check('progress goes 0 -> 1 monotonically', prog and prog[0] == 0.0 and prog[-1] == 1.0 and prog == sorted(prog),
      f'{len(prog)} reports, first {prog[:3]}, last {prog[-2:]}')

print('== P8 pulse pedal (F12): tap-to-confirm')
def run_pulse(taps, end, **kw):
    lg = HoldLogic({'reset': 3.0}, **kw)
    out, t = [], 0.0
    taps = sorted(taps)
    i = 0
    while t <= end + 1e-9:
        while i < len(taps) and taps[i] <= t + 1e-9:
            for key_ev in (1, 0):                         # the pedal sends down + up immediately
                out += [(round(t, 2), a) for a in lg.on_key('reset', key_ev, t, pulse=True)]
            i += 1
        out += [(round(t, 2), a) for a in lg.tick(t)]
        t += 0.02
    return out
out = run_pulse([1.0], 10.0)
check('one tap only arms (nothing fires) and times out after 5 s', not fires(out) and any(a[0] == 'armed' for _, a in out)
      and any(a[0] == 'cancel' and 'second tap' in a[2] for _, a in out), str([a for _, a in out if a[0] in ('armed', 'fire', 'cancel')]))
out = run_pulse([1.0, 2.5], 6.0)
check('two taps 1.5 s apart fire RESET once', fires(out) == [(2.5, 'reset')], str(fires(out)))
out = run_pulse([1.0, 1.2], 6.0)
check('a bounce (second tap after 0.2 s) does NOT fire', not fires(out), str(fires(out)))
out = run_pulse([1.0, 7.5], 9.0)
check('second tap after the 5 s window only re-arms', not fires(out), str(fires(out)))
out = run_pulse([1.0, 2.0, 3.0], 6.0)
check('third tap after a fire arms again (never fires twice by itself)', fires(out) == [(2.0, 'reset')], str(fires(out)))
lg = HoldLogic({'reset': 3.0})
check('key-up / repeat events of a pulse key are ignored', lg.on_key('reset', 0, 0.0, pulse=True) == []
      and lg.on_key('reset', 2, 0.1, pulse=True) == [])

print(f'===== {"PASS" if not fails else "FAIL: " + str(fails)}')
sys.exit(1 if fails else 0)
