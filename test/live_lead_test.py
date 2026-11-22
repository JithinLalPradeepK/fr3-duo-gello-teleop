#!/usr/bin/env python3
"""Unit test of the gate's LIVE lead compensation (scripts/gello_gate.LiveSmoother; no ROS, no robot).
Robot model per joint: the stock joint impedance controller tau = K (q_goal - q) - D dq with a small link inertia
(I dq' = tau), integrated at 1 kHz - the damping lag D/K is what the lead must cancel.
  python3 test/live_lead_test.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
from gello_gate import CTRL_D, CTRL_K, LEAD_CAP, LEAD_S, LiveSmoother  # noqa: E402

INERTIA = (0.25, 0.25, 0.15, 0.1, 0.02, 0.015, 0.005)        # rough FR3 link inertias seen at the joint [kg m^2]
R = []


def check(name, ok, detail=''):
    R.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}{"  - " + detail if detail else ""}')


def run(gello, lead, t_end, stall_after=None, max_vel=3.5):
    """gello(t) -> 7 joint values, sampled at 25 Hz; smoother at 200 Hz; robot at 1 kHz. Returns t, G, CMD, Q arrays."""
    q0 = list(gello(0.0))
    sm = LiveSmoother(q0, max_vel, 0.0, vel_frac=0.75, margin=0.0, lead_scale=lead, lead_max=0.3)
    q, dq = np.array(q0), np.zeros(7)
    K, D, I = np.array(CTRL_K), np.array(CTRL_D), np.array(INERTIA)
    T, G, C, Q = [], [], [], []
    cmd = np.array(q0)
    next_s, next_o = 0.0, 0.0
    for k in range(int(t_end * 1000)):
        t = k / 1000.0
        if t >= next_s - 1e-9:
            if stall_after is None or t < stall_after:
                sm.sample(list(gello(t)), t)
            next_s += 0.04
        if t >= next_o - 1e-9:
            cmd = np.array(sm.step(t))
            next_o += 0.005
        tau = K * (cmd - q) - D * dq
        dq += tau / I * 0.001
        q += dq * 0.001
        T.append(t)
        G.append(gello(t))
        C.append(cmd.copy())
        Q.append(q.copy())
    return np.array(T), np.array(G), np.array(C), np.array(Q)


def main():
    q0 = np.array([0.0, -0.6, 0.0, -2.2, 0.0, 1.6, 0.8])
    v = np.array([0.6, 0.4, 0.6, 0.5, 1.0, 1.0, 1.5])            # constant GELLO speeds [rad/s]
    ramp = lambda t: q0 + v * min(t, 1.5)                          # noqa: E731 - move 1.5 s, then stop
    # 1) scale 0 = the old smoother, bit for bit
    a = LiveSmoother(list(q0), 2.0, 0.0, lead_scale=0.0)
    b = LiveSmoother(list(q0), 2.0, 0.0)
    same = True
    for i in range(50):
        t = i * 0.04
        a.sample(list(ramp(t)), t)
        b.sample(list(ramp(t)), t)
        for k in range(8):
            same &= a.step(t + k * 0.005) == b.step(t + k * 0.005)
    check('lead_scale 0 reproduces the old LiveSmoother exactly', same)
    # 2) steady lag during the constant-speed part (0.8 - 1.4 s): time the robot trails the GELLO
    T, G, C0, Q0 = run(ramp, 0.0, 3.0)
    _, _, C1, Q1 = run(ramp, 1.0, 3.0)
    win = (T > 0.8) & (T < 1.4)
    lag0 = np.mean(G[win] - Q0[win], axis=0) / v * 1000
    lag1 = np.mean(G[win] - Q1[win], axis=0) / v * 1000
    print('     lag without lead [ms]: ' + ' '.join(f'{x:.0f}' for x in lag0))
    print('     lag with lead    [ms]: ' + ' '.join(f'{x:.0f}' for x in lag1))
    check('without lead the model shows the controller lag (~D/K + smoothing)',
          np.all(lag0 > np.array(LEAD_S) * 1000 * 0.8), '')
    need = (np.array(LEAD_S) + 0.04) * v                  # lead the model needs at these speeds
    full = need <= np.array(LEAD_CAP)
    check('with lead: steady lag < 25 ms on every joint whose lead fits its torque cap',
          np.all(np.abs(lag1[full]) < 25), f'joints {np.where(full)[0] + 1}')
    check('with lead: lag at least halved on every joint', np.all(lag1 < lag0 * 0.5))
    # 3) abrupt stop at 1.5 s: overshoot of the ROBOT beyond the final GELLO pose stays small and settles
    final = ramp(10.0)
    after = T > 1.5
    over = np.max((Q1[after] - final) * np.sign(v), axis=0)
    check('stop: robot overshoot <= 0.06 rad on every joint', np.all(over <= 0.06),
          ' '.join(f'{x:.3f}' for x in over))
    err_end = np.max(np.abs(Q1[T > 2.5] - final), axis=0)
    err_end0 = np.max(np.abs(Q0[T > 2.5] - final), axis=0)
    check('stop: 1 s after the stop the robot is as close to the GELLO as without lead (+2 mrad)',
          np.all(err_end <= err_end0 + 0.002), 'lead ' + ' '.join(f'{x:.4f}' for x in err_end) +
          ' | no lead ' + ' '.join(f'{x:.4f}' for x in err_end0))
    # 4) command never further ahead of the GELLO than lead_max (+ one 40 ms segment of motion)
    ahead = np.max(np.abs(C1 - G), axis=0)
    check('command never more than its lead cap + one sample step ahead of the GELLO',
          np.all(ahead <= np.array(LEAD_CAP) + v * 0.045 + 1e-6), ' '.join(f'{x:.3f}' for x in ahead))
    # 5) speed cap still holds with the lead (the command changes every 5 ms)
    dc = np.max(np.abs(np.diff(C1[::5], axis=0)) / 0.005, axis=0)
    check('commanded speed never above 3.5 rad/s (cap applies after the lead)', np.all(dc <= 3.5 * 1.01),
          ' '.join(f'{x:.2f}' for x in dc))
    # 6) GELLO samples stall mid-motion: the lead is dropped, the command returns to the last real sample
    _, Gs, Cs, _ = run(ramp, 1.0, 2.0, stall_after=1.0)
    last = ramp(0.96)
    check('samples stall: lead removed, command back on the last GELLO sample', np.all(np.abs(Cs[-1] - last) < 1e-6),
          ' '.join(f'{x:.4f}' for x in Cs[-1] - last))
    print(f'===== {"PASS" if all(R) else "FAIL"} ({sum(R)}/{len(R)})')
    sys.exit(0 if all(R) else 1)


if __name__ == '__main__':
    main()
