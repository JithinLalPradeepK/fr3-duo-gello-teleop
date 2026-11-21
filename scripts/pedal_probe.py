#!/usr/bin/env python3
"""Show the RAW events of the foot pedal (diagnostics; stop the demo first - the pedal node grabs the device).

  python3 ~/robot_demo/scripts/pedal_probe.py        # then press / hold each pedal; Ctrl-C to end

Prints every key event with a timestamp and, on key-up, how long the key was held. Use it to see whether a pedal
sends a held key (down ... up after you release) or only a quick pulse, and whether it auto-repeats.
"""
import select
import sys
import time

import evdev

DEVICE = '/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd'
try:
    dev = evdev.InputDevice(DEVICE)
except OSError as e:
    sys.exit(f'cannot open {DEVICE}: {e} (is the demo / pedal node running? it grabs the pedal)')
print(f'listening on {dev.name} - press and HOLD each pedal for about 5 s, release; Ctrl-C ends', flush=True)
t0, down, reps = time.monotonic(), {}, {}
try:
    while True:
        r, _, _ = select.select([dev.fd], [], [], 0.5)
        for ev in (dev.read() if r else []):
            if ev.type != evdev.ecodes.EV_KEY:
                continue
            t = time.monotonic() - t0
            name = evdev.ecodes.KEY.get(ev.code, ev.code)
            if ev.value == 1:
                down[ev.code], reps[ev.code] = t, 0
                print(f'{t:8.3f}  {name:8s} DOWN', flush=True)
            elif ev.value == 2:
                reps[ev.code] = reps.get(ev.code, 0) + 1
            elif ev.value == 0:
                held = t - down.pop(ev.code, t)
                print(f'{t:8.3f}  {name:8s} UP    held {held:.2f} s, {reps.pop(ev.code, 0)} auto-repeat events', flush=True)
except KeyboardInterrupt:
    print('\nend')
