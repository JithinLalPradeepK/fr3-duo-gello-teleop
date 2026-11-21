#!/usr/bin/env python3
"""Long-press logic for the foot pedals (no ROS, no evdev: unit-testable with simulated time).

A command with hold > 0 only fires after its pedal has been held continuously for `hold` seconds; releasing earlier
cancels it. A command with hold == 0 (STOP) fires at once on key-down. After firing, the pedal must be released
before the same command can fire again (holding does not repeat the command).

Safety against a stuck / lost key-up: if the pedal delivers auto-repeat events while held (keyboard-style repeat, value 2),
they act as a heartbeat - when they stop for `repeat_gap` seconds without a key-up, the hold is cancelled.
If the pedal never repeats, the hold relies on the key-up event alone.
"""


class HoldLogic:
    def __init__(self, holds, repeat_gap=0.6, progress_every=0.1):
        self.holds = dict(holds)            # cmd -> seconds (0 = immediate)
        self.repeat_gap = repeat_gap
        self.progress_every = progress_every
        self.cur = None                     # active hold: dict(cmd, t0, last, repeats, fired, last_progress)

    def on_key(self, cmd, value, now):
        """value: 1 = key-down, 2 = auto-repeat, 0 = key-up. Returns a list of actions."""
        acts = []
        if value == 1:
            if self.cur and not self.cur['fired'] and self.cur['cmd'] != cmd:
                acts.append(('cancel', self.cur['cmd'], 'another pedal pressed'))
                self.cur = None
            hold = self.holds.get(cmd, 0.0)
            if hold <= 0.0:
                acts.append(('fire', cmd))
            elif not (self.cur and self.cur['cmd'] == cmd):
                self.cur = {'cmd': cmd, 't0': now, 'last': now, 'repeats': 0, 'fired': False, 'last_progress': -1.0}
                acts.append(('progress', cmd, 0.0))
        elif value == 2:
            if self.cur and self.cur['cmd'] == cmd:
                self.cur['last'] = now
                self.cur['repeats'] += 1
        elif value == 0:
            if self.cur and self.cur['cmd'] == cmd:
                if not self.cur['fired']:
                    acts.append(('cancel', cmd, 'released early'))
                acts.append(('done', cmd))
                self.cur = None
        return acts

    def tick(self, now):
        acts = []
        c = self.cur
        if not c:
            return acts
        if c['fired']:
            return acts
        if c['repeats'] > 0 and now - c['last'] > self.repeat_gap:
            acts.append(('cancel', c['cmd'], 'pedal repeat events stopped'))
            acts.append(('done', c['cmd']))
            self.cur = None
            return acts
        held = now - c['t0']
        hold = self.holds[c['cmd']]
        if held >= hold:
            c['fired'] = True
            acts.append(('progress', c['cmd'], 1.0))
            acts.append(('fire', c['cmd']))
        elif now - c['last_progress'] >= self.progress_every:
            c['last_progress'] = now
            acts.append(('progress', c['cmd'], held / hold))
        return acts

    def holding(self):
        """(cmd, progress 0..1, fired) of the active hold, or None."""
        c = self.cur
        return None if not c else (c['cmd'], 1.0 if c['fired'] else 0.0, c['fired'])
