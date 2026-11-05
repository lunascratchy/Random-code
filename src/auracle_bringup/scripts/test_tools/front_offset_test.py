#!/usr/bin/env python3
"""Start/stop PWM of every wheel, for the front offsets, minimum PWMs and kicks in config.h.

Run it YOURSELF in an SSH terminal on the Pi (it reads your key presses). ROS must be stopped.
Robot on a box, wheels in the air, LiPo connected.

    python3 ~/auracle_tools/front_offset_test.py          # all four wheels
    python3 ~/auracle_tools/front_offset_test.py LF       # only some: LF RF LB RB

For each wheel and direction the PWM ramps up slowly. Press ENTER the moment the wheel starts
turning steadily. Then it ramps down: press ENTER the moment it stops. Rear wheels also show
their encoder speed, so you can see the moment they move. Keys: ENTER = mark, s = skip this
one, q = quit (results so far are kept). Results: ~/auracle_tools/runs/front_offsets.json
"""
import json
import os
import select
import sys
import termios
import time
import tty

import serial

PORT, BAUD = '/dev/arduino', 500000
OUT = os.path.expanduser('~/auracle_tools/runs/front_offsets.json')
STEP, STEP_S, MAX = 2, 0.25, 255
# name, side command, slot (0 = rear, 1 = front), telemetry encoder index (None = no encoder)
WHEELS = [('LF', 'p', 1, None), ('RF', 'q', 1, None), ('LB', 'p', 0, 0), ('RB', 'q', 0, 1)]


class Nano:
    def __init__(self):
        self.p = serial.Serial(PORT, BAUD, timeout=0.02)
        end = time.time() + 6
        while time.time() < end:
            if self.p.readline().strip() == b'READY':
                break
        self.last = None   # (ms, l, r)
        self.speed = [0.0, 0.0]

    def send(self, side, rear, front):
        self.p.write(f'{side} {rear} {front}\n'.encode())

    def poll(self):
        while True:
            line = self.p.readline()
            if not line:
                return
            f = line.split()
            if len(f) == 10 and f[0] == b'e':
                ms, l, r = int(f[1]), int(f[2]), int(f[3])
                if self.last and ms > self.last[0]:
                    dt = (ms - self.last[0]) / 1000.0
                    self.speed = [0.7 * self.speed[0] + 0.3 * (l - self.last[1]) / dt,
                                  0.7 * self.speed[1] + 0.3 * (r - self.last[2]) / dt]
                self.last = (ms, l, r)

    def stop(self):
        self.p.write(b's\n')


def key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def ramp(nano, name, side, slot, enc, sign, direction):
    """Returns (start_pwm, stop_pwm), None for a skipped mark, or 'quit'."""
    result = []
    for phase in ('up', 'down'):
        what = 'STARTS turning' if phase == 'up' else 'STOPS'
        print(f'\n{name} {direction}: ramping {phase}. Press ENTER when it {what} (s = skip, q = quit)')
        pwm = 0 if phase == 'up' else min(MAX, (result[0] or 120) + 30)
        nxt = time.time()
        marked = None
        while True:
            nano.poll()
            now = time.time()
            if now >= nxt:
                pwm = pwm + STEP if phase == 'up' else pwm - STEP
                nxt = now + STEP_S
                if pwm > MAX or pwm < 0:
                    break
            val = sign * pwm
            nano.send(side, val if slot == 0 else 0, val if slot == 1 else 0)
            spd = f'  encoder {nano.speed[enc]:7.0f} ticks/s' if enc is not None else ''
            sys.stdout.write(f'\r  PWM {pwm:3d}{spd}   ')
            sys.stdout.flush()
            k = key()
            if k in ('\n', '\r'):
                marked = pwm
                break
            if k == 's':
                break
            if k == 'q':
                nano.send(side, 0, 0)
                return 'quit'
            time.sleep(0.03)
        print(f'\n  -> {what.lower()} at PWM {marked}' if marked is not None else '\n  -> skipped')
        result.append(marked)
        nano.send(side, 0, 0)
        time.sleep(1.0)   # let it stop before the next ramp
    return tuple(result)


def suggest(res):
    def g(name, d, i):
        v = res.get(f'{name}_{d}')
        return v[i] if v else None
    print('\n=== Results (PWM) ===')
    print(f"{'wheel':6} {'dir':4} {'starts':>7} {'stops':>6}")
    for k, v in res.items():
        n, d = k.split('_')
        print(f'{n:6} {d:4} {str(v[0]):>7} {str(v[1]):>6}')
    print('\n=== Suggested config.h values (check before applying) ===')
    for f in ('LF', 'RF'):
        stops = [x for x in (g(f, 'fwd', 1), g(f, 'rev', 1)) if x]
        if stops:
            print(f'{f}_MIN_PWM ~ {int(max(stops) * 1.1)}   (highest stop PWM + 10%: keeps it turning)')
    for r in ('LB', 'RB'):
        for d, tag in (('fwd', 'FWD'), ('rev', 'REV')):
            s = g(r, d, 0)
            if s:
                print(f'{r}_KICK_{tag} ~ {min(255, int(s * 1.15))}   (start PWM + 15%)')
    for f, r in (('LF', 'LB'), ('RF', 'RB')):
        for d in ('fwd', 'rev'):
            fs, rs = g(f, d, 0), g(r, d, 0)
            if fs and rs:
                print(f'{f} vs {r} {d}: front starts {fs - rs:+d} PWM from rear '
                      f'(ratio {fs / rs:.2f}: a first guess for {f}_TRIM_{d.upper()})')


def main():
    if not sys.stdin.isatty():
        print('Run this in an interactive terminal: it needs your key presses.')
        return
    res = {}
    if os.path.exists(OUT):
        res = json.load(open(OUT))
    nano = Nano()
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(__doc__)
        only = [w.upper() for w in sys.argv[1:]]
        for name, side, slot, enc in WHEELS:
            if only and name not in only:
                continue
            for sign, d in ((1, 'fwd'), (-1, 'rev')):
                r = ramp(nano, name, side, slot, enc, sign, d)
                if r == 'quit':
                    raise KeyboardInterrupt
                res[f'{name}_{d}'] = list(r)
                json.dump(res, open(OUT, 'w'), indent=1)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        nano.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        json.dump(res, open(OUT, 'w'), indent=1)
        suggest(res)
        print(f'\nSaved {OUT}')


if __name__ == '__main__':
    main()
