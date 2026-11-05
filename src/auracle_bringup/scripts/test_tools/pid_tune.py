#!/usr/bin/env python3
"""Rear-wheel speed PI tuning: step tests with live gains (firmware 'g'/'h' commands). ROS must be stopped.

  python3 pid_tune.py                 # current config.h gains, one run
  python3 pid_tune.py --sweep         # grid of kp x ki around them, ranked
  python3 pid_tune.py --kp 0.03 --ki 0.15 --kpr 0.025 --kir 0.2

Each run: forward steps 0.50 -> 0.70 -> 0.55 rev/s, then reverse -0.60 rev/s, both sides together
(straight, so the firmware's arc/spin mixing passes the targets through unchanged). Per side and
step it measures rise time (10->90 %), overshoot, steady-state error and ripple over the last
second. Wheels in the air first; repeat on the floor before trusting the numbers.
Results: ~/auracle_tools/runs/pid_tune_<time>.json
"""
import argparse
import itertools
import json
import os
import statistics as st
import time

import serial

PORT, BAUD = '/dev/arduino', 500000
CPR = (985.0, 942.0)            # LB, RB ticks/rev (config.h)
STEPS = [(0.50, 3.0), (0.70, 3.0), (0.55, 3.0), (0.0, 1.5), (-0.60, 3.0), (0.0, 1.5)]   # rev/s, seconds
DEFAULT = dict(kp=0.045, ki=0.30, kpr=0.0375, kir=0.40)   # = config.h (2026-10-25)


def run(p, kp, ki, kpr, kir):
    p.write(f'g {kp} {ki}\n'.encode())
    p.write(f'h {kpr} {kir}\n'.encode())
    rows = []
    t0 = time.time()
    for target, secs in STEPS:
        l, r = target * CPR[0], target * CPR[1]
        ts = time.time()
        last = 0
        while time.time() - ts < secs:
            if time.time() - last > 0.05:
                p.write(f'v {l:.0f} {r:.0f}\n'.encode())
                last = time.time()
            f = p.readline().split()
            if len(f) == 10 and f[0] == b'e':
                rows.append((time.time() - t0, target, int(f[1]), int(f[2]), int(f[3])))
    p.write(b's\n')
    return rows


def metrics(rows):
    """Speed (rev/s) per side from count deltas, then per-step metrics."""
    sp = []
    for a, b in zip(rows, rows[1:]):
        dt = (b[2] - a[2]) / 1000.0
        if dt > 0:
            sp.append((b[0], b[1], (b[3] - a[3]) / dt / CPR[0], (b[4] - a[4]) / dt / CPR[1]))
    # 4-sample (100 ms) moving average: at 40 Hz one tick per sample is a big speed jump, and raw
    # peaks read as overshoot.
    sm = []
    for k in range(len(sp)):
        w = sp[max(0, k - 3):k + 1]
        sm.append((sp[k][0], sp[k][1], sum(x[2] for x in w) / len(w), sum(x[3] for x in w) / len(w)))
    sp = sm
    out = {'L': [], 'R': []}
    t_start, prev = 0.0, 0.0
    for target, secs in STEPS:
        seg = [s for s in sp if t_start <= s[0] < t_start + secs]
        t_start += secs
        if target == 0 or len(seg) < 10:
            prev = target
            continue
        for side, i in (('L', 2), ('R', 3)):
            v = [s[i] for s in seg]
            t = [s[0] - seg[0][0] for s in seg]
            span = target - prev
            def frac(x): return (x - prev) / span if span else 1.0
            t10 = next((tt for tt, x in zip(t, v) if frac(x) >= 0.1), None)
            t90 = next((tt for tt, x in zip(t, v) if frac(x) >= 0.9), None)
            peak = max(v) if span > 0 else min(v)
            over = max(0.0, (peak - target) / abs(span) * 100 if span > 0 else (target - peak) / abs(span) * 100)
            tail = [x for tt, x in zip(t, v) if tt > secs - 1.0]
            out[side].append(dict(target=target,
                                  rise_s=round(t90 - t10, 2) if t10 is not None and t90 is not None else None,
                                  overshoot_pct=round(over, 1),
                                  ss_err_pct=round((st.mean(tail) - target) / abs(target) * 100, 1),
                                  ripple_pct=round(st.pstdev(tail) / abs(target) * 100, 1)))
        prev = target
    return out


def cost(m, sides=('L', 'R')):
    c = 0.0
    for side, steps in m.items():
        if side not in sides:
            continue
        for s in steps:
            c += abs(s['ss_err_pct']) + 0.5 * s['overshoot_pct'] + 2 * s['ripple_pct'] \
                 + 10 * (s['rise_s'] if s['rise_s'] is not None else 3.0)
    return round(c, 1)


def show(label, m):
    print(f'\n{label}   cost {cost(m)}')
    print(f"  {'side':4} {'target':>7} {'rise s':>7} {'over %':>7} {'ss err %':>9} {'ripple %':>9}")
    for side, steps in m.items():
        for s in steps:
            print(f"  {side:4} {s['target']:>7.2f} {str(s['rise_s']):>7} {s['overshoot_pct']:>7} "
                  f"{s['ss_err_pct']:>9} {s['ripple_pct']:>9}")


def main():
    ap = argparse.ArgumentParser()
    for k, v in DEFAULT.items():
        ap.add_argument(f'--{k}', type=float, default=v)
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--kp-mult', type=float, nargs='+', default=[0.5, 1.0, 2.0])
    ap.add_argument('--ki-mult', type=float, nargs='+', default=[0.5, 1.0, 2.0])
    a = ap.parse_args()
    p = serial.Serial(PORT, BAUD, timeout=0.02)
    end = time.time() + 6
    while time.time() < end:
        if p.readline().strip() == b'READY':
            break
    results = []
    try:
        if a.sweep:
            grid = list(itertools.product(a.kp_mult, a.ki_mult))
            for i, (fp, fi) in enumerate(grid):
                g = dict(kp=a.kp * fp, ki=a.ki * fi, kpr=a.kpr * fp, kir=a.kir * fi)
                m = metrics(run(p, **g))
                results.append(dict(gains=g, metrics=m, cost=cost(m)))
                show(f'[{i + 1}/{len(grid)}] kp x{fp} ki x{fi}  {g}', m)
                time.sleep(1.0)
            for side, kk in (('L', ('kp', 'ki')), ('R', ('kpr', 'kir'))):
                b = min(results, key=lambda r: cost(r['metrics'], (side,)))
                print(f"BEST {side}: kp {b['gains'][kk[0]]:.4f}  ki {b['gains'][kk[1]]:.4f}  (cost {cost(b['metrics'], (side,))})")
            print('Put LB_KP/LB_KI and RB_KP/RB_KI in config.h only after a floor run agrees.')
        else:
            g = dict(kp=a.kp, ki=a.ki, kpr=a.kpr, kir=a.kir)
            m = metrics(run(p, **g))
            results.append(dict(gains=g, metrics=m, cost=cost(m)))
            show(str(g), m)
    finally:
        p.write(b's\n')
        out = os.path.expanduser(f'~/auracle_tools/runs/pid_tune_{time.strftime("%H%M%S")}.json')
        json.dump(results, open(out, 'w'), indent=1)
        print(f'\nSaved {out}')


if __name__ == '__main__':
    main()
