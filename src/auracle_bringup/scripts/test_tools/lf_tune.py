#!/usr/bin/env python3
"""Tune the replaced left-front motor (no encoder) on the FLOOR, about 90 s. ROS stopped.
LF has no encoder, so its effect is read from the gyro and the rear encoders.

  1  rears at a fixed PWM, RF as in the firmware, LF = rear + offset, forward then back: LB speed and yaw
  2  current firmware: 0.5 m forward, 0.5 m back, spin 360 left, spin 360 right
"""
import json, os, statistics as st, subprocess, sys, time, serial

PORT, BAUD = '/dev/arduino', 500000
T = os.path.expanduser('~/auracle_tools')
GYRO_LSB = 131.0
REAR = 100                       # raw rear PWM in part 2, ~0.12 m/s
OFFSETS = [None, -20, 0, 20, 40]  # None = RF unpowered, the drag baseline
RF_FWD, RF_REV = REAR + 6, REAR   # RF as in the firmware (rear + RF_OFFSET)
res = dict(time=time.strftime('%F %T'), rear_pwm=REAR)


def open_port():
    p = serial.Serial(PORT, BAUD, timeout=0.02)
    end = time.time() + 6
    while time.time() < end:
        if p.readline().strip() == b'READY':
            break
    return p


def read(p):
    f = p.readline().split()
    if len(f) == 10 and f[0] == b'e':
        try:
            return [int(x) for x in f[1:]]
        except ValueError:
            pass
    return None


def hold(p, cmd, secs):
    """Send cmd until secs pass; return telemetry rows."""
    rows, t0, tc = [], time.time(), 0
    while time.time() - t0 < secs:
        if time.time() - tc > 0.05:
            p.write(cmd() if callable(cmd) else cmd)
            tc = time.time()
        r = read(p)
        if r:
            rows.append(r)
    return rows


def stats(rows, tail_from, bias):
    r = [x for x in rows if x[0] >= rows[0][0] + tail_from]
    dt = (r[-1][0] - r[0][0]) / 1000.0
    return dict(lb=round((r[-1][1] - r[0][1]) / dt), rb=round((r[-1][2] - r[0][2]) / dt),
                yaw=round(st.mean(x[8] / GYRO_LSB - bias for x in r), 1))


def drive(p, sign, off):
    lf = 0 if off is None else max(0, REAR + off)
    rf = RF_FWD if sign > 0 else RF_REV
    rows = hold(p, f'p {sign * REAR} {sign * lf}\nq {sign * REAR} {sign * rf}\n'.encode(), 1.4)
    hold(p, b'p 0 0\nq 0 0\n', 0.6)
    return stats(rows, 0.5, bias)


def sub(args):
    out = subprocess.run([sys.executable, '-u'] + args, capture_output=True, text=True, timeout=60)
    for line in (out.stdout + out.stderr).strip().splitlines():
        print('    ' + line, flush=True)
    return out.stdout


p = open_port()
try:
    rest = []
    t0 = time.time()
    while time.time() - t0 < 2.0:
        r = read(p)
        if r:
            rest.append(r)
    bias = st.mean(r[8] / GYRO_LSB for r in rest)
    res['gz_bias'] = round(bias, 3)

    print(f"1  rears PWM {REAR}, LF = rear + offset (ticks/s, yaw deg/s, + = left)", flush=True)
    res['offsets'] = {}
    for off in OFFSETS:
        f, b = drive(p, 1, off), drive(p, -1, off)
        res['offsets'][str(off)] = dict(fwd=f, rev=b)
        print(f"    offset {str(off):>5}: fwd LB {f['lb']:5d} RB {f['rb']:5d} yaw {f['yaw']:+6.1f} | "
              f"rev LB {b['lb']:5d} RB {b['rb']:5d} yaw {b['yaw']:+6.1f}", flush=True)
finally:
    p.write(b's\n')
    p.close()

print("2  current firmware: forward, back, spin left, spin right", flush=True)
for args in (['drive_test.py', '0.5'], ['drive_test.py', '-0.5'], ['spin_test.py', '360'], ['spin_test.py', '-360']):
    print(f"  {' '.join(args)}", flush=True)
    res.setdefault('firmware', {})[' '.join(args)] = sub([f'{T}/{args[0]}', args[1]])

name = f"{T}/runs/lf_tune_{time.strftime('%m%d_%H%M')}.json"
json.dump(res, open(name, 'w'), indent=1)
print('saved', name)
