#!/usr/bin/env python3
"""Spin-in-place test: turn until the gyro reads the target angle, then compare with the rear encoders.
Effective track width (diff_drive wheel_separation) = (right distance - left distance) / angle in rad.
ROS stopped, robot on the floor with room to spin.

    python3 ~/auracle_tools/spin_test.py 360      # left (counter-clockwise)
    python3 ~/auracle_tools/spin_test.py -360     # right
"""
import json, math, os, sys, time, serial

PORT, BAUD = '/dev/arduino', 500000
CPR = (985.0, 990.0)            # LB, RB ticks per wheel turn (config.h)
WHEEL_R = 0.0375                # m
WHEEL_SPEED = 0.10              # m/s at each side, opposite directions
GYRO_LSB = 131.0                # LSB per deg/s

target = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
sign = 1 if target >= 0 else -1
tpm = [c / (2 * math.pi * WHEEL_R) for c in CPR]

p = serial.Serial(PORT, BAUD, timeout=0.02)
end = time.time() + 6
while time.time() < end:
    if p.readline().strip() == b'READY':
        break

def read():
    f = p.readline().split()
    if len(f) == 10 and f[0] == b'e':
        try:
            return [int(x) for x in f[1:]]
        except ValueError:
            pass
    return None

bias = []
t0 = time.time()
while time.time() - t0 < 2.0:
    r = read()
    if r:
        bias.append(r[8] / GYRO_LSB)
gz_bias = sum(bias) / len(bias)

start = last = None
heading = 0.0
deadline = time.time() + 40
cmd_l, cmd_r = -sign * WHEEL_SPEED * tpm[0], sign * WHEEL_SPEED * tpm[1]
try:
    while time.time() < deadline:
        r = read()
        if not r:
            continue
        if start is None:
            start = last = r
            continue
        dt = (r[0] - last[0]) / 1000.0
        if dt > 0:
            heading += (r[8] / GYRO_LSB - gz_bias) * dt
        last = r
        remaining = abs(target) - sign * heading
        if remaining <= 2.0:          # stop a little early; it coasts the rest
            break
        travel = abs(r[1] - start[1]) / tpm[0] + abs(r[2] - start[2]) / tpm[1]
        if travel > 8.0 * abs(target) / 360:   # allows an effective track up to ~1.27 m; more means gyro or traction trouble
            print("STOPPED: wheels travelled too far for the gyro angle")
            break
        k = 1.0 if remaining > 45 else max(0.5, remaining / 45)
        p.write(f'v {cmd_l * k:.0f} {cmd_r * k:.0f}\n'.encode())
    p.write(b'v 0 0\n')
    t0 = time.time()
    while time.time() - t0 < 1.0:
        r = read()
        if r:
            dt = (r[0] - last[0]) / 1000.0
            if dt > 0:
                heading += (r[8] / GYRO_LSB - gz_bias) * dt
            last = r
finally:
    p.write(b's\n')

dl, dr = (last[1] - start[1]) / tpm[0], (last[2] - start[2]) / tpm[1]
sep = (dr - dl) / math.radians(heading) if heading else float('nan')
res = dict(asked=target, heading_deg=round(heading, 1), enc_left=round(dl, 4), enc_right=round(dr, 4),
           wheel_separation=round(sep, 4), time=time.strftime('%H:%M:%S'))
print(f"gyro turned {heading:+.1f} deg | encoders: left {dl:+.3f}  right {dr:+.3f} m")
print(f"effective wheel_separation = {sep:.3f} m")
with open(os.path.expanduser('~/auracle_tools/runs/spin_tests.jsonl'), 'a') as f:
    f.write(json.dumps(res) + '\n')
