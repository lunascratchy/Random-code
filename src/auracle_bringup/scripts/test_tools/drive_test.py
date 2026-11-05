#!/usr/bin/env python3
"""Floor drive test: drive a set distance by the rear encoders, report heading drift from the gyro.
ROS stopped, robot on the floor. Negative distance drives backward (use it to return to the start).

    python3 ~/auracle_tools/drive_test.py 0.8          # forward 0.8 m
    python3 ~/auracle_tools/drive_test.py -0.8         # back to the start

Measure the real distance moved, then: new wheel_radius = WHEEL_R * measured / encoder distance.
"""
import json, math, os, sys, time, serial

PORT, BAUD = '/dev/arduino', 500000
CPR = (985.0, 990.0)            # LB, RB ticks per wheel turn (config.h)
WHEEL_R = 0.0375                # m, my_controllers_real.yaml
SPEED, MIN_SPEED, ACCEL = 0.12, 0.04, 0.25   # m/s, m/s, m/s^2
GYRO_LSB = 131.0                # LSB per deg/s

dist = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
sign = 1 if dist >= 0 else -1
tpm = [c / (2 * math.pi * WHEEL_R) for c in CPR]   # ticks per metre

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

# Gyro z bias, robot still
bias = []
t0 = time.time()
while time.time() - t0 < 2.0:
    r = read()
    if r:
        bias.append(r[8] / GYRO_LSB)
gz_bias = sum(bias) / len(bias)

start = last = None
heading = 0.0
v_cmd = 0.0
t_last_cmd = time.time()
deadline = time.time() + abs(dist) / MIN_SPEED + 5
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
        dl, dr = (r[1] - start[1]) / tpm[0], (r[2] - start[2]) / tpm[1]
        done = sign * (dl + dr) / 2
        remaining = abs(dist) - done
        if remaining <= 0:
            break
        now = time.time()
        v_cmd = min(SPEED, v_cmd + ACCEL * (now - t_last_cmd), max(MIN_SPEED, math.sqrt(2 * ACCEL * remaining)))
        t_last_cmd = now
        p.write(f'v {sign * v_cmd * tpm[0]:.0f} {sign * v_cmd * tpm[1]:.0f}\n'.encode())
    p.write(b'v 0 0\n')
    t0 = time.time()                                   # let it settle, keep integrating
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
res = dict(asked=dist, enc_left=round(dl, 4), enc_right=round(dr, 4), enc_mean=round((dl + dr) / 2, 4),
           heading_deg=round(heading, 1), gz_bias=round(gz_bias, 3), time=time.strftime('%H:%M:%S'))
print(f"asked {dist:+.3f} m | encoders: left {dl:+.3f}  right {dr:+.3f}  mean {(dl + dr) / 2:+.3f} m")
print(f"heading change (gyro) {heading:+.1f} deg   (+ = turned left)")
log = os.path.expanduser('~/auracle_tools/runs/drive_tests.jsonl')
with open(log, 'a') as f:
    f.write(json.dumps(res) + '\n')
