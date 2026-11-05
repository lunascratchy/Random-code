#!/usr/bin/env python3
"""Floor check, about 50 s, ROS stopped: does right-rear counting hold up under power, and how steady
are the spins at the speeds teleop really uses?

  1  rears at fixed PWM, fronts as in the firmware, forward then back, 3 times: LB vs RB counts and yaw
  2  closed-loop spins at 418 and 786 ticks/s, both ways: wheel speed swing, dropouts, yaw rate
"""
import json, os, statistics as st, time, serial

GYRO_LSB = 131.0
REAR, RF_F, RF_R, LF_F, LF_R = 100, 90, 140, 100, 115   # firmware offsets (config.h)
res = dict(time=time.strftime('%F %T'))
other = []

p = serial.Serial('/dev/arduino', 500000, timeout=0.02)
end = time.time() + 6
while time.time() < end:
    if p.readline().strip() == b'READY':
        break


def hold(cmd, secs):
    rows, t0, tc = [], time.time(), 0
    while time.time() - t0 < secs:
        if time.time() - tc > 0.05:
            p.write(cmd)
            tc = time.time()
        line = p.readline()
        f = line.split()
        if len(f) == 10 and f[0] == b'e':
            try:
                rows.append([int(x) for x in f[1:]])
            except ValueError:
                pass
        elif f and f[0] != b'e':
            other.append(line.decode(errors='replace').strip())
    return rows


rest = hold(b's\n', 2.0)
bias = st.mean(r[8] / GYRO_LSB for r in rest)
print(f"gyro bias {bias:+.3f} deg/s, {len(rest) / 2:.0f} lines/s", flush=True)


def fixed(sign):
    lf, rf = (LF_F, RF_F) if sign > 0 else (LF_R, RF_R)
    rows = hold(f'p {sign * REAR} {sign * lf}\nq {sign * REAR} {sign * rf}\n'.encode(), 1.5)
    hold(b'p 0 0\nq 0 0\n', 0.7)
    r = [x for x in rows if x[0] >= rows[0][0] + 500]
    dt = (r[-1][0] - r[0][0]) / 1000
    return (r[-1][1] - r[0][1]) / dt, (r[-1][2] - r[0][2]) / dt, st.mean(x[8] / GYRO_LSB - bias for x in r)


print(f"1  fixed PWM {REAR} (LB ticks/s, RB ticks/s, RB/LB, yaw deg/s)", flush=True)
res['fixed'] = []
for i in range(3):
    for name, sign in (('fwd', 1), ('rev', -1)):
        lb, rb, yaw = fixed(sign)
        res['fixed'].append(dict(dir=name, lb=round(lb), rb=round(rb), yaw=round(yaw, 1)))
        print(f"    {name}: LB {lb:6.0f}  RB {rb:6.0f}  ratio {abs(rb / lb) if lb else 0:.2f}  yaw {yaw:+5.1f}", flush=True)


def spin(target, sign):
    """sign +1 = left. Speeds are 100 ms averages, ignoring the first second."""
    l, r = -sign * target, sign * target
    rows = hold(f'v {l} {r}\n'.encode(), 5.0)
    hold(b'v 0 0\n', 1.0)
    rows = [x for x in rows if x[0] >= rows[0][0] + 1000]
    out = {}
    for name, i in (('L', 1), ('R', 2)):
        sp = []
        for a, b in zip(rows, rows[4:]):
            dt = (b[0] - a[0]) / 1000
            if dt > 0:
                sp.append(abs(b[i] - a[i]) / dt)
        out[name] = dict(mean=round(st.mean(sp)), sd=round(st.pstdev(sp)), low=sum(s < 0.25 * target for s in sp) * 100 // len(sp),
                         over=sum(s > 1.25 * target for s in sp) * 100 // len(sp))
    gz = [x[8] / GYRO_LSB - bias for x in rows]
    out['yaw'] = round(st.mean(gz), 1)
    out['yaw_sd'] = round(st.pstdev(gz), 1)
    return out


print("2  spins, 5 s each (mean/sd ticks/s, % of time below 25% and above 125% of target, yaw deg/s +- sd)", flush=True)
res['spins'] = []
for target in (418, 786):
    for name, sign in (('left', 1), ('right', -1)):
        o = spin(target, sign)
        res['spins'].append(dict(target=target, dir=name, **o))
        print(f"    {target} {name:5s}: L {o['L']['mean']:4d}/{o['L']['sd']:3d} low {o['L']['low']:2d}% over {o['L']['over']:2d}% | "
              f"R {o['R']['mean']:4d}/{o['R']['sd']:3d} low {o['R']['low']:2d}% over {o['R']['over']:2d}% | yaw {o['yaw']:+6.1f} +- {o['yaw_sd']:.1f}", flush=True)
p.write(b's\n')
p.close()
if other:
    print("Nano messages:", len(other), other[:5], flush=True)
res['other'] = other[:50]
name = os.path.expanduser(f"~/auracle_tools/runs/floor_check_{time.strftime('%m%d_%H%M')}.json")
json.dump(res, open(name, 'w'), indent=1)
print('saved', name)
