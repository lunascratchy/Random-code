#!/usr/bin/env python3
"""Rear feedforward sweep (raw PWM, fronts off). Wheels in the air, ROS stopped.
Fits PWM = BASE + K * speed (1x ticks/s) per rear wheel and direction, for config.h LB/RB_FF_*.
"""
import json, os, statistics as st, time, serial
LEVELS = [40, 60, 80, 100, 130, 160, 190, 220, 250]
p = serial.Serial('/dev/arduino', 500000, timeout=0.02)
end = time.time() + 6
while time.time() < end:
    if p.readline().strip() == b'READY': break
res = {'fwd': [], 'rev': []}
for d, sign in (('fwd', 1), ('rev', -1)):
    for pwm in LEVELS:
        t0 = time.time(); samples = []; last = None
        while time.time() - t0 < 2.5:
            p.write(f'p {sign*pwm} 0\nq {sign*pwm} 0\n'.encode())
            t1 = time.time()
            while time.time() - t1 < 0.06:
                f = p.readline().split()
                if len(f) == 10 and f[0] == b'e':
                    try: cur = (int(f[1]), int(f[2]), int(f[3]))
                    except ValueError: continue
                    if last and cur[0] > last[0] and time.time() - t0 > 1.3:
                        dt = (cur[0] - last[0]) / 1000
                        samples.append(((cur[1]-last[1])/dt, (cur[2]-last[2])/dt))
                    last = cur
        l = st.mean(s[0] for s in samples) if samples else 0; r = st.mean(s[1] for s in samples) if samples else 0
        res[d].append((pwm, l, r))
        print(f"{d} PWM {pwm:3d}: LB {l:7.0f}  RB {r:7.0f} ticks/s", flush=True)
    p.write(b'p 0 0\nq 0 0\n'); time.sleep(1.5)
p.write(b's\n')
json.dump(res, open(os.path.expanduser('~/auracle_tools/runs/ff_sweep.json'), 'w'), indent=1)
def fit(pts):
    pts = [(s, w) for w, s in pts if abs(s) > 150]          # moving points only
    if len(pts) < 3: return None
    xs = [abs(s) for s, _ in pts]; ys = [w for _, w in pts]
    mx, my = st.mean(xs), st.mean(ys)
    k = sum((x-mx)*(y-my) for x, y in zip(xs, ys)) / sum((x-mx)**2 for x in xs)
    b = my - k*mx
    resid = max(abs(y - (b + k*x)) for x, y in zip(xs, ys))
    return b, k, resid
print("\nFits  PWM = BASE + K * |ticks/s|   (max residual in PWM)")
for d in ('fwd', 'rev'):
    for i, w in ((1, 'LB'), (2, 'RB')):
        f = fit([(pt[0], pt[i]) for pt in res[d]])
        print(f"  {w}_FF_{d.upper()}: " + (f"BASE {f[0]:6.1f}  K {f[1]:.4f}   (resid {f[2]:.1f})" if f else "not enough points"))
