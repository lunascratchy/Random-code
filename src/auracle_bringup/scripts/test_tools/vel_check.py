#!/usr/bin/env python3
"""Closed-loop check: sends 'v L R' (1x ticks/s) like the Pi does and prints measured speed.
  python3 vel_check.py   (wheels in the air)"""
import serial, time
STEPS = [("forward", 600, 574, 6), ("stop", 0, 0, 2), ("reverse", -700, -670, 5),
         ("stop", 0, 0, 2), ("spin left", -500, 480, 5), ("stop", 0, 0, 2)]
p = serial.Serial('/dev/arduino', 500000, timeout=0.05)
t_end = time.time() + 6
while time.time() < t_end:
    if p.readline().strip() == b'READY': break
prev = None
for name, l, r, secs in STEPS:
    t0 = time.time(); last_cmd = 0; last_print = 0; win = None
    while time.time() - t0 < secs:
        if time.time() - last_cmd > 0.05:
            p.write(f'v {l} {r}\n'.encode()); last_cmd = time.time()
        line = p.readline().decode(errors='replace').strip()
        if line.startswith('F '): print('  FAULT', line)
        parts = line.split()
        if len(parts) != 10 or parts[0] != 'e': continue
        ms, le, re_ = int(parts[1]), int(parts[2]), int(parts[3])
        if win is None: win = (ms, le, re_)
        if time.time() - last_print > 1.0 and ms > win[0]:
            dt = (ms - win[0]) / 1000
            print(f'{name:10s} cmd L{l:5d} R{r:5d} | L {(le-win[1])/dt:6.0f}  R {(re_-win[2])/dt:6.0f} ticks/s')
            win = (ms, le, re_); last_print = time.time()
p.write(b's\n'); p.close()
