#!/usr/bin/env python3
"""All remaining floor tests in one run (~75 s). ROS stopped.
Start: robot's REAR in a corner of the 1 m square, facing the opposite corner.

  1  straight 0.8 m forward along the diagonal   (gyro drift)
  2  back 0.25 m                                 (ends near the middle)
  3  LF floor sweep: rears PWM 80, LF 140/170/200/230/255, 1 s fwd + 1 s back each
  4  spin left 360                               (wheel_separation)
  5  spin right 360
"""
import os, subprocess, sys, time, serial

T = os.path.expanduser('~/auracle_tools')
t_start = time.time()

def say(msg):
    print(f"[{time.time() - t_start:5.1f} s] {msg}", flush=True)

def run(args):
    out = subprocess.run([sys.executable, '-u'] + args, capture_output=True, text=True, timeout=60).stdout
    for line in out.strip().splitlines():
        print('         ' + line, flush=True)

def sweep():
    p = serial.Serial('/dev/arduino', 500000, timeout=0.02)
    end = time.time() + 6
    while time.time() < end:
        if p.readline().strip() == b'READY':
            break
    def hold(cmd, secs):
        t0 = tc = 0
        t0 = time.time()
        while time.time() - t0 < secs:
            if cmd and time.time() - tc > 0.05:
                p.write(cmd)
                tc = time.time()
            p.readline()
    for lf in (140, 170, 200, 230, 255):
        say(f"   LF {lf}: forward, then back")
        hold(f'p 80 {lf}\nq 80 80\n'.encode(), 1.0)
        hold(f'p -80 -{lf}\nq -80 -80\n'.encode(), 1.0)
        hold(b'p 0 0\nq 0 0\n', 1.0)
    p.write(b's\n')
    p.close()

say("1 straight 0.8 m forward");   run([f'{T}/drive_test.py', '0.8']);   time.sleep(1)
say("2 back 0.25 m");              run([f'{T}/drive_test.py', '-0.25']); time.sleep(1)
say("3 LF floor sweep");           sweep();                              time.sleep(1)
say("4 spin left 360");            run([f'{T}/spin_test.py', '360']);    time.sleep(1)
say("5 spin right 360");           run([f'{T}/spin_test.py', '-360'])
say("done")
