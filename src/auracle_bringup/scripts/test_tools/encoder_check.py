#!/usr/bin/env python3
"""Bench test for the Auracle rear encoders, talking straight to the Nano.

Stop ROS first (the serial port can only have one user).

  python3 encoder_check.py drive 120 20    # both sides at PWM 120 for 20 s (wheels in the air!)
  python3 encoder_check.py drive -120 10   # same, backwards
  python3 encoder_check.py count           # no motors: shows counts while you turn wheels by hand

Prints the left and right speed (1x ticks/s) every 0.5 s. A side that reads 0
while its wheel is visibly spinning has lost its encoder signal.
"""
import sys
import time

import serial

PORT = '/dev/arduino'
BAUD = 500000


def open_nano():
    port = serial.Serial(PORT, BAUD, timeout=0.05)
    print(f'Opened {PORT}. Waiting for the Nano to boot (keep the robot still)...')
    deadline = time.time() + 6
    while time.time() < deadline:
        line = port.readline().decode(errors='replace').strip()
        if line == 'READY':
            print('Nano is READY.\n')
            return port
    print('No READY line seen, continuing anyway.\n')
    return port


def parse(line):
    parts = line.split()
    if len(parts) == 10 and parts[0] == 'e':
        try:
            return int(parts[1]), int(parts[2]), int(parts[3])
        except ValueError:   # a corrupted line: skip it
            return None
    return None


def run(mode, pwm=0, seconds=0.0):
    port = open_nano()
    start = None
    last_print = 0.0
    last_cmd = 0.0
    window = None  # (ms, left, right) at the start of the current 0.5 s window
    try:
        t0 = time.time()
        while True:
            now = time.time()
            if mode == 'drive':
                if now - t0 > seconds:
                    break
                # The Nano stops the motors if it hears nothing for 500 ms, so repeat.
                if now - last_cmd > 0.1:
                    # Ramp up over 2 s so the four motors don't all hit full
                    # start-up current at the same instant.
                    p = int(pwm * min(1.0, (now - t0) / 2.0))
                    port.write(f'o {p} {p}\n'.encode())
                    last_cmd = now
            line = port.readline().decode(errors='replace').strip()
            if line.startswith('F '):
                print(f'FAULT line from Nano: {line}')
            tel = parse(line)
            if tel is None:
                continue
            if start is None:
                start = tel
                window = tel
            if now - last_print >= 0.5:
                dt = (tel[0] - window[0]) / 1000.0
                if dt > 0:
                    ls = (tel[1] - window[1]) / dt
                    rs = (tel[2] - window[2]) / dt
                    flag = ''
                    if mode == 'drive' and now - t0 > 1.0:
                        if abs(ls) < 20:
                            flag += '   <-- LEFT reads ~0'
                        if abs(rs) < 20:
                            flag += '   <-- RIGHT reads ~0'
                    print(f'L {ls:7.0f} ticks/s  R {rs:7.0f} ticks/s   '
                          f'| total L {tel[1] - start[1]:7d}  R {tel[2] - start[2]:7d}{flag}')
                window = tel
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        port.write(b's\n')
        time.sleep(0.1)
        port.close()
        if start is not None and tel is not None:
            print(f'\nFinal counts since start: L {tel[1] - start[1]}  R {tel[2] - start[2]}')
        print('Sent stop. Done.')


if __name__ == '__main__':
    if len(sys.argv) >= 2 and sys.argv[1] == 'count':
        run('count')
    elif len(sys.argv) == 4 and sys.argv[1] == 'drive':
        run('drive', max(-255, min(255, int(sys.argv[2]))), float(sys.argv[3]))
    else:
        print(__doc__)
