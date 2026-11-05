import serial, sys, time
pwm = int(sys.argv[1]) if len(sys.argv) > 1 else 120
p = serial.Serial('/dev/arduino', 500000, timeout=0.05)
t0 = time.time(); last = 0
while time.time() - t0 < 9:
    if time.time() - t0 > 3 and time.time() - last > 0.1:
        p.write(f'o {pwm} {pwm}\n'.encode()); last = time.time()
    l = p.readline().decode(errors='replace').strip()
    if l: print(f'{time.time()-t0:5.2f} {l}')
p.write(b's\n')
