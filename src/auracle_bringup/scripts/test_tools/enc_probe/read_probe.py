import serial, sys, time
p = serial.Serial('/dev/arduino', 115200, timeout=0.2)
end = time.time() + float(sys.argv[1])
while time.time() < end:
    l = p.readline().decode(errors='replace').strip()
    if l: print(time.strftime('%H:%M:%S'), l, flush=True)
