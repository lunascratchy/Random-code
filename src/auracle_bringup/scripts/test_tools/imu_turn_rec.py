import serial, time, json, sys
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60
p = serial.Serial('/dev/arduino', 500000, timeout=0.1)
t_end = time.time() + 6
while time.time() < t_end:
    if p.readline().strip() == b'READY': break
print('RECORDING', flush=True)
rows = []; t_end = time.time() + secs
while time.time() < t_end:
    f = p.readline().split()
    if len(f) == 10 and f[0] == b'e': rows.append([int(x) for x in f[1:]])
json.dump(rows, open('/home/auracle/auracle_tools/runs/imu_turn.json', 'w'))
print('SAVED', len(rows), flush=True)
