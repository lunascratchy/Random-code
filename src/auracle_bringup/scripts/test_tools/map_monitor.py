#!/usr/bin/env python3
"""Watch data health during live mapping. Prints a status line every 30 s and a line per problem;
the same goes to ~/auracle_tools/runs/map_monitor_<time>.log. Stop with Ctrl+C.
"""
import math, os, re, subprocess, time, rclpy
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

NS = '/auracle'
PERIOD = 30.0
GAPS = {'scan': 1.0, 'scan_filtered': 1.0, 'imu': 0.5, 'odom': 0.5, 'ekf': 0.5, 'map_tf': 1.0}
log = open(os.path.expanduser(f"~/auracle_tools/runs/map_monitor_{time.strftime('%m%d_%H%M')}.log"), 'w')


def say(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()


rclpy.init()
n = rclpy.create_node('map_monitor')
count = {k: 0 for k in ('scan', 'scan_filtered', 'imu', 'odom', 'ekf', 'map_tf', 'map', 'cmd')}
last = {}
valid = {'scan': 0, 'scan_filtered': 0}
last_warn = {}
count_seen = {'map_tf': False}
faults = dict(path=None, pos=0, total=0, window=0)
state = dict(cmd_v=0.0, cmd_w=0.0, odom_v=0.0, tf=None, tilt=0.0, gz_max=0.0, jumps=0, map_info='no map yet', dist=0.0, odom=None)


def warn(key, msg, every=10.0):
    if time.time() - last_warn.get(key, 0) >= every:
        last_warn[key] = time.time()
        say('WARN ' + msg)


def on_scan(name):
    def cb(m):
        count[name] += 1
        last[name] = time.time()
        valid[name] += sum(1 for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max)
    return cb


def on_imu(m):
    count['imu'] += 1
    last['imu'] = time.time()
    gz = abs(math.degrees(m.angular_velocity.z))
    state['gz_max'] = max(state['gz_max'], gz)
    if gz > 120:
        warn('gz', f"yaw rate spike {gz:.0f} deg/s: robot knocked or lifted?")
    a = m.linear_acceleration
    tilt = math.degrees(math.atan2(math.hypot(a.x, a.y), a.z))
    state['tilt'] = max(state['tilt'], tilt)
    if tilt > 35:
        warn('tilt', f"tilt {tilt:.0f} deg: or a jolt (single accelerometer sample)", 60)


def on_odom(m):
    count['odom'] += 1
    state['odom_v'] = max(state['odom_v'], abs(m.twist.twist.linear.x))
    last['odom'] = time.time()
    p = m.pose.pose.position
    if state['odom'] is not None:
        d = math.hypot(p.x - state['odom'][0], p.y - state['odom'][1])
        if d > 0.5:
            warn('odomjump', f"wheel odometry jumped {d:.2f} m: Nano or control restart?")
        state['dist'] += d
    state['odom'] = (p.x, p.y)


def on_ekf(m):
    count['ekf'] += 1
    last['ekf'] = time.time()


def on_tf(m):
    for t in m.transforms:
        if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
            count['map_tf'] += 1
            count_seen['map_tf'] = True
            last['map_tf'] = time.time()
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cur = (t.transform.translation.x, t.transform.translation.y, yaw)
            if state['tf'] is not None:
                d = math.hypot(cur[0] - state['tf'][0], cur[1] - state['tf'][1])
                a = abs(math.degrees(math.atan2(math.sin(cur[2] - state['tf'][2]), math.cos(cur[2] - state['tf'][2]))))
                if d > 0.15 or a > 8:
                    state['jumps'] += 1
                    warn('jump', f"SLAM correction {d:.2f} m / {a:.1f} deg (a loop closure, or scan matching slipped)", 2.0)
            state['tf'] = cur


def on_map(m):
    count['map'] += 1
    last['map'] = time.time()
    d = np.asarray(m.data)
    occ, free = int((d > 50).sum()), int((d == 0).sum())
    r = m.info.resolution
    state['map_info'] = f"map {m.info.width * r:.1f}x{m.info.height * r:.1f} m, free {free * r * r:.1f} m2, walls {occ}"


def on_cmd(m):
    count['cmd'] += 1
    state['cmd_v'] = max(state['cmd_v'], abs(m.linear.x))
    state['cmd_w'] = max(state['cmd_w'], abs(m.angular.z))


best = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)
latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(LaserScan, f'{NS}/scan', on_scan('scan'), best)
n.create_subscription(LaserScan, f'{NS}/scan_filtered', on_scan('scan_filtered'), best)
n.create_subscription(Imu, f'{NS}/imu_broadcaster/imu', on_imu, best)
n.create_subscription(Odometry, f'{NS}/diff_cont/odom', on_odom, best)
n.create_subscription(Odometry, f'{NS}/odometry/filtered', on_ekf, best)
n.create_subscription(TFMessage, f'{NS}/tf', on_tf, best)
n.create_subscription(OccupancyGrid, f'{NS}/map', on_map, latched)
n.create_subscription(Twist, f'{NS}/cmd_vel_joy', on_cmd, best)


def pc_offset():
    try:
        pc = open('/etc/chrony/sources.d/pc.sources').read().split()[1]
        out = subprocess.run(['chronyc', '-n', 'sources'], capture_output=True, text=True, timeout=1).stdout
        for line in out.splitlines():
            if pc in line:
                m = re.search(r'([+-]\d+)(ns|us|ms|s)\[', line)
                if m:
                    return int(m[1]) * {'ns': 1e-6, 'us': 1e-3, 'ms': 1.0, 's': 1000.0}[m[2]]
    except Exception:
        pass
    return None


def read_faults():
    """New 'Nano FAULT' lines in the newest ros2_control log."""
    d = os.path.expanduser('~/.ros/log')
    logs = [f for f in os.listdir(d) if f.startswith('ros2_control_node_')]
    if not logs:
        return
    path = os.path.join(d, max(logs, key=lambda f: os.path.getmtime(os.path.join(d, f))))
    if path != faults['path']:
        faults.update(path=path, pos=os.path.getsize(path) if faults['path'] is None else 0, total=0)
    with open(path, errors='replace') as f:
        f.seek(faults['pos'])
        text = f.read()
        faults['pos'] = f.tell()
    for line in text.splitlines():
        m = re.search(r'Nano FAULT on (\w) wheel: code \d (\w+).*speed (-?\d+) ticks/s, target (-?\d+)', line)
        if m:
            faults['total'] += 1
            faults['window'] += 1
            warn('fault', f"{m[1]} wheel {m[2]}: speed {m[3]}, target {m[4]} ticks/s", 5)


def pi_health():
    t = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000
    try:
        th = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=1).stdout.strip().split('=')[-1]
    except Exception:
        th = '?'
    return t, th


say("monitor started")
t0 = tick = time.time()
for k in GAPS:
    last[k] = t0
try:
    while True:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time()
        for k, g in GAPS.items():
            if k == 'map_tf' and not count_seen['map_tf']:
                continue
            if now - last[k] > g:
                warn('gap_' + k, f"{k}: no data for {now - last[k]:.1f} s")
        if now - tick >= PERIOD:
            dt = now - tick
            s = count['scan'] or 1
            removed = valid['scan'] / s - valid['scan_filtered'] / max(count['scan_filtered'], 1)
            off = pc_offset()
            read_faults()
            temp, thr = pi_health()
            say(f"scan {count['scan'] / dt:.1f} Hz ({valid['scan'] / s:.0f} pts, filter removed {removed:.1f}) | "
                f"imu {count['imu'] / dt:.0f} | odom {count['odom'] / dt:.0f} | ekf {count['ekf'] / dt:.0f} | "
                f"slam tf {count['map_tf'] / dt:.0f} | map msgs {count['map']} | teleop {count['cmd'] / dt:.0f} Hz | "
                f"stalls {faults['window']} (total {faults['total']}) | cmd max {state['cmd_v']:.2f} m/s, {state['cmd_w']:.2f} rad/s, odom max {state['odom_v']:.2f} m/s | "
                f"driven {state['dist']:.1f} m | {state['map_info']} | max jolt-tilt {state['tilt']:.0f} deg, yaw {state['gz_max']:.0f} deg/s | "
                f"SLAM jumps {state['jumps']} | PC clock {('%+.0f ms' % off) if off is not None else '?'} | Pi {temp:.0f}C, throttled {thr}")
            if off is not None and abs(off) > 100:
                warn('clock', f"PC clock is {off:+.0f} ms off the Pi", 600)
            if thr not in ('0x0', '?'):
                warn('thr', f"Pi power/heat flag {thr}", 120)
            if temp > 78:
                warn('temp', f"Pi at {temp:.0f} C", 120)
            for k in GAPS:                      # the checks above block this loop; don't count that as a data gap
                last[k] = max(last[k], time.time() - 0.2)
            for k in count:
                count[k] = 0
            valid['scan'] = valid['scan_filtered'] = 0
            state['tilt'] = state['gz_max'] = 0.0
            state['jumps'] = 0
            state['cmd_v'] = state['cmd_w'] = state['odom_v'] = 0.0
            faults['window'] = 0
            tick = now
except KeyboardInterrupt:
    say("monitor stopped")
