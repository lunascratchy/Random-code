#!/usr/bin/env python3
"""Wheel slip on this surface: wheel distance vs lidar. launch_robot running, Nav2/SLAM/teleop off.
Needs only ~1 m clear ahead and any walls or objects in lidar view (no particular wall).
Drives 0.8 m forward and back at 0.20 m/s, stopping to take scans, then finds the real movement
by matching the scans. Slip % = how much further the wheels say it went than the lidar does.

    python3 ~/auracle_tools/slip_test.py grass
"""
import json, math, os, sys, time, rclpy
import numpy as np
from scipy.ndimage import distance_transform_edt
from rclpy.qos import qos_profile_sensor_data as q
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

NS, DIST, SPEED = '/auracle', 0.8, 0.20
label = sys.argv[1] if len(sys.argv) > 1 else 'surface'
rclpy.init()
n = rclpy.create_node('slip_test')
pub = n.create_publisher(Twist, f'{NS}/cmd_vel_joy', 10)
state = dict(pos=None, scan=None, yaw=0.0, t=None)


def on_odom(m):
    p = m.pose.pose.position
    state['pos'] = (p.x, p.y)


def on_imu(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if state['t'] is not None and 0 < t - state['t'] < 0.2:
        state['yaw'] += math.degrees(m.angular_velocity.z) * (t - state['t'])
    state['t'] = t


n.create_subscription(Odometry, f'{NS}/diff_cont/odom', on_odom, q)
n.create_subscription(LaserScan, f'{NS}/scan', lambda m: state.update(scan=m), q)
n.create_subscription(Imu, f'{NS}/imu_broadcaster/imu', on_imu, q)


def spin(secs, v=None):
    end, last = time.time() + secs, 0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
        if v is not None and time.time() - last > 0.05:
            msg = Twist()
            msg.linear.x = v
            pub.publish(msg)
            last = time.time()


def snapshot():
    """Lidar points in metres, a few scans, robot stopped."""
    pts, seen, end = [], None, time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        s = state['scan']
        if s is None or s is seen:
            continue
        seen = s
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > 0.25) & (r < 6.0)
        pts.append(np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
    return np.vstack(pts)


def match(p0, p1, yaw_hint):
    """Best (dx, dy, yaw) putting p1 on p0, coarse to fine, using a distance map of p0."""
    res, lim = 0.02, 7.0
    size = int(2 * lim / res)
    grid = np.ones((size, size), bool)
    idx = ((p0 + lim) / res).astype(int)
    idx = idx[(idx >= 0).all(1) & (idx < size).all(1)]
    grid[idx[:, 0], idx[:, 1]] = False
    dist = distance_transform_edt(grid) * res

    def score(dx, dy, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        x = c * p1[:, 0] - s * p1[:, 1] + dx
        y = s * p1[:, 0] + c * p1[:, 1] + dy
        i, j = ((x + lim) / res).astype(int), ((y + lim) / res).astype(int)
        ok = (i >= 0) & (i < size) & (j >= 0) & (j < size)
        return np.minimum(dist[i[ok], j[ok]], 0.2).mean() if ok.sum() > 20 else 1.0

    best = (9.0, 0, 0, 0)
    hint = math.radians(yaw_hint)
    for yaw in hint + np.radians(np.arange(-10, 10.1, 1.0)):
        for dx in np.arange(-1.4, 1.41, 0.04):
            for dy in np.arange(-1.4, 1.41, 0.04):
                sc = score(dx, dy, yaw)
                if sc < best[0]:
                    best = (sc, dx, dy, yaw)
    for step, ang in ((0.01, 0.25), (0.004, 0.1)):
        _, bx, by, by2 = best
        for yaw in by2 + np.radians(np.arange(-4, 5) * ang):
            for dx in bx + np.arange(-4, 5) * step:
                for dy in by + np.arange(-4, 5) * step:
                    sc = score(dx, dy, yaw)
                    if sc < best[0]:
                        best = (sc, dx, dy, yaw)
    return best


def drive(sign):
    p0, a, y0 = snapshot(), state['pos'], state['yaw']
    t0, deadline = time.time(), time.time() + DIST / SPEED + 4
    while time.time() < deadline:
        spin(0.05, sign * SPEED)
        if math.hypot(state['pos'][0] - a[0], state['pos'][1] - a[1]) >= DIST:
            break
    secs = time.time() - t0
    spin(1.0, 0.0)
    p1, b, yaw = snapshot(), state['pos'], state['yaw'] - y0
    wheels = math.hypot(b[0] - a[0], b[1] - a[1])
    sc, dx, dy, myaw = match(p0, p1, yaw)
    real = math.hypot(dx, dy)
    return dict(dir='fwd' if sign > 0 else 'back', wheels=round(wheels, 3), lidar=round(real, 3), slip_pct=round((wheels - real) / wheels * 100, 1),
                gyro_yaw=round(yaw, 1), lidar_yaw=round(math.degrees(myaw), 1), fit_cm=round(sc * 100, 1), secs=round(secs, 1), points=len(p0))


results = []
try:
    t0 = time.time()
    while (state['pos'] is None or state['scan'] is None) and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.05)
    if state['pos'] is None or state['scan'] is None:
        raise SystemExit('no odometry or scan')
    for sign in (1, -1):
        r = drive(sign)
        results.append(r)
        print(label, r, flush=True)
finally:
    for _ in range(10):
        pub.publish(Twist())
        time.sleep(0.03)
json.dump(results, open(os.path.expanduser(f"~/auracle_tools/runs/slip_{label}_{time.strftime('%m%d_%H%M')}.json"), 'w'), indent=1)
