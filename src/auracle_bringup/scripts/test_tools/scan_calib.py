#!/usr/bin/env python3
"""Lidar / odometry calibration from one short drive. Robot running, SLAM not needed.

  record:   python3 scan_calib.py record 40      (records 40 s of /auracle/scan + odometry)
  analyse:  python3 scan_calib.py analyse

While recording: stand still 3 s, drive STRAIGHT forward ~2-3 s, stop, then SPIN LEFT about
one full turn, stop. Lots of walls around helps.

The analysis answers two questions using the lidar itself as the reference:
  1. Which way is "forward" in the scan? (the beam whose range shrinks while driving forward)
     0 deg = lidar mounting in the URDF is right; ~180 deg = it is mounted backwards, etc.
  2. How far did the robot really turn vs how far wheel odometry says it turned?
     The ratio corrects diff_cont's wheel_separation (yaw from wheels = (vr - vl) / separation).
"""
import math
import sys

import numpy as np

OUT = '/home/auracle/auracle_tools/runs/scan_calib.npz'


def record(seconds):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node('scan_calib_recorder')
    scans, odom, filt = [], [], []
    meta = {}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_scan(m):
        meta.setdefault('amin', m.angle_min)
        meta.setdefault('ainc', m.angle_increment)
        scans.append((now(), np.array(m.ranges, dtype=np.float32)))

    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(store):
        def cb(m):
            p = m.pose.pose
            store.append((now(), p.position.x, p.position.y, yaw(p.orientation),
                          m.twist.twist.linear.x, m.twist.twist.angular.z))
        return cb

    node.create_subscription(LaserScan, '/auracle/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/auracle/diff_cont/odom', on_odom(odom), 20)
    node.create_subscription(Odometry, '/auracle/odometry/filtered', on_odom(filt), 20)
    t_end = now() + seconds
    print(f'Recording {seconds} s ...', flush=True)
    try:
        while now() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('Stopped early, saving what was recorded.')
    n = min(len(s[1]) for s in scans) if scans else 0
    np.savez(OUT, scan_t=np.array([s[0] for s in scans]),
             scans=np.array([s[1][:n] for s in scans]),
             odom=np.array(odom), filt=np.array(filt),
             amin=meta.get('amin', 0.0), ainc=meta.get('ainc', 0.0))
    print(f'Saved {len(scans)} scans, {len(odom)} odom, {len(filt)} filtered -> {OUT}')
    node.destroy_node()
    rclpy.shutdown()


def best_shift(a, b, max_shift):
    """Beam shift k (b[i] ~ a[i - k]) that best matches two scans, by median abs difference."""
    best, best_k = None, 0
    for k in range(-max_shift, max_shift + 1):
        bb = np.roll(b, -k)
        ok = np.isfinite(a) & np.isfinite(bb) & (a > 0.15) & (bb > 0.15)
        if ok.sum() < 50:
            continue
        err = np.median(np.abs(a[ok] - bb[ok]))
        if best is None or err < best:
            best, best_k = err, k
    return best_k, best


def analyse():
    d = np.load(OUT)
    st, scans, odom = d['scan_t'], d['scans'], d['odom']
    ainc = float(d['ainc'])
    nbeam = scans.shape[1]
    scans = np.where(np.isfinite(scans) & (scans > 0.15) & (scans < 11.0), scans, np.nan)
    angles = float(d['amin']) + ainc * np.arange(nbeam)
    print(f'{len(scans)} scans, {nbeam} beams, {math.degrees(ainc):.2f} deg/beam')

    ot, ov, ow, oyaw = odom[:, 0], odom[:, 4], odom[:, 5], np.unwrap(odom[:, 3])
    v_at = np.interp(st, ot, ov)
    w_at = np.interp(st, ot, ow)

    # 1. Forward direction: straight segments (v > 0.05, |w| small), per-beam range rate.
    fwd = (v_at > 0.05) & (np.abs(w_at) < 0.1)
    idx = np.where(fwd)[0]
    if len(idx) > 6:
        rates = []
        for i in idx:
            if i + 1 < len(scans):
                dt = st[i + 1] - st[i]
                rates.append((scans[i + 1] - scans[i]) / dt)
        rates = np.array(rates)
        med = np.nanmedian(rates, axis=0)
        # Smooth over +-10 beams, then find where ranges shrink fastest.
        k = 10
        sm = np.array([np.nanmean(np.take(med, range(i - k, i + k + 1), mode='wrap')) for i in range(nbeam)])
        i_min = int(np.nanargmin(sm))
        v_mean = float(np.mean(v_at[idx]))
        print(f'\n1. FORWARD in the scan frame: {math.degrees(angles[i_min]):+.0f} deg '
              f'(range there shrinks at {sm[i_min]:.3f} m/s while odom says {v_mean:.3f} m/s)')
        print('   ~0 deg: lidar orientation in the URDF is right. ~180: mounted backwards (laser_joint yaw = pi).')
    else:
        print('\n1. Not enough straight driving recorded to check the forward direction.')

    # 2. Rotation, per spin. Left and right spins are kept apart: summed together they cancel.
    spin = np.abs(w_at) > 0.2
    totals = {+1: [0.0, 0.0], -1: [0.0, 0.0]}   # direction -> [|odom|, |lidar|]
    i = 0
    while i < len(st):
        if not spin[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(st) and spin[j + 1]:
            j += 1
        if j - i >= 8:
            lid, ok = 0.0, True
            for k in range(i, j):
                kk, err = best_shift(scans[k], scans[k + 1], max_shift=int(math.radians(30) / ainc))
                if err is None or err > 0.15:
                    ok = False
                    break
                lid += kk * ainc
            od = np.interp(st[j], ot, oyaw) - np.interp(st[i], ot, oyaw)
            true_turn = -lid   # a CCW turn moves walls clockwise in the scan (lidar not mirrored)
            if ok:
                print(f'   spin {st[i] - st[0]:6.1f}s: wheels {math.degrees(od):+5.0f} deg, lidar {math.degrees(true_turn):+5.0f} deg')
                if od * true_turn < 0:
                    print('   Opposite signs: the scan is MIRRORED vs the robot (lidar upside down or flip needed).')
                else:
                    t = totals[1 if od > 0 else -1]
                    t[0] += abs(od)
                    t[1] += abs(true_turn)
        i = j + 1
    all_o = totals[1][0] + totals[-1][0]
    all_l = totals[1][1] + totals[-1][1]
    if all_l > 0.5:
        for dname, dkey in (('left', 1), ('right', -1)):
            o, l = totals[dkey]
            if l > 0.2:
                print(f'   {dname} turns: wheels over-report x{o / l:.2f}')
        ratio = all_o / all_l
        print(f'\n2. TURNING overall: wheels over-report x{ratio:.2f}')
        print(f'   -> multiply diff_cont wheel_separation by {ratio:.2f} (and joystick.yaml wheel_separation)')
    else:
        print('\n2. Not enough clean spinning recorded to check turning.')

if __name__ == '__main__':
    if len(sys.argv) >= 2 and sys.argv[1] == 'record':
        record(float(sys.argv[2]) if len(sys.argv) > 2 else 40.0)
    elif len(sys.argv) >= 2 and sys.argv[1] == 'analyse':
        analyse()
    else:
        print(__doc__)
