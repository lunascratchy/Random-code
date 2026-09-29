#!/usr/bin/env python3
"""Drive a set distance or angle using wheel odometry, then calibrate.

Publishes to cmd_vel_joy (through twist_mux, like teleop) and watches
diff_cont/odom - the raw wheel odometry, not the EKF - so what you're
testing is the encoder -> ticks/rev -> wheel_radius chain.

  ros2 run auracle_bringup drive_test.py --distance 0.05
  ros2 run auracle_bringup drive_test.py --distance 1.0 --speed 0.15
  ros2 run auracle_bringup drive_test.py --angle 360

After it stops, measure what the robot really did and type it in; it
prints the corrected wheel_radius / wheel_separation for
config/my_controllers_real.yaml.

Use 1 m / 360 deg for calibration: over 5 cm, the coast after the stop
command (a few mm) is a big fraction of the move. 5 cm is a good
*check* once calibrated.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

RATE_HZ = 20.0
SETTLE_S = 1.5


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveTest(Node):

    def __init__(self, args):
        super().__init__('drive_test')
        self.args = args
        self.pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.create_subscription(Odometry, 'diff_cont/odom', self.on_odom, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

        self.start = None        # (x, y)
        self.pose = None         # (x, y)
        self.last_yaw = None
        self.yaw_travelled = 0.0  # unwrapped, rad
        self.phase = 'wait'      # wait -> drive -> settle -> done
        self.settle_ticks = 0

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        if self.last_yaw is not None:
            d = yaw - self.last_yaw
            self.yaw_travelled += math.atan2(math.sin(d), math.cos(d))
        self.last_yaw = yaw
        self.pose = (p.position.x, p.position.y)
        if self.start is None:
            self.start = self.pose
            self.yaw_travelled = 0.0

    def distance(self):
        return math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])

    def tick(self):
        cmd = Twist()
        if self.phase == 'wait':
            if self.start is not None:
                self.get_logger().info('Odometry received - driving.')
                self.phase = 'drive'
        elif self.phase == 'drive':
            if self.args.distance is not None:
                target = abs(self.args.distance)
                if self.distance() >= target:
                    self.phase = 'settle'
                else:
                    cmd.linear.x = math.copysign(self.args.speed, self.args.distance)
            else:
                target = math.radians(abs(self.args.angle))
                if abs(self.yaw_travelled) >= target:
                    self.phase = 'settle'
                else:
                    cmd.angular.z = math.copysign(self.args.turn_speed, self.args.angle)
        elif self.phase == 'settle':
            # Keep publishing zero so twist_mux doesn't hand over to anything
            # else while the robot coasts to a stop.
            self.settle_ticks += 1
            if self.settle_ticks >= SETTLE_S * RATE_HZ:
                self.phase = 'done'
        self.pub.publish(cmd)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    goal = parser.add_mutually_exclusive_group(required=True)
    goal.add_argument('--distance', type=float, help='metres, negative = reverse')
    goal.add_argument('--angle', type=float, help='degrees, positive = CCW (left)')
    parser.add_argument('--speed', type=float, default=0.1, help='m/s (default 0.1)')
    parser.add_argument('--turn-speed', type=float, default=0.8, help='rad/s (default 0.8)')
    parser.add_argument('--wheel-radius', type=float, default=0.0239,
                        help='value currently in my_controllers_real.yaml')
    parser.add_argument('--wheel-separation', type=float, default=0.2755,
                        help='value currently in my_controllers_real.yaml')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = DriveTest(args)
    try:
        while rclpy.ok() and node.phase != 'done':
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())

    if node.start is None:
        print('Never received diff_cont/odom - is launch_robot running?')
        node.destroy_node()
        rclpy.shutdown()
        return

    odom_dist = node.distance()
    odom_deg = math.degrees(node.yaw_travelled)
    node.destroy_node()
    rclpy.shutdown()

    if args.distance is not None:
        print(f'\nCommanded {abs(args.distance) * 100:.1f} cm, '
              f'wheel odom says {odom_dist * 100:.1f} cm (includes coast).')
        real = input('Measured real distance in cm (Enter to skip): ').strip()
        if real:
            real_m = float(real) / 100.0
            # odom = wheel_rad * r_cfg, real = wheel_rad * r_true
            new_r = args.wheel_radius * real_m / odom_dist
            print(f'wheel_radius: {args.wheel_radius:.5f} -> {new_r:.5f}')
    else:
        print(f'\nCommanded {abs(args.angle):.1f} deg, wheel odom says {abs(odom_deg):.1f} deg.')
        real = input('Measured real rotation in degrees (Enter to skip): ').strip()
        if real:
            # odom_yaw = (dr - dl) / sep_cfg, real_yaw = (dr - dl) / sep_eff
            new_sep = args.wheel_separation * abs(odom_deg) / float(real)
            print(f'wheel_separation: {args.wheel_separation:.4f} -> {new_sep:.4f}')


if __name__ == '__main__':
    main()
