#!/usr/bin/env python3
"""
course_driver.py

Drives the exact recorded course (see record_waypoints.py) the same way
every single Optuna trial, using ground-truth odom for feedback -
deliberately NOT the SLAM map, NOT Nav2. This isolates what you're
actually trying to measure: how good is the map slam_toolbox produces
with these parameters, not how well Nav2 planned around whatever
partial/broken map it saw mid-run.

Publishes on 'cmd_vel_tracker', which twist_mux.yaml already has wired
up at priority 50 (below the joystick's 100) - so a joystick e-stop still
overrides this if you're supervising a trial.

Exits with code 0 when the last waypoint is reached (so run_trial.py's
subprocess.wait(timeout=...) is all it needs to know the course is done),
or code 1 if it's still running when killed externally on timeout.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml


def yaw_from_quat(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CourseDriver(Node):
    def __init__(self):
        super().__init__('course_driver')
        self.declare_parameter('waypoints_file', '/tmp/course_waypoints.yaml')
        self.declare_parameter('odom_topic', '/model/my_bot/odometry_gt')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_lin_vel', 0.35)
        self.declare_parameter('max_ang_vel', 1.0)
        self.declare_parameter('heading_gate', 0.6)  # rad; above this, rotate in place first

        wp_file = self.get_parameter('waypoints_file').value
        odom_topic = self.get_parameter('odom_topic').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.max_lin = self.get_parameter('max_lin_vel').value
        self.max_ang = self.get_parameter('max_ang_vel').value
        self.heading_gate = self.get_parameter('heading_gate').value

        with open(wp_file) as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        if not self.waypoints:
            self.get_logger().error(f"No waypoints in {wp_file}")
            sys.exit(1)
        self.idx = 0

        self.pose = None  # (x, y, yaw)
        self.pub = self.create_publisher(Twist, 'cmd_vel_tracker', 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {wp_file}; driving course."
        )

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)

    def control_loop(self):
        if self.pose is None:
            return  # no odom yet

        if self.idx >= len(self.waypoints):
            self.pub.publish(Twist())  # stop
            self.get_logger().info("Course complete.")
            self.timer.cancel()
            rclpy.shutdown()
            sys.exit(0)
            return

        gx, gy = self.waypoints[self.idx]
        x, y, yaw = self.pose
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            self.idx += 1
            return

        target_heading = math.atan2(dy, dx)
        heading_err = math.atan2(
            math.sin(target_heading - yaw), math.cos(target_heading - yaw)
        )

        cmd = Twist()
        if abs(heading_err) > self.heading_gate:
            # Large heading error: rotate in place first, don't arc into obstacles.
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.0 * heading_err))
        else:
            cmd.linear.x = max(0.0, min(self.max_lin, 0.6 * dist))
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.5 * heading_err))

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = CourseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
