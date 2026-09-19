#!/usr/bin/env python3
"""
record_waypoints.py

I don't know the real-world coordinates of your gamefield's ramps, cone
zone, barrier, etc. - only you know where those are on the actual course
geometry. So instead of guessing waypoints, this node lets you drive the
full course ONCE by joystick/teleop in Gazebo and records the path you
drove as a list of (x, y) waypoints in the ground-truth odom frame.

That recorded path becomes the fixed, repeatable course that
course_driver.py replays identically on every Optuna trial - so you're
comparing SLAM parameters against each other on the same path, not
partly measuring path variance.

Usage:
    1. Launch your sim + joystick as normal:
         ros2 launch auracle_bringup launch_sim.launch.py
    2. In another terminal:
         ros2 run auracle_slam_tuning record_waypoints \
             --ros-args -p output_file:=/path/to/course_waypoints.yaml
    3. Drive the ENTIRE course with the joystick: screen stop -> dock stop
       -> ramp -> dynamic obstacle zone -> barrier/ramp choice -> offload.
    4. Ctrl+C this node when you reach the offload zone. It writes the
       waypoint file on shutdown.

Notes:
    - Subscribes to the ground-truth odom topic (see gt_odom.xacro), NOT
      the noisy EKF odom, so the recorded path is not itself corrupted by
      whatever odometry noise you're trying to tune around.
    - Points are appended only every `min_spacing` meters of travel, so
      the file stays small and course_driver.py isn't fighting jittery
      near-duplicate points.
    - This only needs to be done ONCE per course layout. If the gamefield
      geometry changes, re-record.
"""
import signal
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import yaml


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('output_file', '/tmp/course_waypoints.yaml')
        self.declare_parameter('odom_topic', '/model/my_bot/odometry_gt')
        self.declare_parameter('min_spacing', 0.25)  # meters between recorded points

        self.output_file = self.get_parameter('output_file').value
        odom_topic = self.get_parameter('odom_topic').value
        self.min_spacing = self.get_parameter('min_spacing').value

        self.waypoints = []
        self.last_xy = None

        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.get_logger().info(
            f"Recording waypoints from '{odom_topic}' to '{self.output_file}'. "
            f"Drive the full course now; Ctrl+C when done."
        )

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is None:
            self.waypoints.append([round(x, 3), round(y, 3)])
            self.last_xy = (x, y)
            return
        dx = x - self.last_xy[0]
        dy = y - self.last_xy[1]
        if (dx * dx + dy * dy) ** 0.5 >= self.min_spacing:
            self.waypoints.append([round(x, 3), round(y, 3)])
            self.last_xy = (x, y)

    def save(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints recorded - nothing written.")
            return
        with open(self.output_file, 'w') as f:
            yaml.safe_dump({'frame': 'ground_truth_odom', 'waypoints': self.waypoints},
                            f, default_flow_style=False)
        self.get_logger().info(
            f"Wrote {len(self.waypoints)} waypoints to {self.output_file}"
        )


def main():
    rclpy.init()
    node = WaypointRecorder()

    def handle_sigint(signum, frame):
        node.save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
