#!/usr/bin/env python3
"""
gt_trajectory_logger.py

Logs two trajectories in parallel, in TUM format (timestamp tx ty tz qx qy
qz qw), so evo can compute APE/RPE between them at the end of a trial:

  - ground truth: read directly off the ground-truth odom topic (gt_odom.xacro)
  - estimate:     the map->base_footprint TF that slam_toolbox is publishing,
                   sampled on a timer so both trajectories are regularly spaced

Files are only flushed to disk on shutdown (SIGINT), matching how
run_trial.py will stop this node once the course is finished.
"""
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
import tf2_ros


class GtTrajectoryLogger(Node):
    def __init__(self):
        super().__init__('gt_trajectory_logger')
        self.declare_parameter('gt_odom_topic', '/model/my_bot/odometry_gt')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sample_hz', 10.0)
        self.declare_parameter('gt_out', '/tmp/traj_gt.tum')
        self.declare_parameter('est_out', '/tmp/traj_est.tum')

        self.gt_out_path = self.get_parameter('gt_out').value
        self.est_out_path = self.get_parameter('est_out').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.gt_lines = []
        self.est_lines = []

        gt_topic = self.get_parameter('gt_odom_topic').value
        self.sub = self.create_subscription(Odometry, gt_topic, self.gt_cb, 20)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        hz = self.get_parameter('sample_hz').value
        self.timer = self.create_timer(1.0 / hz, self.sample_estimate)

        self.get_logger().info(
            f"Logging ground truth from '{gt_topic}' and estimate from "
            f"TF {self.map_frame}->{self.base_frame} at {hz} Hz."
        )

    def gt_cb(self, msg: Odometry):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.gt_lines.append(
            f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        )

    def sample_estimate(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return  # slam_toolbox hasn't published this transform yet
        t = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        tr, rot = tf.transform.translation, tf.transform.rotation
        self.est_lines.append(
            f"{t:.6f} {tr.x:.6f} {tr.y:.6f} {tr.z:.6f} "
            f"{rot.x:.6f} {rot.y:.6f} {rot.z:.6f} {rot.w:.6f}\n"
        )

    def save(self):
        with open(self.gt_out_path, 'w') as f:
            f.writelines(self.gt_lines)
        with open(self.est_out_path, 'w') as f:
            f.writelines(self.est_lines)
        self.get_logger().info(
            f"Wrote {len(self.gt_lines)} GT poses -> {self.gt_out_path}, "
            f"{len(self.est_lines)} estimated poses -> {self.est_out_path}"
        )


def main():
    rclpy.init()
    node = GtTrajectoryLogger()

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
