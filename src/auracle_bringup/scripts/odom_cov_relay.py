#!/usr/bin/env python3
"""odom_cov_relay: give rf2o's laser odometry a real covariance so the EKF can weigh it.

rf2o_laser_odometry publishes nav_msgs/Odometry with all-zero covariances. robot_localization
treats that as "perfectly certain", so a bad scan match (long plain wall, lidar lockup) would
be believed over everything else. This node copies the message and fills the diagonal.

  in : odom_rf2o_raw   (nav_msgs/Odometry, from rf2o)
  out: odom_rf2o       (same, with covariance)  -> fused by ekf_lidar_odom.yaml as odom1

If the scan matcher stops publishing, nothing is relayed and the EKF simply coasts on the IMU.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomCovRelay(Node):
    def __init__(self):
        super().__init__('odom_cov_relay')
        # Variance on the diagonal of the 6x6 (x, y, z, roll, pitch, yaw) twist / pose blocks.
        self.declare_parameter('twist_vx_var', 0.01)    # (0.1 m/s)^2: rf2o is good to ~cm/s
        self.declare_parameter('twist_vyaw_var', 0.02)
        self.declare_parameter('pose_var', 0.05)        # pose is not fused; kept for RViz
        self.vx = float(self.get_parameter('twist_vx_var').value)
        self.vyaw = float(self.get_parameter('twist_vyaw_var').value)
        self.pv = float(self.get_parameter('pose_var').value)
        self.pub = self.create_publisher(Odometry, 'odom_rf2o', 10)
        self.create_subscription(Odometry, 'odom_rf2o_raw', self.on_odom, 10)

    def on_odom(self, m: Odometry):
        pose = [0.0] * 36
        twist = [0.0] * 36
        for i in range(6):
            pose[i * 6 + i] = self.pv if i in (0, 1, 5) else 1e6
            twist[i * 6 + i] = 1e6
        twist[0] = self.vx          # vx
        twist[7] = self.vx          # vy (skid-steer: should be ~0)
        twist[35] = self.vyaw       # vyaw
        m.pose.covariance = pose
        m.twist.covariance = twist
        self.pub.publish(m)


def main():
    rclpy.init()
    node = OdomCovRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()