#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState
import numpy as np

class KalmanFilterNode(Node):
    """ROS2 Action Server that plans path to a goal pose using A* and then follows the path using P-controller"""

    def __init__(self):
        super().__init__('kalman_filter')
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.encoder_sub = self.create_subscription(JointState, '/joint_states', self.encoder_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.odom_pub = self.create_publisher(Odometry, '/odom_kf', 10)
        self.odom_msg = Odometry()

    def imu_callback(self, msg: Imu):
        w = msg.angular_velocity.z # omega
        a_x = msg.linear_acceleration.x # linear acceleration in x direction
        self.get_logger().info(f"Angular Velocity: {w}")

    def encoder_callback(self, msg: JointState):
        left_wheel_speed = msg.velocity[0] # rad/s?
        right_wheel_speed = msg.velocity[1] # rad/s?
        left_wheel_pos = msg.velocity[0] # rad
        right_wheel_pos = msg.velocity[1] # rad

        # TODO: run kalman filter

    def publish_odom(self, x,theta):
        self.odom_msg.pose.pose.position.x = x
        self.odom_pub.publish(self.odom_msg)

def main(args=None):
    rclpy.init(args=args)
    a_star_action_server = KalmanFilterNode()
    while rclpy.ok():
        rclpy.spin_once(a_star_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()