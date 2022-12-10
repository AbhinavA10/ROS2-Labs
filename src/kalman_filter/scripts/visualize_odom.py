#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import ReliabilityPolicy, QoSProfile

class PathVisualizer(Node):
    """Visualizes odom topic for Rviz"""

    def __init__(self):
        super().__init__('path_visualizer')
        self.path = Path()
        self.kf_path = Path()
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.odom_sub = self.create_subscription(Odometry, '/kf_odom', self.kf_odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.path_pub_odom = self.create_publisher(Path, '/path_viz_odom', 10)
        self.path_pub_kf = self.create_publisher(Path, '/path_viz_kf', 10)

    def odom_callback(self, msg: Odometry):
        # Create RViz message for visualizing path
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub_odom.publish(self.path)

    def kf_odom_callback(self, msg: Odometry):
        # Create RViz message for visualizing path
        self.kf_path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.kf_path.poses.append(pose)
        self.path_pub_kf.publish(self.kf_path)
        #TODO: do some mse between KF topic and odom topic to see how close we are to ground truth

def main(args=None):
    rclpy.init(args=args)
    visualizer = PathVisualizer()
    while rclpy.ok():
        rclpy.spin_once(visualizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()