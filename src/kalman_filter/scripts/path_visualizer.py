#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import ReliabilityPolicy, QoSProfile
class PathVisualizer(Node):
    """Visualizes path topics for Rviz"""

    def __init__(self):
        super().__init__('path_visualizer')
        self.odom_path = Path()
        self.kf_path = Path()
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.kf_sub = self.create_subscription(PoseStamped, '/kf_global_frame', self.kf_odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.odom_path_pub = self.create_publisher(Path, '/path_viz_odom', 10)
        self.kf_path_pub = self.create_publisher(Path, '/path_viz_kf', 10)

    def odom_callback(self, msg: Odometry):
        """Create RViz message for visualizing /odom path"""
        self.odom_path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose.pose.position.z = 0.0
        self.odom_path.poses.append(pose)
        self.odom_path_pub.publish(self.odom_path)

    def kf_odom_callback(self, msg: PoseStamped):
        """Create RViz message for visualizing /kf_global_frame path"""
        self.kf_path.header = msg.header
        self.kf_path.poses.append(msg)
        self.kf_path_pub.publish(self.kf_path)

def main(args=None):
    rclpy.init(args=args)
    visualizer = PathVisualizer()
    try:
        while rclpy.ok():
            rclpy.spin_once(visualizer)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()