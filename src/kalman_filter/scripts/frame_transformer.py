#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import ReliabilityPolicy, QoSProfile
from kalman_filter_interfaces.msg import RobotFrameState
import numpy as np
import math

class FrameTransformer(Node):
    """Converts between Robot Frame to global frame"""

    def __init__(self):
        super().__init__('frame_transformer')
        self.kf_global_pose = PoseStamped()
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.kf_correct_sub = self.create_subscription(RobotFrameState, '/kf_corrected', self.kf_correct_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.kf_global_pub = self.create_publisher(PoseStamped, '/kf_global_pose', 10)
        self.odom_robot_pub = self.create_publisher(RobotFrameState, '/odom_robot_frame', 10)

        # Conversion from Robot Frame to Global frame
        self.kf_pose_corrected_prev = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # x, x_dot, theta, omega
        self.global_pose_corrected = np.matrix([0.0, 0.0]).transpose() # x,y
        self.odom_global_pose_prev = np.matrix([0.0, 0.0, 0.0]).transpose() # x,y, theta
        self.odom_robotframe_pose = RobotFrameState()
        self.odom_robotframe_pose.x = 0.0

    def odom_callback(self, msg: Odometry):
        # convert odom globalFrame message to robot frame
        delta_x = msg.pose.pose.position.x - self.odom_global_pose_prev[0, 0]
        delta_y = msg.pose.pose.position.y - self.odom_global_pose_prev[1, 0]
        delta_x_robot_frame = np.sqrt(delta_x**2 + delta_y**2)
        # get theta from  msg.pose.pose.orientation quaternion
        quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
                )
        _, _, theta = self.euler_from_quaternion(quaternion)

        self.odom_robotframe_pose.header.frame_id = 'odom'
        self.odom_robotframe_pose.header.stamp = self.get_clock().now().to_msg()
        self.odom_robotframe_pose.x += delta_x_robot_frame
        self.odom_robotframe_pose.theta = theta
        self.odom_robot_pub.publish(self.odom_robotframe_pose)

        self.odom_global_pose_prev[0,0] = msg.pose.pose.position.x
        self.odom_global_pose_prev[1,0] = msg.pose.pose.position.y
        self.odom_global_pose_prev[2,0] = theta
    
    def kf_correct_callback(self, msg: RobotFrameState):
        theta = msg.theta
        delta_x = msg.x - self.kf_pose_corrected_prev[0, 0]
        self.global_pose_corrected[0,0] += delta_x*np.cos(theta)
        self.global_pose_corrected[1,0] += delta_x*np.sin(theta)
        self.publish_kf_global_frame()
        # Store last KF message
        self.kf_pose_corrected_prev[0, 0] = msg.x
        self.kf_pose_corrected_prev[1, 0] = msg.x_dot
        self.kf_pose_corrected_prev[2, 0] = msg.theta
        self.kf_pose_corrected_prev[3, 0] = msg.omega
        #TODO: do some mse between KF topic and odom topic to see how close we are to ground truth in another node
    
    def publish_kf_global_frame(self):
        self.kf_global_pose.header.frame_id = 'odom'
        self.kf_global_pose.header.stamp = self.get_clock().now().to_msg()
        self.kf_global_pose.pose.position.x = self.global_pose_corrected[0, 0]
        self.kf_global_pose.pose.position.y = self.global_pose_corrected[1, 0]
        self.kf_global_pub.publish(self.kf_global_pose)

    def euler_from_quaternion(self, quaternion):       
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        (x,y,z,w) = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)
    transformer = FrameTransformer()
    try:
        while rclpy.ok():
            rclpy.spin_once(transformer)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()