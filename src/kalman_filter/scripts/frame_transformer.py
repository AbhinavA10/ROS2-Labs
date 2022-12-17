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
    """Converts between Robot Frame and global frame"""

    # ---- Explanation on Reference Frames ------ 
    # The 'robot frame' tracks states of [x, x_dot, theta, omega]". The `x` in this frame is
    # the distance along the 'path'. `theta` is the orientation of the robot, with respect to the global frame
    # The 'global frame' tracks position of the robot as [x,y,theta]. `x` and `y` are global positions 
    # of the robot. `theta` is again the orientation of the robot, with respect to the global frame

    def __init__(self):
        super().__init__('frame_transformer')
        # Subscribe to ground truth and Kalman Filter state updates
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.kf_correct_sub = self.create_subscription(RobotFrameState, '/kf_corrected_robot_frame', self.kf_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Publish above states in the oppisite frame
        self.kf_global_pub = self.create_publisher(PoseStamped, '/kf_global_frame', 10)
        self.odom_robot_pub = self.create_publisher(RobotFrameState, '/odom_robot_frame', 10)
        
        self.kf_global_frame_pose_msg = PoseStamped()
        self.kf_corrected_robot_frame_state_prev = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # x, x_dot, theta, omega
        self.kf_global_frame_pose = np.matrix([0.0, 0.0]).transpose() # x,y
        self.odom_global_frame_pose_prev = np.matrix([0.0, 0.0, 0.0]).transpose() # x, y, theta
        self.odom_robot_frame_state = RobotFrameState()

    def odom_callback(self, msg: Odometry):
        """Convert odom message from Global Frame to Robot Frame, for use in MSE comparison"""
        quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
                )
        _, _, theta = self.euler_from_quaternion(quaternion)
        delta_x = msg.pose.pose.position.x - self.odom_global_frame_pose_prev[0, 0]
        delta_y = msg.pose.pose.position.y - self.odom_global_frame_pose_prev[1, 0]
        delta_x_robot_frame = np.sqrt(delta_x**2 + delta_y**2)
 
        self.odom_robot_frame_state.header.frame_id = 'odom' # Robot frame origin coincides with odom origin
        self.odom_robot_frame_state.header.stamp = self.get_clock().now().to_msg()
        self.odom_robot_frame_state.x += delta_x_robot_frame
        self.odom_robot_frame_state.theta = theta
        # Store last message
        self.odom_global_frame_pose_prev[0,0] = msg.pose.pose.position.x
        self.odom_global_frame_pose_prev[1,0] = msg.pose.pose.position.y
        self.odom_global_frame_pose_prev[2,0] = theta

        self.odom_robot_pub.publish(self.odom_robot_frame_state)
    
    def kf_callback(self, msg: RobotFrameState):
        """Convert Kalman Filter's corrected state from Robot Frame to Global frame, for use in RViz"""
        theta = msg.theta
        delta_x = msg.x - self.kf_corrected_robot_frame_state_prev[0, 0]
        self.kf_global_frame_pose[0,0] += delta_x*np.cos(theta)
        self.kf_global_frame_pose[1,0] += delta_x*np.sin(theta)
        # Store last message
        self.kf_corrected_robot_frame_state_prev[0, 0] = msg.x
        self.kf_corrected_robot_frame_state_prev[1, 0] = msg.x_dot
        self.kf_corrected_robot_frame_state_prev[2, 0] = msg.theta
        self.kf_corrected_robot_frame_state_prev[3, 0] = msg.omega

        self.publish_kf_global_frame()
    
    def publish_kf_global_frame(self):
        """Publish Kalman Filter's state in the global frame"""
        self.kf_global_frame_pose_msg.header.frame_id = 'odom' # Set origin as odom, to visualize in RViz
        self.kf_global_frame_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.kf_global_frame_pose_msg.pose.position.x = self.kf_global_frame_pose[0, 0]
        self.kf_global_frame_pose_msg.pose.position.y = self.kf_global_frame_pose[1, 0]
        self.kf_global_pub.publish(self.kf_global_frame_pose_msg)

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