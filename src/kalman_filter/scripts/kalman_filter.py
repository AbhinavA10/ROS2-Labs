#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState
import numpy as np
from kalman_filter_interfaces.msg import RobotFrameState

# in meters
WHEEL_RADIUS = 0.066/2
WHEEL_BASE = 0.16

class KalmanFilterNode(Node):
    """Kalman Filter to track Robot states in robot reference frame"""

    def __init__(self):
        super().__init__('kalman_filter')
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.encoder_sub = self.create_subscription(JointState, '/joint_states', self.encoder_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.state_predicted_pub = self.create_publisher(RobotFrameState, '/kf_predicted', 10)
        self.state_corrected_pub = self.create_publisher(RobotFrameState, '/kf_corrected', 10)
        self.state_msg = RobotFrameState()

        # Time step of analysis
        self.dt = 1.0/25.0
        self.xhat = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # mean(mu) estimate for the "first" step
        self.xhat_predicted = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # mean(mu) estimate for the "first" step
        self.P = np.identity(4)
        self.A = np.matrix([[1, self.dt, 0, 0], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, 0],
                            [0, 0, 0, 0]]) # this is the state space
        self.B = np.matrix([[1/2*self.dt**2, 0],
                            [self.dt, 0],
                            [0, self.dt],
                            [0, 1]])
        self.Qa = np.matrix([[0.000289, 0], 
                            [0, 4*10**(-8)]]) 
        self.Q = self.B*self.Qa*self.B.transpose()
        self.C = np.matrix([[0, 1/WHEEL_RADIUS, 0, -WHEEL_BASE/2], 
                            [0, 1/WHEEL_RADIUS, 0, +WHEEL_BASE/2],
                            [0, 0, 0, 1]])
        self.R = np.matrix([[0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]])
        self.u = np.matrix([0.0, 0.0]).transpose() # a_x, omega
        self.y = np.matrix([0.0, 0.0, 0.0]).transpose() #u_l, u_r, omega_calculated

        # Run Kalman Filter at fixed rate, slower than all subscribers
        self.timer = self.create_timer(self.dt, self.run_kalman_filter)

    def imu_callback(self, msg: Imu):
        w = msg.angular_velocity.z # omega
        a_x = msg.linear_acceleration.x # linear acceleration in x direction
        #TODO: apply imu_link to base_footprint transform
        self.u = np.matrix([a_x, w]).transpose()
        # self.get_logger().info(f"IMU Angular Velocity: {w}")

    def encoder_callback(self, msg: JointState):
        left_wheel_speed = msg.velocity[0] # rad/s?
        right_wheel_speed = msg.velocity[1] # rad/s?
        omega_calculated = WHEEL_RADIUS * (left_wheel_speed-right_wheel_speed)/WHEEL_BASE
        # self.get_logger().info(f"Encoder Angular Velocity: {omega_calculated}")
        self.y = np.matrix([left_wheel_speed, right_wheel_speed, omega_calculated]).transpose()

    def run_kalman_filter(self):
        # Prediction update
        self.xhat_predicted = self.A * self.xhat + self.B * self.u
        P_predict = self.A*self.P*self.A.transpose() + self.Q
        # Measurement Update and Kalman Gain
        K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)
        self.xhat = self.xhat_predicted + K * (self.y - self.C * self.xhat_predicted)
        self.P = (np.identity(4) - K * self.C) * P_predict
        # Publish predicted and corrected states for other nodes
        self.publish_states()

    def publish_states(self):
        self.state_msg.header.frame_id = 'odom'
        self.state_msg.header.stamp = self.get_clock().now().to_msg()
        # Prediction
        self.state_msg.x = self.xhat_predicted[0, 0]
        self.state_msg.x_dot = self.xhat_predicted[1, 0]
        self.state_msg.theta = self.xhat_predicted[2, 0]
        self.state_msg.omega = self.xhat_predicted[3, 0]
        self.state_predicted_pub.publish(self.state_msg)
        # Correction
        self.state_msg.x = self.xhat[0, 0]
        self.state_msg.x_dot = self.xhat[1, 0]
        self.state_msg.theta = self.xhat[2, 0]
        self.state_msg.omega = self.xhat[3, 0]
        self.state_corrected_pub.publish(self.state_msg)

def main(args=None):
    rclpy.init(args=args)
    kalman_filter = KalmanFilterNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(kalman_filter)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()