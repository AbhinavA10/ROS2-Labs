#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState

from kalman_filter_interfaces.msg import RobotFrameState # Custom message type

# Parameters from Turtlebot3 datasheet, in meters
# https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
WHEEL_RADIUS = 0.066/2
WHEEL_BASE = 0.16

class KalmanFilterNode(Node):
    """Kalman Filter to track Robot states in robot reference frame"""

    def __init__(self):
        super().__init__('kalman_filter')
        # Subscribers for receiving IMU input and Encoder velocity measurements
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.encoder_sub = self.create_subscription(JointState, '/joint_states', self.encoder_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Publishers for notifying other nodes on tracked robot position, in the robot reference frame
        self.state_predicted_pub = self.create_publisher(RobotFrameState, '/kf_predicted_robot_frame', 10)
        self.state_corrected_pub = self.create_publisher(RobotFrameState, '/kf_corrected_robot_frame', 10)
        self.state_msg = RobotFrameState() 
        
        self.dt = 1.0/25.0 # Time step of Kalman Filter
        self.xhat = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # mean(mu) estimate for the "first" step
        self.xhat_predicted = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # mean(mu) estimate for the "first" step
        self.P = np.identity(4)
        self.A = np.matrix([[1, self.dt, 0, 0], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, 0],
                            [0, 0, 0, 0]]) # state space transition
        self.B = np.matrix([[1/2*self.dt**2, 0],
                            [self.dt, 0],
                            [0, self.dt],
                            [0, 1]])
        self.Qa = np.matrix([[0.000289, 0], 
                            [0, 4*10**(-8)]]) # Covariance values used from IMU message
        self.Q = self.B*self.Qa*self.B.transpose() # Motion Model / state transition covariance
        self.C = np.matrix([[0, 1/WHEEL_RADIUS, 0, -WHEEL_BASE/2], 
                            [0, 1/WHEEL_RADIUS, 0, +WHEEL_BASE/2],
                            [0, 0, 0, 1]]) # Measurement Model. Translates tracked states to a corresponding expected measurement
        self.R = np.matrix([[0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]]) # Sensor Model Covariance
        self.u = np.matrix([0.0, 0.0]).transpose() # State Transition Inputs: linear acceleration in x (forward direction of robot), and omega
        self.y = np.matrix([0.0, 0.0, 0.0]).transpose() # Sensor Measurements: u_l, u_r, omega_calculated

        # Run Kalman Filter at fixed rate, slower than all subscribers. This synchronizes the measurements
        self.timer = self.create_timer(self.dt, self.run_kalman_filter)

    def imu_callback(self, msg: Imu):
        """Record latest IMU Input"""
        w = msg.angular_velocity.z # omega
        a_x = msg.linear_acceleration.x # linear acceleration in x direction
        #Note: we do not need to apply a transform from imu_link to base_footprint as
        # the static transform shows a offset in the x-direction, and no rotational offset
        # Thus, the x-axis of the IMU aligns with x-axis of the robot.
        self.u = np.matrix([a_x, w]).transpose()
        # self.get_logger().info(f"IMU Angular Velocity: {w}")

    def encoder_callback(self, msg: JointState):
        """Record latest Sensor Measurement"""
        left_wheel_speed = msg.velocity[0] # rad/s
        right_wheel_speed = msg.velocity[1] # rad/s
        omega_calculated = WHEEL_RADIUS * (left_wheel_speed-right_wheel_speed)/WHEEL_BASE
        self.y = np.matrix([left_wheel_speed, right_wheel_speed, omega_calculated]).transpose()

    def run_kalman_filter(self):
        """Run 1 iteration of Kalman Filter Algorithim"""
        # Prediction update
        self.xhat_predicted = self.A * self.xhat + self.B * self.u
        P_predict = self.A*self.P*self.A.transpose() + self.Q
        # Measurement Update and Kalman Gain (Correction)
        K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)
        self.xhat = self.xhat_predicted + K * (self.y - self.C * self.xhat_predicted)
        self.P = (np.identity(4) - K * self.C) * P_predict

        self.publish_states()

    def publish_states(self):
        """Publish predicted and corrected states for other nodes"""
        self.state_msg.header.frame_id = 'odom' # set origin to be coincident with 'odom' frame
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