#!/usr/bin/env python3

import rclpy
import matplotlib.pyplot as plt
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from rclpy.qos import ReliabilityPolicy, QoSProfile
from kalman_filter_interfaces.msg import RobotFrameState # Custom message type

class Plotter(Node):
    """Creates plots and calculates MSE between Kalman Filter states and ground truth"""

    def __init__(self):
        super().__init__('plotter')
        self.create_subscription(RobotFrameState, '/odom_robot_frame', self.odom_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(RobotFrameState, '/kf_predicted_robot_frame', self.kf_prediction_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(RobotFrameState, '/kf_corrected_robot_frame', self.kf_correction_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        # create the subscriber object to RViz `Clicked Point`
        self.create_subscription(PointStamped, '/clicked_point', self.perform_analysis, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.ground_truth = RobotFrameState()
        self.kf_prediction = RobotFrameState()
        self.kf_correction = RobotFrameState()
        
        self.times = []
        self.ground_truths = []
        self.kf_predictions = []
        self.kf_corrections = []

        self.start_time = self.get_clock().now()
    
    def get_seconds_since_start(self) -> float:
        """Return seconds since this node started"""
        return ((self.get_clock().now()-self.start_time).nanoseconds /(10**9))

    def odom_callback(self, msg: RobotFrameState):
        """Store latest odom message"""
        self.ground_truth = msg
        self.get_logger().info(f"Ground Truth x_dot: {msg.x_dot}")

    def kf_prediction_callback(self, msg: RobotFrameState):
        """Store latest Kalman Filter Prediction message"""
        self.kf_prediction = msg

    def kf_correction_callback(self, msg: RobotFrameState):
        """Store latest Kalman Filter Correction message"""
        self.kf_correction = msg
        # self.get_logger().info(f"Corrected x_dot: {msg.x_dot}")
        # Upon receiving Kalman filter correction, save latest data. This synchronizes ground truth and Kalman Filter output
        self.record_data()
    
    def record_data(self):
        """Save current data to an array, for later analysis"""
        self.times.append(self.get_seconds_since_start())
        self.ground_truths.append([self.ground_truth.x, self.ground_truth.x_dot, self.ground_truth.theta, self.ground_truth.omega])
        self.kf_predictions.append([self.kf_prediction.x, self.kf_prediction.x_dot, self.kf_prediction.theta, self.kf_prediction.omega])
        self.kf_corrections.append([self.kf_correction.x, self.kf_correction.x_dot, self.kf_correction.theta, self.kf_correction.omega])

    def perform_analysis(self, msg: PointStamped):
        """Plot and calculate MSE"""
        # Convert to Numpy arrays for analysis
        times = np.array(self.times)
        ground_truths = np.array(self.ground_truths)
        kf_predictions = np.array(self.kf_predictions)
        kf_corrections = np.array(self.kf_corrections)
        
        plt.figure(1)
        plt.plot(times, kf_corrections[:,0])
        plt.plot(times, kf_corrections[:,1])
        plt.plot(times, kf_corrections[:,2])
        plt.plot(times, kf_corrections[:,3]) 
        plt.plot(times, ground_truths[:,0])
        plt.plot(times, ground_truths[:,1])
        plt.plot(times, ground_truths[:,2])
        plt.plot(times, ground_truths[:,3])
        plt.legend(['x est','x_dot est', 'theta est', 'omega est', 'x true','x_dot true', 'theta true', 'omega true'])
        plt.figure(2)
        plt.plot(times, kf_predictions[:,0])
        plt.plot(times, kf_predictions[:,1])
        plt.plot(times, kf_predictions[:,2])
        plt.plot(times, kf_predictions[:,3])
        plt.plot(times, ground_truths[:,0])
        plt.plot(times, ground_truths[:,1])
        plt.plot(times, ground_truths[:,2])
        plt.plot(times, ground_truths[:,3])
        plt.legend(['x pred','x_dot pred', 'theta pred', 'omega pred', 'x true','x_dot true', 'theta true', 'omega true'])
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    visualizer = Plotter()
    try:
        while rclpy.ok():
            rclpy.spin_once(visualizer)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()