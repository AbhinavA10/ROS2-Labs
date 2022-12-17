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
        # self.get_logger().info(f"Ground Truth x_dot: {msg.x_dot}")

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
        # Print data point at ~middle of data
        self.get_logger().info(f"SAMPLE DATA POINT")
        self.get_logger().info(f"Data format: [x x_dot theta omega]")
        self.get_logger().info(f"Time: {times[int(times.shape[0]/2)]}")
        self.get_logger().info(f"Ground Truth: {ground_truths[int(ground_truths.shape[0]/2),:]}")
        self.get_logger().info(f"Prediction: {kf_corrections[int(kf_corrections.shape[0]/2),:]}")
        self.get_logger().info(f"Correction: {kf_predictions[int(kf_predictions.shape[0]/2),:]}")
        
        self.get_logger().info(f"MSE FOR PREDICTIONS")
        self.make_plots(times,
                    [kf_predictions[:,0], ground_truths[:,0], kf_predictions[:,1], ground_truths[:,1], kf_predictions[:,2], ground_truths[:,2], kf_predictions[:,3],  ground_truths[:,3]],
                    [r"$x_{predicted}$", r"$x_{true}$", r"$\dot x_{predicted}$", r"$\dot x_{true}$", r"$\theta_{predicted}$", r"$\theta_{true}$", r"$\omega_{predicted}$", r"$\omega_{true}$"],
                    ['Distance [m]', 'Velocity [m/s]', 'Angle [rad]', 'Angular Velocity [rad/s]'],
                    "Prediction vs. Ground Truth")
        self.get_logger().info(f"MSE FOR ESTIMATES")
        self.make_plots(times,
                    [kf_corrections[:,0], ground_truths[:,0], kf_corrections[:,1], ground_truths[:,1], kf_corrections[:,2], ground_truths[:,2], kf_corrections[:,3],  ground_truths[:,3]],
                    [r"$x_{estimated}$", r"$x_{true}$", r"$\dot x_{estimated}$", r"$\dot x_{true}$", r"$\theta_{estimated}$", r"$\theta_{true}$", r"$\omega_{estimated}$", r"$\omega_{true}$"],
                    ['Distance [m]', 'Velocity [m/s]', 'Angle [rad]', 'Angular Velocity [rad/s]'],
                    "Estimate vs. Ground Truth")
        plt.show()
    
    def make_plots(self, times: np.ndarray, datas: list, line_labels: list, ylabels: list, title: str):
        # Make overall plot
        plt.figure(figsize=(10,8))
        plt.title(title)
        for i in range(len(line_labels)):
            plt.plot(times, datas[i], label=line_labels[i])
        plt.legend(loc="upper left")
        plt.xlabel("Time [s]")
        plt.grid()

        # Make detailed subplots
        fig, axs = plt.subplots(int(len(line_labels)/2), figsize=(7, 11))
        fig.suptitle(title)
        for i in range(int(len(line_labels)/2)):
            axs[i].plot(times, datas[2*i], label=line_labels[2*i])
            axs[i].plot(times, datas[2*i+1], label=line_labels[2*i+1])
            mse = (np.square(datas[2*i] - datas[2*i+1])).mean()
            self.get_logger().info(f"MSE {line_labels[2*i][1:-1]}<->{line_labels[2*i+1][1:-1]}: {mse}")
            axs[i].legend()
            axs[i].set(ylabel=ylabels[i])
            axs[i].grid()
        axs[-1].set(xlabel="Time [s]")
        fig.tight_layout()

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