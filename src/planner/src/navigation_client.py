#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from planner_action_interfaces.action import Move2Goal

class AStarClient(Node):

    def __init__(self):
        super().__init__('a_star_client')
        self._action_client = ActionClient(self, Move2Goal, 'planner')
        # create the subscriber object to RViz `2D Goal Pose`
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_rviz_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Used for finding TF between base_link frame and map (i.e. robot position)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose = None
        self.goal_pose = None

        # ROS2 params for setting goal through node launch
        self.declare_parameter('predefined_goal', False)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)

    def get_current_pose(self):
        """Get current pose of Turtlebot"""
        # Find the transform between base_link and map
        from_frame_rel = 'base_link'
        to_frame_rel = 'map'
        for i in range(500):
            rclpy.spin_once(self)
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
                curr_pose_x = t.transform.translation.x
                curr_pose_y = t.transform.translation.y
                base_map_rot = t.transform.rotation
                quaternion = (
                    base_map_rot.x,
                    base_map_rot.y,
                    base_map_rot.z,
                    base_map_rot.w
                )

                initial_pose = PoseWithCovarianceStamped()
                initial_pose.pose.pose.position.x = curr_pose_x
                initial_pose.pose.pose.position.y = curr_pose_y
                initial_pose.pose.pose.orientation.x = quaternion[0]
                initial_pose.pose.pose.orientation.y = quaternion[1]
                initial_pose.pose.pose.orientation.z = quaternion[2]
                initial_pose.pose.pose.orientation.w = quaternion[3]
                return initial_pose

            except TransformException as ex:
                pass

    def goal_pose_rviz_callback(self, msg):
        """Store `2D Goal Pose` from RViz"""
        if self.goal_pose is None:
            self.goal_pose = msg
            self.get_logger().info(f"Goal position: {self.goal_pose.pose.position.x, self.goal_pose.pose.position.y}")

    def send_goal(self, initial_x=None, initial_y=None, goal_x=None, goal_y=None):
        """Send ActionServer a new goal"""

        goal_msg = Move2Goal.Goal()
        if not (initial_x and initial_y and goal_x is not None and goal_y is not None):
            # Will be getting a goal pose from RViz
            while True:
                # Wait till we get an initial pose from tf
                if self.initial_pose is None:
                    self.initial_pose = self.get_current_pose()
                    rclpy.spin_once(self)
                else:
                    break
            while True:
                # Wait till we get an goal pose from RViz
                if self.goal_pose is None:
                    rclpy.spin_once(self)
                else:
                    break
            
            goal_msg.initial_pose = self.initial_pose
            goal_msg.goal_x = self.goal_pose.pose.position.x
            goal_msg.goal_y = self.goal_pose.pose.position.y

        else:
            # Received a predefined goal
            goal_msg.initial_pose = PoseWithCovarianceStamped()
            goal_msg.initial_pose.pose.pose.position.x = initial_x
            goal_msg.initial_pose.pose.pose.position.y = initial_y
            goal_msg.goal_x = goal_x
            goal_msg.goal_y = goal_y
        
        # Send goal to ActionServer
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Parse whether ActionServer has accepted new goal"""
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_pose = None
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        # Setup callback for ActionServer result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Parse ActionServer result"""

        result = future.result().result
        self.get_logger().info(f"Goal Reached: {result.reached_goal}, Estimated Heuristic Cost: {result.estimated_heuristic_cost:.3f}, Total Distance Travelled: {result.total_distance_travelled:.3f}")
        self.goal_pose = None
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Parse ActionServer feedback"""

        feedback = feedback_msg.feedback
        curr_pose_x = feedback.current_pose.pose.position.x
        curr_pose_y = feedback.current_pose.pose.position.y
        dist = feedback.distance_travelled
        elapsed_duration = feedback.navigation_time
        elapsed_time = elapsed_duration.sec + elapsed_duration.nanosec/1.0E9
        self.get_logger().info(f"Feedback: Robot Pose [{curr_pose_x:.3f},{curr_pose_y:.3f}], Elapsed Time [{elapsed_time:.3f}], Distance Travelled [{dist:.3f}]")

def main(args=None):
    rclpy.init(args=args)

    action_client = AStarClient()
    
    initial_pose = None

    if action_client.get_parameter('predefined_goal').value:
        # Receiving a goal pose through node launch
        # Therefore, get initial pose of robot ourselves
        initial_pose = action_client.get_current_pose()
    
    if action_client.get_parameter('predefined_goal').value and initial_pose is not None:
        # Ensure we were able to get initial pose of robot ourselves
        goal_x = action_client.get_parameter('goal_x').value
        goal_y = action_client.get_parameter('goal_y').value
        initial_x = initial_pose.pose.pose.position.x
        initial_y = initial_pose.pose.pose.position.y
        action_client.send_goal(initial_x, initial_y, goal_x, goal_y)
    else:
        # Goal will be received through RViz
        action_client.send_goal()
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
