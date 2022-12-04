#!/usr/bin/env python3

import rclpy

from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Path
from a_star import find_path
import numpy as np
from planner_action_interfaces.action import Move2Goal
from skimage.transform import downscale_local_mean # scale map

import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class AStarActionServer(Node):
    """ROS2 Action Server that plans path to a goal pose using A* and then follows the path using P-controller"""

    def __init__(self):
        super().__init__('a_star_action_server')
        # Setup Action server for receiving goal poses
        self._action_server = ActionServer(
            self,
            Move2Goal,
            'planner',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
        )
        # create the subscriber object to Nav2 costmap
        self.global_map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Create path visualizer publisher 
        self.path_pub = self.create_publisher(Path, '/path_viz', 10)

        # Initialize map parameters to default values. Will be correctly loaded later
        self.origin = [0,0,0]
        self.map_res = 0.05
        # Global costmap variable
        self.occupancy_map = None
        
        # List of cartisian points containing A* path to goal, in map frame in meters
        self.path_cart = None
        
        self.estimated_heuristic = None
        # P-control publisher
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # The P-controller runs in the global frame.
        self.curr_pose = Pose() # holds current global position of turtlebot.
        self.setpoint_pose = Pose() # defaults to 0 till we receive a new setpoint from external node
        self.vel_msg = Twist() # holds velocity command to send to turtlebot
        self.reached_intermediate_goal = False

        # Used for finding TF between base_link frame and map (i.e. robot position)
        # See https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def goal_callback(self, goal_request):
        """Determine whether to accept or reject the goal"""

        # Access global map and pass to A* function
        while True:
            # wait till global costmap has been received
            if self.occupancy_map is None:
                pass
            else:
                break
        initial_pose = goal_request.initial_pose # client also sends initial pose of the robot

        # Get start and goal positions in map pixels
        start = self.coord_2_pixel((initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y))
        goal =  self.coord_2_pixel((goal_request.goal_x, goal_request.goal_y))

        # Run A*
        [path, cost] = find_path(start, goal, self.occupancy_map)

        self.estimated_heuristic = cost
        if path.size == 0:
            # Failed to find a path to the goal
            self.get_logger().info(f"No path found")
            return GoalResponse.REJECT

        # Convert path from pixels to map frame in meters
        path_scale = path*self.map_res
        self.path_cart = path_scale + self.origin

        # Create RViz message for visualizing path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for pose in self.path_cart:
            point = PoseStamped()
            point.pose.position.x = pose[0]
            point.pose.position.y = pose[1]
            path_msg.poses.append(point)
        self.path_pub.publish(path_msg)

        self.get_logger().info(f"Number of points: {self.path_cart.shape[0]}")

        return GoalResponse.ACCEPT

    def map_callback(self, msg):
        """Load received global costmap"""

        if self.occupancy_map is None:
            self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            self.height = msg.info.height
            self.width = msg.info.width
            self.map_res = msg.info.resolution
            self.occupancy_map = np.reshape(msg.data, (self.height, self.width))
            
            # downsample map for faster processing
            self.occupancy_map = downscale_local_mean(self.occupancy_map, (2,2))
            self.map_res *= 2
            
            # Convert costmap message from y-by-x to x-by-y coordinates as it is row-major order, with (0,0) at bottom left
            self.occupancy_map = np.transpose(self.occupancy_map)

    def coord_2_pixel(self, point):
        """Convert a coordinate in map frame (meters) to pixels"""
        return (
            round((point[0] - self.origin[0])/(self.map_res)), 
            round((point[1] - self.origin[1])/(self.map_res))
        ) 

    def execute_callback(self, goal_handle):
        """Follow planned path using a P controller"""
        start_time = self.get_clock().now()
        self.get_logger().info('Executing goal...') 
        
        prev_pose = [self.curr_pose.x, self.curr_pose.y]
        dist_travelled = 0
        for point in self.path_cart:
            self.get_logger().info(f"Current Pose: [{self.curr_pose.x:.3f},{self.curr_pose.y:.3f},{self.curr_pose.theta:.3f}]")
            self.get_logger().info(f"Going to: {point}")
            self.setpoint_pose.x = point[0]
            self.setpoint_pose.y = point[1]
            self.reached_intermediate_goal = False
            while not self.reached_intermediate_goal:
                self.update_current_pose() # Get latest position of turtlebot
                self.run_control_loop_once() # run 1 iteration of P control
                
                # Create feedback message with updated info
                dist_travelled += math.sqrt(
                    math.pow(self.curr_pose.x - prev_pose[0], 2) + math.pow(self.curr_pose.y - prev_pose[1], 2)
                )

                prev_pose = [self.curr_pose.x, self.curr_pose.y]
                rclpy.spin_once(self, timeout_sec=0.1) # 10Hz spin
            
            feedback_msg = Move2Goal.Feedback()
            feedback_msg.current_pose.pose.position.x = self.curr_pose.x
            feedback_msg.current_pose.pose.position.y = self.curr_pose.y
            elapsed_time = self.get_clock().now() - start_time
            feedback_msg.navigation_time =  elapsed_time.to_msg()
            feedback_msg.distance_travelled =  dist_travelled
            goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info('Reached Final goal')
        self.occupancy_map = None # Reset Occupancy map for next iteration
        
        # Notify client
        goal_handle.succeed()
        result = Move2Goal.Result()
        result.estimated_heuristic_cost = self.estimated_heuristic
        result.total_distance_travelled = dist_travelled
        result.reached_goal = True  
        return result
    
    #P-controller helper functions
    def update_current_pose(self):
        """Get current pose of Turtlebot"""
        # Find the transform between base_link and map
        from_frame_rel = 'base_link'
        to_frame_rel = 'map'
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.curr_pose.x = t.transform.translation.x
            self.curr_pose.y = t.transform.translation.y
            base_map_rot = t.transform.rotation
            quaternion = (
                base_map_rot.x,
                base_map_rot.y,
                base_map_rot.z,
                base_map_rot.w
                )
            _, _, self.curr_pose.theta = self.euler_from_quaternion(quaternion)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

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

    def bound(self, val:float, bound:float) ->float:
        """Limit value to given symmetric range"""
        return min(max(val, -bound), bound)
    
    def get_position_error(self):
        """Calculate error in position between current pose and the setpoint pose"""
        # As we independantly have another P controller to turn towards the setpoint, we can 
        # think of the position error as a straight line away from our current position.
        # So, use Euclidean distance as the error function.
        return math.sqrt(math.pow((self.setpoint_pose.x - self.curr_pose.x), 2) +
                         math.pow((self.setpoint_pose.y - self.curr_pose.y), 2))

    def get_linear_velocity(self, Kp=1.2) -> float:
        """Proportional controller for Position"""
        # if we need to rotate more than 60 degrees (arbitrarily), we should first turn and then move forward later
        # This will allow robot to stay on course
        if abs(self.get_angle_error()) > math.pi/3.0:
            return 0.0
        # Calculate proportional linear velocity
        return self.bound(Kp * self.get_position_error(), 0.3) # Max linear velocity from Turtlebot4 datasheet: 0.3m/s

    def get_required_theta(self):
        """Calculate angle needed for turtlebot to face the setpoint"""
        return math.atan2(self.setpoint_pose.y - self.curr_pose.y, self.setpoint_pose.x - self.curr_pose.x)

    def get_angle_error(self):
        '''Calculate change in angle needed for turtlebot to face the setpoint'''
        angle_error = self.get_required_theta() - self.curr_pose.theta
        # if current theta is 3.14 and desired theta is -3.1415, the smallest rotation angle would be 0.0015 rad 'left' 
        # and the rotation shouldn't be 1 whole round rotation : = -6.2 rad
        # So, Accounts for discontinous jump at pi -> -pi
        # by adding or subtracting a full turn (2pi rad) when the angle resulting angle is outside the (-pi, pi) range.
        if angle_error < -math.pi:
            angle_error = angle_error + 2*math.pi
        elif (angle_error > math.pi):
            angle_error = angle_error - 2*math.pi
        return angle_error

    def get_angular_velocity(self, Kp=6.2) -> float:
        '''Proportional controller for orientation to face towards setpoint position'''
        # Calculate proprotional angular velocity
        return self.bound(Kp * self.get_angle_error(), 1.9) # Max angular velocity from Turtlebot4 datasheet: 1.9rad/s

    def run_control_loop_once(self):
        """Run 1 iteration of P-controller"""
        # If our control loop is still active when the robot is really close, then we will start seeing unnecessary osciliations
        # due to control inaccuracies. e.g. Turtlebot jittering back and forth. 
        # So, if turtlebot is closer than 0.1 to setpoint, mark that we've reached the intermediate goal
        # This avoids oscillation, as well as allowing us to quickly move to next intermediate goal point
        if self.get_position_error() >= 0.1:
            # generate control signals (velocities) required to reach intermediate goal pose
            self.vel_msg.linear.x = self.get_linear_velocity() # move towards setpoint
            self.vel_msg.angular.z = self.get_angular_velocity() # orient towards setpoint
        else:
            self.reached_intermediate_goal = True
            # Stopping our robot after the movement is over.
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
        self.publisher_vel.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    a_star_action_server = AStarActionServer()
    while True:
        rclpy.spin_once(a_star_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()