import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os


def generate_launch_description():

	# Default initial robot positions for gazebo simulation.
	# Should be modified to match robot's actual position as needed.
	initial_pose_x = -2
	initial_pose_y = -0.5
	initial_yaw = 0

	lifecycle_nodes = [
		'map_server',
		'planner_server'
	]

	map_file = os.path.join(get_package_share_directory("planner"), 'map', 'map.yaml')
	
	return launch.LaunchDescription([

		# Start node after 1.5 seconds 
		# launch.actions.TimerAction(
		# 	period=1.5,
		# 	actions=[
		launch_ros.actions.Node(
			package = "tf2_ros", 
			executable = "static_transform_publisher",
			arguments = [str(initial_pose_x), str(initial_pose_y), "0", str(initial_yaw), "0", "0", "odom", "map"]
		),
		# 	],
		# ),

        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[
				{'use_sim_time':False},
				{'autostart': True},
				{'node_names': lifecycle_nodes},
			]),
		launch_ros.actions.Node(
			package='rviz2',
			executable='rviz2',
			arguments=['-d', [os.path.join(get_package_share_directory("planner"), 'launch', 'map_rviz.rviz')]]
		),

		launch_ros.actions.Node(
			package='planner',
			executable='navigation_server.py',
		),
    ])