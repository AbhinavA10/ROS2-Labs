import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

	# Default initial robot positions for gazebo simulation.
	# Should be modified to match robot's actual position as needed.
	initial_pose_x = 0
	initial_pose_y = 0
	initial_yaw = 0
	gazebo_world_launch_file = ''
	map_file_house = './src/planner/map/office_map/map.yaml'
	map_file_maze = './src/planner/map/Entry_4/Entry_4.yaml'

	lifecycle_nodes = [
		'map_server',
		'planner_server'
	]

	# Change below map as needed
	map_file = map_file_house


	if map_file is map_file_house:
		# Gazebo launch file has an odom (world frame) offset
		initial_pose_x = -2
		initial_pose_y = -0.5
		initial_yaw = 0
		gazebo_world_launch_file = 'launch/turtlebot3_house.launch.py'
	elif map_file is map_file_maze:
		# odom and map frame already coincide
		initial_pose_x = 0
		initial_pose_y = 0
		initial_yaw = 0
		gazebo_world_launch_file = 'launch/empty_world.launch.py'
	
	return LaunchDescription([
		# include another launch file
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('turtlebot3_gazebo'),
					gazebo_world_launch_file))
		),
		
		
		# Start after Gazebo loads
		TimerAction(
			period=5.0,
			actions=[
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory('turtlebot3_navigation2'),
					'launch/navigation2.launch.py')),
			launch_arguments={
				'use_sim_time': 'True',
				'map': map_file,
			}.items()
		),
		Node(
			package = "tf2_ros", 
			executable = "static_transform_publisher",
			arguments = [str(initial_pose_x), str(initial_pose_y), "0", str(initial_yaw), "0", "0", "odom", "map"]
		),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[
				{'use_sim_time':False},
				{'autostart': True},
				{'node_names': lifecycle_nodes},
			]),
		Node(
			package='rviz2',
			executable='rviz2',
			arguments=['-d', [os.path.join(get_package_share_directory("planner"), 'launch', 'map_rviz.rviz')]]
		),
		Node(
			package='planner',
			executable='navigation_server.py',
		),
		
			],
		),

    ])