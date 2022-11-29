import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	
	map_server_config_path = os.path.join(get_package_share_directory("particle_filter"), 'map', 'map_maze_1.yaml')

	lifecycle_nodes = ['map_server']
	use_sim_time = False
	autostart = True

	# https://answers.ros.org/question/406036/how-to-publish-map-in-ros2-galactic/
	start_lifecycle_manager_cmd = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager',
		output='screen',
		parameters=[{'use_sim_time': use_sim_time},
		            {'autostart': autostart},
		            {'node_names': lifecycle_nodes}])

	map_server_cmd = Node(
	    package='nav2_map_server',
	    executable='map_server',
	    output='screen',
	    parameters=[{'yaml_filename': map_server_config_path}]
	)
	
	rviz2_node = Node(
		package='rviz2',
		executable='rviz2',
		arguments=['-d', [os.path.join(get_package_share_directory("particle_filter"), 'launch', 'map_rviz.rviz')]]
	)

	particle_filter_node = Node(
	    package='particle_filter',
	    executable='particle_filter',
	    output='screen',
	)
	
	# bag_play = ExecuteProcess(
    # 	cmd = ['ros2', 'bag', 'play',  '/home/cmele/Map1/testPoint'],
    # )
	
	return LaunchDescription([
		rviz2_node,

		# Start map_server and lifecycle node after 2 seconds 
		TimerAction(
			period=2.0,
			actions=[map_server_cmd, start_lifecycle_manager_cmd],
		),

		# Start particle_filter node after 2.5 seconds
		TimerAction(
			period=2.5,
			actions=[particle_filter_node],
		),
	])
