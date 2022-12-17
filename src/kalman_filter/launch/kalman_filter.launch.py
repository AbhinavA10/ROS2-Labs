import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():  
    return LaunchDescription([
        # include Gazebo launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('turtlebot3_gazebo'),
        #             'launch/empty_world.launch.py'))
        # ),
        # Start after Gazebo loads
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', [os.path.join(get_package_share_directory("kalman_filter"), 'launch', 'kf_rviz.rviz')]]
                ),
                Node(
                    package='kalman_filter',
                    executable='path_visualizer.py',
                ),
                Node(
                    package='kalman_filter',
                    executable='frame_transformer.py',
                ),
                Node(
                    package='kalman_filter',
                    executable='kalman_filter.py',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', os.path.join(get_package_share_directory("kalman_filter"), 'bag_files', 'path')],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-o', os.path.join('./src/kalman_filter/bag_files', 'kalman_filter_output'), '--all'],
                    output='screen'
                ),
                # Node(
                #     package='turtlebot3_teleop',
                #     executable='teleop_keyboard',
                # ),
                # Node(
                #     package='rqt_plot',
                #     executable='rqt_plot',
                #     arguments=['/kf_corrected_robot_frame/x /odom_robot_frame/x']
                # ),
            ],
        ),
    ])