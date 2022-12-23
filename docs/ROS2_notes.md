# Learning ROS2 
[Ros2 Tutorials](https://docs.ros.org/en/galactic/Tutorials.html)

## tf2

> tf2 is the transform library, which lets the user keep track of multiple coordinate frames over time. tf2 maintains the relationship between coordinate frames in a tree structure.  [Source](https://docs.ros.org/en/galactic/Concepts/About-Tf2.html)

TurtleBot3 broadcasts tf transforms. In python, we can listen for these as shown [here](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html).

- While a rosbag is running, run `ros2 run tf2_tools view_frames`. This output the file: `./src/particle_filter/bag_files/frames.pdf`

RViz and [Foxglove Studio](https://foxglove.dev/) are  useful for viewing the association between TF frames within the ROS system. For `Foxglove Studio`, can use the `./docs/09_MTE544-final-project-foxglove-layout.json` file.

To output the transform between any 2 frames:
- `ros2 bag play src/particle_filter/bag_files/point2`
- `ros2 run rviz2 rviz2 -d rviz_configs/lab2_tf.rviz`

[Additional info about ROS2 tf2](https://articulatedrobotics.xyz/ready-for-ros-6-tf/)

## Useful ROS commands:
- `ros2 run rqt_graph rqt_graph`
- `ros2 topic list`
- `ros2 interface show sensor_msgs/msg/LaserScan`
- `ros2 daemon stop`
- `ros2 daemon start`

## Other info
- [ROS2 Launch files](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html)
- [ROS2 Launch Files substitution](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Substitutions.html)
