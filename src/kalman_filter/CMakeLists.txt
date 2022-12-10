cmake_minimum_required(VERSION 3.5)
project(kalman_filter)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(planner_action_interfaces REQUIRED)


install(DIRECTORY
  launch
  scripts
  bag_files
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/kalman_filter.py
  scripts/visualize_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
