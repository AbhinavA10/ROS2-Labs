Where is the `turtlebot3_gazebo`'s `turtlebot3_house.launch.py` file? 
- `/opt/ros/galactic/share/turtlebot3_gazebo/launch/turtlebot3_house.launch.py`
- it has a default position of -2.5, -0.5

The robot was located in -2, -0.5 wrt to the world in gazebo. So we had to correct for it by moving the odom frame by 2, 0.5. This is why the static tf publisher says map to odom is 2, 0.5 (trial and error)

The gazebo world frame (`odom`) does not correspond with the map frame. (Origins do not coincide).
If we want the origins to coincide, need to copy the gazebo house world and gazebo launch file, and edit the files so the frames coincide (would move the house model `house.world` to match with map frame)
Then can publish (0,0) tf between odom and map

For the `Entry_4` map and using `empty_world.launch`, the map frame and odom frame already coincide

The whole idea with all this stuff is to make the `odom` and `map` frame line up. But in part 1 of the lab, 
- The original `turtlebot3_gazebo` launch file puts the `odom`/gazebo world frame at some non-0 value.
- Initially we tried using a non-zero static tf publisher for odom <--> map in order to show the map on `Rviz`. This located the `map` frame at the starting pose of the Robot in gazebo.
- But for some reason, the original Gazebo launch file put the world frame at the wrong spot.
- So instead, we made our own copy of gazebo launch file and house.world, which had correction changes to align `odom` and `map` by moving the "house world" model. i.e. you undid the above point. Then the static tf publisher could publish 0.

For part 2 of the lab
- Since the empty_world gazebo launch file had a `0,0` starting pose, we could make the `odom` and `map` frame line up through our static publisher automatically (publishing `0`)

Other Notes
- Gazebo only publishes tf tree from odom downwards
- Based on running `Nav2` stack in `house_launch` with amcl, `/tf` has `map` -> `odom` -> `base_footprint` -> `base_link`.
- "with AMCL node" means entire nav stack, and have set the "2D pose" estimate in Rviz
- tf contains transform b/w odom and base_footprint. And also odom to map if given
- map to odom tf publisher in the launch file is altnerative to AMCL, to visualize map in ROS. This static tf assumes odom sensors are perfect and localization is perfect

Idea
- Instead of running the nav2_planner independantly as we have, we could have also run the entire Nav2 stack (with AMCL node) as shown in the lab manual, and would see similar behaviour. In this case, would not need to publish map<->odom tf.
- Or, could also publish the map to odom tf, in which case we don't need to set 2D pose estimate for AMCL.



To run the Nav2 stack with on house map simulation, while still generating a costmap, run:
- `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`
- `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=./src/planner/map/office_map/map.yaml`
- `ros2 launch planner planner.launch.py`
- For defining goals in RViz:
  - `ros2 run planner navigation_client.py`
  - Then through RViz, give a `2D Goal Pose`
- For pre-defining a goal from terminal (modify x and y as needed): 
  - `ros2 run planner navigation_client.py --ros-args -p predefined_goal:=True -p goal_x:=4.0 -p goal_y:=0.8`

To run the Nav2 stack with an empty world gazebo simulation, while still generating a costmap, run:
- `ros2 launch turtlebot3_gazebo empty_world.launch.py`
- `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=./src/planner/map/office_map/map.yaml`
- `ros2 launch mte544_a_star mte544_a_star.launch.py`
- `ros2 run mte544_a_star mte544_navigation_client.py --ros-args -p predefined_goal:=True -p goal_x:=4.0 -p goal_y:=.8`
- `ros2 run rviz2 rviz2 -d map_rviz.rviz`

# Lab 3 Physical
- Got it working in simulation
- pass in pose of 2.33, 0 as goal in client
- To generate costmap run gazebo with empty_world.launch

# A-* note
you should have costs assigned to the map, and then have on top of that also the heuristic. The heuristic is what would make your search A*. The ones you mentioned are for the heuristic and not the cost, the cost of each node should be the cost from one node to the immediate neighboring ones. For example, if there’s an obstacle there this should be infinite as you’d not want to go there. You have the information of whether there’s an obstacle or not from the map. We briefly talked about this in L19.

# Origin of map
When you load in the map the top left corner is labeled as (0,0) because it is based on the indexing of the array (top left is row 0 and column 0, as you go to the right your columns increase, as you go down your rows increase). But the actual origin of the map is in the bottom left corner.

Where the yaml file origin comes in is that it describes the location of the bottom left corner realtive to the odom frame. Therefore the bottom left corner of the map is at (-0.843,-3.94) in the odom frame for this map. You can then relabel the axis knowing this is what the bottom left corner should be and knowing the resolution per pixel specifed in the yaml file.