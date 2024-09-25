Robotics Studio 1 - Scamazon Smart Warehouse
This repository holds all my individual and group work for Robotics Studio 1. Our project group, Scamazon, is developing an Amazon Smart Warehouse using TurtleBot3.

Key Package: Scamazon Navigation
The scamazon_navigation package allows TurtleBot3 to localize and navigate through goals in a 2D warehouse using a .pgm map and .yaml configuration, employing Nav2 for navigation. An optimization algorithm is being developed, using the Nearest Neighbor method, to optimize the path between goals.

Scamazon Navigation Commands
Package
bash
Copy code
# Compile
cd ~/ros2_ws
colcon build --packages-select scamazon_navigation

# Node List
reach_goals

# Launch Package
ros2 run scamazon_navigation reach_goals
ros2 run scamazon_navigation initial_pose_publisher
Testing
bash
Copy code
# Launch World
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_GroupWarehouse.launch.py

# Launch Nav2 with Obstacle Avoidance and Map
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/student/ros2_ws/src/tejasWarehouse/updatedWarehouse_map2.yaml

# Send Test Goal
ros2 topic pub -1 /scam_goals geometry_msgs/msg/PoseArray ...
ROS2 Commands
Colcon Workflow
bash
Copy code
# Create New Package
ros2 pkg create <package_name> --build-type ament_cmake --dependencies <dependencies>

# Build Packages
colcon build --packages-select <package_name>

# Source Workspace
source install/setup.bash
Launch File Workflow
bash
Copy code
# Create a launch file
cd <package_name>/launch
touch my_launch_file.py
ROS2 Bag
bash
Copy code
# Record all topics
ros2 bag record -a

# Play Bag File
ros2 bag play /path/to/bagfile
Visualizations
bash
Copy code
# Open RViz
rviz2

# Open rqt Image Viewer
rqt --standalone rqt_image_view

