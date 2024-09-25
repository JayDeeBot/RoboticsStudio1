cmake_minimum_required(VERSION 3.5)
project(laser_scan_filter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include the header directory
include_directories(include)

# Declare the executables
add_executable(laser_scan_filter_node src/main.cpp src/laser_scan_filter_node.cpp)
add_executable(laser_scan_visualizer_node src/laser_scan_visualizer_node.cpp)

# Link against dependencies
ament_target_dependencies(laser_scan_filter_node rclcpp sensor_msgs)
ament_target_dependencies(laser_scan_visualizer_node rclcpp sensor_msgs visualization_msgs geometry_msgs)

# Install the executables
install(TARGETS
  laser_scan_filter_node
  laser_scan_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

# Install the header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Ament package
ament_package()
