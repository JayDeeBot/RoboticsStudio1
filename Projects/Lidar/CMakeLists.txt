cmake_minimum_required(VERSION 3.5)
project(turtlebot3_lidar_processing)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(point_cloud_node src/point_cloud_node.cpp)
ament_target_dependencies(point_cloud_node rclcpp sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs PCL pcl_conversions)

add_executable(three_d_model_node src/three_d_model_node.cpp)
ament_target_dependencies(three_d_model_node rclcpp sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs visualization_msgs PCL pcl_conversions)

install(TARGETS
  point_cloud_node
  three_d_model_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
