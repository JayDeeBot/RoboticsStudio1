cmake_minimum_required(VERSION 3.5)
project(autonomous_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(autonomous_mapping src/autonomous_mapping.cpp src/autonomous_mapping_main.cpp)
ament_target_dependencies(autonomous_mapping rclcpp sensor_msgs nav_msgs geometry_msgs nav2_msgs rtabmap_ros std_srvs)

install(TARGETS
  autonomous_mapping
  DESTINATION lib/${PROJECT_NAME})

ament_package()
