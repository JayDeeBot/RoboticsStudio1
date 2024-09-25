cmake_minimum_required(VERSION 3.5)
project(laser_scan_to_image)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add executable
add_executable(scan_matching_localizer src/scan_matching_localizer.cpp)

# Link necessary libraries
ament_target_dependencies(scan_matching_localizer
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  geometry_msgs
)

# Install target
install(TARGETS
scan_matching_localizer
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
