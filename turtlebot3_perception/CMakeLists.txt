cmake_minimum_required(VERSION 3.5)
project(turtlebot3_perception)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(laser_scan_processor src/laser_scan_processor.cpp)

# Link dependencies
ament_target_dependencies(laser_scan_processor rclcpp sensor_msgs std_msgs)

# Install the target
install(TARGETS
  laser_scan_processor
  DESTINATION lib/${PROJECT_NAME})

# Install launch files if you have any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing if you have tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
