cmake_minimum_required(VERSION 3.5)
project(move_turtlebot)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  include  # Assuming you have header files in the include directory
)

# Create the executable for the move_robot_node
add_executable(move_robot_node src/move_robot_node.cpp)

# Link against necessary libraries
ament_target_dependencies(move_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  PCL
  pcl_conversions
)

# Link PCL and OpenCV libraries
target_link_libraries(move_robot_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS
  move_robot_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)

# Install RViz configuration files (if any)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz)

# Install params files (if any)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/params)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for installation
ament_package()
