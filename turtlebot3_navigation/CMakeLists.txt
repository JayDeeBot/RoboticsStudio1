cmake_minimum_required(VERSION 3.5)
project(turtlebot3_navigation)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  # For NavigateToPose action
find_package(rclcpp_action REQUIRED)  # For using action clients

# Add executable for navigation_launcher node
add_executable(navigation_launcher src/navigation_launcher.cpp)
ament_target_dependencies(navigation_launcher rclcpp)

# Add executable for goal_publisher node
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher rclcpp geometry_msgs nav2_msgs rclcpp_action)

# Install the executables
install(TARGETS
  navigation_launcher
  goal_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install the config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Finalize the package
ament_package()
