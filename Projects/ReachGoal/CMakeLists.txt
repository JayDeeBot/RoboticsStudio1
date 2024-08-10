cmake_minimum_required(VERSION 3.5)
project(turtlebot3_goal_sender)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(goal_publisher src/goal_publisher.cpp)
add_executable(goal_executor src/goal_executor.cpp)

ament_target_dependencies(goal_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(goal_executor
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
)

install(TARGETS
  goal_publisher
  goal_executor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
