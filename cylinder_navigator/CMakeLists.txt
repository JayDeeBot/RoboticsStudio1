cmake_minimum_required(VERSION 3.8)
project(cylinder_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  # Added dependency for action client
find_package(rclcpp_action REQUIRED)


add_executable(circum_navigation src/circum_navigation.cpp)

ament_target_dependencies(circum_navigation
  rclcpp 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  rclcpp_action 
  nav2_msgs  # Added dependency
)

add_executable(goal_publisher src/goal_publisher.cpp)

ament_target_dependencies(goal_publisher
  rclcpp 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  rclcpp_action 
  nav2_msgs  # Added dependency
)

install(TARGETS 
  circum_navigation
  goal_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
