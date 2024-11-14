cmake_minimum_required(VERSION 3.5)
project(test_pgm_circle)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_circle_node src/test_circle.cpp)

# Link against OpenCV libraries
target_link_libraries(test_circle_node ${OpenCV_LIBS})

ament_target_dependencies(test_circle_node rclcpp)

# Install the executable
install(TARGETS
  test_circle_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
