cmake_minimum_required(VERSION 3.5)
project(turtlebot3_image_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_subscriber_node src/image_subscriber_node.cpp)
ament_target_dependencies(image_subscriber_node rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp geometry_msgs)

target_include_directories(image_subscriber_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_subscriber_node ${OpenCV_LIBS})

install(TARGETS
  image_subscriber_node
  teleop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

