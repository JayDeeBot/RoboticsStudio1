# Add this content to CMakeLists.txt
# CMakeLists.txt

cmake_minimum_required(VERSION 3.5)
project(turtlebot3_image_processing)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(image_processor src/image_processor.cpp)

# Link libraries
ament_target_dependencies(image_processor
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

install(TARGETS
  image_processor
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
