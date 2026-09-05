cmake_minimum_required(VERSION 3.5)
project(vanjee_ros2_driver)

# Default to C++14 to match the SDK
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Threads
find_package(Threads REQUIRED)

# Include directories
include_directories(
  include
  src
  ${CMAKE_CURRENT_LIST_DIR}/../vanjee_driver/src
  ${EIGEN3_INCLUDE_DIR}
)

# Build the node executable
add_executable(vanjee_ros2_driver_node
  src/vanjee_ros2_driver_node.cpp
)

target_link_libraries(vanjee_ros2_driver_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  Threads::Threads
  pcap
)

# Install
install(TARGETS vanjee_ros2_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
