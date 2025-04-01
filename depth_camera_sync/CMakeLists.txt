cmake_minimum_required(VERSION 3.8)
project(depth_camera_sync)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(depth_sync_node src/depth_sync_node.cpp)
ament_target_dependencies(depth_sync_node 
  rclcpp
  sensor_msgs
  message_filters
)

install(
  TARGETS depth_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
