cmake_minimum_required(VERSION 3.8)
project(gripper_hardware)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/gripper_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch & config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
