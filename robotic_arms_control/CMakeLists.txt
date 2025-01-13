cmake_minimum_required(VERSION 3.5)
project(robotic_arms_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# (Optional) Add C++ executables here
# add_executable(my_cpp_executable src/my_cpp_executable.cpp)
# ament_target_dependencies(my_cpp_executable
#   rclcpp
#   std_msgs
#   geometry_msgs
#   moveit_task_constructor_core
# )
# install(TARGETS my_cpp_executable
#   DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/joint_controller_test.py
  scripts/jtc_multi_goals.py
  scripts/kinematic_solver.py
  scripts/rtb_ikine_fkine.py
  scripts/transform_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(ament_cmake)

ament_package()
