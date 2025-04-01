cmake_minimum_required(VERSION 3.8)
project(fairino_mtc_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############################################################################
# Find dependencies
##############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

# Add MoveIt Core and related packages
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add message packages
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add Eigen3 for spline interpolation
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

##############################################################################
# Create library from shared source (task_builder.cpp)
##############################################################################
add_library(${PROJECT_NAME}_lib
  src/task_builder.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  Eigen3
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  yaml-cpp
)

##############################################################################
# Build Executables
##############################################################################

# 1) mtc_node
add_executable(mtc_node src/mtc_node.cpp)
target_compile_features(mtc_node PUBLIC cxx_std_17)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
)
target_link_libraries(mtc_node
  ${PROJECT_NAME}_lib
)

# 2) add_collision_objects
add_executable(add_collision_objects src/add_collision_objects.cpp)
target_compile_features(add_collision_objects PUBLIC cxx_std_17)
ament_target_dependencies(add_collision_objects
  rclcpp
  moveit_task_constructor_core
)
target_link_libraries(add_collision_objects
  ${PROJECT_NAME}_lib
)

# 3) task_generator
add_executable(task_generator src/task_generator.cpp)
target_compile_features(task_generator PUBLIC cxx_std_17)
ament_target_dependencies(task_generator
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  Eigen3
  tf2_geometry_msgs
  yaml-cpp
)
target_link_libraries(task_generator
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)

# 4) task_executor
add_executable(task_executor src/task_executor.cpp)
target_compile_features(task_executor PUBLIC cxx_std_17)
ament_target_dependencies(task_executor
  rclcpp
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  Eigen3
  tf2_geometry_msgs
  yaml-cpp
)
target_link_libraries(task_executor
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)

##############################################################################
# Installation
##############################################################################

install(
  TARGETS
    mtc_node
    add_collision_objects
    task_generator
    task_executor  # <-- install the new executable
  DESTINATION lib/${PROJECT_NAME}
)

# (Optional) If you have headers in `include/`:
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# Install launch files
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

##############################################################################
# Tests / Linters
##############################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

##############################################################################
# Finalize
##############################################################################
ament_package()
