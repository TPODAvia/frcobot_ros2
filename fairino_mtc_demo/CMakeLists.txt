cmake_minimum_required(VERSION 3.8)
project(fairino_mtc_demo)

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

##############################################################################
# Create library from shared source (task_builder.cpp)
##############################################################################
add_library(${PROJECT_NAME}_lib
  src/task_builder.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  # adjust if header is in 'include/'
    $<INSTALL_INTERFACE:include>
)

# Link the library with required dependencies
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  moveit_task_constructor_core
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
)
target_link_libraries(task_generator
  ${PROJECT_NAME}_lib
)

##############################################################################
# Installation
##############################################################################
# Install all executables
install(
  TARGETS 
    mtc_node
    add_collision_objects
    task_generator
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally install headers if you move them to `include/`:
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(
  DIRECTORY launch
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
