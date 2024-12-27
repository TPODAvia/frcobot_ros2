cmake_minimum_required(VERSION 3.8)
project(fairino_mtc_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

# # Create a library for TaskBuilder
# add_library(task_builder_lib
#   src/task_builder.cpp
# )

# ament_target_dependencies(task_builder_lib
#   rclcpp
#   moveit_task_constructor_core
#   # ... plus anything else you need
# )
# target_include_directories(task_builder_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )


add_executable(mtc_node src/mtc_node.cpp)
add_executable(add_collision_objects src/add_collision_objects.cpp)
# add_executable(task_generator src/task_generator.cpp)


target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(add_collision_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(add_collision_objects PUBLIC c_std_99 cxx_std_17)

# target_include_directories(task_generator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(task_generator PUBLIC c_std_99 cxx_std_17)


# Use ament_target_dependencies for packages (not local targets)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
)
ament_target_dependencies(add_collision_objects
  rclcpp
  moveit_task_constructor_core
)

# ament_target_dependencies(task_generator
#   rclcpp
#   moveit_task_constructor_core
# )

# If you need to link them together, do this:
# target_link_libraries(mtc_node add_collision_objects)

install(TARGETS 
  mtc_node 
  add_collision_objects 
  # task_generator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
