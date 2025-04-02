cmake_minimum_required(VERSION 3.5)
project(depth_camera_description)

find_package(ament_cmake REQUIRED)

# Install the URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
