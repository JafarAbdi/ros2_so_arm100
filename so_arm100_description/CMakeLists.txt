cmake_minimum_required(VERSION 3.22)
project(so_arm100_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install environment hooks for Gazebo to find the models
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

install(DIRECTORY urdf mjcf meshes control rviz launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
