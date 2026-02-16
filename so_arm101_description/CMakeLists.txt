cmake_minimum_required(VERSION 3.22)
project(so_arm101_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install environment hooks for Gazebo to find the models
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

install(DIRECTORY urdf control meshes mjcf rviz launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
