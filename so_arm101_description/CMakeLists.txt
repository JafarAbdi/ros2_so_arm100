cmake_minimum_required(VERSION 3.22)
project(so_arm101_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY urdf control meshes mjcf rviz launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
