cmake_minimum_required(VERSION 3.22)
project(so_arm100_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY urdf mjcf meshes control rviz launch DESTINATION share/${PROJECT_NAME}/)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME})

ament_package()
