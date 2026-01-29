cmake_minimum_required(VERSION 3.22)
project(so_arm100_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install environment hooks for Gazebo to find the models
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

install(DIRECTORY urdf mjcf control rviz launch models DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY SO-ARM100/Simulation/
        DESTINATION share/${PROJECT_NAME}/models/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME})

ament_package()
