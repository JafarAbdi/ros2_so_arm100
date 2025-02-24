cmake_minimum_required(VERSION 3.22)
project(so_arm100_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz launch config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
