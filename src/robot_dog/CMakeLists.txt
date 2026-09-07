cmake_minimum_required(VERSION 3.8)

project(robot_dog)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    config
    launch
    rviz
    worlds
    mapas_robot
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
