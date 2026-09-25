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
    models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joint_trajectory_bridge.py
    scripts/imu_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
