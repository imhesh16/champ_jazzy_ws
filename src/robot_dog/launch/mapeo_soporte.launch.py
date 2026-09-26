"""Mapeo con el robot FISICO quieto sobre su soporte (sin caminar).

Levanta, en la Raspberry Pi:
  - el driver del LiDAR T-mini Plus (/dev/lidar -> /scan, y la TF
    base_link -> laser_frame que publica su propio launch),
  - una TF fija odom -> base_link (el robot no se mueve, asi que la
    odometria es la identidad),
  - SLAM Toolbox en modo online asincrono (config/slam_real.yaml).

Con el robot quieto, el mapa cubre lo que el LiDAR ve desde ese punto.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot_dog_share = get_package_share_directory("robot_dog")
    ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_share, "launch", "ydlidar_launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(
                robot_dog_share, "config", "ydlidar_tmini_plus.yaml"
            ),
        }.items(),
    )

    odom_fija = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_fija",
        arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
    )

    # Se usa el launch oficial de SLAM Toolbox porque su nodo es de ciclo de
    # vida (lifecycle) y ese launch lo configura y activa automaticamente.
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": os.path.join(
                robot_dog_share, "config", "slam_real.yaml"
            ),
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription([lidar, odom_fija, slam])
