#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # ARCHIVO DE PARÁMETROS
    # =====================================================

    robot_dog_share = get_package_share_directory("robot_dog")

    # Variante para simulacion: se lee directo del codigo fuente (no requiere recompilar)
    nav2_params = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "config",
        "nav2_params_sim.yaml",
    )

    # =====================================================
    # REMAPEOS COMUNES DE TF
    # =====================================================

    tf_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    # =====================================================
    # NODOS LIFECYCLE
    #
    # Collision Monitor se omite temporalmente.
    # =====================================================

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
        "waypoint_follower",
    ]

    # =====================================================
    # CONTROLLER SERVER
    #
    # Salida:
    #   /cmd_vel_nav
    # =====================================================

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=(
            tf_remappings
            + [
                ("cmd_vel", "cmd_vel_nav"),
            ]
        ),
    )

    # =====================================================
    # SMOOTHER SERVER
    # Suaviza la geometría de la trayectoria.
    # =====================================================

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=tf_remappings,
    )

    # =====================================================
    # PLANNER SERVER
    # =====================================================

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=tf_remappings,
    )

    # =====================================================
    # BEHAVIOR SERVER
    #
    # Sus comandos también ingresan al velocity_smoother.
    # =====================================================

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=(
            tf_remappings
            + [
                ("cmd_vel", "cmd_vel_nav"),
            ]
        ),
    )

    # =====================================================
    # BT NAVIGATOR
    # =====================================================

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=tf_remappings,
    )

    # =====================================================
    # WAYPOINT FOLLOWER
    # =====================================================

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=tf_remappings,
    )

    # =====================================================
    # VELOCITY SMOOTHER
    #
    # Entrada:
    #   /cmd_vel_nav
    #
    # Salida directa a CHAMP:
    #   /cmd_vel
    # =====================================================

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
        remappings=(
            tf_remappings
            + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ]
        ),
    )

    # =====================================================
    # LIFECYCLE MANAGER
    # =====================================================

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 4.0,
                "attempt_respawn_reconnection": True,
                "bond_respawn_max_duration": 10.0,
                "node_names": lifecycle_nodes,
            }
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
    )

    logging_buffer = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM",
        "1",
    )

    return LaunchDescription([
        logging_buffer,

        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        velocity_smoother,
        bt_navigator,
        waypoint_follower,

        lifecycle_manager_navigation,
    ])
