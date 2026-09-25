#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =====================================================
    # RUTAS DE LOS PAQUETES
    # =====================================================

    robot_dog_share = get_package_share_directory("robot_dog")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    xacro_file = os.path.join(
        robot_dog_share,
        "urdf",
        "spot_micro.urdf.xacro",
    )

    world_file = os.path.join(
        robot_dog_share,
        "worlds",
        "turtlebot3_world.sdf",
    )

    joints_config = os.path.join(
        robot_dog_share,
        "config",
        "joints.yaml",
    )

    links_config = os.path.join(
        robot_dog_share,
        "config",
        "links.yaml",
    )

    gait_config = os.path.join(
        robot_dog_share,
        "config",
        "gait.yaml",
    )

    amcl_params = os.path.join(
        robot_dog_share,
        "config",
        "amcl.yaml",
    )

    map_file = os.path.join(
        robot_dog_share,
        "mapas_robot",
        "mapa_chico.yaml",
    )

    # =====================================================
    # RECURSOS PARA GAZEBO
    # =====================================================

    gazebo_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(robot_dog_share),
    )

    # =====================================================
    # DESCRIPCIÓN URDF/XACRO DEL ROBOT
    # =====================================================

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]),
        value_type=str,
    )

    # =====================================================
    # ROBOT STATE PUBLISHER
    # Publica base_link -> enlaces del robot y lidar_link
    # =====================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # =====================================================
    # GAZEBO SIM
    # =====================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 3 {world_file}",
        }.items(),
    )

    # =====================================================
    # RVIZ
    # =====================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        additional_env={
            "QT_QPA_PLATFORM": "xcb",
        },
        emulate_tty=True,
    )

    # Se abre cuando Gazebo y los demás nodos ya están iniciando
    delayed_rviz = TimerAction(
        period=12.0,
        actions=[
            rviz_node,
        ],
    )

    # =====================================================
    # BRIDGE DEL RELOJ DE GAZEBO
    # =====================================================

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # =====================================================
    # BRIDGE DEL LIDAR
    # Gazebo -> ROS 2 /scan
    # =====================================================

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        output="screen",
        arguments=[
            (
                "/lidar_link/lidar_sensor/scan"
                "@sensor_msgs/msg/LaserScan"
                "[gz.msgs.LaserScan"
            ),
        ],
        remappings=[
            (
                "/lidar_link/lidar_sensor/scan",
                "/scan",
            ),
        ],
    )

    # =====================================================
    # BRIDGE DE ODOMETRÍA
    # Gazebo -> ROS 2 /odom
    # =====================================================

    odom_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odom_bridge",
        output="screen",
        arguments=[
            (
                "/model/robot_dog/odometry"
                "@nav_msgs/msg/Odometry"
                "[gz.msgs.Odometry"
            ),
        ],
        remappings=[
            (
                "/model/robot_dog/odometry",
                "/odom",
            ),
        ],
    )

    # =====================================================
    # BRIDGE DE TF
    # Aporta principalmente odom -> base_link
    # =====================================================

    tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tf_bridge",
        output="screen",
        arguments=[
            (
                "/model/robot_dog/tf"
                "@tf2_msgs/msg/TFMessage"
                "[gz.msgs.Pose_V"
            ),
        ],
        remappings=[
            (
                "/model/robot_dog/tf",
                "/tf",
            ),
        ],
    )

    # =====================================================
    # INSERTAR ROBOT EN GAZEBO
    # =====================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot_dog",
        output="screen",
        arguments=[
            "-world",
            "default",

            "-name",
            "robot_dog",

            "-topic",
            "/robot_description",

            "-x",
            "0.0",

            "-y",
            "-2.0",

            # Aparece sobre el suelo y luego se estabiliza
            "-z",
            "0.25",

            "-allow_renaming",
            "true",
        ],
    )

    # =====================================================
    # JOINT STATE BROADCASTER
    # =====================================================

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # =====================================================
    # CONTROLADOR DE LAS 12 ARTICULACIONES
    # =====================================================

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="trajectory_controller_spawner",
        output="screen",
        arguments=[
            "all_legs_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # =====================================================
    # CONTROLADOR CHAMP
    # =====================================================

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        name="quadruped_controller_node",
        output="screen",
        parameters=[
            joints_config,
            links_config,
            gait_config,
            {
                "use_sim_time": True,
                "gazebo": True,
                "hardware_connected": False,

                # /joint_states lo entrega joint_state_broadcaster
                "publish_joint_states": False,

                "publish_joint_control": True,
                "publish_foot_contacts": True,

                "close_loop_odom": True,

                "joint_controller_topic":
                    "all_legs_trajectory_controller/joint_trajectory",

                "loop_rate": 60.0,

                "urdf": robot_description,
            },
        ],
        remappings=[
            (
                "/cmd_vel/smooth",
                "/cmd_vel",
            ),
        ],
    )

    # =====================================================
    # MAP SERVER + AMCL
    # Sustituye completamente a SLAM Toolbox
    # =====================================================

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_share,
                "launch",
                "localization_launch.py",
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": amcl_params,
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    # Espera después de iniciar los controladores
    delayed_localization = TimerAction(
        period=3.0,
        actions=[
            localization_launch,
        ],
    )

    # =====================================================
    # SECUENCIA DE ARRANQUE
    # =====================================================

    # Esperar a que el mundo de Gazebo cargue
    delayed_spawn = TimerAction(
        period=4.0,
        actions=[
            spawn_robot,
        ],
    )

    # Después del spawn, iniciar joint_state_broadcaster
    start_joint_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    # Después iniciar el controlador de las patas
    start_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                trajectory_controller_spawner,
            ],
        )
    )

    # Después iniciar CHAMP y la localización AMCL
    start_champ_and_amcl = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_controller_spawner,
            on_exit=[
                quadruped_controller,
                delayed_localization,
            ],
        )
    )

    # =====================================================
    # LANZAMIENTO COMPLETO
    # =====================================================

    return LaunchDescription([
        gazebo_resource_path,

        # Descripción y TF internos del robot
        robot_state_publisher,

        # Bridges de Gazebo
        clock_bridge,
        lidar_bridge,
        odom_bridge,
        tf_bridge,

        # Simulador y visualizador
        gazebo,
        delayed_rviz,

        # Secuencia de control y localización
        start_joint_broadcaster,
        start_trajectory_controller,
        start_champ_and_amcl,
        delayed_spawn,
    ])
