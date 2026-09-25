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
    # RUTAS DE PAQUETES
    # =====================================================

    robot_dog_share = get_package_share_directory("robot_dog")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

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

    slam_params_file = os.path.join(
        slam_toolbox_share,
        "config",
        "mapper_params_online_async.yaml",
    )

    # =====================================================
    # RECURSOS DE GAZEBO
    # =====================================================

    gazebo_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(robot_dog_share),
    )

    # =====================================================
    # DESCRIPCIÓN DEL ROBOT
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
    # RVIZ - VERSIÓN ROBUSTA CON TIMER Y RESPAWN
    # =====================================================

    # Definimos el nodo de RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        additional_env={"QT_QPA_PLATFORM": "xcb"},
        respawn=True,  # Reintenta si falla
        respawn_delay=2.0,
        emulate_tty=True,  # Mejor salida en terminal
    )

    # RViz se ejecutará después de 3 segundos para asegurar que todo esté listo
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node],
    )

    # =====================================================
    # BRIDGE DEL RELOJ
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
    # BRIDGE DE TF DE GAZEBO
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
    # INSERTAR EL ROBOT EN GAZEBO
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
    # SLAM TOOLBOX
    # =====================================================

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share,
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    # =====================================================
    # SECUENCIA DE ARRANQUE
    # =====================================================

    # Esperar 4 segundos para cargar el mundo antes de insertar el robot
    delayed_spawn = TimerAction(
        period=4.0,
        actions=[spawn_robot],
    )

    # Cuando termine de aparecer el robot, iniciar joint_state_broadcaster
    start_joint_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Cuando joint_state_broadcaster esté listo, iniciar el controlador de piernas
    start_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[trajectory_controller_spawner],
        )
    )

    # Cuando las articulaciones estén controladas, iniciar CHAMP y SLAM
    start_champ_and_slam = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_controller_spawner,
            on_exit=[
                quadruped_controller,
                slam_toolbox_launch,
            ],
        )
    )

    # =====================================================
    # LANZAMIENTO COMPLETO
    # =====================================================

    return LaunchDescription([
        # Recursos y servicios
        gazebo_resource_path,
        robot_state_publisher,
        
        # Bridges
        clock_bridge,
        lidar_bridge,
        odom_bridge,
        tf_bridge,
        
        # Interfaces gráficas
        gazebo,
        delayed_rviz,  # RViz se abre automáticamente después de 3 segundos
        
        # Secuencia del robot
        start_joint_broadcaster,
        start_trajectory_controller,
        start_champ_and_slam,
        delayed_spawn,
    ])