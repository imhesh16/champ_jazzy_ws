import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # headless:=true corre Gazebo solo como servidor de fisica, sin la
    # ventana 3D (flag nativo "-s" de gz sim). Util en equipos sin GPU
    # compatible con OpenGL 3.3 (ej. Raspberry Pi 5) o sin monitor --
    # lo que realmente hace falta para generar la trayectoria real
    # hacia el puente ESP32 es el servidor, no la GUI.
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="true = Gazebo sin GUI (solo servidor de fisica)",
    )
    headless = LaunchConfiguration("headless")

    robot_dog_share = get_package_share_directory("robot_dog")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # Permite que Gazebo encuentre:
    # model://robot_dog/urdf/stl/archivo.stl
    gazebo_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(robot_dog_share),
    )

    xacro_file = os.path.join(
        robot_dog_share,
        "urdf",
        "spot_micro.urdf.xacro",
    )

    world_file = os.path.join(
        robot_dog_share,
        "worlds",
        "empty_world.sdf",
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

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]),
        value_type=str,
    )

    # Publica robot_description y las transformaciones TF.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # Inicia Gazebo Harmonic con el mundo vacío. Dos variantes segun
    # "headless": con GUI (normal) o solo servidor (flag "-s" nativo
    # de gz sim, sin ventana 3D).
    gz_sim_launch = os.path.join(
        ros_gz_sim_share,
        "launch",
        "gz_sim.launch.py",
    )

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": f"-r -v 3 {world_file}",
        }.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": f"-r -v 3 -s {world_file}",
        }.items(),
        condition=IfCondition(headless),
    )

    # Publica el reloj de Gazebo en ROS 2.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # Inserta el robot desde /robot_description.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot_dog",
        output="screen",
        arguments=[
            "-world", "empty",
            "-name", "robot_dog",
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.25",
            "-allow_renaming", "true",
        ],
    )

    # Publica los estados articulares reales de Gazebo.
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
            "120",
        ],
    )

    # Controlador que recibe JointTrajectory desde CHAMP.
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
            "120",
        ],
    )

    # Nodo principal de CHAMP.
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

                # Gazebo y joint_state_broadcaster publican /joint_states.
                "publish_joint_states": False,

                # CHAMP publica los 12 ángulos como JointTrajectory.
                "publish_joint_control": True,

                "publish_foot_contacts": True,

                "joint_controller_topic":
                    "all_legs_trajectory_controller/joint_trajectory",

                "loop_rate": 200.0,
                "urdf": robot_description,
            },
        ],
        remappings=[
            ("/cmd_vel/smooth", "/cmd_vel"),
        ],
    )

    # Espera a que Gazebo termine de cargar antes de insertar el robot.
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot],
    )

    # Después de insertar el robot, activa joint_state_broadcaster.
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    # Después activa all_legs_trajectory_controller.
    start_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                trajectory_controller_spawner,
            ],
        )
    )

    # Después inicia CHAMP.
    start_champ = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_controller_spawner,
            on_exit=[
                quadruped_controller,
            ],
        )
    )

    return LaunchDescription([
        headless_arg,

        # Debe ejecutarse antes de iniciar Gazebo.
        gazebo_resource_path,

        robot_state_publisher,
        gazebo_with_gui,
        gazebo_headless,
        clock_bridge,

        delayed_spawn,
        start_joint_state_broadcaster,
        start_trajectory_controller,
        start_champ,
    ])