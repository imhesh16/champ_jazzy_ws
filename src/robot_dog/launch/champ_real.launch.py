"""Arranca el stack de CHAMP en el robot FISICO real, sin Gazebo.

Equivalente a champ_gazebo.launch.py pero para hardware real: levanta
robot_state_publisher y quadruped_controller_node (el que calcula la
marcha real y publica la trayectoria articular), usando el reloj de
pared normal (use_sim_time=false) en vez del reloj simulado de
Gazebo. No lanza ninguna simulacion, ninguna fisica simulada, ningun
sensor simulado -- solo el calculo de cinematica/marcha de CHAMP.

Requiere, en paralelo (no los levanta este launch):
  - El agente micro-ROS corriendo y el ESP32 conectado.
  - El puente joint_trajectory_bridge.py
    (ros2 run robot_dog joint_trajectory_bridge.py)

No borra ni reemplaza champ_gazebo.launch.py -- ese sigue siendo el
punto de entrada para simulacion en Gazebo (en esta PC). Este es el
equivalente para cuando NO hay simulacion de por medio (ej. en la
Raspberry Pi a bordo del robot real).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_dog_share = get_package_share_directory("robot_dog")

    xacro_file = os.path.join(
        robot_dog_share,
        "urdf",
        "spot_micro.urdf.xacro",
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

    # Usa gait_real.yaml (no gait.yaml) -- valores separados para el
    # robot fisico real (mas lento, mas agachado) sin afectar como se
    # ve la simulacion en champ_gazebo.launch.py, que sigue usando
    # gait.yaml con los valores "naturales" originales.
    gait_config = os.path.join(
        robot_dog_share,
        "config",
        "gait_real.yaml",
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]),
        value_type=str,
    )

    # Publica robot_description y las transformaciones TF, con el
    # reloj real del sistema (no hay /clock simulado sin Gazebo).
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )

    # Nodo principal de CHAMP: calcula la marcha real y publica los
    # 12 angulos articulares como JointTrajectory, ademas de la
    # odometria (a partir de la cinematica de las patas, no de
    # ningun sensor simulado).
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
                "use_sim_time": False,
                "gazebo": False,

                # Sin Gazebo/joint_state_broadcaster no hay quien mas
                # publique /joint_states -- lo hace CHAMP directo.
                "publish_joint_states": True,

                # CHAMP publica los 12 angulos como JointTrajectory,
                # igual que en la version de Gazebo -- el puente hacia
                # el ESP32 se suscribe a este mismo topico.
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

    return LaunchDescription([
        robot_state_publisher,
        quadruped_controller,
    ])
