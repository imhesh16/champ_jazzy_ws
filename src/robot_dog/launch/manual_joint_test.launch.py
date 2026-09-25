"""Visualizacion manual del modelo del robot, SIN CHAMP.

Muestra el URDF en RViz con sliders (joint_state_publisher_gui) para
mover cada una de las 12 articulaciones a mano, de forma independiente
del generador de marcha. Sirve como referencia visual de "verdad
conocida" para comparar un angulo especifico contra lo que hace la
pata real, sin la complejidad de un gait completo caminando.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_dog_share = get_package_share_directory('robot_dog')

    xacro_file = os.path.join(
        robot_dog_share,
        'urdf',
        'spot_micro.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': False
            }
        ]
    )

    # Sliders manuales para las 12 articulaciones (publica /joint_states).
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {
                'use_sim_time': False
            }
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
