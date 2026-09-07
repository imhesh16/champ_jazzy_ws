import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Ruta instalada del paquete robot_dog
    robot_dog_share = get_package_share_directory('robot_dog')

    # Archivo Xacro del robot
    xacro_file = os.path.join(
        robot_dog_share,
        'urdf',
        'spot_micro.urdf.xacro'
    )

    # Archivos de configuración de CHAMP
    joints_config = os.path.join(
        robot_dog_share,
        'config',
        'joints.yaml'
    )

    links_config = os.path.join(
        robot_dog_share,
        'config',
        'links.yaml'
    )

    gait_config = os.path.join(
        robot_dog_share,
        'config',
        'gait.yaml'
    )

    # Procesar el Xacro y convertirlo en robot_description
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file
        ]),
        value_type=str
    )

    # Publica la estructura TF del robot usando el URDF
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

    # Controlador principal de CHAMP
    quadruped_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        name='quadruped_controller_node',
        output='screen',

        parameters=[
            {
                'use_sim_time': False,
                'gazebo': False,

                # CHAMP publicará /joint_states para visualizar en RViz
                'publish_joint_states': True,

                # CHAMP publicará JointTrajectory para las 12 articulaciones
                'publish_joint_control': True,

                # Publicación estimada del contacto de las patas
                'publish_foot_contacts': True,

                # Tópico de salida de las posiciones articulares
                'joint_controller_topic':
                    'all_legs_trajectory_controller/joint_trajectory',

                # Frecuencia interna del controlador
                'loop_rate': 200.0,

                # URDF utilizado por CHAMP para calcular la cinemática
                'urdf': robot_description
            },

            joints_config,
            links_config,
            gait_config
        ],

        # CHAMP escucha cmd_vel/smooth, pero nosotros enviaremos /cmd_vel
        remappings=[
            ('/cmd_vel/smooth', '/cmd_vel')
        ]
    )

    # RViz
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
        quadruped_controller_node,
        rviz_node
    ])