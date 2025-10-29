import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Trova la directory del pacchetto
    pkg_armando_description = get_package_share_directory('armando_description')
    
    # 2. Definisci i percorsi dei file
    urdf_path = os.path.join(pkg_armando_description, 'urdf', 'arm.urdf')
    # Configurazione Rviz (richiesta per il punto 1.b)
    rviz_config_path = os.path.join(pkg_armando_description, 'config', 'armando.rviz')
    
    # 3. Definisci le configurazioni di lancio
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)

    # Leggi il contenuto dell'URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Nodo 1: robot_state_publisher (Carica l'URDF e pubblica i frame)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Nodo 2: joint_state_publisher (Permette di muovere le giunture con gli slider)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui', 
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    # Nodo 3: Rviz2 (Visualizza il robot)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Carica il file di configurazione armando.rviz (punto 1.b)
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        # Argomento per il percorso del file Rviz
        DeclareLaunchArgument(
            name='rviz_config_file', 
            default_value=rviz_config_path,
            description='Percorso del file di configurazione Rviz'
        ),
        
        # Avvio dei nodi
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
