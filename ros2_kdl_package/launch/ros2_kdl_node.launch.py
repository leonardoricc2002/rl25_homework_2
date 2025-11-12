from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Percorso al package e al file YAML
    pkg_path = get_package_share_directory('ros2_kdl_package')
    yaml_file = os.path.join(pkg_path, 'config', 'ros2_kdl_node.yaml')
    
    # Argomento per selezionare il tipo di controller
    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='velocity_ctrl',
        description='Controller mode: velocity_ctrl or velocity_ctrl_null'
    )

    # Argomento per decidere se avviare automaticamente il nodo
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='If true, the node moves autonomously; if false, waits for Action client'
    )

    # Nodo principale del KDL controller
    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        output='screen',
        parameters=[
            yaml_file,
            {'ctrl_mode': LaunchConfiguration('ctrl')},
            {'auto_start': LaunchConfiguration('auto_start')}
        ],
        # Se necessario, abilita la rimappatura:
        # remappings=[('/joint_states', '/iiwa/joint_states')]
    )

    # TimerAction: ritarda l’avvio di 5 secondi per dare tempo al robot di pubblicare /joint_states
    delayed_start = TimerAction(
        period=5.0,  # secondi di attesa prima di eseguire il nodo
        actions=[ros2_kdl_node]
    )

    return LaunchDescription([
        ctrl_arg,
        auto_start_arg,
        delayed_start
    ])
