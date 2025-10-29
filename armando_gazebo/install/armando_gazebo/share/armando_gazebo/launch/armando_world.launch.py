from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from xacro import process_file

def generate_launch_description():
    # 1. PERCORSI
    description_pkg = get_package_share_directory('armando_description')
    
    # CORREZIONE 1: Definizione di gazebo_pkg per evitare NameError
    gazebo_pkg = get_package_share_directory('armando_gazebo')
    
    # Percorso al file URDF/XACRO
    urdf_path = os.path.join(description_pkg, 'urdf', 'arm.urdf.xacro') 
    
    # Questa riga ora è pulita, ma non usata nell'espansione XACRO in questo file.
    # camera_xacro_path = os.path.join(gazebo_pkg, 'urdf', 'armando_camera.xacro') 
    
    # Percorso al file YAML dei controller
    controller_config = os.path.join(description_pkg, 'config', 'armando_controllers.yaml')
    
    # 2. ESPANSIONE XACRO
    # NOTA: Assicurati di aver corretto il percorso di inclusione nel file arm.urdf.xacro 
    # come discusso nell'ultima risposta, altrimenti questa riga fallirà ancora!
    # Se hai usato il fix con 'gazebo_pkg_path' come argomento, devi usare 'process_file' con 'mappings' qui.
    robot_description = process_file(urdf_path).toxml()

    # 3. NODI BASE (RSP + Gazebo)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'armando',
            '-x', '0', '-y', '0', '-z', '0.00'
        ],
        output='screen'
    )
    
    # 4. NODI CONTROLLER MANAGER
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '-p', controller_config],
        output='screen'
    )

    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller', '--controller-manager', '/controller_manager', '-p', controller_config],
        output='screen'
    )
    
    # 5. BRIDGE DELLA TELECAMERA (Punto 3c: integrazione)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Mappa il topic dell'immagine da Gazebo al sistema ROS 2
            # Topic: /armando/camera_sensor/image
            '/armando/camera_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        output='screen'
    )

    # 6. EVENT HANDLERS
    register_load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    register_load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_position_controller],
        )
    )

    return LaunchDescription([
        # Lancio dei nodi base
        gazebo,
        robot_state_publisher_node,
        
        # Spawna il robot (trigger per i controller)
        spawn_robot,
        
        # Attiva il bridge della telecamera
        camera_bridge, 
        
        # Carica i controller in sequenza
        register_load_jsb,
        register_load_controllers
    ])
