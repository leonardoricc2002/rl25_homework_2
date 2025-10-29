from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from xacro import process_file
from xml.dom import minidom

def generate_launch_description():
    description_pkg = get_package_share_directory('armando_description')
    gazebo_pkg = get_package_share_directory('armando_gazebo')

    urdf_path = os.path.join(description_pkg, 'urdf', 'arm.urdf.xacro')
    camera_xacro_path = os.path.join(gazebo_pkg, 'urdf', 'armando_camera.xacro')
    controller_config = os.path.join(description_pkg, 'config', 'armando_controllers.yaml')

    # Espansione XACRO
    robot_xml = process_file(urdf_path).toprettyxml(indent='  ')
    camera_xml = process_file(camera_xacro_path).toprettyxml(indent='  ')

    robot_dom = minidom.parseString(robot_xml)
    camera_dom = minidom.parseString(camera_xml)
    for node in camera_dom.documentElement.childNodes:
        robot_dom.documentElement.appendChild(node)
    robot_description = robot_dom.toxml()

    # Argomento controller_mode
    declare_controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='position',
        description='Controller mode to use: position or trajectory'
    )

    # Nodi base
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'armando', '-x', '0', '-y', '0', '-z', '0.0'],
        output='screen'
    )

    # Controller manager
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
        output='screen',
        condition=LaunchConfigurationEquals('controller_mode', 'position')
    )

    load_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager', '-p', controller_config],
        output='screen',
        condition=LaunchConfigurationEquals('controller_mode', 'trajectory')
    )

    register_load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    register_load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_position_controller, load_trajectory_controller],
        )
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    arm_controller_node = Node(
        package='armando_controller',
        executable='arm_controller_node',
        name='arm_controller_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('controller_mode')
        }]
    )

    return LaunchDescription([
        declare_controller_mode_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        register_load_jsb,
        register_load_controllers,
        ros_gz_bridge,
        arm_controller_node
    ])
