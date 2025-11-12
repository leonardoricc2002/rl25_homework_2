# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, OrSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --------------------- DECLARE ARGUMENTS ---------------------
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='iiwa_description',
            description='Package with the controller\'s configuration in "config" folder. Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='iiwa_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa.config.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. If changed than also the namespace in the controllers configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start robot with Moveit2 servoing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='iiwa_arm_controller',
            description='Robot controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.170.10.2',
            description='Robot IP of FRI interface',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_port',
            default_value='30200',
            description='Robot port of FRI interface.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value='initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='position',
            description='Robot command interface [position|velocity|effort].',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )
    iiwa_aruco_world = PathJoinSubstitution(
        [FindPackageShare('iiwa_description'),
         'gazebo/worlds',
         'iiwa_aruco_world.world']
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_args',
            default_value=[iiwa_aruco_world, ' -r -v 1'],
            description='Arguments for gz_sim (Gazebo world to load)',
        )
    )

    # ------------------ INITIALIZE ARGUMENTS ------------------
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')

    # ------------------ ROBOT DESCRIPTION ------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'config', description_file]),
        ' ',
        'prefix:=', prefix, ' ',
        'use_sim:=', use_sim, ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ', 'robot_ip:=', robot_ip, ' ',
        'robot_port:=', robot_port, ' ',
        'initial_positions_file:=', initial_positions_file, ' ',
        'command_interface:=', command_interface, ' ',
        'base_frame_file:=', base_frame_file, ' ',
        'description_package:=', description_package, ' ',
        'runtime_config_package:=', runtime_config_package, ' ',
        'controllers_file:=', controllers_file, ' ',
        'namespace:=', namespace,
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ------------------ PLANNING/SERVOING ------------------
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('iiwa_bringup'), 'launch', 'iiwa_planning.launch.py'])
        ]),
        launch_arguments={
            'description_package': description_package, 
            'description_file': description_file, 
            'prefix': prefix, 
            'start_rviz': start_rviz, 
            'base_frame_file': base_frame_file, 
            'namespace': namespace, 
            'use_sim': use_sim,
        }.items(),
        condition=IfCondition(use_planning),
    )

    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('iiwa_bringup'), 'launch', 'iiwa_servoing.launch.py'])
        ]),
        launch_arguments={
            'description_package': description_package, 
            'description_file': description_file, 
            'prefix': prefix, 
            'base_frame_file': base_frame_file, 
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_servoing),
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'iiwa.rviz']
    )

    # ------------------ NODES ------------------
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description],
        condition=UnlessCondition(OrSubstitution(use_planning, use_sim)),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
        condition=IfCondition(use_sim),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'iiwa',
                   '-allow_renaming', 'true',],
        condition=IfCondition(use_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', [namespace, 'controller_manager']],
    )

    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager', [namespace, 'controller_manager']],
        condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    # ------------------ ARUCO NODE ------------------
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_detect',
        output='screen',
        remappings=[
            ('/image', '/camera/color/image_raw'), 
            ('/camera_info', '/camera/color/camera_info'), 
        ],
        parameters=[{
            'camera_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'marker_size': 0.5,
            'marker_id': 201,
            'camera_frame': 'camera_link', 
            'marker_frame': 'aruco_marker_static_instance/base' 
        }],
        condition=IfCondition(use_sim)
    )
    # ------------------ CAMERA BRIDGE ------------------
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
        condition=IfCondition(use_sim),
    )
    # ------------------ STATIC TF PUBLISHER PER ARUCO ------------------
    aruco_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-1.2', '-0.9', '0.55',      # x, y, z (Position)
                   '-0.4', '1.5708', '0',      # roll, pitch, yaw (Orientation)
                   'iiwa_base',                # Parent Frame
                   'aruco_marker_static_instance/base'], # Child Frame
        condition=IfCondition(use_sim),
    )
    
    # ------------------ ARUCO SET POSE SERVICE BRIDGE ------------------
    aruco_set_pose_service_bridge = Node(
        package='ros_gz_bridge',             
        executable='parameter_bridge',
        name='aruco_set_pose_service_bridge',
        arguments=[ 
 
            '/world/iiwa_aruco_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ], 
        output='screen',
        condition=IfCondition(use_sim),
    )
    
    # ------------------ EVENT HANDLERS ------------------


    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_aruco_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[aruco_node],
        ),
        condition=IfCondition(use_sim),
    )

    # ------------------ RETURN LAUNCH DESCRIPTION ------------------
    nodes = [
        gazebo,
        control_node,
        iiwa_planning_launch,
        iiwa_servoing_launch,
        aruco_tf_publisher, 
        spawn_entity,
        
        aruco_set_pose_service_bridge, 
        
        robot_state_pub_node,
        camera_bridge,
        
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        external_torque_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_aruco_after_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
