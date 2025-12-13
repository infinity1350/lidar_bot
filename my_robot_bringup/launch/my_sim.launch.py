#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set Gazebo model paths
    models_path = os.path.join(get_package_share_directory("my_robot_bringup"), 'models')
    gazebo_models_path = os.path.expanduser('~/.gazebo/models')
    
    system_model_paths = [
        '/usr/share/gazebo-11/models',
        '/usr/share/gazebo/models', 
        '/opt/ros/humble/share/gazebo_plugins/worlds/../models',
        '/usr/share/ign-gazebo-*/models'
    ]
    
    resource_paths = [models_path, gazebo_models_path]
    resource_paths.extend([p for p in system_model_paths if os.path.exists(p)])
    resource_path_str = ':'.join(resource_paths)
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = resource_path_str + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = resource_path_str
        
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = resource_path_str + ':' + os.environ['IGN_GAZEBO_RESOURCE_PATH']
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = resource_path_str
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = resource_path_str + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = resource_path_str

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    default_world = os.path.join(get_package_share_directory("my_robot_bringup"), 'world', 'small_house.world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to world file (.world)'
    )

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]),
        " ",
        "use_sim_time:=true",
    ])
    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("my_robot_bringup"),
        "config",
        "my_robot_controllers.yaml",
    ])

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("my_robot_description"), "rviz", "urdf_config.rviz"
    ])

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'ign_args': ['-r -v 4 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot', '-z', '0.1'],
        output='screen'
    )

    # Bridge configuration
    bridge_params = os.path.join(get_package_share_directory("my_robot_bringup"), 'config', 'gz_bridge.yaml')
    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Control node with remapping
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {'use_sim_time': True}],
        remappings=[('/diff_drive_controller/cmd_vel', '/cmd_vel')],
        output="both"
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff drive controller spawner (simple, no extra args)
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # Delayed spawners
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[spawn_entity],
        )
    )

    nodes = [
        use_sim_time_arg,
        world_arg,
        gazebo,
        robot_state_pub_node,
        bridge,
        control_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_rviz,
        delay_robot_controller,
        delay_spawn,
    ]

    return LaunchDescription(nodes)