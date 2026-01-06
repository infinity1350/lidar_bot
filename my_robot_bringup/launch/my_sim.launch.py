#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    EmitEvent,
    LogInfo,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
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
    
    # NEW: Map yaml argument for navigation
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/optimus/bumperbot_ws/src/bumperbot_mapping/maps/small_house/map.yaml',
        description='Full path to map yaml file'
    )

    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')

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
            'ign_args': ['-r -v 4 --render-engine ogre2 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description', '-z', '0.1'],
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

    # RViz (from original code)
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

    # === NEW NAVIGATION COMPONENTS ===
    
    # Static TF: map -> odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Map Server (Lifecycle Node)
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',  # Required parameter in ROS2 Humble
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time
        }]
    )

    # === ORIGINAL DELAYED SPAWNERS ===
    
    # Delayed RViz (from original code) - starts after joint_state_broadcaster
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delayed robot controller (from original code)
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delayed spawn (from original code)
    delay_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[spawn_entity],
        )
    )

    # === NEW: Start navigation components after controllers are ready ===
    
    delay_navigation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg='Controllers ready, starting navigation components...'),
                TimerAction(
                    period=2.0,
                    actions=[
                        static_tf_map_odom,
                        map_server,
                    ]
                )
            ],
        )
    )

    # === NEW: Map server lifecycle management ===
    
    # Configure map_server after it starts
    configure_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.name == 'map_server',
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.name == 'map_server',
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[
                LogInfo(msg='Map server started, configuring...'),
                TimerAction(
                    period=2.0,
                    actions=[configure_map_server]
                )
            ]
        )
    )

    activate_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[
                        LogInfo(msg='Activating map server...'),
                        activate_map_server
                    ]
                )
            ]
        )
    )

    # === FINAL NODE LIST ===
    nodes = [
        # Arguments
        use_sim_time_arg,
        world_arg,
        map_yaml_arg,  # NEW
        
        # Original nodes
        gazebo,
        robot_state_pub_node,
        bridge,
        control_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        
        # Original event handlers
        delay_rviz,
        delay_robot_controller,
        delay_spawn,
        
        # NEW: Navigation components and lifecycle management
        delay_navigation,
        configure_event_handler,
        activate_event_handler,
    ]

    return LaunchDescription(nodes)