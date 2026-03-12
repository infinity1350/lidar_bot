#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    
    # Map yaml argument for navigation
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
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

    # Diff drive controller spawner
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # === NAVIGATION COMPONENTS ===
    
    # Static transform: base_footprint -> laser frame
    # This bridges the gap between Gazebo's namespaced laser and AMCL's expected frame
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'my_robot/base_footprint/laser'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Map Server (Lifecycle Node)
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time
        }]
    )

    # AMCL (Lifecycle Node) - Provides map->odom transform
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_footprint',  # Use non-namespaced frame
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 12.0,  # YDLidar X4 Pro max range
            'laser_min_range': 0.12,  # YDLidar X4 Pro min range
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',  # Use non-namespaced frame
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,  # This publishes map->odom transform
            'transform_tolerance': 2.0,  # Increased tolerance for Gazebo
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': 'scan',
            'set_initial_pose': True,
            'initial_pose': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0
            }
        }]
    )

    # Lifecycle Manager for AUTOMATIC state transitions
    # This will automatically configure and activate both map_server and amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,  # CRITICAL: This auto-configures and activates nodes
            'node_names': ['map_server', 'amcl']
        }]
    )

    # === DELAYED SPAWNERS ===
    
    # Delayed RViz - starts after joint_state_broadcaster
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delayed robot controller
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delayed spawn
    delay_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[spawn_entity],
        )
    )

    # Start navigation components after controllers are ready
    # The lifecycle_manager will automatically transition map_server and amcl
    delay_navigation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg='Controllers ready, starting navigation components...'),
                TimerAction(
                    period=2.0,  # Give 2 seconds for everything to stabilize
                    actions=[
                        LogInfo(msg='Publishing static laser transform...'),
                        static_tf_laser,
                    ]
                ),
                TimerAction(
                    period=3.0,  # Give 3 seconds for everything to stabilize
                    actions=[
                        LogInfo(msg='Launching map_server...'),
                        map_server,
                    ]
                ),
                TimerAction(
                    period=4.0,  # Start AMCL 1 second after map_server
                    actions=[
                        LogInfo(msg='Launching AMCL...'),
                        amcl,
                    ]
                ),
                TimerAction(
                    period=5.0,  # Start lifecycle_manager last
                    actions=[
                        LogInfo(msg='Launching lifecycle_manager (will auto-activate nodes)...'),
                        lifecycle_manager,
                    ]
                )
            ],
        )
    )

    # === FINAL NODE LIST ===
    nodes = [
        # Arguments
        use_sim_time_arg,
        world_arg,
        map_yaml_arg,
        
        # Core nodes
        gazebo,
        robot_state_pub_node,
        bridge,
        control_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        
        # Event handlers
        delay_rviz,
        delay_robot_controller,
        delay_spawn,
        delay_navigation,
    ]

    return LaunchDescription(nodes)