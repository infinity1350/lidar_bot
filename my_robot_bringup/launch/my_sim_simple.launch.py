#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    package_name = 'my_robot_bringup'
    
    # Set Gazebo model paths
    models_path = os.path.join(get_package_share_directory(package_name), 'models')
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
    
    # World file argument
    default_world = os.path.join(get_package_share_directory(package_name), 'world', 'small_house.world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to world file'
    )
    
    # Map yaml argument for navigation
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(
            get_package_share_directory('my_robot_navigation'), 'maps', 'small_house', 'map.yaml'
        ),
        description='Full path to map yaml file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map_yaml')

    # Robot description with simulation flag
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]),
        " ",
        "use_sim_time:=true",
        " ",
        "is_sim:=true",
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("my_robot_description"), "rviz", "urdf_config.rviz"
    ])

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Include Gazebo Ignition launch (SIMPLIFIED APPROACH FROM EXAMPLE)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            )
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
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-z', '0.1'
        ],
        output='screen'
    )

    # twist_mux configuration
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    # Bridge configuration
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_ign_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # === NAVIGATION COMPONENTS (with delays) ===
    
    # Static transform: map -> odom (identity, used as fallback before AMCL activates)
    # robot_state_publisher already handles base_footprint->base_link->laser_frame from the URDF.
    # The old 'base_footprint'->'my_robot/base_footprint/laser' transform was wrong and useless.
    static_tf_laser = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_map_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # Map Server (Lifecycle Node)
    map_server = TimerAction(
        period=6.0,
        actions=[
            LifecycleNode(
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
        ]
    )

    # AMCL (Lifecycle Node)
    amcl = TimerAction(
        period=7.0,
        actions=[
            LifecycleNode(
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
                    'base_frame_id': 'base_footprint',
                    'beam_skip_distance': 0.5,
                    'beam_skip_error_threshold': 0.9,
                    'beam_skip_threshold': 0.3,
                    'do_beamskip': False,
                    'global_frame_id': 'map',
                    'lambda_short': 0.1,
                    'laser_likelihood_max_dist': 2.0,
                    'laser_max_range': 12.0,
                    'laser_min_range': 0.12,
                    'laser_model_type': 'likelihood_field',
                    'max_beams': 60,
                    'max_particles': 2000,
                    'min_particles': 500,
                    'odom_frame_id': 'odom',
                    'pf_err': 0.05,
                    'pf_z': 0.99,
                    'recovery_alpha_fast': 0.0,
                    'recovery_alpha_slow': 0.0,
                    'resample_interval': 1,
                    'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                    'save_pose_rate': 0.5,
                    'sigma_hit': 0.2,
                    'tf_broadcast': True,
                    'transform_tolerance': 2.0,
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
        ]
    )

    # Lifecycle Manager
    lifecycle_manager = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']
                }]
            )
        ]
    )

    # Launch them all!
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_arg,
        map_yaml_arg,
        
        # Core nodes (start immediately)
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        ros_ign_bridge,
        twist_mux,
        rviz_node,
        
        # Navigation nodes (with delays)
        static_tf_laser,
        map_server,
        amcl,
        lifecycle_manager,
    ])