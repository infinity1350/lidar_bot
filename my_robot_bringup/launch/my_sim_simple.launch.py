#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set model paths
    models_path = os.path.join(get_package_share_directory("my_robot_bringup"), 'models')
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path + ':' + os.environ['IGN_GAZEBO_RESOURCE_PATH']
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path

    # World file
    world_file = os.path.join(get_package_share_directory("my_robot_bringup"), 'world', 'small_house.world')
    
    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]),
        " use_sim_time:=true"
    ])
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': f'-r -v4 {world_file}'}.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # Spawn robot - spawn at ground level, let physics settle it properly
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot', '-z', '0.2', '-x', '0.0', '-y', '0.0'],
        output='screen'
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        output='screen'
    )
    
    # Delayed spawn
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        delayed_spawn,
    ])