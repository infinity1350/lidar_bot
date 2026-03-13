#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (true for Gazebo, false for real robot)',
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/optimus/cntrl_ws/src/my_robot_navigation/maps/small_house/map.yaml',
        description='Full path to the map yaml file',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('my_robot_navigation'),
            'config', 'nav2_params.yaml'
        ),
        description='Full path to nav2 params yaml file',
    )

    # ── Resolved configurations ───────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map_yaml')
    params_file  = LaunchConfiguration('params_file')

    # ── Nav2 nodes ────────────────────────────────────────────────────────────

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        # Nav2 outputs cmd_vel; remap to nav_vel so twist_mux handles priority
        remappings=[('cmd_vel', 'nav_vel')],
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',
        output='screen',
        remappings=[
            ('cmd_vel', 'nav_vel'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── Lifecycle manager ─────────────────────────────────────────────────────
    # Automatically configures and activates all Nav2 nodes in order
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                # 'map_server',
                # 'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    # ── Assemble ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        # map_yaml_arg,
        params_file_arg,
        # map_server,
        # amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])