#!/usr/bin/env python3
"""Nav2 navigation-only stack (NO localization).

Same as nav2_bringup.launch.py but without map_server + amcl — localization and
the map come from slam_toolbox (map -> odom + /map). Used by slam_nav2.launch.py.

Key difference from upstream nav2_bringup/navigation_launch.py: the
lifecycle_manager is given **bond_timeout: 0.0**. With the default 4 s bond
timeout, a node that misses its heartbeat under heavy load (Gazebo GUI + RViz +
SLAM + Nav2 on one machine) is declared unresponsive and the manager tears the
whole stack down a few seconds after bringup — which made bt_navigator go
active then inactive and reject all goals. bond_timeout 0.0 disables that.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    # No map_server / amcl — slam_toolbox provides /map and map->odom.
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use the simulation clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_jetank_nav, 'config', 'nav2', 'nav2_params.yaml'),
        description='Nav2 parameters file')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically start the nav2 lifecycle nodes')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')

    load_nodes = GroupAction([
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # velocity_smoother: takes the controller's cmd_vel_nav and republishes
        # the smoothed result on /cmd_vel (which the cmd_vel bridge relays to the
        # diff_drive_controller in sim).
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),
                                     ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes},
                        {'bond_timeout': 0.0}]),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)
    return ld
