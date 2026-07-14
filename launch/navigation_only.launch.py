#!/usr/bin/env python3
"""
Nav2 navigation-only stack (NO localization).

Thin wrapper around nav2_bringup.launch.py with ``use_localization:=False``:
no map_server + amcl — localization and the map come from slam_toolbox
(map -> odom + /map). Used by slam_nav2.launch.py.

The wrapper keeps a stable entry point for callers and a sim-friendly
``use_sim_time`` default (True here, False in nav2_bringup.launch.py).
See nav2_bringup.launch.py for the bond_timeout: 0.0 rationale.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')

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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={
            'use_localization': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(nav2)
    return ld
