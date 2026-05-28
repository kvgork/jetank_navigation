#!/usr/bin/env python3
"""SLAM Toolbox launch file for the JeTank robot.

Launches SLAM Toolbox in mapping mode against the RPLidar /scan topic.
The generated map can later be saved with the ``save_map.sh`` helper and
re-used for navigation with AMCL.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SLAM Toolbox."""

    pkg_jetank_nav = get_package_share_directory('jetank_navigation')
    default_slam_config = os.path.join(
        pkg_jetank_nav, 'config', 'slam', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_config,
        description='Full path to the SLAM Toolbox parameters file')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/scan')],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(slam_toolbox_node)
    return ld
