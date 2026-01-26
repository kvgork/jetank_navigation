#!/usr/bin/env python3
"""
SLAM Toolbox launch file for JeTank robot.

Launches SLAM Toolbox in mapping mode to build maps of unknown environments.
The generated map can be saved and used later for navigation with AMCL.

Supports two configurations:
  - pointcloud: Optimized for stereo camera (1.5m range)
  - rplidar: Optimized for RPLidar C1M1 (12m range)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup launch nodes with resolved configuration."""

    # Get package directory
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')

    # Resolve launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_source = LaunchConfiguration('lidar_source').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)

    # Select config based on lidar_source if default params file is used
    default_pointcloud_config = os.path.join(
        pkg_jetank_nav, 'config', 'slam', 'slam_toolbox.yaml')
    default_rplidar_config = os.path.join(
        pkg_jetank_nav, 'config', 'slam', 'slam_toolbox_rplidar.yaml')

    # If using default config, select based on lidar source
    if slam_params_file == default_pointcloud_config:
        if lidar_source == 'rplidar':
            slam_params_file = default_rplidar_config

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    return [slam_toolbox_node]


def generate_launch_description():
    """Generate launch description for SLAM Toolbox."""

    # Get package directory
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_lidar_source_cmd = DeclareLaunchArgument(
        'lidar_source',
        default_value='pointcloud',
        description='LiDAR source: "pointcloud" or "rplidar"')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            pkg_jetank_nav,
            'config',
            'slam',
            'slam_toolbox.yaml'),
        description='Full path to the SLAM Toolbox parameters file')

    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_lidar_source_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add opaque function for dynamic setup
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
