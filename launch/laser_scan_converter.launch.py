#!/usr/bin/env python3
"""
Launch file for PointCloud2 to LaserScan conversion node.

This launch file starts the laser_data_node which converts stereo camera
point cloud data into a 2D laser scan for navigation and SLAM.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for laser scan converter."""

    # Path to configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare('jetank_navigation'),
        'config',
        'laser_data.yaml'
    ])

    # Laser data conversion node
    laser_data_node = Node(
        package='jetank_navigation',
        executable='laser_data_node',
        name='laser_data_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Remap topics if needed
            # ('/stereo_camera/points', '/points'),
            # ('/scan', '/scan'),
        ]
    )

    return LaunchDescription([
        laser_data_node
    ])
