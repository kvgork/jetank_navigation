#!/usr/bin/env python3
"""
Launch file for hardware LiDAR laser scan.

Launches the RPLidar hardware driver which publishes directly to /scan.

Usage:
  ros2 launch jetank_navigation lidar.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_navigation'), 'launch', 'rplidar.launch.py'])
        )
    )

    return LaunchDescription([rplidar_launch])
