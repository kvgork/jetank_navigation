#!/usr/bin/env python3
"""
Launch file for laser scan sources.

Supports two LiDAR source modes:
  - pointcloud: PointCloud2 to LaserScan conversion (stereo camera)
  - rplidar: Hardware RPLidar C1M1 sensor (use rplidar.launch.py directly for standalone use)

Usage:
  # Default (stereo camera pointcloud conversion)
  ros2 launch jetank_navigation laser_scan_converter.launch.py

  # Hardware RPLidar (standalone)
  ros2 launch jetank_navigation rplidar.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for laser scan sources."""

    lidar_source = LaunchConfiguration('lidar_source')

    declare_lidar_source = DeclareLaunchArgument(
        'lidar_source',
        default_value='pointcloud',
        description='LiDAR source: "pointcloud" (stereo camera) or "rplidar" (hardware)')

    laser_data_config = PathJoinSubstitution([
        FindPackageShare('jetank_navigation'), 'config', 'laser_data.yaml'
    ])

    # PointCloud2 to LaserScan conversion node (stereo camera)
    laser_data_node = Node(
        package='jetank_navigation',
        executable='laser_data_node',
        name='laser_data_node',
        output='screen',
        parameters=[laser_data_config],
        condition=IfCondition(PythonExpression(["'", lidar_source, "' == 'pointcloud'"]))
    )

    # RPLidar hardware (delegates to dedicated launch file)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jetank_navigation'), 'launch', 'rplidar.launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", lidar_source, "' == 'rplidar'"]))
    )

    return LaunchDescription([
        declare_lidar_source,
        laser_data_node,
        rplidar_launch,
    ])
