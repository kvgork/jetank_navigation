#!/usr/bin/env python3
"""
Launch file for laser scan sources.

Supports two LiDAR source modes:
  - pointcloud: PointCloud2 to LaserScan conversion (stereo camera)
  - rplidar: Hardware RPLidar C1M1 sensor

Usage:
  # Default (stereo camera pointcloud conversion)
  ros2 launch jetank_navigation laser_scan_converter.launch.py

  # Hardware RPLidar
  ros2 launch jetank_navigation laser_scan_converter.launch.py lidar_source:=rplidar
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for laser scan sources."""

    # Launch configuration
    lidar_source = LaunchConfiguration('lidar_source')

    # Declare launch arguments
    declare_lidar_source = DeclareLaunchArgument(
        'lidar_source',
        default_value='pointcloud',
        description='LiDAR source: "pointcloud" (stereo camera) or "rplidar" (hardware)')

    # Path to configuration files
    laser_data_config = PathJoinSubstitution([
        FindPackageShare('jetank_navigation'),
        'config',
        'laser_data.yaml'
    ])

    rplidar_config = PathJoinSubstitution([
        FindPackageShare('jetank_navigation'),
        'config',
        'rplidar_c1m1.yaml'
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

    # RPLidar hardware node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[rplidar_config],
        condition=IfCondition(PythonExpression(["'", lidar_source, "' == 'rplidar'"]))
    )

    return LaunchDescription([
        declare_lidar_source,
        laser_data_node,
        rplidar_node
    ])
