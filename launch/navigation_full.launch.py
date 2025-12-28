#!/usr/bin/env python3
"""
Full navigation system launch file for JeTank robot.

Launches complete autonomous navigation stack:
  - Stereo camera perception
  - PointCloud2 to LaserScan conversion
  - Motor controller (odometry)
  - Robot state publisher (URDF/TF)
  - SLAM (mapping mode) OR Nav2 (navigation mode)
  - RViz visualization

Usage:
  # For mapping (SLAM):
  ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

  # For navigation (with existing map):
  ros2 launch jetank_navigation navigation_full.launch.py mode:=nav2 map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Generate launch description for full navigation system."""

    # Get package directories
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')
    pkg_jetank_main = get_package_share_directory('jetank_ros2_main')

    # Launch configuration variables
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    # Declare launch arguments
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Navigation mode: "slam" for mapping or "nav2" for navigation')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file (required for nav2 mode)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz visualization')

    # 1. Robot state publisher (URDF/TF tree)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'urdf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Stereo camera perception
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'stereo_camera.launch.py')
        )
    )

    # 3. Motor controller (odometry)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'motor_controller.launch.py')
        )
    )

    # 4. PointCloud2 to LaserScan conversion
    laser_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'laser_scan_converter.launch.py')
        )
    )

    # 5a. SLAM Toolbox (mapping mode)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5b. Nav2 stack (navigation mode)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav2'"])),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 6. RViz (optional)
    rviz_config_file = os.path.join(pkg_jetank_nav, 'rviz', 'navigation.rviz')

    rviz_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/rviz_launch.py'
            ]),
            launch_arguments={
                'rviz_config': rviz_config_file,
                'use_sim_time': use_sim_time,
            }.items(),
        )
    ], condition=IfCondition(use_rviz))

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)

    # Add all launches
    ld.add_action(urdf_launch)
    ld.add_action(camera_launch)
    ld.add_action(motor_launch)
    ld.add_action(laser_scan_launch)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_launch)

    return ld
