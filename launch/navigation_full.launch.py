#!/usr/bin/env python3
r"""
Full navigation launch for the JeTank robot.

Launches the complete autonomous-navigation stack against the RPLidar:
  - Robot state publisher (URDF/TF tree)
  - Motor controller (odometry + /cmd_vel sink)
  - IMU (ICM-20948)
  - RPLidar (publishes /scan)
  - SLAM (mapping mode) OR Nav2 (navigation mode)
  - RViz visualisation (uses the nav2_bringup rviz_launch wrapper)

Usage::

    # Build a map
    ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

    # Navigate using a saved map
    ros2 launch jetank_navigation navigation_full.launch.py \
        mode:=nav2 map:=$HOME/maps/jetank_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the full navigation system."""
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')
    pkg_jetank_main = get_package_share_directory('jetank_ros_main')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

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
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz visualisation')

    # Hardware bring-up (URDF/TF, motor controller, IMU, RPLidar) plus SLAM/
    # Nav2 are unified.launch.py's job (jetank_ros_main) — this file used to
    # re-implement that same six-include graph with its own argument
    # plumbing, which meant every hardware change had to be patched twice.
    # Delegate to unified.launch.py instead, with its perception/web/MoveIt
    # layers switched off (this file never included them) and 'mode'/'map'
    # mapped onto unified's navigation_mode/map_file. unified.launch.py
    # itself gates the hardware layer UnlessCondition(use_sim_time), so this
    # still behaves correctly when called with use_sim_time:=true (e.g. from
    # sim_demo.launch.py, which only wants the SLAM branch).
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'unified.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_web_control': 'false',
            'enable_perception': 'false',
            'enable_moveit': 'false',
            'enable_navigation': 'true',
            'navigation_mode': mode,
            'map_file': map_file,
        }.items(),
    )

    # RViz (uses nav2_bringup rviz wrapper to attach the navigation panel)
    rviz_config_file = os.path.join(pkg_jetank_nav, 'rviz', 'navigation.rviz')
    rviz_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/rviz_launch.py',
            ]),
            launch_arguments={
                'rviz_config': rviz_config_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ], condition=IfCondition(use_rviz))

    ld = LaunchDescription()
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_launch)
    return ld
