#!/usr/bin/env python3
"""Full navigation launch for the JeTank robot.

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
    ros2 launch jetank_navigation navigation_full.launch.py \\
        mode:=nav2 map:=$HOME/maps/jetank_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


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
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz visualisation')

    # Hardware bringup (items 1-4) only runs on the real robot. In simulation
    # (use_sim_time:=true) Gazebo already provides robot_state_publisher, the
    # /scan, /imu and /odom topics, and gz_ros2_control — so launching the
    # hardware nodes here would spawn a second robot_state_publisher and a
    # motor driver that spams I2C errors and publishes invalid JointStates.
    # Gate them behind UnlessCondition(use_sim_time).

    # 1. Robot state publisher (URDF/TF tree) — always includes RPLidar frame
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'urdf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(use_sim_time),
    )

    # 2. Motor controller (publishes /odom, subscribes to /cmd_vel)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'motor_controller.launch.py')
        ),
        condition=UnlessCondition(use_sim_time),
    )

    # 3. IMU (ICM-20948 on the Waveshare IMX219-83 stereo camera module)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'imu.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(use_sim_time),
    )

    # 4. RPLidar (hardware driver publishing /scan)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'lidar.launch.py')
        ),
        condition=UnlessCondition(use_sim_time),
    )

    # 5a. SLAM Toolbox (mapping mode)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 5b. Nav2 stack (navigation mode)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav2'"])),
        launch_arguments={'map': map_file, 'use_sim_time': use_sim_time}.items(),
    )

    # 6. RViz (uses nav2_bringup rviz wrapper to attach the navigation panel)
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
    ld.add_action(urdf_launch)
    ld.add_action(motor_launch)
    ld.add_action(imu_launch)
    ld.add_action(lidar_launch)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_launch)
    return ld
