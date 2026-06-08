#!/usr/bin/env python3
"""SLAM + Nav2 (navigation-only) for live mapping AND navigation at once.

LEGACY: combined SLAM+nav2 shifts the live map under loop-closure; the web UI
now uses slam.launch.py (mapping only) and nav2_bringup.launch.py (saved-map
nav) separately.

slam_toolbox (async, mapping mode) provides both the live ``/map`` and the
``map -> odom`` transform, so Nav2 runs **navigation-only** (no AMCL, no
map_server) via the upstream ``nav2_bringup/navigation_launch.py``. This lets
you drive to build a map and send NavigateToPose goals at the same time.

Used by the web control's "Mapping Mode" (jetank_web_control). Assumes a robot
is already publishing /scan + /odom (real robot or Gazebo with use_sim_time).

    ros2 launch jetank_navigation slam_nav2.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('jetank_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the simulation clock')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2', 'nav2_params.yaml'),
        description='Nav2 params (amcl/map_server sections are ignored here)')

    # slam_toolbox (async mapping) -> /map + map->odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Nav2 navigation-only (controller/planner/behaviors/bt_navigator/smoother
    # + lifecycle_manager_navigation). No localization — slam_toolbox supplies it.
    # Uses our navigation_only.launch.py (not upstream navigation_launch.py) so the
    # lifecycle_manager gets bond_timeout:=0.0 — otherwise the stack tears itself
    # down a few seconds after bringup under heavy load and rejects all goals.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_nav, 'launch', 'navigation_only.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([declare_use_sim_time, declare_params, slam, nav2])
