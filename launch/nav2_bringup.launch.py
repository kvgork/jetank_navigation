#!/usr/bin/env python3
"""
Launch file for the Nav2 stack.

Two modes, selected by the ``use_localization`` argument:

- ``use_localization:=True`` (default) — full stack against a saved map:
  map_server + AMCL + controller / planner / behaviors / bt_navigator +
  lifecycle manager.
- ``use_localization:=False`` — navigation-only: no map_server/amcl;
  localization and ``/map`` come from slam_toolbox. This is what
  navigation_only.launch.py (used by slam_nav2.launch.py) selects.

waypoint_follower, velocity_smoother and (SLAM variant) smoother_server are
intentionally not launched: nothing in this workspace calls FollowWaypoints
or NavigateThroughPoses/SmoothPath (RViz GoalTool and web_control_node only
issue NavigateToPose), and velocity_smoother duplicated DWB's own
acc_lim_*/decel_lim_* limiting on every /cmd_vel message. controller_server
now publishes cmd_vel directly instead of through velocity_smoother.

Key difference from upstream nav2_bringup: the lifecycle_manager is given
**bond_timeout: 0.0**. With the default 4 s bond timeout, a node that misses
its heartbeat under heavy load (Gazebo GUI + RViz + SLAM + Nav2 on one
machine) is declared unresponsive and the manager tears the whole stack down
a few seconds after bringup — which made bt_navigator go active then inactive
and reject all goals. bond_timeout 0.0 disables that.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    """Build the Nav2 node set for the selected mode."""
    use_localization = (
        LaunchConfiguration('use_localization').perform(context).lower()
        in ('true', '1'))

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    map_yaml_file = LaunchConfiguration('map')

    # Map fully qualified names to relative ones so the node's namespace can
    # be prepended.
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    if use_localization:
        # Full stack against a saved map.
        lifecycle_nodes = ['map_server',
                           'amcl',
                           'controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator']
    else:
        # No map_server / amcl — slam_toolbox provides /map and map->odom.
        lifecycle_nodes = ['controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator']

    # node name -> (package, executable, extra remappings). controller_server
    # publishes the final /cmd_vel directly (no velocity_smoother hop; DWB's
    # own acc_lim_*/decel_lim_* already limit acceleration).
    specs = {
        'map_server': ('nav2_map_server', 'map_server', []),
        'amcl': ('nav2_amcl', 'amcl', []),
        'controller_server': ('nav2_controller', 'controller_server', []),
        'planner_server': ('nav2_planner', 'planner_server', []),
        'behavior_server': ('nav2_behaviors', 'behavior_server', []),
        'bt_navigator': ('nav2_bt_navigator', 'bt_navigator', []),
    }

    nodes = [
        Node(
            package=specs[name][0],
            executable=specs[name][1],
            name=name,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + specs[name][2])
        for name in lifecycle_nodes
    ]

    nodes.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes},
                        # Disable the lifecycle bond timeout: under heavy load
                        # a node misses its heartbeat and the manager would
                        # otherwise tear the whole stack down (bt_navigator ->
                        # inactive -> goals rejected).
                        {'bond_timeout': 0.0}]))

    return [GroupAction(nodes)]


def generate_launch_description():
    """Generate launch description for the Nav2 stack."""
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_jetank_nav, 'config', 'nav2', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization',
        default_value='True',
        description='Launch map_server + AMCL (False when slam_toolbox '
                    'provides /map and map->odom)')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_localization_cmd)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
