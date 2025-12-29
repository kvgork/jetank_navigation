# JeTank Navigation Implementation Plan
**Created**: 2025-12-25
**Status**: Ready for Implementation
**Objective**: Complete PointCloud2→LaserScan conversion and implement Nav2 stack for autonomous navigation

---

## 📋 Current State

### ✅ Completed
- Node skeleton (`laser_data_node.cpp`) with full framework
- Parameter system configured
- TF2 integration ready
- Subscribers/publishers set up
- Configuration file (`config/laser_data.yaml`)

### 🚧 To Complete
- PointCloud2 to LaserScan conversion logic (TODO at line 223-239)
- Nav2 stack integration
- SLAM configuration
- Full system integration

---

## 🎯 Implementation Phases

## Phase 1: Complete PointCloud2 to LaserScan Conversion

### 1.1 Implement Conversion Logic
**File**: `src/laser_data_node.cpp:222-239`

**Implementation**:

```cpp
void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. Validate input
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(get_logger(), "Received empty point cloud");
        return;
    }

    // 2. Lookup transform
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            config_.frame_id,
            msg->header.frame_id,
            msg->header.stamp,
            rclcpp::Duration::from_seconds(config_.transform_tolerance)
        );
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
        return;
    }

    // 3. Initialize LaserScan message
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = config_.frame_id;
    scan.angle_min = config_.laser_angle_min;
    scan.angle_max = config_.laser_angle_max;
    scan.angle_increment = config_.laser_angle_increment;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = config_.laser_min_range;
    scan.range_max = config_.laser_max_range;

    int num_ranges = static_cast<int>(
        (config_.laser_angle_max - config_.laser_angle_min) / config_.laser_angle_increment
    );
    scan.ranges.assign(num_ranges, config_.laser_max_range);

    // 4. Iterate through point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // Transform point to base_link
        geometry_msgs::msg::PointStamped point_in, point_out;
        point_in.point.x = *iter_x;
        point_in.point.y = *iter_y;
        point_in.point.z = *iter_z;

        tf2::doTransform(point_in, point_out, transform);

        // 5. Height filtering
        if (point_out.point.z < config_.conversion_height - config_.conversion_range ||
            point_out.point.z > config_.conversion_height + config_.conversion_range) {
            continue;
        }

        // 6. Calculate angle and distance
        double angle = atan2(point_out.point.y, point_out.point.x);
        double distance = sqrt(point_out.point.x * point_out.point.x +
                              point_out.point.y * point_out.point.y);

        // Check if in range
        if (distance < config_.laser_min_range || distance > config_.laser_max_range) {
            continue;
        }

        // 7. Calculate bin index
        if (angle < config_.laser_angle_min || angle > config_.laser_angle_max) {
            continue;
        }

        int index = static_cast<int>(
            (angle - config_.laser_angle_min) / config_.laser_angle_increment
        );

        if (index >= 0 && index < num_ranges) {
            // Keep minimum distance (closest obstacle)
            if (distance < scan.ranges[index]) {
                scan.ranges[index] = distance;
            }
        }
    }

    // 8. Apply noise filtering
    for (int i = 1; i < num_ranges - 1; ++i) {
        if (scan.ranges[i] < config_.laser_max_range) {
            // Check distance to neighbors
            float diff_prev = std::abs(scan.ranges[i] - scan.ranges[i-1]);
            float diff_next = std::abs(scan.ranges[i] - scan.ranges[i+1]);

            if (diff_prev > config_.filter_max_distance &&
                diff_next > config_.filter_max_distance) {
                // Isolated point - likely noise
                scan.ranges[i] = config_.laser_max_range;
            }
        }
    }

    // 9. Publish
    laser_scan_pub_->publish(scan);
    RCLCPP_DEBUG(get_logger(), "Published LaserScan with %d ranges", num_ranges);
}
```

**Dependencies to add** to `CMakeLists.txt`:
```cmake
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_target_dependencies(laser_data_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
```

**Add includes** to `laser_data_node.cpp`:
```cpp
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
```

---

### 1.2 Create Launch File
**File**: `launch/laser_scan_converter.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('jetank_navigation'),
        'config',
        'laser_data.yaml'
    ])

    laser_data_node = Node(
        package='jetank_navigation',
        executable='laser_data_node',
        name='laser_data_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        laser_data_node
    ])
```

**Update CMakeLists.txt** to install launch files:
```cmake
# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

---

### 1.3 Build and Test

```bash
cd /home/koen/workspaces/ros2_ws
colcon build --packages-select jetank_navigation
source install/setup.bash

# Test launch
ros2 launch jetank_navigation laser_scan_converter.launch.py

# In another terminal, check output
ros2 topic echo /scan --once
ros2 topic hz /scan
```

**Validate in RViz**:
```bash
rviz2
# Add LaserScan display: /scan
# Add PointCloud2 display: /stereo_camera/points
# Compare visual alignment
```

---

## Phase 2: Nav2 Stack Integration

### 2.1 Install Dependencies

```bash
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-robot-localization
```

---

### 2.2 Extract Robot Dimensions

**Check** `src/jetank_description/urdf/jetank_parameters.xacro` for:
- Wheelbase
- Track width
- Robot height
- Camera mounting position

**Estimate footprint** (rectangular or circular):
- Add 5cm safety margin to physical dimensions

---

### 2.3 Create Costmap Configuration

**Directory structure**:
```
config/nav2/
├── nav2_params.yaml          # Combined params file
├── costmap_common.yaml       # Shared costmap params
├── global_costmap.yaml       # Global planning costmap
├── local_costmap.yaml        # Local control costmap
└── controller.yaml           # Controller params
```

#### `config/nav2/costmap_common.yaml`
```yaml
costmap_common:
  ros__parameters:
    robot_radius: 0.15  # Adjust based on actual robot
    footprint_padding: 0.05

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        sensor_frame: base_link
        data_type: "LaserScan"
        marking: true
        clearing: true
        min_obstacle_height: 0.0
        max_obstacle_height: 2.0
        obstacle_max_range: 1.5
        obstacle_min_range: 0.0
        raytrace_max_range: 1.6
        raytrace_min_range: 0.0

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      enabled: true
      inflation_radius: 0.30
      cost_scaling_factor: 3.0
```

#### `config/nav2/global_costmap.yaml`
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
```

#### `config/nav2/local_costmap.yaml`
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.025
      plugins: ["obstacle_layer", "inflation_layer"]
```

#### `config/nav2/controller.yaml`
```yaml
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.15

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.3
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.3
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0
      vx_samples: 20
      vy_samples: 0
      vth_samples: 40
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
```

---

### 2.4 Create Combined Nav2 Parameters

#### `config/nav2/nav2_params.yaml`
```yaml
# Main Nav2 parameter file
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    # ... (rest from controller.yaml)

local_costmap:
  local_costmap:
    ros__parameters:
      # ... (from local_costmap.yaml)

global_costmap:
  global_costmap:
    ros__parameters:
      # ... (from global_costmap.yaml)

planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.1
      use_astar: true
      allow_unknown: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.3, 0.0, 1.0]
    min_velocity: [-0.3, 0.0, -1.0]
    max_accel: [0.5, 0.0, 1.0]
    max_decel: [-0.5, 0.0, -1.0]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

---

### 2.5 AMCL Configuration

#### `config/nav2/amcl.yaml`
```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 1.5
    laser_min_range: 0.3
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
```

---

### 2.6 Create Nav2 Launch File

#### `launch/nav2_bringup.launch.py`
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')

    # Paths
    nav2_params = os.path.join(pkg_jetank_nav, 'config', 'nav2', 'nav2_params.yaml')
    map_file = LaunchConfiguration('map')

    # Declare launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_jetank_nav, 'maps', 'default_map.yaml'),
        description='Full path to map yaml file'
    )

    # Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'map': map_file,
            'use_sim_time': 'False'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
```

---

## Phase 3: SLAM Integration

### 3.1 SLAM Toolbox Configuration

#### `config/slam/slam_toolbox.yaml`
```yaml
slam_toolbox:
  ros__parameters:
    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    # Solver params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Mapping params
    resolution: 0.05
    max_laser_range: 1.5
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
    map_update_interval: 5.0
    enable_interactive_mode: true

    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation params
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # Misc
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
```

---

### 3.2 SLAM Launch File

#### `launch/slam.launch.py`
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')
    slam_params = os.path.join(pkg_jetank_nav, 'config', 'slam', 'slam_toolbox.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        slam_toolbox_node
    ])
```

---

### 3.3 Map Save/Load Scripts

#### `scripts/save_map.sh`
```bash
#!/bin/bash
MAP_NAME=${1:-my_map}
MAP_DIR="${HOME}/maps"

mkdir -p ${MAP_DIR}

echo "Saving map to ${MAP_DIR}/${MAP_NAME}"
ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/${MAP_NAME}

echo "Map saved successfully!"
echo "YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
echo "PGM:  ${MAP_DIR}/${MAP_NAME}.pgm"
```

Make executable:
```bash
chmod +x scripts/save_map.sh
```

---

## Phase 4: Full System Integration

### 4.1 Complete System Launch

#### `launch/navigation_full.launch.py`
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_jetank_nav = get_package_share_directory('jetank_navigation')
    pkg_jetank_main = get_package_share_directory('jetank_ros_main')

    # Launch arguments
    use_slam = LaunchConfiguration('slam', default='false')
    map_file = LaunchConfiguration('map')

    # 1. Robot state publisher (URDF/TF)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'urdf.launch.py')
        )
    )

    # 2. Stereo camera
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

    # 4. Laser scan converter
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'laser_scan_converter.launch.py')
        )
    )

    # 5. Nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'nav2_bringup.launch.py')
        ),
        launch_arguments={'map': map_file}.items()
    )

    # 6. SLAM (optional)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_nav, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(use_slam)
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(DeclareLaunchArgument('slam', default_value='false'))
    ld.add_action(DeclareLaunchArgument('map', default_value=''))

    # Launches
    ld.add_action(urdf_launch)
    ld.add_action(camera_launch)
    ld.add_action(motor_launch)
    ld.add_action(laser_launch)
    ld.add_action(nav2_launch)
    ld.add_action(slam_launch)

    return ld
```

---

### 4.2 RViz Configuration

Create `rviz/navigation.rviz` with displays:
- RobotModel
- TF
- Map
- LaserScan (`/scan`)
- PointCloud2 (`/stereo_camera/points`)
- GlobalCostmap (`/global_costmap/costmap`)
- LocalCostmap (`/local_costmap/costmap`)
- GlobalPath (`/plan`)
- LocalPath (`/local_plan`)
- ParticleCloud (`/particle_cloud`) - for AMCL

Save and reference in launch file.

---

### 4.3 Update CMakeLists.txt

```cmake
# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install maps directory
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
  OPTIONAL
)

# Install RViz configs
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

# Install scripts
install(PROGRAMS
  scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## 📦 Final Deliverables

### File Structure
```
jetank_navigation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── IMPLEMENTATION_PLAN.md
├── config/
│   ├── laser_data.yaml
│   ├── nav2/
│   │   ├── nav2_params.yaml
│   │   ├── costmap_common.yaml
│   │   ├── global_costmap.yaml
│   │   ├── local_costmap.yaml
│   │   ├── controller.yaml
│   │   └── amcl.yaml
│   └── slam/
│       └── slam_toolbox.yaml
├── include/jetank_navigation/
│   └── utils.hpp
├── launch/
│   ├── laser_scan_converter.launch.py
│   ├── nav2_bringup.launch.py
│   ├── slam.launch.py
│   └── navigation_full.launch.py
├── maps/
│   └── (generated map files)
├── rviz/
│   └── navigation.rviz
├── scripts/
│   └── save_map.sh
└── src/
    └── laser_data_node.cpp
```

---

## 🚀 Usage

### Build
```bash
cd /home/koen/workspaces/ros2_ws
colcon build --packages-select jetank_navigation
source install/setup.bash
```

### Test Laser Scan Conversion
```bash
# Terminal 1: Launch camera
ros2 launch jetank_ros_main stereo_camera.launch.py

# Terminal 2: Launch converter
ros2 launch jetank_navigation laser_scan_converter.launch.py

# Terminal 3: Visualize
rviz2 -d install/jetank_navigation/share/jetank_navigation/rviz/navigation.rviz
```

### Run SLAM (Mapping)
```bash
ros2 launch jetank_navigation navigation_full.launch.py slam:=true
```

### Run Navigation (with existing map)
```bash
ros2 launch jetank_navigation navigation_full.launch.py map:=/path/to/map.yaml
```

### Save Map
```bash
ros2 run jetank_navigation save_map.sh my_office_map
```

---

## ⏱️ Implementation Timeline

| Phase | Tasks | Duration |
|-------|-------|----------|
| Phase 1 | Laser scan conversion | 2-3 days |
| Phase 2 | Nav2 configuration | 3-4 days |
| Phase 3 | SLAM integration | 2 days |
| Phase 4 | Full integration & testing | 2-3 days |
| **Total** | | **9-12 days** |

---

## 🎯 Next Steps

1. Implement PointCloud2 callback (Phase 1.1)
2. Build and test laser scan conversion
3. Extract robot dimensions from URDF (Phase 2.2)
4. Create Nav2 configuration files (Phase 2.3-2.5)
5. Test navigation with Nav2
6. Configure SLAM
7. Full system integration
