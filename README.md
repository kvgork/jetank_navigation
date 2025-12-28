# JeTank Navigation Package

Complete autonomous navigation stack for the JeTank AI robot platform.

## Overview

This package provides a complete navigation solution for the JeTank robot, including:

- **PointCloud2 to LaserScan conversion** - Converts stereo camera depth data to 2D laser scans
- **SLAM mapping** - Create maps of unknown environments using SLAM Toolbox
- **Autonomous navigation** - Navigate to goals using Nav2 stack with AMCL localization
- **Full system integration** - Launch complete autonomous navigation with a single command

## Features

### ✅ Implemented

- **Laser Scan Generation**
  - Converts stereo camera point clouds to 2D laser scans
  - 30Hz operation with configurable parameters
  - Height filtering and noise reduction
  - Supports 73° FOV, 0.3-1.5m range

- **SLAM Mapping**
  - SLAM Toolbox integration for autonomous mapping
  - Real-time map building and loop closure
  - Map saving/loading utilities
  - Optimized for small indoor environments

- **Autonomous Navigation**
  - Complete Nav2 stack integration
  - AMCL localization for known maps
  - DWB local planner for dynamic obstacle avoidance
  - NavFn global planner for optimal path planning
  - Behavior-based recovery system

- **Visualization**
  - Pre-configured RViz setup
  - Real-time costmap visualization
  - Path planning visualization
  - Particle cloud display (AMCL)

## Package Contents

```
jetank_navigation/
├── config/
│   ├── laser_data.yaml              # LaserScan conversion parameters
│   ├── nav2/
│   │   ├── nav2_params.yaml         # Complete Nav2 configuration
│   │   └── amcl.yaml                # AMCL localization parameters
│   └── slam/
│       └── slam_toolbox.yaml        # SLAM Toolbox configuration
├── launch/
│   ├── laser_scan_converter.launch.py    # LaserScan conversion only
│   ├── slam.launch.py                     # SLAM mapping only
│   ├── nav2_bringup.launch.py             # Nav2 navigation only
│   └── navigation_full.launch.py          # Complete system integration
├── maps/                                   # Saved maps directory
├── rviz/
│   └── navigation.rviz                    # RViz configuration
├── scripts/
│   └── save_map.sh                        # Map saving utility
├── src/
│   └── laser_data_node.cpp               # PointCloud2→LaserScan converter
└── include/jetank_navigation/
    └── utils.hpp                          # Utility functions
```

## Dependencies

### Required ROS2 Packages

```bash
# Navigation stack
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup

# SLAM
sudo apt install ros-humble-slam-toolbox

# Already installed with base system
# - ros-humble-sensor-msgs
# - ros-humble-geometry-msgs
# - ros-humble-tf2-ros
```

### System Requirements

- ROS2 Humble
- Ubuntu 22.04
- JeTank robot with:
  - Stereo camera (IMX219-83)
  - Motor controller (odometry)
  - Complete TF tree

## Quick Start

### 1. Build the Package

```bash
cd ~/workspaces/ros2_ws
colcon build --packages-select jetank_navigation
source install/setup.bash
```

### 2. Create a Map (SLAM Mode)

```bash
# Launch full system in mapping mode
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

# Drive the robot around to build the map
# (Use teleop or manual control)

# Save the map when complete
ros2 run jetank_navigation save_map.sh my_map
```

### 3. Navigate Autonomously (Nav2 Mode)

```bash
# Launch full system with your saved map
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=nav2 \
  map:=$HOME/maps/my_map.yaml

# In RViz:
# 1. Set initial pose with "2D Pose Estimate" tool
# 2. Send goals with "Nav2 Goal" tool
```

## Usage Guide

### Launch Files

#### Full System Integration

**navigation_full.launch.py** - Launches everything needed for autonomous navigation:

```bash
# SLAM mode (for mapping)
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

# Nav2 mode (for navigation with existing map)
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=nav2 \
  map:=/path/to/map.yaml

# Without RViz
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=slam \
  rviz:=false
```

**What it launches**:
- Robot state publisher (URDF/TF)
- Stereo camera
- Motor controller (odometry)
- LaserScan converter
- SLAM Toolbox OR Nav2 (based on mode)
- RViz (optional)

#### Individual Components

**LaserScan Converter Only**:
```bash
ros2 launch jetank_navigation laser_scan_converter.launch.py
```

**SLAM Only** (requires laser scan and odometry):
```bash
ros2 launch jetank_navigation slam.launch.py
```

**Nav2 Only** (requires laser scan and odometry):
```bash
ros2 launch jetank_navigation nav2_bringup.launch.py \
  map:=/path/to/map.yaml
```

### Map Management

**Save Current Map**:
```bash
# Default saves to ~/maps/jetank_map
ros2 run jetank_navigation save_map.sh

# Specify name
ros2 run jetank_navigation save_map.sh office_map
```

**List Available Maps**:
```bash
ls ~/maps/*.yaml
```

### RViz Operation

**Set Initial Pose (AMCL)**:
1. Click "2D Pose Estimate" toolbar button
2. Click on map where robot is located
3. Drag to set orientation
4. Release mouse
5. Wait for particles to converge (green cloud)

**Send Navigation Goal**:
1. Click "Nav2 Goal" toolbar button
2. Click on map where you want robot to go
3. Drag to set goal orientation
4. Release mouse
5. Robot will plan and execute path

**Monitor Navigation**:
- **Yellow path**: Global plan
- **Green path**: Local plan
- **Red laserscan**: Obstacles
- **Costmaps**: Inflation zones around obstacles

## Configuration

### LaserScan Conversion

Edit `config/laser_data.yaml`:

```yaml
laser_data_node:
  ros__parameters:
    # Scan range
    laser.min_range: 0.3      # Minimum detection (m)
    laser.max_range: 1.5      # Maximum detection (m)

    # Height filtering
    conversion.height: 0.05   # Slice height above base_link (m)
    conversion.range: 0.005   # Tolerance (±5mm)

    # Noise filtering
    filter.max_distance: 0.05 # Max gap between points (m)
```

### SLAM Parameters

Edit `config/slam/slam_toolbox.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Map resolution
    resolution: 0.05          # 5cm grid cells

    # Movement thresholds
    minimum_travel_distance: 0.1   # 10cm
    minimum_travel_heading: 0.2    # ~11 degrees

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0  # meters
```

### Nav2 Parameters

Edit `config/nav2/nav2_params.yaml`:

**Robot footprint**:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.15      # Robot radius (m)
```

**Velocity limits**:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.3          # Linear velocity (m/s)
      max_vel_theta: 1.0      # Angular velocity (rad/s)
```

**Costmap inflation**:
```yaml
inflation_layer:
  inflation_radius: 0.30      # Safety buffer (m)
  cost_scaling_factor: 3.0
```

## Workflow Examples

### Example 1: Map Your Office

```bash
# 1. Launch SLAM mode
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

# 2. Drive around slowly (use teleop)
# - Cover all areas
# - Drive slowly near walls
# - Complete loop closures

# 3. Save the map
ros2 run jetank_navigation save_map.sh office_floor1

# Map saved to ~/maps/office_floor1.yaml
```

### Example 2: Patrol Route

```bash
# 1. Launch Nav2 with your map
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=nav2 \
  map:=$HOME/maps/office_floor1.yaml

# 2. Set initial pose in RViz

# 3. Send multiple waypoints using "Nav2 Goal"
# Robot will navigate to each goal sequentially
```

### Example 3: Test Individual Components

```bash
# Terminal 1: Camera + LaserScan
ros2 launch jetank_ros2_main stereo_camera.launch.py &
ros2 launch jetank_navigation laser_scan_converter.launch.py

# Terminal 2: Check laser scan
ros2 topic echo /scan --once
ros2 topic hz /scan

# Terminal 3: Visualize
rviz2
# Add LaserScan display on topic /scan
```

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/stereo_camera/points` | sensor_msgs/PointCloud2 | Input point cloud from stereo camera |
| `/odom` | nav_msgs/Odometry | Robot odometry from motor controller |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Generated 2D laser scan |
| `/map` | nav_msgs/OccupancyGrid | SLAM-generated map (SLAM mode) |
| `/particle_cloud` | geometry_msgs/PoseArray | AMCL particles (Nav2 mode) |
| `/plan` | nav_msgs/Path | Global navigation plan |
| `/local_plan` | nav_msgs/Path | Local navigation plan |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands to motors |

## Troubleshooting

### LaserScan Issues

**Problem**: No laser scan published
```bash
# Check point cloud
ros2 topic hz /stereo_camera/points

# Check TF
ros2 run tf2_ros tf2_echo base_link camera_left_link

# Check node
ros2 node info /laser_data_node
```

**Problem**: Laser scan has many max_range values
- Increase camera exposure (better depth data)
- Adjust height filtering parameters
- Check camera calibration

### SLAM Issues

**Problem**: Map quality is poor
- Drive slower during mapping
- Ensure good lighting for camera
- Check odometry accuracy
- Increase `minimum_travel_distance`

**Problem**: No loop closure
- Drive complete loops
- Ensure distinct features in environment
- Adjust `loop_search_maximum_distance`

### Navigation Issues

**Problem**: Robot won't move
- Check `/cmd_vel` topic is published
- Verify motor controller is running
- Check costmaps for obstacles

**Problem**: AMCL won't localize
- Set better initial pose estimate
- Increase particle count
- Check odometry is published

**Problem**: Path planning fails
- Check global costmap for valid path
- Adjust inflation radius if too conservative
- Verify map is loaded correctly

### Common Errors

**"Transform failed" warnings**:
```bash
# Verify TF tree is complete
ros2 run tf2_tools view_frames
evince frames.pdf

# Check for missing transforms
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_left_link
```

**Nav2 nodes won't start**:
```bash
# Check parameter file syntax
ros2 param describe /controller_server use_sim_time

# Verify all dependencies installed
ros2 pkg list | grep nav2
```

## Performance Tips

### Optimization

**Reduce CPU usage**:
- Lower costmap update frequency
- Reduce particle count in AMCL
- Decrease planner frequency

**Improve accuracy**:
- Increase costmap resolution
- Use finer angular resolution
- Tune AMCL noise parameters

**Faster navigation**:
- Increase velocity limits
- Reduce safety margins (inflation)
- Use simpler planner

### Parameter Tuning Priority

1. **Robot footprint** - Get this right first
2. **Velocity limits** - Safe but not too slow
3. **Costmap inflation** - Balance safety vs agility
4. **AMCL particles** - More = accurate but slower
5. **Controller critics** - Fine-tune behavior

## Hardware Specifications

### JeTank Robot

- **Chassis**: 0.2m × 0.1m × 0.08m
- **Wheels**: 0.03m radius, differential drive
- **Footprint**: ~0.25m × 0.25m (with safety margin)
- **Ground clearance**: 0.01m

### Sensors

- **Camera**: IMX219-83 Stereo Camera
  - FOV: 73° horizontal, 50° vertical
  - Resolution: 640×360 @ 30fps
  - Depth range: 0.3-1.5m
- **Odometry**: Wheel encoders (motor controller)

## Development

### Adding Custom Behaviors

Create behavior plugins in `src/behaviors/`:

```cpp
#include <nav2_behaviors/behavior.hpp>

class CustomBehavior : public nav2_behaviors::Behavior {
  // Implementation
};
```

Register in `config/nav2/nav2_params.yaml`.

### Extending the Stack

- Add recovery behaviors
- Implement custom planners
- Create mission planning layer
- Add obstacle detection sensors

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [JeTank Robot Description](../jetank_description/)
- [Implementation Plan](IMPLEMENTATION_PLAN.md)

## License

MIT License - See main repository LICENSE file

## Maintainer

JeTank Navigation Package
- Package: `jetank_navigation`
- ROS2 Distro: Humble
- Platform: Ubuntu 22.04

---

**Ready for autonomous navigation!** 🤖🗺️
