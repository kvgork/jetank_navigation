# JeTank Navigation Package

Navigation, SLAM and sensor bring-up for the JeTank AI robot — Nav2 +
slam_toolbox + RPLidar + IMU, for both Gazebo simulation and the real robot.

## Overview

This package wires the JeTank into the standard ROS 2 navigation ecosystem:

- **2D lidar** — RPLidar (real) or Gazebo `gpu_lidar` (sim) publishing `/scan`.
- **IMU** — first-party ICM-20948 driver (`icm20948_node`) over I2C (real robot).
- **SLAM mapping** — `slam_toolbox` builds `/map` and the `map → odom` transform.
- **Autonomous navigation** — the full Nav2 stack (planner / controller /
  behaviors / bt_navigator), with AMCL localization against a saved map.
- **One-command bring-up** — `navigation_full.launch.py` is sim-aware and
  dispatches `mode:=slam` or `mode:=nav2`.

> The only node this package *builds itself* is the IMU driver. SLAM, Nav2 and
> the lidar driver are upstream packages (`slam_toolbox`, `nav2_*`,
> `rplidar_ros`) that this package's launch files configure and start. (Earlier
> revisions described a `laser_data_node` PointCloud2→LaserScan converter — that
> approach was dropped; the lidar publishes `/scan` directly.)

## Running in Simulation (Gazebo)

> Most of this README describes the **real-robot** path. For Gazebo, the robot
> model already publishes a real `gpu_lidar` `/scan` and `/imu`, and
> `gz_ros2_control` provides odometry.

`navigation_full.launch.py` is **sim-aware**: with `use_sim_time:=true` it
**skips the hardware bringup** (urdf/motor/IMU/RPLidar — gated behind
`UnlessCondition(use_sim_time)`) and consumes Gazebo's topics instead. Without
that flag it spawns the hardware nodes (a second `robot_state_publisher` and the
motor driver, which spams I2C errors in sim).

```bash
# 1) start the sim (separate terminal): Gazebo + robot + sensors + controllers
ros2 launch jetank_ros_main gazebo_sim.launch.py world:=obstacle_course

# 2) SLAM against the simulated lidar
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam use_sim_time:=true

# 2') Nav2 with a saved map
ros2 launch jetank_navigation navigation_full.launch.py mode:=nav2 use_sim_time:=true map:=<map.yaml>
```

Or use the all-in-one `ros2 launch jetank_ros_main sim_demo.launch.py`
(Gazebo + unified.rviz + SLAM). slam_toolbox subscribes to `/scan`, publishes
`/map`, and `map → odom`; verified building a ~5.3 m map in `obstacle_course`.

### Sim launch files used by the web control

- **`slam_nav2.launch.py`** — slam_toolbox (online mapping = `/map` + `map→odom`)
  **+** `navigation_only.launch.py`. The web "Start Mapping" entry; you can map and
  navigate at the same time (no AMCL).
- **`navigation_only.launch.py`** — Nav2 navigation stack (controller / smoother /
  planner / behaviors / bt_navigator / waypoint / velocity_smoother) **with NO**
  map_server/AMCL, and the lifecycle_manager set to **`bond_timeout: 0.0`**.
  Upstream `nav2_bringup/navigation_launch.py` does not pass the params file to its
  lifecycle_manager, and under heavy sim load a missed bond heartbeat (default 4 s)
  tears the whole stack down → bt_navigator flaps to inactive → goals rejected.
  `bond_timeout: 0.0` disables that.
- **`nav2_bringup.launch.py`** — full localization + navigation (map_server + AMCL).
  Used by the web "Navigate (saved map)". Also given `bond_timeout: 0.0`, and AMCL
  has `set_initial_pose` + `initial_pose` (0,0,0) so it can self-localize at the
  robot spawn. The web node additionally publishes `/initialpose` until AMCL
  converges (the param alone is not always honored through RewrittenYaml).

Costmaps use `robot_radius: 0.12` and `inflation_radius: 0.18` tuned for the small
JeTank footprint.

## ROS 2 API

This package builds exactly **one** runtime node of its own — the ICM-20948 IMU driver (`icm20948_node`). All other navigation/SLAM/LiDAR functionality is delivered by *launching third-party nodes* (`slam_toolbox`, the `nav2_*` stack, `rplidar_ros`) via this package's launch files; those nodes' interfaces belong to their upstream packages and are not redefined here. This package declares **no** custom messages, services, or actions.

### Nodes (built by this package)

| Node name | Executable | Role |
|---|---|---|
| `icm20948_imu` | `icm20948_node` | InvenSense ICM-20948 9-axis IMU driver. Reads accelerometer, gyroscope, magnetometer (AK09916) and die temperature over I2C and publishes them. |

### Published topics (`icm20948_imu`)

| Topic | Type |
|---|---|
| `imu/data_raw` | `sensor_msgs/msg/Imu` |
| `imu/magnetic_field` | `sensor_msgs/msg/MagneticField` |
| `imu/temperature` | `sensor_msgs/msg/Temperature` |

`Imu.orientation_covariance[0]` is set to `-1` (no orientation estimate — raw accel/gyro only); accel and gyro carry diagonal covariances from datasheet noise specs; `Temperature` is published at 1/10 the IMU rate.

### Parameters (`icm20948_imu`)

| Parameter | Type | Node default | Launched value (`config/icm20948.yaml`) |
|---|---|---|---|
| `i2c_bus` | int | `7` | `1` (`/dev/i2c-1`) |
| `i2c_address` | int | `0x68` | `0x68` |
| `frame_id` | string | `imu_link` | `imu_link` |
| `publish_rate` | double (Hz) | `100.0` | `100.0` |

### Launch entrypoints (start third-party nodes)

| Launch file | Brings up |
|---|---|
| `imu.launch.py` | `icm20948_imu` (this package's node) with `config/icm20948.yaml` |
| `rplidar.launch.py` / `lidar.launch.py` | `rplidar_ros/rplidar_node` → publishes `/scan` (`frame_id: laser`) |
| `slam.launch.py` | `slam_toolbox/async_slam_toolbox_node` (subscribes `/scan`, publishes `/map`, provides `map → odom`) |
| `nav2_bringup.launch.py` | Full Nav2 + localization: `map_server`, `amcl`, `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `lifecycle_manager` |
| `navigation_only.launch.py` | Nav2 **without** `map_server`/`amcl` (expects SLAM to supply `map → odom`) |
| `slam_nav2.launch.py` | `slam.launch.py` + `navigation_only.launch.py` (live map + navigate) |
| `navigation_full.launch.py` | Real robot (gated by `UnlessCondition(use_sim_time)`): URDF + motor controller + `imu` + `lidar`, then SLAM (`mode:=slam`) **or** Nav2 (`mode:=nav2`), plus optional RViz |

### Key wire names set in launch + `config/nav2/nav2_params.yaml`

These are upstream Nav2 nodes; the names below are how this package wires them:

- `controller_server`: `cmd_vel` → `cmd_vel_nav`; `velocity_smoother` outputs the final velocity on `/cmd_vel` (input `cmd_vel_nav`, output `cmd_vel_smoothed` → `cmd_vel`).
- `slam_toolbox`: `scan_topic: /scan`, frames `map`/`odom`/`base_link`.
- Nav2 costmaps: observation source `scan` on `/scan`, `odom_topic: /odom`, `robot_radius: 0.12`, `inflation_radius: 0.18`.

## Tests

`test/test_utils.cpp` — gtests for the header-only math helpers in
`include/jetank_navigation/utils.hpp` (it is the first/only consumer that
`#include`s the header, so it also acts as a compile guard):

- `DegToRadKnownValues` — `deg_to_rad` returns correct radians for 0/90/180/360/-90°.
- `CalculateAngleIncrement` — `calculate_angle_increment` spaces N beams over `[min,max]`
  into `N-1` equal gaps.
- `DiagonalFovToHorizontal` — `diagonal_fov_to_horizontal` equals the diagonal FOV for a
  square image, stretches for wider-than-tall images, and grows with aspect ratio.

Run them (built and run by colcon):

```bash
colcon test --packages-select jetank_navigation
colcon test-result --verbose
```

## Package Contents

```
jetank_navigation/
├── config/
│   ├── icm20948.yaml                 # ICM-20948 IMU driver parameters
│   ├── rplidar_c1m1.yaml             # RPLidar driver parameters
│   ├── nav2/
│   │   └── nav2_params.yaml          # Full Nav2 config (planner/controller/AMCL/costmaps)
│   └── slam/
│       └── slam_toolbox.yaml         # slam_toolbox configuration
├── launch/
│   ├── imu.launch.py                 # ICM-20948 IMU driver
│   ├── rplidar.launch.py             # RPLidar driver (publishes /scan)
│   ├── lidar.launch.py               # wrapper around rplidar.launch.py
│   ├── slam.launch.py                # slam_toolbox mapping
│   ├── nav2_bringup.launch.py        # Nav2 + map_server + AMCL
│   ├── navigation_only.launch.py     # Nav2 without map_server/AMCL
│   ├── slam_nav2.launch.py           # SLAM + navigate simultaneously
│   └── navigation_full.launch.py     # Complete system (sim-aware, mode:=slam|nav2)
├── rviz/
│   └── navigation.rviz               # RViz configuration
├── scripts/
│   └── save_map.sh                   # Map saving utility
├── src/
│   └── icm20948_node.cpp             # ICM-20948 IMU driver node
├── include/jetank_navigation/
│   └── utils.hpp                     # Pure FOV/angle math helpers (header-only)
└── test/
    └── test_utils.cpp                # gtests for utils.hpp helpers
```

## Dependencies

```bash
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-rplidar-ros
```

(In this workspace these come from the pixi/RoboStack env — see the root `pixi.toml`.)

## Real-robot Quick Start

### 1. Build

```bash
pixi run build-nav          # or: colcon build --packages-select jetank_navigation
```

### 2. Create a map (SLAM)

```bash
# Full hardware bring-up (URDF + motor + IMU + lidar) in SLAM mode
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

# Drive around (teleop), then save the map
ros2 run jetank_navigation save_map.sh my_map     # -> ~/maps/my_map.yaml
```

### 3. Navigate autonomously (Nav2)

```bash
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=nav2 map:=$HOME/maps/my_map.yaml

# In RViz: set "2D Pose Estimate", then send goals with "Nav2 Goal".
```

## Configuration

| File | Tunes |
|---|---|
| `config/icm20948.yaml` | IMU I2C bus/address, frame, publish rate |
| `config/rplidar_c1m1.yaml` | Lidar serial port, scan mode, frame |
| `config/slam/slam_toolbox.yaml` | Map resolution, travel thresholds, loop closure |
| `config/nav2/nav2_params.yaml` | Robot radius/footprint, velocity limits, costmap inflation, AMCL, BT |

Key small-robot values already set: `robot_radius: 0.12`, `inflation_radius: 0.18`,
costmap observation source `scan` on `/scan`, `odom_topic: /odom`.

## Troubleshooting

**No `/scan`** — check the lidar driver: `ros2 topic hz /scan`; on real hardware
verify the RPLidar serial port in `config/rplidar_c1m1.yaml`; in sim verify the
Gazebo `gpu_lidar` and the `ros_gz` bridge.

**Robot won't move** — confirm Nav2's final velocity reaches the base: the
`velocity_smoother` republishes to `/cmd_vel`; in sim the web `cmd_vel_bridge`
maps that to `/diff_drive_controller/cmd_vel` (TwistStamped).

**AMCL won't localize** — set a better initial pose (or publish `/initialpose`);
check `map → odom → base_link` is complete (`ros2 run tf2_tools view_frames`).

**Nav2 stack tears down under load** — ensure `bond_timeout: 0.0` is applied (see
the sim launch notes above).

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [JeTank Robot Description](../jetank_description/)

## License

MIT — see the workspace `LICENSE`.
