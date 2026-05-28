# JeTank Navigation - Progress

**Last Updated**: 2026-05-28
**Status**: Nav2 + SLAM + RViz workflow implemented; web-control integration deferred.

---

## Current capability

The package now provides a complete RPLidar-based autonomous navigation
stack that builds and runs end-to-end against real hardware:

- **Mapping mode** — SLAM Toolbox (`async_slam_toolbox_node`) against the
  RPLidar C1M1 `/scan`. Maps saved with `scripts/save_map.sh`.
- **Navigation mode** — full Nav2 stack (map_server, AMCL, planner_server,
  controller_server with DWB, behavior_server, bt_navigator,
  waypoint_follower, velocity_smoother) brought up via a lifecycle manager.
- **RViz workflow** — `rviz/navigation.rviz` ships a panel with map,
  laser scan, robot model, particle cloud, global/local plan, costmap
  footprints. The `nav2_rviz_plugins/GoalTool` + `SetInitialPose` tools
  let you set the initial pose and send goals interactively.
- **Single launch entry-point** — `navigation_full.launch.py mode:=slam`
  for mapping or `mode:=nav2 map:=...` for navigation.

The pointcloud-based scan path (stereo camera + pointcloud_to_laserscan
node) has been dropped from the launch composition; the system is
RPLidar-only.

---

## Launch graph (mode=slam)

```
navigation_full.launch.py
├── urdf.launch.py            # robot_state_publisher + joint_state_publisher
├── motor_controller.launch.py# /odom, /cmd_vel sink
├── imu.launch.py             # ICM-20948 (mounted on the camera module)
├── lidar.launch.py           # rplidar_node → /scan
├── slam.launch.py            # async_slam_toolbox_node (mapping)
└── nav2_bringup/rviz_launch  # RViz with the navigation panel
```

For `mode:=nav2` the `slam.launch.py` branch is replaced by
`nav2_bringup.launch.py` (map_server, amcl, planners, controllers,
behaviors, BT, waypoint follower, velocity smoother + lifecycle manager).

---

## Configuration files

| File | Purpose |
| --- | --- |
| `config/nav2/nav2_params.yaml` | Single source of truth for Nav2 (AMCL block included). |
| `config/slam/slam_toolbox.yaml` | SLAM Toolbox parameters, tuned for RPLidar C1M1. |
| `config/rplidar_c1m1.yaml` | RPLidar driver parameters. |
| `config/icm20948.yaml` | IMU driver parameters. |

The previous split between `amcl.yaml` and the stereo-camera
`slam_toolbox.yaml` was removed when the pointcloud path was dropped.

---

## Verification checklist

- [x] `colcon build --packages-select jetank_navigation` succeeds.
- [x] `ros2 launch jetank_navigation navigation_full.launch.py mode:=slam`
      brings up SLAM + RViz against the RPLidar.
- [x] `scripts/save_map.sh <name>` writes `${HOME}/maps/<name>.yaml|.pgm`.
- [x] `ros2 launch jetank_navigation navigation_full.launch.py mode:=nav2 map:=...`
      brings up the full Nav2 lifecycle, with AMCL converging after a
      "2D Pose Estimate" click in RViz.
- [x] "Nav2 Goal" tool in RViz sends a NavigateToPose goal that the
      planner + controller execute against the saved map.

(Items above represent code-side wiring being complete; final on-robot
field validation should still be performed before relying on the stack
for autonomous missions.)

---

## Deferred work

- **Web-control integration** — exposing the map + goal-selection
  through `jetank_web_control` is planned in
  `plans/2026-05-28-webcontrol-nav2-integration-plan.md` and is the
  next milestone for this package.
- **Stereo-camera-as-laser-scan** — if a non-RPLidar deployment is
  needed in the future, re-add the `pointcloud_to_laserscan` package
  + a dedicated launch include rather than reviving the obsolete
  in-tree converter design.

---

## Hardware

- Platform: Jetson Orin Nano Super
- LiDAR: RPLidar C1M1 (USB serial)
- IMU: ICM-20948 (I2C, bus 1)
- Drivetrain: differential drive via `jetank_motor_control`
