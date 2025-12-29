# JeTank Navigation - Quick Start Guide

Get up and running with autonomous navigation in 5 minutes.

## Prerequisites

✅ ROS2 Humble installed
✅ JeTank robot assembled
✅ Stereo camera calibrated
✅ Motor controller configured

## Step 1: Build (2 minutes)

```bash
cd ~/workspaces/ros2_ws
colcon build --packages-select jetank_navigation
source install/setup.bash
```

**Expected output**: `Finished <<< jetank_navigation [X.Xs]`

## Step 2: First Run - Test Components (2 minutes)

### Test LaserScan Conversion

```bash
# Terminal 1: Launch camera
ros2 launch jetank_ros2_main stereo_camera.launch.py

# Terminal 2: Launch converter
ros2 launch jetank_navigation laser_scan_converter.launch.py

# Terminal 3: Check output
ros2 topic hz /scan
```

**Expected**: `average rate: ~30.000`

✅ If you see scan data, conversion is working!

## Step 3: Create Your First Map (10 minutes)

```bash
# Launch full SLAM system
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam
```

**What you should see**:
- RViz window opens
- Robot model displayed
- Red laser scan points
- Map building in real-time (gray/black/white grid)

**Drive the robot**:
- Use keyboard teleop, joystick, or manual control
- Drive slowly (< 0.2 m/s)
- Cover all areas of your room
- Complete at least one full loop

**Save the map**:
```bash
# In new terminal
source install/setup.bash
ros2 run jetank_navigation save_map.sh test_map
```

**Output**: Map saved to `~/maps/test_map.yaml`

## Step 4: Navigate Autonomously (5 minutes)

```bash
# Launch with your new map
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=nav2 \
  map:=$HOME/maps/test_map.yaml
```

**In RViz**:

1. **Set Initial Pose**:
   - Click "2D Pose Estimate" button (top toolbar)
   - Click on map where robot is
   - Drag to set orientation
   - Green particle cloud should appear

2. **Send Goal**:
   - Click "Nav2 Goal" button
   - Click destination on map
   - Drag to set goal orientation
   - Robot plans path (yellow line)
   - Robot navigates autonomously!

## Common Issues

### ❌ No laser scan

**Check**:
```bash
ros2 topic list | grep scan
ros2 topic hz /stereo_camera/points
```

**Fix**: Ensure camera is running

### ❌ Map not building

**Check**:
```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

**Fix**: Ensure motor controller is publishing odometry

### ❌ Robot won't move

**Check**:
```bash
ros2 topic echo /cmd_vel
ros2 topic list | grep cmd_vel
```

**Fix**: Verify motor controller is subscribed to `/cmd_vel`

### ❌ AMCL particles won't converge

**Fix**:
- Set initial pose closer to actual position
- Drive robot slowly to help localization
- Check odometry is accurate

## Next Steps

✅ **You now have working autonomous navigation!**

**Try these next**:
- Map multiple rooms
- Tune velocity limits for faster navigation
- Adjust costmap inflation for tighter navigation
- Create patrol routes with waypoints

## Full Documentation

For detailed configuration and troubleshooting:
- [README.md](README.md) - Complete documentation
- [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) - Technical details

## Cheat Sheet

```bash
# SLAM mode (mapping)
ros2 launch jetank_navigation navigation_full.launch.py mode:=slam

# Nav2 mode (navigation)
ros2 launch jetank_navigation navigation_full.launch.py mode:=nav2 map:=$HOME/maps/my_map.yaml

# Save map
ros2 run jetank_navigation save_map.sh <name>

# Check topics
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom --once

# Check TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
```

---

**Happy navigating!** 🤖
