# JeTank Navigation - Troubleshooting Guide

Comprehensive troubleshooting for common navigation issues.

## Quick Diagnostics

Run this first to check system health:

```bash
#!/bin/bash
echo "=== JeTank Navigation Diagnostics ==="

echo -e "\n1. Checking ROS2 topics..."
ros2 topic list | grep -E '(scan|odom|points|cmd_vel|map)'

echo -e "\n2. Checking topic rates..."
timeout 3 ros2 topic hz /scan 2>&1 | grep "average" &
timeout 3 ros2 topic hz /odom 2>&1 | grep "average" &
timeout 3 ros2 topic hz /stereo_camera/points 2>&1 | grep "average" &
wait

echo -e "\n3. Checking TF tree..."
ros2 run tf2_ros tf2_echo base_link camera_left_link 2>&1 | head -5

echo -e "\n4. Checking nodes..."
ros2 node list | grep -E '(laser|slam|controller|planner|amcl)'

echo "=== Diagnostics Complete ==="
```

---

## Problem Categories

- [LaserScan Issues](#laserscan-issues)
- [SLAM/Mapping Issues](#slam-mapping-issues)
- [Navigation Issues](#navigation-issues)
- [Localization (AMCL) Issues](#localization-amcl-issues)
- [TF/Transform Issues](#tf-transform-issues)
- [Performance Issues](#performance-issues)
- [Launch File Issues](#launch-file-issues)

---

## LaserScan Issues

### Problem: No `/scan` topic

**Symptoms**:
```bash
$ ros2 topic list | grep scan
# No output
```

**Diagnosis**:
```bash
# Check if laser_data_node is running
ros2 node list | grep laser_data

# Check point cloud input
ros2 topic hz /stereo_camera/points
```

**Solutions**:

1. **Camera not running**:
   ```bash
   ros2 launch jetank_ros2_main stereo_camera.launch.py
   ```

2. **Laser node not launched**:
   ```bash
   ros2 launch jetank_navigation laser_scan_converter.launch.py
   ```

3. **Check node logs**:
   ```bash
   ros2 node info /laser_data_node
   ```

### Problem: LaserScan published but all ranges are max_range

**Symptoms**:
```bash
$ ros2 topic echo /scan --once
ranges: [1.5, 1.5, 1.5, ...]  # All max values
```

**Causes**:
- Poor point cloud quality
- Incorrect height filtering
- Transform issues

**Solutions**:

1. **Check point cloud quality**:
   ```bash
   ros2 topic echo /stereo_camera/points --once
   # Should have valid x,y,z data
   ```

2. **Adjust height filtering** in `config/laser_data.yaml`:
   ```yaml
   conversion.height: 0.05   # Try 0.0 to 0.10
   conversion.range: 0.01    # Increase tolerance
   ```

3. **Improve camera lighting**:
   - Ensure good illumination
   - Avoid direct sunlight
   - Check camera exposure settings

4. **Verify transform**:
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_left_link
   ```

### Problem: Laser scan has noise/spikes

**Symptoms**:
- Random distant points in scan
- Unstable scan readings

**Solutions**:

1. **Adjust noise filtering** in `config/laser_data.yaml`:
   ```yaml
   filter.max_distance: 0.03  # Stricter filtering
   filter.group_amount: 6     # Larger window
   ```

2. **Check camera calibration**:
   - Recalibrate stereo camera
   - Verify calibration file loaded

3. **Reduce ambient interference**:
   - Avoid reflective surfaces
   - Remove transparent objects
   - Check for IR interference

---

## SLAM Mapping Issues

### Problem: Map not building

**Symptoms**:
- RViz shows gray grid but no map
- `/map` topic not publishing

**Diagnosis**:
```bash
# Check SLAM node
ros2 node list | grep slam

# Check map topic
ros2 topic hz /map

# Check odometry
ros2 topic hz /odom
```

**Solutions**:

1. **SLAM node not running**:
   ```bash
   ros2 launch jetank_navigation slam.launch.py
   ```

2. **No odometry**:
   ```bash
   # Verify motor controller running
   ros2 launch jetank_ros2_main motor_controller.launch.py

   # Check odom topic
   ros2 topic echo /odom --once
   ```

3. **Robot not moving**:
   - Drive robot at least 10cm
   - SLAM only updates on movement

### Problem: Poor map quality

**Symptoms**:
- Walls are thick/blurry
- Features misaligned
- Map drifts over time

**Solutions**:

1. **Drive slower**:
   - Max 0.2 m/s during mapping
   - Pause at corners
   - Smooth acceleration

2. **Improve odometry**:
   - Calibrate wheel encoders
   - Check for wheel slippage
   - Verify motor controller accuracy

3. **Tune SLAM parameters** in `config/slam/slam_toolbox.yaml`:
   ```yaml
   # More conservative
   minimum_travel_distance: 0.15  # From 0.1
   minimum_travel_heading: 0.3    # From 0.2

   # Better scan matching
   correlation_search_space_dimension: 0.3  # From 0.5
   ```

4. **Better lighting**:
   - Even illumination
   - Avoid shadows
   - Consistent brightness

### Problem: No loop closure

**Symptoms**:
- Map drifts when returning to start
- Duplicate rooms in map
- Features don't align

**Solutions**:

1. **Complete full loops**:
   - Return to starting position
   - Revisit recognizable features
   - Drive same path twice

2. **Increase loop search distance** in `config/slam/slam_toolbox.yaml`:
   ```yaml
   loop_search_maximum_distance: 5.0  # From 3.0
   ```

3. **Ensure distinct features**:
   - Avoid featureless walls
   - Add landmarks if needed
   - Vary environment texture

---

## Navigation Issues

### Problem: Robot won't move to goal

**Symptoms**:
- Goal accepted but no movement
- No `/cmd_vel` published

**Diagnosis**:
```bash
# Check cmd_vel
ros2 topic echo /cmd_vel

# Check controller server
ros2 node info /controller_server

# Check local costmap
ros2 topic echo /local_costmap/costmap --once
```

**Solutions**:

1. **Motor controller not receiving commands**:
   ```bash
   # Check remapping
   ros2 topic info /cmd_vel

   # Verify motor controller subscribes
   ros2 node info /motor_controller_node
   ```

2. **Robot stuck in costmap**:
   - Reduce inflation radius
   - Clear costmaps: RViz → Navigation 2 → Clear All Costmaps
   - Check for phantom obstacles

3. **Goal unreachable**:
   - Verify goal is in free space
   - Check global planner found path
   - Try closer goal first

### Problem: Robot rotates in place / won't reach goal

**Symptoms**:
- Spins continuously
- Gets close to goal but doesn't stop
- Oscillates

**Solutions**:

1. **Tune goal tolerance** in `config/nav2/nav2_params.yaml`:
   ```yaml
   general_goal_checker:
     xy_goal_tolerance: 0.15  # From 0.10 (more relaxed)
     yaw_goal_tolerance: 0.20 # From 0.15
   ```

2. **Adjust DWB critics**:
   ```yaml
   FollowPath:
     GoalDist.scale: 30.0     # From 24.0 (prioritize reaching goal)
     RotateToGoal.scale: 30.0 # From 32.0 (less rotation emphasis)
   ```

3. **Check odometry drift**:
   - Verify odom accuracy
   - Recalibrate encoders
   - Check for wheel slip

### Problem: Robot too cautious / won't navigate tight spaces

**Symptoms**:
- Refuses to go through doorways
- Takes very wide paths
- Declares goal unreachable

**Solutions**:

1. **Reduce inflation** in `config/nav2/nav2_params.yaml`:
   ```yaml
   inflation_layer:
     inflation_radius: 0.20   # From 0.30
     cost_scaling_factor: 2.0 # From 3.0
   ```

2. **Reduce robot footprint**:
   ```yaml
   robot_radius: 0.12  # From 0.15
   ```

3. **Verify actual robot dimensions**:
   - Measure physical robot
   - Account for protruding sensors
   - Add minimal safety margin

---

## Localization (AMCL) Issues

### Problem: Particles won't converge

**Symptoms**:
- Green particle cloud stays spread out
- Robot position jumps around
- Never gets good localization

**Solutions**:

1. **Better initial pose**:
   - Set pose very accurately
   - Match orientation precisely
   - Set pose in distinctive area

2. **Increase particle count** in `config/nav2/amcl.yaml`:
   ```yaml
   max_particles: 3000  # From 2000
   min_particles: 1000  # From 500
   ```

3. **Drive robot slowly**:
   - Move forward/backward
   - Rotate in place
   - Give AMCL time to update

4. **Improve scan matching**:
   ```yaml
   laser_likelihood_max_dist: 3.0  # From 2.0
   max_beams: 120                  # From 60
   ```

### Problem: Localization drifts over time

**Symptoms**:
- Good initial localization
- Slowly drifts off course
- Position becomes incorrect

**Solutions**:

1. **Tune odometry noise** in `config/nav2/amcl.yaml`:
   ```yaml
   # Increase if odometry is poor
   alpha1: 0.3  # From 0.2 (rotation from rotation)
   alpha2: 0.3  # From 0.2 (rotation from translation)
   alpha3: 0.3  # From 0.2 (translation from translation)
   alpha4: 0.3  # From 0.2 (translation from rotation)
   ```

2. **Calibrate odometry**:
   - Measure actual vs reported distance
   - Adjust wheel diameter in motor controller
   - Check for systematic errors

3. **Increase update frequency**:
   ```yaml
   update_min_d: 0.05  # From 0.1 (update more often)
   update_min_a: 0.1   # From 0.2
   ```

---

## TF Transform Issues

### Problem: "Transform failed" warnings

**Symptoms**:
```
[WARN] Transform failed: [...]
```

**Diagnosis**:
```bash
# View complete TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check specific transforms
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_left_link
```

**Solutions**:

1. **Missing transform**:
   - Ensure robot_state_publisher running
   - Verify URDF loads correctly
   - Check SLAM or AMCL publishing map→odom

2. **Stale transform**:
   - Increase transform tolerance
   - Check node update rates
   - Verify clock synchronization

3. **Wrong frame names**:
   - Check config files for typos
   - Verify frame_id in messages
   - Ensure consistent naming

### Problem: TF tree disconnected

**Symptoms**:
- Frames don't connect
- Missing parent/child relationships

**Solutions**:

```bash
# Check what's publishing each transform
ros2 run tf2_ros tf2_monitor

# Verify static transforms
ros2 topic echo /tf_static

# Check URDF
ros2 param get /robot_state_publisher robot_description
```

---

## Performance Issues

### Problem: High CPU usage

**Diagnosis**:
```bash
top -p $(pgrep -f 'ros2|slam|nav2')
```

**Solutions**:

1. **Reduce costmap update rates**:
   ```yaml
   local_costmap:
     update_frequency: 3.0   # From 5.0
   global_costmap:
     update_frequency: 0.5   # From 1.0
   ```

2. **Reduce particle count**:
   ```yaml
   max_particles: 1000  # From 2000
   ```

3. **Lower planner frequency**:
   ```yaml
   expected_planner_frequency: 10.0  # From 20.0
   ```

4. **Reduce laser scan beams**:
   ```yaml
   max_beams: 40  # From 60
   ```

### Problem: Slow navigation

**Symptoms**:
- Robot moves very slowly
- Takes long time to plan

**Solutions**:

1. **Increase velocity limits**:
   ```yaml
   max_vel_x: 0.4  # From 0.3
   max_vel_theta: 1.2  # From 1.0
   ```

2. **Increase acceleration**:
   ```yaml
   acc_lim_x: 0.7  # From 0.5
   acc_lim_theta: 1.5  # From 1.0
   ```

3. **Simplify planner**:
   - Use NavFn instead of Smac (already using NavFn ✓)
   - Reduce costmap resolution if needed

---

## Launch File Issues

### Problem: Package not found

**Error**:
```
PackageNotFoundError: "package 'jetank_ros_main' not found"
```

**Solution**:
```bash
# Check correct package name
ros2 pkg list | grep jetank

# Rebuild workspace
colcon build
source install/setup.bash
```

### Problem: Parameter file not loaded

**Symptoms**:
- Nodes use default parameters
- Warning about missing parameters

**Solutions**:

1. **Check file path**:
   ```bash
   ls install/jetank_navigation/share/jetank_navigation/config/nav2/
   ```

2. **Verify parameter syntax**:
   ```bash
   # YAML syntax checker
   python3 -c "import yaml; yaml.safe_load(open('config/nav2/nav2_params.yaml'))"
   ```

3. **Check launch file**:
   - Verify params_file path
   - Ensure file gets installed (CMakeLists.txt)

---

## Getting Help

### Collect Debug Information

```bash
#!/bin/bash
# Save debug info
echo "=== System Info ===" > debug.txt
ros2 doctor >> debug.txt

echo -e "\n=== Topics ===" >> debug.txt
ros2 topic list >> debug.txt

echo -e "\n=== Nodes ===" >> debug.txt
ros2 node list >> debug.txt

echo -e "\n=== TF Tree ===" >> debug.txt
ros2 run tf2_ros tf2_monitor >> debug.txt

echo -e "\n=== Param Values ===" >> debug.txt
ros2 param list /controller_server >> debug.txt

echo "Debug info saved to debug.txt"
```

### Enable Debug Logging

```bash
# Launch with debug level
ros2 launch jetank_navigation navigation_full.launch.py \
  mode:=nav2 \
  log_level:=debug
```

### Useful Commands Reference

```bash
# Check all topics
ros2 topic list

# Monitor topic rate
ros2 topic hz <topic>

# Check message content
ros2 topic echo <topic> --once

# View TF tree
ros2 run tf2_tools view_frames

# Check node health
ros2 node info <node>

# List parameters
ros2 param list <node>

# Get parameter value
ros2 param get <node> <param>

# Set parameter
ros2 param set <node> <param> <value>
```

---

## Still Having Issues?

1. Check [README.md](README.md) for detailed documentation
2. Review [QUICKSTART.md](QUICKSTART.md) for basic setup
3. Check ROS2 Nav2 documentation: https://navigation.ros.org/
4. Verify hardware connections and calibration

**Most issues are caused by**:
- Missing or incorrect transforms (TF tree)
- Poor odometry quality
- Incorrect parameter values
- Hardware/sensor problems
