# JeTank Navigation Learning Progress

**Last Updated**: 2025-10-30
**Current Phase**: Phase 2 - Design & Architecture
**Status**: In Progress

---

## ✅ Completed

### Phase 1: Understanding the Problem Space
- [x] Analyzed PointCloud2 data structure
- [x] Researched LaserScan message structure
- [x] Understood 3D to 2D projection concepts
- [x] Investigated coordinate frames with tf2

**Key Learnings:**
- PointCloud2 uses binary format for efficiency (640x360 = 230,400 points)
- LaserScan uses polar coordinates (angle + range)
- Projection: slice 3D cloud by height, convert to 2D polar
- Stereo camera publishes in `camera_left_link` frame
- LaserScan should be in `base_link` frame for SLAM

### Phase 2: Design & Architecture (Partial)

#### Task 2.1: Node Architecture Design ✅
- [x] Completed

#### Task 2.2: Utility Functions Created ✅
- [x] Created `include/jetank_navigation/utils.hpp`
- [x] Implemented `deg_to_rad()` - Convert degrees to radians
- [x] Implemented `diagonal_fov_to_horizontal()` - Calculate horizontal FOV from diagonal
- [x] Implemented `calculate_angle_increment()` - Calculate laser beam spacing
- [x] All functions are `constexpr` for compile-time optimization
- [x] Added M_PI fallback for portability
- [x] Documented all functions with parameter units

**Key Learnings:**
- `constexpr` functions work at both compile-time and runtime
- Integer division truncates - must cast to double for accurate calculations
- Always document units (degrees/radians) in robotics code
- Header-only implementations for simple utility functions

#### Task 2.3: Camera FOV Research ✅
- [x] Researched IMX219-83 camera specifications

**IMX219-83 FOV Specifications:**
- Diagonal FOV: 83°
- **Horizontal FOV: 73°** ← Used for laser scan
- Vertical FOV: 50°
- Source: Waveshare product documentation

---

## 🔄 In Progress

### Phase 2: Design & Architecture

#### Task 2.1: Node Architecture Design

**Decisions Made:**
1. **Node Type**: Composable component
   - Reasoning: Better efficiency with intra-process communication
   - Note: Need to research if this actually provides zero-copy

2. **Transform Strategy**: Transform after validation
   - Check data validity first (non-empty, valid timestamp)
   - Then transform from camera_link to base_link
   - Minimize overhead by filtering before transforming

3. **QoS Matching**: Match pointcloud publisher
   - Discovered: `/stereo_camera/points` uses RELIABLE, VOLATILE
   - Config should use same settings

4. **Config File Started**: `config/laser_data.yaml`

#### Current Config Parameters:
```yaml
laser_data:
  laser_data_node:
    ros__parameters:
      # Conversion parameters
      conversion.height: 0.05      # 5cm above base_link
      conversion.range: 0.005       # ±5mm height tolerance

      # Laser configuration
      laser.density: 640            # Number of laser beams
      laser.min_range: 0.3          # Match camera minimum
      laser.max_range: 1.5          # Match camera maximum
      laser.angle_min: ???          # TODO: Based on camera FOV
      laser.angle_max: ???          # TODO: Based on camera FOV
      laser.angle_increment: ???    # TODO: Calculate from density

      # Noise filtering
      filter.max_distance: 0.05     # Max distance between adjacent points [m]
      filter.group_amount: 4        # Outlier detection window
```

---

## 📋 Next Tasks (To Resume)

### Immediate TODO:

1. **Calculate Laser Scan Angle Parameters** ← NEXT
   - Convert horizontal FOV (73°) to radians: `deg_to_rad(73.0)`
   - Calculate centered scan angles:
     - `angle_min = -horizontal_fov_radians / 2`
     - `angle_max = +horizontal_fov_radians / 2`
   - Calculate `angle_increment` using `calculate_angle_increment(angle_min, angle_max, 640)`
   - Update `config/laser_data.yaml` with calculated values

2. **Add Missing Config Parameters**
   ```yaml
   # Frame configuration
   frame_id: "base_link"           # Target frame for laser scan
   source_frame: "camera_left_link" # Or check actual pointcloud frame

   # Topic names
   input_topic: "/stereo_camera/points"
   output_topic: "/scan"

   # Transform parameters
   transform_tolerance: 0.066  # 2 * message_period (30Hz = 0.033s)

   # QoS settings (match pointcloud publisher)
   qos.reliability: "reliable"
   qos.durability: "volatile"
   qos.depth: 1
   ```

### Pending Tasks (Phase 2):

- [ ] **Task 2.2**: Select algorithm for multi-point handling
  - Decide: min/max/median for multiple points at same angle
  - Reasoning based on SLAM requirements

- [ ] **Task 2.3**: Complete parameter design
  - Add all necessary parameters
  - Document sensible defaults
  - Create justifications for each value

- [ ] **Task 2.4**: Plan package structure
  - Header files location
  - Source files organization
  - Dependencies in package.xml and CMakeLists.txt

---

## 🎓 Key Questions Still to Answer

### Node Architecture:
- Does ROS2 composition actually provide zero-copy message passing?
- Research: `rclcpp::Node` vs `rclcpp::Component`
- Intra-process communication requirements

### Transform Validation:
- What checks before transforming?
  - Empty point cloud?
  - Invalid timestamp?
  - Transform availability?
  - Transform age (staleness)?
- Use `tf2_ros::Buffer::canTransform()`

### QoS Configuration:
- RELIABLE vs BEST_EFFORT trade-offs
- Queue depth for 30fps processing
- What happens if processing takes >33ms?

### Design Trade-offs:
- Transform entire cloud vs individual points?
- Process every cloud or skip if behind?
- Angular resolution: detail vs noise balance

---

## 📚 Resources Referenced

- ROS2 sensor_msgs/PointCloud2 documentation
- ROS2 sensor_msgs/LaserScan documentation
- Stereo camera config: `src/jetank_perception/config/stereo_camera_config.yaml`
- Learning plan: `LEARNING_PLAN.md`

---

## 🔗 System Specifications

**Hardware:**
- Platform: Jetson Orin Nano
- Camera: IMX219-83 Stereo Camera (60mm baseline)
- Resolution: 640x360 @ 30fps
- Depth Range: 0.3-1.5m

**ROS2:**
- Distro: Humble
- Frames: `base_link`, `camera_left_link`, `camera_right_link`
- Existing topics: `/stereo_camera/points` (PointCloud2)

---

## 💡 Notes for Next Session

1. Use learning agents proactively - they provide better guidance through discovery
2. Always ask for direct feedback output from coordinator/mentors
3. Research IMX219-83 FOV specs (check Waveshare documentation)
4. Consider robot's max speed when setting transform_tolerance
5. Height slice (5cm) seems reasonable for ground-level obstacles

**Learning Approach:**
- Let agents guide through Socratic questions
- Don't expect complete answers - discover through guided thinking
- Document design decisions with justifications

---

## 🎯 Phase Completion Criteria

**Phase 2 Success Criteria:**
- [ ] Node architecture designed and documented
- [ ] Algorithm selected with justification
- [ ] Parameter file complete with sensible defaults
- [ ] Package structure planned
- [ ] Dependencies identified

**After Phase 2:** Ready to start Phase 3 (Implementation)
