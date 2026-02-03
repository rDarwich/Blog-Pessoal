---
title: Setting Up Sensor TF Based on base_link
author: hangyo-cho
date: 2026-02-02 10:00:00 +0900
categories: [build, beginner]
tags: [TF, cartographer, imu, lidar, vesc]
image:
  path: /assets/img/posts/sensor-tf-setup/tf-tree-visualization.png
lang: en
lang_ref: sensor-tf-setup
---

## Introduction

One of the first challenges you'll encounter when developing a robot is **sensor coordinate system setup**. To fuse data from multiple sensors like LiDAR, IMU, and cameras, you need to precisely define where each sensor is mounted on the robot.

This guide covers how to set up sensor coordinate systems using ROS's TF (Transform) system on the Roboracer platform (SRX1 vehicle).

---

## What is TF (Transform)?

In ROS, **TF** is a system that manages the transformation relationships between different coordinate frames.

For example:
- The LiDAR is 30cm away from the robot's center (`base_link`)
- The IMU is located 6cm to the left of the robot's center

TF's role is to define and manage these **spatial relationships**.

### Why Do We Need TF?

Each sensor publishes data in its own coordinate frame:
- LiDAR measures obstacle distances based on the `laser` frame
- IMU measures acceleration/angular velocity based on the `imu` frame
- Camera captures images based on the `camera_link` frame

To fuse this data, we need to transform it to a **common coordinate system**, and TF provides the position/orientation information for each sensor.

---

## UNICORN SRX1 TF Structure

### Frame Hierarchy

```
map
 └─ odom (or carto_odom)
     └─ base_link ← Robot's center coordinate frame
         ├─ imu (Microstrain IMU)
         │   └─ imu_rot (rotation correction)
         ├─ laser (Hokuyo LiDAR)
         ├─ vesc_imu (VESC built-in IMU)
         │   └─ vesc_imu_rot
         └─ chassis (vehicle body)
             └─ zed_camera_link (ZED camera)
                 ├─ camera_link (left lens)
                 └─ zed_camera_right_link (right lens)
```

### Main Frame Descriptions

| Frame | Description | Parent Frame |
|-------|-------------|--------------|
| `map` | Global fixed coordinate frame (SLAM map) | - |
| `odom` | Odometry coordinate frame (with drift) | map |
| `base_link` | Robot center (center of rear axle, ground level) | odom |
| `imu` | IMU sensor position | base_link |
| `laser` | LiDAR sensor position | base_link |
| `vesc_imu` | VESC motor controller IMU | base_link |
| `chassis` | Vehicle body (5cm above base_link) | base_link |
| `zed_camera_link` | ZED stereo camera | chassis |

---

## Coordinate System Convention

Several coordinate system conventions are used in robotics. UNICORN follows the **ROS standard** (REP-105).

### base_link: FLU (Forward-Left-Up)

![FLU Coordinate](/assets/img/posts/sensor-tf-setup/flu-coordinate.png)

- **X-axis**: Forward (direction the vehicle travels)
- **Y-axis**: Left (driver's left side)
- **Z-axis**: Up (toward the sky)

### map/odom: ENU (East-North-Up)

- **X-axis**: East
- **Y-axis**: North
- **Z-axis**: Up

### Why Use Different Conventions?

- `base_link` is a **robot-centric** coordinate frame → based on robot movement
- `map`/`odom` are **world coordinate frames** → based on the map

It's natural to process sensor data relative to the robot's center and express the final position in the map coordinate frame.

---

## SRX1 Sensor Layout

Let's examine where the sensors are mounted on the actual SRX1 vehicle.

### Sensor Positions (relative to base_link)

| Sensor | X (m) | Y (m) | Z (m) | Description |
|--------|-------|-------|-------|-------------|
| **IMU** | 0.085 | 0.06 | 0.065 | Front of vehicle, offset to left |
| **LiDAR** | 0.270 | 0.0 | 0.127 | Vehicle centerline, front |
| **VESC IMU** | 0.10 | 0.0 | 0.127 | Built into VESC motor controller |
| **ZED Camera** | 0.390 | 0.0 | 0.09 | Far front, center |

### Sensor Layout Characteristics

```
      Front
        ↑
    [LiDAR]  (0.27, 0, 0.127)
        |
     [IMU]   (0.085, 0.06, 0.065) ← 6cm left
        |
    [VESC]   (0.10, 0, 0.127)
        |
    base_link (0, 0, 0)
```

The SRX1's Microstrain IMU is mounted **6cm to the left** because the vehicle structure makes it difficult to mount at the center. Even with this offset, the TF system automatically compensates.

---

## TF Configuration File

The SRX1's sensor TF is defined in the following file:

```
UNICORN/stack_master/config/SRX1/devices/static_transforms.launch.xml
```

### File Contents

```xml
<!-- -*- mode: XML -*- -->
<launch>
   <!-- TFs -->
  <arg name="pub_map_to_odom" default="False"/>

  <!-- 1. IMU Transform -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_imu"
        args="0.085 0.06 0.065 0.0 1.0 0.0 0.0 base_link imu" />

  <node pkg="tf2_ros" type="static_transform_publisher" name="imu_to_imu_rot"
        args="0.0 0.0 0.0 0.0 1.0 0.0 0.0 imu imu_rot" />

  <!-- 2. LiDAR Transform -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_laser"
        args="0.270 0.0 0.127 0.0 0.0 0.0 1.0 base_link laser" />

  <!-- 3. VESC IMU Transform -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_vesc_imu"
        args="0.10 0.0 0.127 0.0 0.0 -0.7071 0.7071 base_link vesc_imu" />

  <node pkg="tf2_ros" type="static_transform_publisher" name="vesc_imu_to_vesc_imu_rot"
        args="0.0 0.0 0.0 0.0 0.0 0.7071 0.7071 vesc_imu vesc_imu_rot" />

  <!-- 4. Map to Odom (optional) -->
  <group if="$(arg pub_map_to_odom)">
    <node pkg="tf" type="static_transform_publisher" name="map_to_odom"
          args="0 0 0 0 0 0 map odom 10" />
  </group>
</launch>
```

---

## Interpreting TF Parameters

### static_transform_publisher args Format

```
args="x y z qx qy qz qw parent_frame child_frame"
```

- **x, y, z**: Position (unit: meters)
- **qx, qy, qz, qw**: Rotation (Quaternion format)
- **parent_frame**: Parent coordinate frame
- **child_frame**: Child coordinate frame

### Example 1: LiDAR Transform

```xml
args="0.270 0.0 0.127 0.0 0.0 0.0 1.0 base_link laser"
```

- Position: (0.27, 0, 0.127) m
  - 27cm forward
  - Centered left-right (0)
  - 12.7cm up
- Rotation: (0, 0, 0, 1) = No rotation (identity)
- Transform from `base_link` to `laser`

This means the LiDAR is located 27cm forward and 12.7cm up from the robot's center, facing the same direction as base_link.

### Example 2: IMU Transform

```xml
args="0.085 0.06 0.065 0.0 1.0 0.0 0.0 base_link imu"
```

- Position: (0.085, 0.06, 0.065) m
  - 8.5cm forward
  - 6cm left
  - 6.5cm up
- Rotation: (0, 1, 0, 0) = **180-degree pitch rotation**
- Transform from `base_link` to `imu`

This means the IMU is offset to the left and physically mounted upside down (180-degree rotation).

---

## Understanding Quaternion Rotation

### What is a Quaternion?

It's one way to represent 3D rotation. Rotation is expressed with 4 numbers (qx, qy, qz, qw).

### Commonly Used Rotations

| Rotation | Quaternion (qx, qy, qz, qw) | Description |
|----------|---------------------------|-------------|
| No rotation | (0, 0, 0, 1) | Identity |
| Z-axis 90° | (0, 0, 0.7071, 0.7071) | Turn left |
| Z-axis -90° | (0, 0, -0.7071, 0.7071) | Turn right |
| Y-axis 180° | (0, 1, 0, 0) | Flipped |
| X-axis 90° | (0.7071, 0, 0, 0.7071) | Tilt left |

### Why is the IMU Rotated 180 Degrees?

```
Normal:    Flipped (180° pitch):
  ↑ Z         Z  ↓
  |              |
  o--→ X    X ←--o
```

The IMU sensor is physically mounted upside down. However, with TF software compensation, **the data can be used normally**.

### Why is imu_rot Needed?

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="imu_to_imu_rot"
      args="0.0 0.0 0.0 0.0 1.0 0.0 0.0 imu imu_rot" />
```

`imu_rot` is used as the **tracking_frame** for Cartographer (SLAM algorithm). It rotates the IMU another 180 degrees to restore the original orientation.

```
base_link --[180° pitch]--> imu --[180° pitch]--> imu_rot
```

As a result, `imu_rot` has the same orientation as `base_link`, but passes through the `imu` frame in between.

---

## VESC IMU Rotation

```xml
<!-- VESC IMU: Z-axis -90 degree rotation -->
args="0.10 0.0 0.127 0.0 0.0 -0.7071 0.7071 base_link vesc_imu"

<!-- VESC IMU Rotation: Z-axis +90 degree rotation -->
args="0.0 0.0 0.0 0.0 0.0 0.7071 0.7071 vesc_imu vesc_imu_rot"
```

### Why Two Rotations?

The IMU built into the VESC motor controller is **rotated 90 degrees around the Z-axis** according to the PCB board orientation.

```
base_link: X→ Y↑     vesc_imu: X↑ Y←     vesc_imu_rot: X→ Y↑
```

1. **vesc_imu**: Physical mount direction (-90°)
2. **vesc_imu_rot**: Software correction (+90°)

Finally, `vesc_imu_rot` has the same orientation as `base_link`.

---

## TF Configuration Loading Process

### Launch File Call Order

```
base_system.launch
    └─ middle_level.launch
        └─ low_level.launch  ← TF loaded here!
            ├─ LiDAR driver
            ├─ IMU driver
            ├─ VESC driver
            └─ static_transforms.launch.xml  ← TF configuration
```

### Importance of CAR_NAME Environment Variable

```bash
export CAR_NAME=SRX1
```

This environment variable must be set for the correct vehicle's TF file to load:

```bash
$(find stack_master)/config/$(arg racecar_version)/devices/static_transforms.launch.xml
→ UNICORN/stack_master/config/SRX1/devices/static_transforms.launch.xml
```

> Using the wrong CAR_NAME will load sensor position information for a different vehicle, causing inaccurate sensor fusion and degraded SLAM performance.
{: .prompt-warning }

---

## Verifying TF

### 1. TF Tree Visualization

```bash
# After running ROS
rosrun tf view_frames

# PDF file generated
evince frames.pdf
```

### 2. Check Specific Transform

```bash
# Transform between LiDAR and base_link
rosrun tf tf_echo base_link laser

# Output:
# At time 1234.567
# - Translation: [0.270, 0.000, 0.127]
# - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
#            in RPY (radian) [0.000, 0.000, 0.000]
#            in RPY (degree) [0.000, 0.000, 0.000]
```

```bash
# Transform between IMU and base_link
rosrun tf tf_echo base_link imu

# Output:
# - Translation: [0.085, 0.060, 0.065]
# - Rotation: in Quaternion [0.000, 1.000, 0.000, 0.000]
#            in RPY (degree) [0.000, 180.000, 0.000]  ← 180 degree pitch
```

### 3. Check Static TF

```bash
rostopic echo /tf_static -n 20
```

You can verify that all static transforms are being published.

### 4. Visualize with RViz

```bash
rosrun rviz rviz
```

**RViz Settings**:
1. Set `Fixed Frame` to `base_link`
2. `Add` → Select `TF`
3. `Show Names`: Check → Display frame names
4. `Show Axes`: Check → Display coordinate axes
5. `Show Arrows`: Check → Display parent-child relationship arrows

**What You See**:
- Red arrow: X-axis (forward)
- Green arrow: Y-axis (left)
- Blue arrow: Z-axis (up)

You can intuitively verify the position and orientation of each sensor.

---

## Modifying Sensor TF

### When to Modify TF?

1. **Sensor position change**: Physically mounting a sensor in a different location
2. **Adding sensors**: Installing new sensors
3. **Calibration**: Updating after more accurate position measurement

### Modification Procedure

#### 1. Measure Physical Position

Measure the exact position of the sensor:

```
Reference point: base_link (center of rear axle, ground level)

Measurement items:
- X: Forward distance (m)
- Y: Left-right distance (m, left is +)
- Z: Height (m)
- Rotation: roll, pitch, yaw (degrees or radians)
```

> **Measurement tips**: Measure to cm accuracy with a tape measure. X-axis is vehicle forward (usually +), Y-axis is + for left, - for right, Z-axis is height from ground to sensor.
{: .prompt-tip }

#### 2. Calculate Quaternion

Convert Roll, Pitch, Yaw to Quaternion:

```python
#!/usr/bin/env python3
from tf.transformations import quaternion_from_euler
import math

# Convert angles to radians
roll = math.radians(0)    # X-axis rotation
pitch = math.radians(180) # Y-axis rotation (e.g., flipped IMU)
yaw = math.radians(0)     # Z-axis rotation

# Calculate Quaternion
quat = quaternion_from_euler(roll, pitch, yaw)

print(f"Quaternion:")
print(f"  qx = {quat[0]:.4f}")
print(f"  qy = {quat[1]:.4f}")
print(f"  qz = {quat[2]:.4f}")
print(f"  qw = {quat[3]:.4f}")

# Example output:
# Quaternion:
#   qx = 0.0000
#   qy = 1.0000
#   qz = 0.0000
#   qw = 0.0000
```

#### 3. Modify Launch File

```bash
cd ~/unicorn_ws/UNICORN/stack_master/config/SRX1/devices
code static_transforms.launch.xml
```

**Example: Change LiDAR position to 30cm forward**

Before:
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_laser"
      args="0.270 0.0 0.127 0.0 0.0 0.0 1.0 base_link laser" />
```

After:
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_laser"
      args="0.300 0.0 0.127 0.0 0.0 0.0 1.0 base_link laser" />
```

↑ Changed X value from 0.270 to 0.300

#### 4. Restart and Verify

```bash
# Restart ROS
roslaunch stack_master base_system.launch map:=YOUR_MAP

# Verify in new terminal
rosrun tf tf_echo base_link laser
```

---

## Adding Sensors

Let's assume we're adding a new sensor (e.g., RealSense camera).

### 1. Determine Sensor Position

- Position: (0.35, 0.0, 0.10) - 35cm forward, center, 10cm height
- Rotation: Pointing 15 degrees down (pitch = -15 degrees)

### 2. Calculate Quaternion

```python
from tf.transformations import quaternion_from_euler
import math

pitch = math.radians(-15)  # 15 degrees down
quat = quaternion_from_euler(0, pitch, 0)

# Result: (0.0, -0.1305, 0.0, 0.9914)
```

### 3. Add to static_transforms.launch.xml

```xml
<!-- RealSense Camera -->
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_realsense"
      args="0.35 0.0 0.10 0.0 -0.1305 0.0 0.9914 base_link realsense_link" />
```

---

## Cartographer and TF

### mapping.lua Configuration

```lua
-- SRX1/SE/slam/mapping.lua

options = {
  map_frame = "map",
  tracking_frame = "imu_rot",      -- ← Frame Cartographer will track
  published_frame = "base_link",   -- ← Frame to publish results
  odom_frame = "carto_odom",

  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  num_laser_scans = 1,
  use_odometry = false,
}
```

### Meaning of tracking_frame = "imu_rot"

Cartographer tracks the **orientation** of the `imu_rot` frame.

**Why imu_rot instead of base_link?**
- IMU provides **high-frequency pose data** (100-250Hz)
- base_link has no sensor (virtual coordinate frame)
- Tracking IMU provides **smooth and accurate** pose estimation

**Meaning of published_frame = "base_link"**:
- However, final results are published as `base_link`
- Automatic transformation via TF: `imu_rot` → `imu` → `base_link`

---

## Troubleshooting

### Problem 1: TF Not Being Published

**Symptoms**:
```bash
rosrun tf tf_echo base_link laser
# Exception thrown: Frame laser does not exist
```

**Causes**:
1. `static_transforms.launch.xml` not loaded
2. Wrong `CAR_NAME` environment variable
3. Launch file syntax error

**Solutions**:
```bash
# 1. Check CAR_NAME
echo $CAR_NAME
# Output: SRX1 (normal) or empty (abnormal)

# 2. Check static_transform_publisher nodes
rosnode list | grep static_transform_publisher
# Output: /base_link_to_imu, /base_link_to_laser, etc.

# 3. Check launch file syntax
roslaunch --check stack_master low_level.launch

# 4. Test TF publishing manually
rosrun tf2_ros static_transform_publisher 0.27 0 0.127 0 0 0 1 base_link laser
```

### Problem 2: Cartographer Can't Find IMU

**Symptoms**:
```
[ERROR] [cartographer]: Failed to compute relative pose.
        Lookup would require extrapolation at time 1234.567,
        but only time 1234.560 is in the buffer.
```

**Causes**:
- `imu_rot` frame doesn't exist
- Wrong frame_id in IMU data

**Solutions**:
```bash
# 1. Check imu_rot frame
rosrun tf tf_echo base_link imu_rot

# 2. Check IMU data frame_id
rostopic echo /imu/data -n 1 | grep frame_id
# Output: frame_id: "imu"  ← Should be "imu"

# 3. Check frame_id in imu.yaml
cat ~/unicorn_ws/UNICORN/stack_master/config/SRX1/devices/imu.yaml | grep frame_id
# Output: frame_id : 'imu'
```

### Problem 3: Sensor Data Displayed in Wrong Position

**Symptoms**:
- LiDAR scan displayed behind the vehicle in RViz
- IMU orientation is reversed

**Causes**:
- Wrong position/rotation values in TF
- Quaternion sign error

**Solutions**:
```bash
# 1. Check TF values
rosrun tf tf_echo base_link laser

# 2. Compare with expected values
# X is negative? → Sensor is recognized as behind
# Quaternion sign is reversed? → 180 degrees off

# 3. Modify static_transforms.launch.xml
# Change X from negative to positive, or change quaternion sign
```

---

## Summary

Without accurate TF:
- SLAM won't work properly
- Sensor fusion is impossible
- Path planning will have errors

However, once configured, **all sensor data is automatically aligned**.

### Key Points

1. **base_link is the center**: All sensors are defined relative to base_link
2. **Static TF**: Since sensor positions don't change, use static_transform_publisher
3. **FLU Convention**: ROS standard (Forward-Left-Up)
4. **Quaternion**: Rotation is expressed in quaternion format
5. Always verify with `tf_echo`, `view_frames`, and RViz

### Reference

- [ROS TF Tutorial](http://wiki.ros.org/tf/Tutorials)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Quaternion Basics](http://wiki.ros.org/tf2/Tutorials/Quaternions)
- [Cartographer ROS Integration](https://google-cartographer-ros.readthedocs.io/)

---

## SRX1 Complete TF Configuration

### static_transforms.launch.xml

```xml
<!-- -*- mode: XML -*- -->
<launch>
   <!-- TFs -->
  <arg name="pub_map_to_odom" default="False"/>

  <!-- IMU (Microstrain) -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_imu"
        args="0.085 0.06 0.065 0.0 1.0 0.0 0.0 base_link imu" />
  <node pkg="tf2_ros" type="static_transform_publisher" name="imu_to_imu_rot"
        args="0.0 0.0 0.0 0.0 1.0 0.0 0.0 imu imu_rot" />

  <!-- LiDAR (Hokuyo) -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_laser"
        args="0.270 0.0 0.127 0.0 0.0 0.0 1.0 base_link laser" />

  <!-- VESC IMU (built-in) -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_vesc_imu"
        args="0.10 0.0 0.127 0.0 0.0 -0.7071 0.7071 base_link vesc_imu" />
  <node pkg="tf2_ros" type="static_transform_publisher" name="vesc_imu_to_vesc_imu_rot"
        args="0.0 0.0 0.0 0.0 0.0 0.7071 0.7071 vesc_imu vesc_imu_rot" />

  <!-- Map to Odom (optional) -->
  <group if="$(arg pub_map_to_odom)">
    <node pkg="tf" type="static_transform_publisher" name="map_to_odom"
          args="0 0 0 0 0 0 map odom 10" />
  </group>
</launch>
```

### Sensor Position Summary (SRX1)

| Frame | Parent | X | Y | Z | Rotation (deg) |
|-------|--------|---|---|---|----------------|
| `imu` | `base_link` | 0.085 | 0.06 | 0.065 | pitch=180 |
| `imu_rot` | `imu` | 0 | 0 | 0 | pitch=180 |
| `laser` | `base_link` | 0.270 | 0 | 0.127 | none |
| `vesc_imu` | `base_link` | 0.10 | 0 | 0.127 | yaw=-90 |
| `vesc_imu_rot` | `vesc_imu` | 0 | 0 | 0 | yaw=+90 |
| `chassis` | `base_link` | 0 | 0 | 0.05 | none |
| `zed_camera_link` | `chassis` | 0.390 | 0 | 0.04 | none |
