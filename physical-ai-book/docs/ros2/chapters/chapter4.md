---
title: "Chapter 4: Modeling Humanoid Robots with URDF"
description: "Understanding URDF for humanoid robot structure and simulation consistency"
sidebar_position: 4
---

# Chapter 4: Modeling Humanoid Robots with URDF

## Introduction

URDF (Unified Robot Description Format) serves as the fundamental language for describing robot structure in ROS-based systems. For humanoid robots, URDF provides the essential framework that enables both simulation and real-world deployment to share consistent models. This chapter explores how to model humanoid robots using URDF, covering the essential components and best practices.

## What is URDF and Why It Matters for Humanoid Robots

### Definition and Purpose

URDF is an XML-based format used to describe robot models in ROS systems. It defines:

- **Physical structure**: Links connected by joints
- **Visual properties**: How the robot appears in simulation
- **Collision properties**: How the robot interacts with the environment
- **Kinematic properties**: Inertial characteristics for physics simulation
- **Sensor and actuator placements**: Where to mount devices on the robot

### Importance for Humanoid Robots

Humanoid robots have complex kinematic structures with multiple degrees of freedom. URDF provides:

- **Consistency**: Same model works in simulation and real deployment
- **Visualization**: Tools like RViz can display the robot structure
- **Physics Simulation**: Gazebo uses URDF for accurate physics
- **Kinematic Solvers**: Tools like KDL and MoveIt use URDF for planning
- **Standardization**: Common format across the ROS ecosystem

## URDF Fundamentals: Links, Joints, and Kinematic Chains

### Links

A **link** represents a rigid body in the robot structure. Each link has:

- **Inertial properties**: Mass, center of mass, and inertia tensor
- **Visual properties**: How the link appears in simulation
- **Collision properties**: How the link interacts with other objects

Basic link structure:
```xml
<link name="link_name">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </collision>
</link>
```

### Joints

A **joint** connects two links with specific kinematic properties. Joint types include:

- **Fixed**: No movement between links
- **Revolute**: Rotational movement around one axis (with limits)
- **Continuous**: Rotational movement without limits
- **Prismatic**: Linear movement along one axis
- **Floating**: 6 DOF movement (for base)

Basic joint structure:
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0.1" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
</joint>
```

### Kinematic Chains

Kinematic chains are sequences of links connected by joints. For humanoid robots, common chains include:

- **Arm chains**: Shoulder → elbow → wrist
- **Leg chains**: Hip → knee → ankle
- **Spine chain**: Multiple vertebra-like links
- **Neck chain**: Base → head

## Defining Sensors and Actuators in URDF

### Sensor Mounting

Sensors are typically attached to links using fixed joints:

```xml
<!-- Mount a camera to the head -->
<joint name="head_camera_joint" type="fixed">
  <parent link="head_link" />
  <child link="camera_link" />
  <origin xyz="0.05 0 0.05" rpy="0 0 0" />
</joint>

<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02" />
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02" />
    </geometry>
  </collision>
</link>

<!-- Gazebo plugin for the camera -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Actuator Integration

Actuators (motors) are defined in joints and often have associated control interfaces:

```xml
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso_link" />
  <child link="upper_arm_link" />
  <origin xyz="0.0 0.15 0.0" rpy="0 0 0" />
  <axis xyz="1 0 0" />
  <limit lower="-1.57" upper="1.57" effort="50.0" velocity="2.0" />
  <!-- Safety limits -->
  <safety_controller k_position="100.0" k_velocity="10.0" soft_lower_limit="-1.5" soft_upper_limit="1.5" />
</joint>
```

## Complete Humanoid URDF Example

Here's a simplified humanoid model showing the essential structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base/Fixed link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2" />
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso_link" />
    <origin xyz="0 0 0.5" />
  </joint>

  <link name="torso_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0.2" />
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" />
      <geometry>
        <box size="0.3 0.3 0.4" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.2" />
      <geometry>
        <box size="0.3 0.3 0.4" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link" />
    <child link="head_link" />
    <origin xyz="0 0 0.4" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <link name="head_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0.1" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso_link" />
    <child link="left_upper_arm" />
    <origin xyz="0.2 0 0.1" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" />
      <geometry>
        <cylinder length="0.2" radius="0.05" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" />
      <geometry>
        <cylinder length="0.2" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <origin xyz="0 0 -0.2" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="2.0" effort="15.0" velocity="1.0" />
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.3" />
      <origin xyz="0 0 -0.075" />
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.0005" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" />
      <geometry>
        <cylinder length="0.15" radius="0.04" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" />
      <geometry>
        <cylinder length="0.15" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <!-- Right Arm (similar structure) -->
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso_link" />
    <child link="right_upper_arm" />
    <origin xyz="-0.2 0 0.1" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" />
      <geometry>
        <cylinder length="0.2" radius="0.05" />
      </geometry>
      <material name="blue" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" />
      <geometry>
        <cylinder length="0.2" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip" type="revolute">
    <parent link="torso_link" />
    <child link="left_thigh" />
    <origin xyz="0.07 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.0" upper="1.0" effort="50.0" velocity="1.0" />
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh" />
    <child link="left_shin" />
    <origin xyz="0 0 -0.4" />
    <axis xyz="1 0 0" />
    <limit lower="0" upper="2.0" effort="40.0" velocity="1.0" />
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="black" />
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
  </link>

</robot>
```

## URDF Best Practices for Humanoid Robots

### Modular Design with Xacro

For complex humanoid models, use Xacro (XML Macros) to create reusable components:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define macros for common components -->
  <xacro:macro name="simple_arm" params="side parent_link *origin">
    <joint name="${side}_shoulder_pitch" type="revolute">
      <parent link="${parent_link}" />
      <child link="${side}_upper_arm" />
      <xacro:insert_block name="origin" />
      <axis xyz="1 0 0" />
      <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
    </joint>

    <link name="${side}_upper_arm">
      <inertial>
        <mass value="0.5" />
        <origin xyz="0 0 -0.1" />
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin xyz="0 0 -0.1" />
        <geometry>
          <cylinder length="0.2" radius="0.05" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" />
        <geometry>
          <cylinder length="0.2" radius="0.05" />
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <link name="torso_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0.2" />
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1" />
    </inertial>
  </link>

  <xacro:simple_arm side="left" parent_link="torso_link">
    <origin xyz="0.2 0 0.1" />
  </xacro:simple_arm>

  <xacro:simple_arm side="right" parent_link="torso_link">
    <origin xyz="-0.2 0 0.1" />
  </xacro:simple_arm>

</robot>
```

### Proper Inertial Properties

Accurate inertial properties are crucial for realistic physics simulation:

```xml
<!-- Calculate inertia properly -->
<!-- For a box: Ixx = 1/12 * m * (h² + d²), etc. -->
<link name="example_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <!-- For a 0.1x0.1x0.2 box -->
    <inertia ixx="0.0017" ixy="0.0" ixz="0.0"
             iyy="0.0017" iyz="0.0"
             izz="0.0008" />
  </inertial>
</link>
```

### Gazebo-Specific Elements

Add Gazebo-specific plugins and properties:

```xml
<gazebo reference="link_name">
  <!-- Material for visualization -->
  <material>Gazebo/Blue</material>

  <!-- Physics properties -->
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>

  <!-- Sensors -->
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
      </angular_velocity>
    </imu>
  </sensor>
</gazebo>
```

## Simulation and Real-World Deployment Consistency

### Why Consistency Matters

The same URDF model serves multiple purposes:

1. **Simulation**: Physics engines use it for realistic simulation
2. **Visualization**: Tools like RViz display the robot structure
3. **Planning**: Motion planners use it for collision checking
4. **Control**: Inverse kinematics solvers use it for joint calculations

### Ensuring Consistency

- **Accurate Dimensions**: Model dimensions should match the real robot
- **Proper Inertial Properties**: For realistic physics simulation
- **Correct Joint Limits**: Reflect actual robot capabilities
- **Realistic Mass Distribution**: Affects dynamics simulation

### Verification Approaches

```bash
# Validate URDF syntax
check_urdf /path/to/robot.urdf

# Visualize the robot
ros2 run rviz2 rviz2

# Check kinematic chains
ros2 run urdf_tutorial check_urdf /path/to/robot.urdf
```

## Common URDF Pitfalls and Solutions

### 1. Floating Base Issue

Humanoid robots often need a fixed base for simulation:

```xml
<!-- Add a fixed joint to ground if needed for simulation -->
<joint name="base_ground_joint" type="fixed">
  <parent link="ground_plane" />
  <child link="base_link" />
</joint>
```

### 2. Kinematic Loop Issues

Complex humanoid structures might create kinematic loops. Consider using transmissions or additional planning:

```xml
<!-- For complex multi-loop structures, consider using dedicated packages -->
<!-- like moveit_servo or ros2_controllers for better handling -->
```

### 3. Inertial Matrix Issues

Always ensure the inertia matrix is physically valid:

- Diagonal elements must be positive
- Must satisfy triangle inequalities: |ixx| ≤ |iyy + izz|, etc.

## Tools for Working with URDF

### Visualization Tools

- **RViz**: Real-time visualization of robot structure
- **Gazebo**: Physics simulation with URDF models
- **urdf_tutorial**: Basic tools for checking and visualizing URDF

### Validation Tools

- `check_urdf`: Validates URDF syntax and structure
- `gz model`: For Gazebo-specific validation
- **MeshLab/Blender**: For visual inspection of mesh files

## Summary

URDF is the essential format for describing humanoid robot structure in ROS systems:

- Links represent rigid bodies with physical properties
- Joints connect links with specific kinematic constraints
- Sensors and actuators are mounted using the same framework
- Proper inertial properties ensure realistic simulation
- Modular design with Xacro improves maintainability
- Consistency between simulation and real robot is crucial

The humanoid robot model defined in URDF provides the structural foundation that AI agents, as covered in Chapter 3, can use to control and interact with the physical robot system. This completes our exploration of ROS 2 as the robotic nervous system, covering its architecture, communication patterns, AI integration, and structural modeling.