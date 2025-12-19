---
title: "Chapter 2: Sensor Simulation in Gazebo"
description: "Simulating LiDAR, Depth Camera, and IMU sensors with ROS 2 integration"
sidebar_position: 6
---

# Chapter 2: Sensor Simulation in Gazebo

## Introduction to Sensor Simulation

Sensor simulation is a critical component of effective robotics simulation, enabling the development and testing of perception algorithms without requiring physical hardware. In Gazebo, realistic sensor simulation bridges the gap between virtual and real-world robotics by generating data that closely matches the characteristics and noise patterns of actual sensors.

For humanoid robots, accurate sensor simulation is essential for developing robust perception, navigation, and interaction capabilities. The three primary sensor types we'll focus on are:

- **LiDAR sensors**: Provide 2D or 3D distance measurements for obstacle detection and mapping
- **Depth cameras**: Offer color and depth information for 3D scene understanding
- **IMUs**: Deliver orientation and motion data for state estimation

## LiDAR Sensor Simulation

### LiDAR Physics and Principles

LiDAR (Light Detection and Ranging) sensors work by emitting laser pulses and measuring the time it takes for the light to return after reflecting off objects. In Gazebo, LiDAR simulation uses ray tracing to detect obstacles and calculate distances:

```xml
<!-- LiDAR sensor configuration -->
<sensor name="lidar_sensor" type="ray">
  <pose>0.2 0 0.1 0 0 0</pose>  <!-- Position on robot -->
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>        <!-- Number of rays per revolution -->
        <resolution>1</resolution>     <!-- Angular resolution -->
        <min_angle>-3.14159</min_angle> <!-- -π radians (-180°) -->
        <max_angle>3.14159</max_angle>  <!-- π radians (180°) -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>     <!-- Minimum detection range (m) -->
      <max>30.0</max>    <!-- Maximum detection range (m) -->
      <resolution>0.01</resolution>  <!-- Range resolution (m) -->
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

### Realistic Parameter Configuration

Configuring realistic LiDAR parameters is essential for effective simulation:

- **Range**: Minimum and maximum detection distances should match the physical sensor
- **Resolution**: Angular and distance resolution should reflect the actual sensor capabilities
- **Noise**: Add realistic noise models to simulate real-world sensor imperfections
- **Update rate**: Should match the physical sensor's update frequency

```xml
<!-- Enhanced LiDAR with noise model -->
<sensor name="lidar_sensor" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>    <!-- Higher resolution -->
        <resolution>1</resolution>
        <min_angle>-2.35619</min_angle> <!-- -135° -->
        <max_angle>2.35619</max_angle>   <!-- 135° -->
      </horizontal>
    </scan>
    <range>
      <min>0.08</min>     <!-- Hokuyo UTM-30LX like -->
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>true</always_on>
  <update_rate>40</update_rate>  <!-- 40 Hz update rate -->
  <visualize>true</visualize>    <!-- Visualize rays in GUI -->

  <!-- Noise model -->
  <noise type="gaussian">
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
  </noise>

  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

### LiDAR Data Processing with ROS 2

Once the LiDAR data is published to ROS 2 topics, it can be processed using standard message types:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to LiDAR scan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/lidar/scan',
            self.lidar_callback,
            10
        )

        # Publisher for processed data
        self.obstacle_publisher = self.create_publisher(
            # Custom message type for obstacle detection
            # This would be defined in your package
        )

        self.get_logger().info('LiDAR processor initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR scan data"""
        # Extract ranges from message
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Calculate minimum distance (closest obstacle)
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')

        # Detect obstacles within threshold
        obstacle_threshold = 1.0  # meters
        obstacles = ranges[ranges < obstacle_threshold]

        if len(obstacles) > 0:
            self.get_logger().warn(f'{len(obstacles)} obstacles detected within {obstacle_threshold}m')

        # Calculate free space statistics
        free_space_distances = ranges[ranges > obstacle_threshold]
        if len(free_space_distances) > 0:
            avg_free_space = np.mean(free_space_distances)
            self.get_logger().info(f'Average free space: {avg_free_space:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()

    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Camera Simulation

### Depth Camera Physics and Optics

Depth cameras capture both color and depth information, providing 3D spatial data essential for navigation and manipulation. In Gazebo, depth cameras use ray tracing to calculate depth values:

```xml
<!-- Depth camera sensor configuration -->
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.2 0 0 0</pose>  <!-- Position on robot -->
  <camera name="depth_cam">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees in radians -->
    <image>
      <width>640</width>      <!-- Image width -->
      <height>480</height>    <!-- Image height -->
      <format>R8G8B8</format> <!-- Color format -->
    </image>
    <clip>
      <near>0.1</near>    <!-- Near clipping distance -->
      <far>10.0</far>     <!-- Far clipping distance -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- Noise standard deviation -->
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>  <!-- 30 FPS -->
  <visualize>true</visualize>

  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
      <remapping>image_raw:=image_color</remapping>
      <remapping>camera_info:=camera_info</remapping>
    </ros>
    <frame_name>camera_link</frame_name>
    <min_depth>0.1</min_depth>
    <max_depth>10.0</max_depth>
  </plugin>
</sensor>
```

### Depth Data Processing

Depth camera data provides both color images and depth information that can be processed for 3D understanding:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to color and depth images
        self.color_sub = self.create_subscription(
            Image,
            '/camera/image_color',
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Subscribe to camera info for intrinsic parameters
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

        self.camera_info = None
        self.get_logger().info('Depth camera processor initialized')

    def info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_info = msg

    def color_callback(self, msg):
        """Process color image data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the color image (example: edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Display the result
            cv2.imshow('Color Image', cv_image)
            cv2.imshow('Edges', edges)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing color image: {e}')

    def depth_callback(self, msg):
        """Process depth image data"""
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            # Process depth data
            height, width = depth_image.shape

            # Calculate distance statistics
            valid_depths = depth_image[np.isfinite(depth_image)]
            if len(valid_depths) > 0:
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                avg_depth = np.mean(valid_depths)

                self.get_logger().info(
                    f'Depth stats - Min: {min_depth:.2f}m, '
                    f'Max: {max_depth:.2f}m, Avg: {avg_depth:.2f}m'
                )

            # Find objects at specific distance range
            distance_threshold = 2.0  # meters
            close_objects = depth_image[depth_image < distance_threshold]

            if len(close_objects) > 0:
                percentage = (len(close_objects) / (height * width)) * 100
                self.get_logger().info(
                    f'{len(close_objects)} pixels within {distance_threshold}m '
                    f'({percentage:.1f}% of image)'
                )

            # Display depth image (normalized for visualization)
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            cv2.imshow('Depth Image', depth_normalized)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    depth_processor = DepthCameraProcessor()

    try:
        rclpy.spin(depth_processor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        depth_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Simulation

### IMU Physics and Components

An IMU (Inertial Measurement Unit) combines accelerometers and gyroscopes to measure linear acceleration and angular velocity. In Gazebo, IMU simulation models the physics of these sensors:

```xml
<!-- IMU sensor configuration -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>  <!-- 100 Hz update rate -->
  <pose>0 0 0.1 0 0 0</pose>  <!-- Position on robot -->

  <imu>
    <!-- Noise parameters for accelerometer -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1 deg/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </angular_velocity>

    <!-- Noise parameters for gyroscope -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- ~0.017 m/s^2 -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

### IMU Data Processing

IMU data provides crucial information about the robot's orientation and motion:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for processed orientation data
        self.orientation_publisher = self.create_publisher(
            Vector3,
            '/robot/orientation_euler',
            10
        )

        self.previous_orientation = None
        self.get_logger().info('IMU processor initialized')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Create rotation object from quaternion
        r = R.from_quat([x, y, z, w])
        # Convert to Euler angles in radians
        euler = r.as_euler('xyz')
        return euler

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        # Extract orientation quaternion
        orientation = msg.orientation
        euler = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # Convert to degrees for easier interpretation
        roll_deg = np.degrees(euler[0])
        pitch_deg = np.degrees(euler[1])
        yaw_deg = np.degrees(euler[2])

        # Log orientation data
        self.get_logger().info(
            f'Orientation - Roll: {roll_deg:.2f}°, '
            f'Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°'
        )

        # Extract angular velocity
        angular_vel = msg.angular_velocity
        self.get_logger().info(
            f'Angular Vel - X: {angular_vel.x:.3f}, '
            f'Y: {angular_vel.y:.3f}, Z: {angular_vel.z:.3f}'
        )

        # Extract linear acceleration
        linear_acc = msg.linear_acceleration
        total_acc = np.sqrt(
            linear_acc.x**2 + linear_acc.y**2 + linear_acc.z**2
        )
        self.get_logger().info(
            f'Linear Acc - X: {linear_acc.x:.3f}, '
            f'Y: {linear_acc.y:.3f}, Z: {linear_acc.z:.3f}, '
            f'Total: {total_acc:.3f}'
        )

        # Publish Euler angles for other nodes to use
        euler_msg = Vector3()
        euler_msg.x = roll_deg
        euler_msg.y = pitch_deg
        euler_msg.z = yaw_deg
        self.orientation_publisher.publish(euler_msg)

        # Detect significant orientation changes
        if self.previous_orientation is not None:
            prev_euler = self.previous_orientation
            change = abs(euler - prev_euler)
            max_change = np.max(change)

            if max_change > np.radians(5):  # 5 degree threshold
                self.get_logger().warn(
                    f'Significant orientation change detected: {np.degrees(max_change):.2f}°'
                )

        self.previous_orientation = euler

def main(args=None):
    rclpy.init(args=args)
    imu_processor = IMUProcessor()

    try:
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Integration and Standard Messages

### Standard Message Types

Gazebo maps sensor data to standard ROS 2 message types that ensure compatibility across the robotics ecosystem:

- **sensor_msgs/LaserScan**: For LiDAR data with ranges, intensities, and metadata
- **sensor_msgs/Image**: For camera images with encoding and size information
- **sensor_msgs/Imu**: For IMU data with orientation, angular velocity, and linear acceleration
- **sensor_msgs/CameraInfo**: For camera intrinsic and extrinsic parameters

### Data Acquisition Patterns

The typical pattern for sensor data acquisition in ROS 2 involves:

1. **Sensor publishing**: Gazebo plugins publish sensor data to ROS 2 topics
2. **Message subscription**: Robot nodes subscribe to sensor topics
3. **Data processing**: Nodes process sensor data for perception or control
4. **Result publication**: Processed data is published for other nodes to use

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

class MultiSensorFusion(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Create subscribers for multiple sensors
        lidar_sub = Subscriber(self, LaserScan, '/robot/lidar/scan')
        imu_sub = Subscriber(self, Imu, '/imu/data')
        camera_sub = Subscriber(self, Image, '/camera/image_color')

        # Synchronize messages from different sensors
        ats = ApproximateTimeSynchronizer(
            [lidar_sub, imu_sub, camera_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        ats.registerCallback(self.sensor_fusion_callback)

        self.get_logger().info('Multi-sensor fusion node initialized')

    def sensor_fusion_callback(self, lidar_msg, imu_msg, camera_msg):
        """Process synchronized sensor data"""
        self.get_logger().info('Received synchronized sensor data')

        # Perform sensor fusion operations here
        # Example: combine LiDAR obstacle detection with IMU orientation
        # to determine robot's pose in the environment

        # Extract relevant information from each sensor
        lidar_ranges = lidar_msg.ranges
        imu_orientation = imu_msg.orientation
        camera_encoding = camera_msg.encoding

        # Log fusion result
        self.get_logger().info(
            f'Fusion: {len(lidar_ranges)} LiDAR points, '
            f'Camera encoding: {camera_encoding}'
        )

def main(args=None):
    rclpy.init(args=args)
    fusion_node = MultiSensorFusion()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Textual Visualization of Sensor Outputs

### LiDAR Scan Visualization

LiDAR sensors provide 2D or 3D distance measurements that can be visualized as point clouds or occupancy grids:

```
LiDAR Scan Visualization (Top-down view):

     3m
      ↑
-3m ← → +3m
      ↓
    -3m

    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·
    · · · · · · · · · · · · · · · · · · · ·

Legend:
· = Free space
█ = Obstacle detected
O = Robot position
```

### Depth Camera Output Visualization

Depth cameras provide both color and depth information that can be processed for 3D understanding:

```
Depth Camera Output Structure:

Color Image (RGB):
┌─────────────────┐
│ R G B R G B ... │ ← Each pixel has RGB values
│ R G B R G B ... │
│ R G B R G B ... │
│ ...     ...     │
└─────────────────┘

Depth Image (Float32):
┌─────────────────┐
│ 1.2 1.5 2.1 ... │ ← Each pixel has depth in meters
│ 1.3 1.4 2.0 ... │
│ 1.4 1.3 1.9 ... │
│ ...     ...     │
└─────────────────┘

Point Cloud (X,Y,Z):
┌─────────────────┐
│(1,2,1.2)(1,3,1.5)│ ← Each pixel becomes 3D point
│(2,2,1.4)(2,3,1.3)│
│(3,2,1.5)(3,3,1.2)│
│ ...      ...     │
└─────────────────┘
```

### IMU Data Representation

IMU sensors provide orientation, angular velocity, and linear acceleration data:

```
IMU Data Structure:

Orientation (Quaternion):
┌─────────────────────────┐
│ x: 0.0, y: 0.0,       │
│ z: 0.7, w: 0.7        │ ← 45° rotation around Z-axis
└─────────────────────────┘

Angular Velocity:
┌─────────────────────────┐
│ x: 0.1, y: 0.0,       │ ← 0.1 rad/s rotation around X
│ z: 0.0                │
└─────────────────────────┘

Linear Acceleration:
┌─────────────────────────┐
│ x: 0.0, y: 0.0,       │ ← 9.8 m/s² due to gravity
│ z: 9.8                │   when Z-axis is up
└─────────────────────────┘
```

## Performance Considerations

### Sensor Update Rates

Different sensors have different optimal update rates based on their purpose:

- **LiDAR**: 10-40 Hz for navigation and mapping
- **Depth Camera**: 15-30 Hz for 3D perception
- **IMU**: 100-200 Hz for accurate motion tracking

### Computational Requirements

Sensor simulation places different computational loads on the system:

- **LiDAR**: Moderate CPU usage, ray tracing calculations
- **Depth Camera**: High GPU usage, rendering calculations
- **IMU**: Low computational requirements, physics integration

## Summary

This chapter has covered the essential sensor types for humanoid robot simulation in Gazebo: LiDAR, depth cameras, and IMUs. Each sensor type provides different information that is crucial for robot perception and navigation:

- **LiDAR sensors** provide 2D or 3D distance measurements for obstacle detection and mapping
- **Depth cameras** offer color and depth information for 3D scene understanding
- **IMUs** deliver orientation and motion data for state estimation

The chapter demonstrated how to configure these sensors in Gazebo with realistic parameters, process their data using ROS 2 standard message types, and visualize the results for debugging and validation. Proper sensor simulation is fundamental to developing robust robotics applications that can successfully transition from simulation to reality.

In the next chapter, we'll explore Unity as a platform for high-fidelity visualization and human-robot interaction simulation.