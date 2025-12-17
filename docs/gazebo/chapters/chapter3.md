---
title: "Chapter 3: Unity for High-Fidelity Interaction"
description: "Using Unity for high-fidelity visualization and human-robot interaction simulation"
sidebar_position: 7
---

# Chapter 3: Unity for High-Fidelity Interaction

## Introduction to Unity in Robotics

While Gazebo excels at physics simulation and sensor modeling, Unity provides unparalleled capabilities for high-fidelity rendering and human-robot interaction simulation. Unity's advanced graphics engine, intuitive development environment, and extensive asset ecosystem make it an ideal platform for creating visually compelling digital twins that enhance the realism and usability of robotics simulations.

For humanoid robots, Unity's strength lies in creating immersive visualization environments that can help developers understand robot behavior, test interaction scenarios, and present robotics concepts to stakeholders. Unity complements Gazebo's physics simulation with high-quality visualization and interactive human-robot interfaces.

## Unity Environment Setup for Robotics

### Unity Installation and Configuration

Unity Hub provides a centralized way to manage Unity installations and projects. For robotics applications, we recommend using Unity 2022.3 LTS (Long Term Support) for stability and compatibility:

1. **Install Unity Hub**: Download from the Unity website
2. **Install Unity Editor**: Select 2022.3 LTS version
3. **Install modules**: Include Visual Studio integration and required build targets
4. **Configure for robotics**: Set up project templates and asset repositories

### ROS Integration Setup

The Unity ROS TCP Connector enables communication between Unity and ROS 2 systems. This bridge allows Unity to send and receive ROS messages, creating a seamless integration:

```csharp
// Example Unity C# script for ROS connection
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class UnityROSBridge : MonoBehaviour
{
    private ROS2UnityComponent ros2;
    private UDPBase udpBase;

    [Header("ROS Connection Settings")]
    public string rosMasterUrl = "127.0.0.1";
    public int rosMasterPort = 11345;

    void Start()
    {
        // Initialize ROS2 component
        ros2 = GetComponent<ROS2UnityComponent>();
        ros2.ROSConnectionSettings.protocol = ConnectionProtocol.TCP;
        ros2.ROSConnectionSettings.node_name = "unity_robot_sim";

        // Initialize UDP for sensor data (if needed)
        udpBase = new UDPBase();
        udpBase.Connect(rosMasterUrl, rosMasterPort);

        // Start ROS connection
        ros2.Initialize();

        Debug.Log("ROS connection initialized");
    }

    void OnDestroy()
    {
        if (ros2 != null)
        {
            ros2.Shutdown();
        }
    }
}
```

### Project Structure for Robotics Simulation

A well-organized Unity project structure enhances maintainability and collaboration:

```
UnityRoboticsProject/
├── Assets/
│   ├── Scripts/           # C# scripts for robot control and ROS integration
│   ├── Models/            # 3D models for robots and environments
│   ├── Materials/         # Material definitions and textures
│   ├── Scenes/            # Unity scene files
│   ├── Prefabs/           # Reusable robot and environment components
│   ├── Plugins/           # Third-party plugins (ROS bridge, etc.)
│   └── Resources/         # Runtime-loadable assets
├── ProjectSettings/       # Unity project configuration
└── Packages/              # Unity package dependencies
```

## Rendering Humanoid Robots with Realistic Materials

### Importing Robot Models

Robot models created in CAD software or defined in URDF can be imported into Unity using several approaches:

1. **Direct import**: Import STL, FBX, or OBJ files directly
2. **URDF importer**: Use Unity's URDF Importer package for automatic conversion
3. **Custom import pipeline**: Create scripts to process URDF and generate Unity objects

```csharp
// Example script for importing and configuring robot parts
using UnityEngine;

public class RobotImporter : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "HumanoidRobot";
    public Transform robotRoot;

    [Header("Visual Configuration")]
    public Material defaultMaterial;
    public Material highlightMaterial;

    void Start()
    {
        SetupRobotMaterials();
        ConfigureRobotJoints();
    }

    void SetupRobotMaterials()
    {
        // Apply materials to robot parts
        Renderer[] renderers = robotRoot.GetComponentsInChildren<Renderer>();

        foreach (Renderer renderer in renderers)
        {
            // Apply default material if none exists
            if (renderer.material == null)
            {
                renderer.material = defaultMaterial;
            }

            // Configure material properties for realism
            ConfigureMaterial(renderer.material);
        }
    }

    void ConfigureMaterial(Material material)
    {
        // Set realistic material properties
        material.SetFloat("_Metallic", 0.2f);
        material.SetFloat("_Smoothness", 0.5f);
        material.EnableKeyword("_NORMALMAP");
    }

    void ConfigureRobotJoints()
    {
        // Configure Unity joints to match URDF joint limits
        ConfigurableJoint[] joints = robotRoot.GetComponentsInChildren<ConfigurableJoint>();

        foreach (ConfigurableJoint joint in joints)
        {
            ConfigureJointLimits(joint);
        }
    }

    void ConfigureJointLimits(ConfigurableJoint joint)
    {
        // Example: Configure revolute joint limits
        SoftJointLimit limit = joint.angularYLimit;
        limit.limit = 90f; // Example: 90 degree limit
        joint.angularYLimit = limit;
    }
}
```

### Realistic Material Creation

Creating realistic materials enhances the visual fidelity of robot models:

```csharp
// Material configuration script for realistic robot appearance
using UnityEngine;

[CreateAssetMenu(fileName = "RobotMaterialConfig", menuName = "Robotics/Material Config")]
public class RobotMaterialConfig : ScriptableObject
{
    [Header("Base Properties")]
    public Color baseColor = Color.gray;
    public float metallic = 0.1f;
    public float smoothness = 0.5f;

    [Header("Surface Details")]
    public Texture2D normalMap;
    public Texture2D roughnessMap;
    public float surfaceScale = 1.0f;

    [Header("Special Effects")]
    public bool hasEmission = false;
    public Color emissionColor = Color.black;
    public float emissionIntensity = 1.0f;
}

public class MaterialApplier : MonoBehaviour
{
    public RobotMaterialConfig materialConfig;
    public Material robotMaterial;

    void Start()
    {
        if (materialConfig != null)
        {
            ApplyMaterialConfig();
        }
    }

    void ApplyMaterialConfig()
    {
        // Apply base properties
        robotMaterial.SetColor("_BaseColor", materialConfig.baseColor);
        robotMaterial.SetFloat("_Metallic", materialConfig.metallic);
        robotMaterial.SetFloat("_Smoothness", materialConfig.smoothness);

        // Apply textures
        if (materialConfig.normalMap != null)
        {
            robotMaterial.SetTexture("_NormalMap", materialConfig.normalMap);
        }

        if (materialConfig.roughnessMap != null)
        {
            robotMaterial.SetTexture("_MetallicGlossMap", materialConfig.roughnessMap);
        }

        // Configure emission
        if (materialConfig.hasEmission)
        {
            robotMaterial.EnableKeyword("_EMISSION");
            robotMaterial.SetColor("_EmissionColor",
                materialConfig.emissionColor * materialConfig.emissionIntensity);
        }
    }
}
```

### Animation and Joint Movement

Animating robot joints in Unity provides visual feedback for robot movement:

```csharp
// Robot animation controller
using UnityEngine;

public class RobotAnimator : MonoBehaviour
{
    [Header("Joint Configuration")]
    public Transform[] jointTransforms;
    public ConfigurableJoint[] joints;

    [Header("Animation Parameters")]
    public float animationSpeed = 1.0f;
    public AnimationCurve jointMovementCurve;

    private float[] targetJointPositions;
    private float[] currentJointPositions;

    void Start()
    {
        InitializeJoints();
    }

    void InitializeJoints()
    {
        joints = GetComponentsInChildren<ConfigurableJoint>();
        targetJointPositions = new float[joints.Length];
        currentJointPositions = new float[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            // Initialize with current positions
            currentJointPositions[i] = GetJointPosition(joints[i]);
            targetJointPositions[i] = currentJointPositions[i];
        }
    }

    void Update()
    {
        AnimateJoints();
    }

    void AnimateJoints()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] != null)
            {
                // Smoothly interpolate to target position
                currentJointPositions[i] = Mathf.Lerp(
                    currentJointPositions[i],
                    targetJointPositions[i],
                    Time.deltaTime * animationSpeed
                );

                // Apply position to joint
                SetJointPosition(joints[i], currentJointPositions[i]);
            }
        }
    }

    float GetJointPosition(ConfigurableJoint joint)
    {
        // Extract current joint position based on joint type
        // This is a simplified example - actual implementation depends on joint configuration
        return joint.transform.localEulerAngles.y;
    }

    void SetJointPosition(ConfigurableJoint joint, float position)
    {
        // Set joint to new position
        // Actual implementation depends on joint configuration
        joint.transform.localEulerAngles = new Vector3(0, position, 0);
    }

    // Public method to set target joint positions from external source (e.g., ROS)
    public void SetTargetJointPositions(float[] positions)
    {
        for (int i = 0; i < Mathf.Min(targetJointPositions.Length, positions.Length); i++)
        {
            targetJointPositions[i] = positions[i];
        }
    }
}
```

## Human-Robot Interaction Simulation

### UI and Interaction Systems

Creating intuitive interfaces for human-robot interaction in Unity:

```csharp
// Human-robot interaction interface
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class HumanRobotInterface : MonoBehaviour
{
    [Header("UI Elements")]
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;
    public Slider speedSlider;
    public TextMeshProUGUI statusText;

    [Header("Robot Control")]
    public RobotController robotController;

    void Start()
    {
        SetupUI();
    }

    void SetupUI()
    {
        // Setup button click events
        if (moveForwardButton != null)
            moveForwardButton.onClick.AddListener(() => SendCommand("forward"));

        if (moveBackwardButton != null)
            moveBackwardButton.onClick.AddListener(() => SendCommand("backward"));

        if (turnLeftButton != null)
            turnLeftButton.onClick.AddListener(() => SendCommand("turn_left"));

        if (turnRightButton != null)
            turnRightButton.onClick.AddListener(() => SendCommand("turn_right"));

        // Setup speed slider
        if (speedSlider != null)
            speedSlider.onValueChanged.AddListener(OnSpeedChanged);

        UpdateStatus("Ready for interaction");
    }

    void SendCommand(string command)
    {
        if (robotController != null)
        {
            robotController.ExecuteCommand(command, speedSlider.value);
            UpdateStatus($"Command sent: {command} at speed {speedSlider.value:F1}");
        }
    }

    void OnSpeedChanged(float speed)
    {
        if (robotController != null)
        {
            robotController.SetSpeed(speed);
        }
    }

    void UpdateStatus(string message)
    {
        if (statusText != null)
        {
            statusText.text = message;
        }
    }
}

// Robot controller that handles commands
public class RobotController : MonoBehaviour
{
    [Header("Movement Configuration")]
    public float maxSpeed = 2.0f;
    public float rotationSpeed = 90.0f; // degrees per second

    private float currentSpeed = 1.0f;

    public void ExecuteCommand(string command, float speed)
    {
        switch (command)
        {
            case "forward":
                MoveForward(speed);
                break;
            case "backward":
                MoveBackward(speed);
                break;
            case "turn_left":
                RotateLeft(speed);
                break;
            case "turn_right":
                RotateRight(speed);
                break;
        }
    }

    public void SetSpeed(float speed)
    {
        currentSpeed = Mathf.Clamp(speed, 0.1f, 1.0f);
    }

    void MoveForward(float speed)
    {
        Vector3 movement = transform.forward * maxSpeed * speed * Time.deltaTime;
        transform.position += movement;
    }

    void MoveBackward(float speed)
    {
        Vector3 movement = transform.forward * maxSpeed * speed * Time.deltaTime;
        transform.position -= movement;
    }

    void RotateLeft(float speed)
    {
        transform.Rotate(Vector3.up, -rotationSpeed * speed * Time.deltaTime);
    }

    void RotateRight(float speed)
    {
        transform.Rotate(Vector3.up, rotationSpeed * speed * Time.deltaTime);
    }
}
```

### Gesture Recognition and Input Systems

Implementing gesture recognition for natural interaction:

```csharp
// Gesture recognition system
using UnityEngine;

public class GestureRecognition : MonoBehaviour
{
    [Header("Gesture Detection")]
    public float minGestureDistance = 0.1f;
    public float maxGestureTime = 1.0f;

    private Vector3 gestureStartPos;
    private float gestureStartTime;
    private bool isGestureActive = false;

    void Update()
    {
        HandleMouseGestures();
        HandleTouchGestures();
    }

    void HandleMouseGestures()
    {
        if (Input.GetMouseButtonDown(0))
        {
            StartGesture(Input.mousePosition);
        }
        else if (Input.GetMouseButtonUp(0))
        {
            EndGesture(Input.mousePosition);
        }
    }

    void HandleTouchGestures()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Began)
            {
                StartGesture(touch.position);
            }
            else if (touch.phase == TouchPhase.Ended)
            {
                EndGesture(touch.position);
            }
        }
    }

    void StartGesture(Vector3 startPos)
    {
        gestureStartPos = startPos;
        gestureStartTime = Time.time;
        isGestureActive = true;
    }

    void EndGesture(Vector3 endPos)
    {
        if (isGestureActive)
        {
            float gestureTime = Time.time - gestureStartTime;
            Vector3 gestureVector = endPos - gestureStartPos;
            float gestureDistance = gestureVector.magnitude;

            if (gestureDistance >= minGestureDistance && gestureTime <= maxGestureTime)
            {
                RecognizeGesture(gestureVector, gestureDistance, gestureTime);
            }

            isGestureActive = false;
        }
    }

    void RecognizeGesture(Vector3 gestureVector, float distance, float time)
    {
        // Normalize gesture vector for direction recognition
        Vector2 direction = gestureVector.normalized;

        // Determine gesture type based on direction
        if (Mathf.Abs(direction.x) > Mathf.Abs(direction.y))
        {
            // Horizontal gesture
            if (direction.x > 0.5f)
            {
                ExecuteGesture("swipe_right");
            }
            else if (direction.x < -0.5f)
            {
                ExecuteGesture("swipe_left");
            }
        }
        else
        {
            // Vertical gesture
            if (direction.y > 0.5f)
            {
                ExecuteGesture("swipe_up");
            }
            else if (direction.y < -0.5f)
            {
                ExecuteGesture("swipe_down");
            }
        }
    }

    void ExecuteGesture(string gestureName)
    {
        Debug.Log($"Gesture recognized: {gestureName}");

        // Send gesture to robot controller via ROS
        SendGestureToRobot(gestureName);
    }

    void SendGestureToRobot(string gesture)
    {
        // This would typically send a ROS message to the robot
        // For example, publish to a gesture topic
        Debug.Log($"Sending gesture {gesture} to robot");
    }
}
```

## ROS 2 Integration in Unity

### Message Publishing and Subscribing

Integrating Unity with ROS 2 for bidirectional communication:

```csharp
// ROS 2 message publisher for Unity
using UnityEngine;
using ROS2;

public class UnityROSPublisher : MonoBehaviour
{
    private ROS2UnityComponent ros2;
    private Publisher<std_msgs.msg.String> statusPublisher;
    private Publisher<geometry_msgs.msg.Twist> cmdVelPublisher;

    [Header("ROS Topics")]
    public string statusTopic = "/unity_robot_status";
    public string cmdVelTopic = "/cmd_vel";

    void Start()
    {
        ros2 = GetComponent<ROS2UnityComponent>();
        ros2.Initialize();

        // Initialize publishers
        statusPublisher = ros2.CreatePublisher<std_msgs.msg.String>(statusTopic);
        cmdVelPublisher = ros2.CreatePublisher<geometry_msgs.msg.Twist>(cmdVelTopic);

        // Start publishing robot status
        InvokeRepeating("PublishRobotStatus", 0.0f, 1.0f);
    }

    void PublishRobotStatus()
    {
        if (statusPublisher != null)
        {
            var statusMsg = new std_msgs.msg.String();
            statusMsg.Data = $"Unity robot at position: {transform.position}";
            statusPublisher.Publish(statusMsg);
        }
    }

    public void PublishVelocityCommand(float linearX, float angularZ)
    {
        if (cmdVelPublisher != null)
        {
            var cmdMsg = new geometry_msgs.msg.Twist();
            cmdMsg.Linear.X = linearX;
            cmdMsg.Angular.Z = angularZ;
            cmdVelPublisher.Publish(cmdMsg);
        }
    }
}

// ROS 2 message subscriber for Unity
using UnityEngine;
using ROS2;

public class UnityROSSubscriber : MonoBehaviour
{
    private ROS2UnityComponent ros2;
    private Subscription<sensor_msgs.msg.LaserScan> lidarSubscriber;
    private Subscription<sensor_msgs.msg.Imu> imuSubscriber;

    [Header("Sensor Data")]
    public float[] lidarRanges;
    public Vector3 imuOrientation;

    void Start()
    {
        ros2 = GetComponent<ROS2UnityComponent>();
        ros2.Initialize();

        // Initialize subscribers
        lidarSubscriber = ros2.CreateSubscription<sensor_msgs.msg.LaserScan>(
            "/robot/lidar/scan",
            ProcessLidarData
        );

        imuSubscriber = ros2.CreateSubscription<sensor_msgs.msg.Imu>(
            "/imu/data",
            ProcessIMUData
        );
    }

    void ProcessLidarData(ROS2Msg msg)
    {
        var lidarMsg = (sensor_msgs.msg.LaserScan)msg;
        lidarRanges = new float[lidarMsg.Ranges.Length];

        for (int i = 0; i < lidarMsg.Ranges.Length; i++)
        {
            lidarRanges[i] = lidarMsg.Ranges[i];
        }
    }

    void ProcessIMUData(ROS2Msg msg)
    {
        var imuMsg = (sensor_msgs.msg.Imu)msg;
        imuOrientation = new Vector3(
            imuMsg.Orientation.X,
            imuMsg.Orientation.Y,
            imuMsg.Orientation.Z
        );
    }
}
```

### Visualization of ROS Data

Visualizing ROS data within Unity enhances the simulation experience:

```csharp
// Visualize sensor data in Unity
using UnityEngine;

public class SensorVisualizer : MonoBehaviour
{
    [Header("Visualization Configuration")]
    public UnityROSSubscriber sensorSubscriber;
    public GameObject lidarPointPrefab;
    public Material obstacleMaterial;
    public float visualizationRange = 10.0f;

    private GameObject[] lidarPoints;

    void Start()
    {
        CreateLidarVisualization();
    }

    void Update()
    {
        UpdateLidarVisualization();
    }

    void CreateLidarVisualization()
    {
        if (sensorSubscriber.lidarRanges != null)
        {
            lidarPoints = new GameObject[sensorSubscriber.lidarRanges.Length];

            for (int i = 0; i < sensorSubscriber.lidarRanges.Length; i++)
            {
                lidarPoints[i] = Instantiate(lidarPointPrefab);
                lidarPoints[i].transform.SetParent(transform);
                lidarPoints[i].SetActive(false);
            }
        }
    }

    void UpdateLidarVisualization()
    {
        if (sensorSubscriber.lidarRanges != null)
        {
            for (int i = 0; i < sensorSubscriber.lidarRanges.Length; i++)
            {
                float distance = sensorSubscriber.lidarRanges[i];

                if (distance > 0 && distance < visualizationRange)
                {
                    // Calculate angle for this laser beam
                    float angle = Mathf.Lerp(-60f, 60f, (float)i / sensorSubscriber.lidarRanges.Length);

                    // Convert polar to Cartesian coordinates
                    float x = distance * Mathf.Cos(angle * Mathf.Deg2Rad);
                    float z = distance * Mathf.Sin(angle * Mathf.Deg2Rad);

                    // Position the visualization point
                    lidarPoints[i].transform.position = transform.position + new Vector3(x, 0.1f, z);
                    lidarPoints[i].SetActive(true);

                    // Change color based on distance
                    Renderer renderer = lidarPoints[i].GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        float colorIntensity = 1.0f - (distance / visualizationRange);
                        renderer.material.color = Color.Lerp(Color.red, Color.green, colorIntensity);
                    }
                }
                else
                {
                    lidarPoints[i].SetActive(false);
                }
            }
        }
    }
}
```

## Unity Scene Architecture for Robotics

### Scene Organization

Unity scenes for robotics applications follow a structured organization pattern:

```
Robotics Scene Hierarchy:
├── Environment/
│   ├── Ground/
│   ├── Obstacles/
│   └── Lighting/
├── Robots/
│   ├── HumanoidRobot/
│   │   ├── Base/
│   │   ├── Torso/
│   │   ├── Arms/
│   │   └── Legs/
│   └── Sensors/
│       ├── LiDAR/
│       ├── Camera/
│       └── IMU/
├── UI/
│   ├── ControlPanel/
│   └── StatusDisplay/
└── Managers/
    ├── ROSBridge/
    ├── SceneController/
    └── PhysicsController/
```

### Performance Considerations

High-fidelity rendering requires careful optimization to maintain performance:

```csharp
// Graphics optimization manager
using UnityEngine;

public class GraphicsOptimizer : MonoBehaviour
{
    [Header("Quality Settings")]
    public int targetFrameRate = 60;
    public LODGroup[] lodGroups;
    public int maxVisibleObjects = 100;

    [Header("Dynamic Optimization")]
    public float optimizationInterval = 1.0f;

    private float lastOptimizationTime;

    void Start()
    {
        // Set target frame rate
        Application.targetFrameRate = targetFrameRate;

        // Initialize LOD groups
        InitializeLODs();

        lastOptimizationTime = Time.time;
    }

    void Update()
    {
        // Periodic optimization
        if (Time.time - lastOptimizationTime > optimizationInterval)
        {
            PerformOptimization();
            lastOptimizationTime = Time.time;
        }
    }

    void InitializeLODs()
    {
        lodGroups = FindObjectsOfType<LODGroup>();
    }

    void PerformOptimization()
    {
        // Dynamic LOD adjustment based on performance
        float currentFrameRate = 1.0f / Time.unscaledDeltaTime;

        if (currentFrameRate < targetFrameRate * 0.8f)
        {
            // Reduce quality if frame rate is too low
            ReduceLODQuality();
        }
        else if (currentFrameRate > targetFrameRate * 0.95f)
        {
            // Increase quality if frame rate is good
            IncreaseLODQuality();
        }
    }

    void ReduceLODQuality()
    {
        foreach (LODGroup lodGroup in lodGroups)
        {
            // Switch to lower LOD level
            lodGroup.ForceLOD(1); // Use medium detail
        }
    }

    void IncreaseLODQuality()
    {
        foreach (LODGroup lodGroup in lodGroups)
        {
            // Switch to higher LOD level
            lodGroup.ForceLOD(0); // Use high detail
        }
    }
}
```

## Integration with Gazebo Data

Unity can receive and visualize data from Gazebo simulations:

```python
# Python script to send Gazebo data to Unity (bridge example)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
import socket
import json

class GazeboUnityBridge(Node):
    def __init__(self):
        super().__init__('gazebo_unity_bridge')

        # Setup socket connection to Unity
        self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.unity_socket.connect(('127.0.0.1', 5555))

        # Subscribe to Gazebo sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan, '/robot/lidar/scan', self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/robot/imu/data', self.imu_callback, 10
        )

        self.get_logger().info('Gazebo-Unity bridge initialized')

    def lidar_callback(self, msg):
        """Send LiDAR data to Unity"""
        lidar_data = {
            'type': 'lidar',
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

        try:
            self.unity_socket.send(json.dumps(lidar_data).encode())
        except Exception as e:
            self.get_logger().error(f'Error sending lidar data: {e}')

    def imu_callback(self, msg):
        """Send IMU data to Unity"""
        imu_data = {
            'type': 'imu',
            'orientation': [msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y,
                                msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y,
                                   msg.linear_acceleration.z]
        }

        try:
            self.unity_socket.send(json.dumps(imu_data).encode())
        except Exception as e:
            self.get_logger().error(f'Error sending imu data: {e}')

def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboUnityBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.unity_socket.close()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter has explored Unity's capabilities for high-fidelity rendering and human-robot interaction simulation. Unity complements Gazebo's physics simulation by providing:

- **Advanced graphics**: High-quality rendering for realistic visualization
- **Interactive interfaces**: Intuitive human-robot interaction systems
- **ROS integration**: Bidirectional communication with ROS 2 systems
- **Performance optimization**: Techniques to maintain frame rates with complex scenes

Unity's strength lies in creating visually compelling and interactive experiences that enhance the realism of robotics simulations. The integration with ROS 2 enables Unity to serve as a sophisticated visualization and interaction layer that can receive sensor data from physics simulations and send control commands back to the system.

In the next chapter, we'll explore how to bridge Gazebo and Unity environments to create a comprehensive digital twin that combines the physics accuracy of Gazebo with the visual fidelity of Unity.