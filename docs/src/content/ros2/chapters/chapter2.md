---
title: "Chapter 2: ROS 2 Communication Primitives"
description: "Understanding ROS 2 communication patterns: nodes, topics, services, and actions"
---

# Chapter 2: ROS 2 Communication Primitives

## Introduction

In the previous chapter, we established ROS 2 as the distributed nervous system for humanoid robots. This chapter delves into the specific communication primitives that enable this distributed architecture: nodes, topics, services, and actions. Understanding these primitives is crucial for designing effective robot systems.

## Nodes and Node Lifecycle

### What is a Node?

A **node** in ROS 2 is an executable process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically handles a specific task or capability, such as:

- Sensor data processing
- Path planning
- Motion control
- User interface
- Behavior management

### Node Lifecycle States

ROS 2 nodes follow a well-defined lifecycle with the following states:

1. **Unconfigured**: Initial state after node creation
2. **Inactive**: Node is configured but not executing
3. **Active**: Node is running and processing data
4. **Finalized**: Node is shutting down

This lifecycle enables more robust system management and resource allocation.

### Basic Node Example

Here's a simple Python node using rclpy:

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publishers and Subscribers

### Topic-Based Communication

Topics enable asynchronous, decoupled communication between nodes. Key characteristics:

- **Publish-subscribe pattern**: Publishers send data without knowing subscribers
- **Continuous data flow**: Perfect for sensor streams, state updates
- **Many-to-many**: Multiple publishers and subscribers can use the same topic
- **Data types**: Defined by message definitions (`.msg` files)

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(0.5, self.publish_status)  # Publish every 0.5 seconds
        self.counter = 0

    def publish_status(self):
        msg = String()
        msg.data = f'Robot operational - count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS) Settings

Topics support various QoS settings to control reliability and performance:

- **Reliability**: Reliable vs best-effort delivery
- **Durability**: Volatile vs transient-local (reliable for late-joining subscribers)
- **History**: Keep-all vs keep-last N messages
- **Depth**: Buffer size for message history

## Services: Synchronous Communication

### Service-Based Communication

Services provide synchronous request-response communication:

- **Request-response pattern**: Client sends request, server responds
- **Synchronous**: Client waits for response before continuing
- **One-to-one**: Typically one server serves multiple clients
- **Use cases**: Transform lookup, planning requests, configuration changes

### Service Definition

Services are defined in `.srv` files:

```
# Request
string goal_pose
---
# Response
bool success
string message
```

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.send_request(2, 3)

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Long-Running Tasks with Feedback

### Action-Based Communication

Actions handle long-running tasks that require:

- **Feedback**: Continuous updates on task progress
- **Goals**: Request to start a long-running task
- **Results**: Final outcome when task completes
- **Cancelation**: Ability to stop ongoing tasks

### Use Cases for Actions

- Navigation to a goal pose
- Trajectory execution
- Object manipulation tasks
- Calibration procedures

### Action Example Structure

Actions are defined in `.action` files:

```
# Goal
geometry_msgs/PoseStamped target_pose
---
# Result
bool succeeded
string message
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

## Communication Pattern Decision Guide

### When to Use Each Pattern

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topics** | Continuous data streams | Sensor data, robot state |
| **Services** | Request-response, quick operations | Transform lookup, simple calculations |
| **Actions** | Long-running tasks with feedback | Navigation, manipulation |

### Message Flow Diagrams

```
Topics (Continuous):
Publisher → Topic → Subscriber
   ↓                    ↓
Sensor    → /sensor_data → Processing Node
   ↓                    ↓
Camera    → /image_raw   → Image Processing Node

Services (Request-Response):
Client → Service Request → Server
   ↓                        ↓
Path Planner ← Service ← Navigation Node
   ↓        ← Response
Goal Pose

Actions (Long-running with Feedback):
Client → Goal → Action Server
   ↓            ↓
Navigation ← Current Pose ← Robot
   ↓        ← Feedback
Result
```

## Practical Example: Sensor-Controller Communication

Here's a complete example showing how different communication patterns work together:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from example_interfaces.srv import SetBool
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
import threading
import time

class SensorControllerNode(Node):
    def __init__(self):
        super().__init__('sensor_controller')

        # Topic: Publish sensor data
        self.sensor_publisher = self.create_publisher(Float32, 'sensor_value', 10)

        # Service: Control sensor on/off
        self.control_service = self.create_service(
            SetBool, 'control_sensor', self.control_callback)

        # Action: Long-running monitoring task
        self.action_server = ActionServer(
            self, Fibonacci, 'monitor_sensor', self.execute_monitor)

        # Simulate sensor data
        self.is_monitoring = False
        self.timer = self.create_timer(1.0, self.publish_sensor_data)

    def publish_sensor_data(self):
        if self.is_monitoring:
            import random
            msg = Float32()
            msg.data = random.uniform(0.0, 100.0)
            self.sensor_publisher.publish(msg)
            self.get_logger().info(f'Sensor value: {msg.data}')

    def control_callback(self, request, response):
        self.is_monitoring = request.data
        response.success = True
        response.message = f'Sensor monitoring set to {self.is_monitoring}'
        self.get_logger().info(response.message)
        return response

    def execute_monitor(self, goal_handle):
        self.get_logger().info('Monitoring started')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0]

        for i in range(1, goal_handle.request.order + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            feedback_msg.sequence.append(i)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SensorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 communication primitives provide flexible patterns for different robot system needs:

- **Topics** for continuous data streams and sensor information
- **Services** for synchronous request-response interactions
- **Actions** for long-running tasks with progress feedback

Understanding when to use each pattern is essential for designing efficient and maintainable robot systems. In the next chapter, we'll explore how to bridge AI agents with these ROS 2 communication patterns.