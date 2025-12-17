---
title: "Chapter 3: Action Planning & Execution"
description: "Connecting VLM outputs to robot actions with ROS 2 integration"
sidebar_position: 6
---

# Chapter 3: Action Planning & Execution

## Overview

Action planning and execution form the critical bridge between the perception capabilities provided by Vision-Language Models (VLMs) and the physical manifestation of robot behavior. In Vision-Language-Action (VLA) systems, the action component must translate high-level linguistic commands and visual understanding into specific motor commands that achieve the desired goals while maintaining safety and efficiency.

This chapter explores how VLA systems connect the rich multimodal understanding provided by VLMs to concrete robot actions, focusing on the integration with ROS 2 for reliable and standardized robot control. We'll examine decision-making frameworks, feedback mechanisms, and the practical implementation of action execution in humanoid robots.

## Decision-Making Frameworks

### From VLM Output to Action Space

The transition from VLM output to executable actions involves several key steps:

1. **Semantic Interpretation**: Converting VLM outputs into meaningful action concepts
2. **Action Space Mapping**: Translating high-level concepts to specific robot capabilities
3. **Constraint Validation**: Ensuring actions are feasible and safe
4. **Execution Planning**: Generating detailed motion plans for action execution

The challenge lies in bridging the gap between the abstract representations learned by VLMs and the concrete physical actions that robots can perform. This requires careful design of the interface between perception and action systems.

### Hierarchical Action Planning

VLA systems often employ hierarchical action planning to handle complex tasks:

- **High-Level Planning**: Decomposing complex goals into subtasks
- **Mid-Level Planning**: Generating sequences of primitive actions
- **Low-Level Execution**: Controlling individual joints and actuators

This hierarchy allows the system to handle both high-level commands like "set the table for dinner" and low-level precision tasks like "carefully place the wine glass on the table."

### Task and Motion Planning Integration

Modern VLA systems integrate task planning (what to do) with motion planning (how to do it) to create coherent action sequences. This integration considers:

- **Geometric Constraints**: Physical reachability and collision avoidance
- **Temporal Constraints**: Sequencing of actions over time
- **Resource Constraints**: Energy, time, and computational limitations
- **Environmental Constraints**: Dynamic obstacles and changing conditions

## Connecting VLM Outputs to Robot Actions

### Action Vocabulary Design

The action vocabulary defines the set of actions that the robot can perform and how they connect to VLM outputs. A well-designed vocabulary includes:

- **Basic Motion Primitives**: Move, turn, approach, retreat
- **Manipulation Actions**: Grasp, release, push, pull
- **Navigation Actions**: Go to location, follow path, avoid obstacles
- **Social Actions**: Wave, nod, maintain eye contact

The vocabulary should be designed to match both the linguistic concepts that VLMs can understand and the physical capabilities of the humanoid robot.

### Command Mapping Algorithms

Mapping natural language commands to robot actions requires sophisticated algorithms that handle ambiguity and context:

```python
import re
from typing import Dict, List, Tuple, Optional

class ActionCommandMapper:
    def __init__(self):
        # Define action vocabulary
        self.action_patterns = {
            'move_forward': [
                r'move forward (\d+(?:\.\d+)?)\s*(m|meter|cm|centimeter)',
                r'go forward (\d+(?:\.\d+)?)\s*(m|meter|cm|centimeter)',
                r'walk forward (\d+(?:\.\d+)?)\s*(m|meter|cm|centimeter)'
            ],
            'turn_left': [
                r'turn left (\d+(?:\.\d+)?)\s*(deg|degree)',
                r'turn counterclockwise (\d+(?:\.\d+)?)\s*(deg|degree)',
                r'rotate left (\d+(?:\.\d+)?)\s*(deg|degree)'
            ],
            'grasp': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'pick (.+)'
            ],
            'place': [
                r'place (.+) on (.+)',
                r'put (.+) on (.+)',
                r'place (.+) at (.+)'
            ],
            'navigate': [
                r'go to (.+)',
                r'move to (.+)',
                r'go near (.+)',
                r'approach (.+)'
            ]
        }

    def parse_command(self, command: str) -> Dict:
        """Parse a natural language command into structured action."""
        command_lower = command.lower().strip()

        for action_type, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command_lower, re.IGNORECASE)
                if match:
                    params = list(match.groups())
                    return {
                        'action_type': action_type,
                        'parameters': params,
                        'confidence': 0.9  # Placeholder confidence
                    }

        # If no specific pattern matches, return a generic response
        return {
            'action_type': 'unknown',
            'parameters': [command],
            'confidence': 0.1
        }

    def refine_with_vlm_context(self, vlm_output: Dict, parsed_command: Dict) -> Dict:
        """Refine action based on VLM understanding of the scene."""
        # Use VLM output to disambiguate commands
        if parsed_command['action_type'] == 'grasp':
            if len(parsed_command['parameters']) == 1:
                target_obj = parsed_command['parameters'][0]
                # Use VLM to identify specific object in scene
                if 'objects' in vlm_output:
                    for obj in vlm_output['objects']:
                        if target_obj.lower() in obj['name'].lower():
                            parsed_command['parameters'].append(obj['position'])
                            break

        return parsed_command
```

### Context-Aware Action Selection

VLA systems must consider contextual information when selecting actions:

- **Environmental Context**: Current scene layout, object positions, and obstacles
- **Social Context**: Human presence, social norms, and interaction protocols
- **Task Context**: Current task state, previous actions, and expected outcomes
- **Physical Context**: Robot state, battery level, and available resources

## ROS 2 Integration for Action Execution

### Action Server Architecture

ROS 2 provides a robust framework for action execution through action servers that handle long-running tasks with feedback and goal management:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Example action definition (this would be defined in .action files)
# In practice, you would define your own action types like:
# action/VLAAction.action
# string action_type
# string[] parameters
# ---
# bool success
# string message
# ---
# float32 progress
# string status

class VLAActionServer(Node):
    def __init__(self):
        super().__init__('vla_action_server')

        # Create action server with reentrant callback group for concurrent handling
        self._action_server = ActionServer(
            self,
            # VLAAction,  # Your custom action type
            'vla_execute_action',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Robot control interfaces
        self.robot_controller = RobotController()
        self.get_logger().info('VLA Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request: {goal_request.action_type}')

        # Validate goal feasibility
        if self.is_goal_feasible(goal_request):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Initialize feedback
        feedback_msg = None  # VLAAction.Feedback()
        feedback_msg.progress = 0.0
        feedback_msg.status = "Initializing"

        # Execute the action based on goal request
        action_result = self.execute_action(goal_handle.request, goal_handle)

        # Determine success based on action execution
        goal_handle.succeed()

        result = None  # VLAAction.Result()
        result.success = action_result['success']
        result.message = action_result['message']

        self.get_logger().info(f'Returning result: {result.success}')
        return result

    def is_goal_feasible(self, goal_request):
        """Check if the requested action is feasible."""
        # Implement feasibility checks
        return True

    def execute_action(self, request, goal_handle):
        """Execute the specific action based on request parameters."""
        action_type = request.action_type
        params = request.parameters

        try:
            if action_type == 'move_forward':
                return self.move_forward(float(params[0]) if params else 1.0, goal_handle)
            elif action_type == 'turn_left':
                return self.turn_left(float(params[0]) if params else 90.0, goal_handle)
            elif action_type == 'grasp':
                return self.grasp_object(params[0] if params else 'object', goal_handle)
            elif action_type == 'place':
                return self.place_object(params[0] if len(params) > 0 else 'object',
                                       params[1] if len(params) > 1 else 'table', goal_handle)
            elif action_type == 'navigate':
                return self.navigate_to_location(params[0] if params else 'location', goal_handle)
            else:
                return {'success': False, 'message': f'Unknown action: {action_type}'}
        except Exception as e:
            self.get_logger().error(f'Error executing action: {e}')
            return {'success': False, 'message': f'Execution error: {str(e)}'}

    def move_forward(self, distance, goal_handle):
        """Execute forward movement."""
        self.get_logger().info(f'Moving forward {distance} meters')
        # Implement actual movement logic
        return {'success': True, 'message': f'Moved forward {distance} meters'}

    def turn_left(self, angle, goal_handle):
        """Execute left turn."""
        self.get_logger().info(f'Turning left {angle} degrees')
        # Implement actual turning logic
        return {'success': True, 'message': f'Turned left {angle} degrees'}

    def grasp_object(self, object_name, goal_handle):
        """Execute grasping action."""
        self.get_logger().info(f'Attempting to grasp {object_name}')
        # Implement actual grasping logic
        return {'success': True, 'message': f'Grasped {object_name}'}

    def place_object(self, object_name, location, goal_handle):
        """Execute placing action."""
        self.get_logger().info(f'Placing {object_name} at {location}')
        # Implement actual placing logic
        return {'success': True, 'message': f'Placed {object_name} at {location}'}

    def navigate_to_location(self, location, goal_handle):
        """Execute navigation to location."""
        self.get_logger().info(f'Navigating to {location}')
        # Implement actual navigation logic
        return {'success': True, 'message': f'Navigated to {location}'}

def main(args=None):
    rclpy.init(args=args)

    vla_action_server = VLAActionServer()

    # Use multi-threaded executor to handle multiple concurrent actions
    executor = MultiThreadedExecutor()
    rclpy.spin(vla_action_server, executor=executor)

    vla_action_server.destroy_node()
    rclpy.shutdown()
```

### Action Client Implementation

Action clients allow other nodes to request action execution:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

class VLAActionClient(Node):
    def __init__(self):
        super().__init__('vla_action_client')
        self._action_client = ActionClient(
            self,
            # VLAAction,  # Your custom action type
            'vla_execute_action'
        )

    def send_action_goal(self, action_type, parameters):
        """Send an action goal to the server."""
        goal_msg = None  # VLAAction.Goal()
        goal_msg.action_type = action_type
        goal_msg.parameters = parameters

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: {action_type} with params {parameters}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle action result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')

    def feedback_callback(self, feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.status}, {feedback.progress:.2f}')
```

## Feedback Loops and Adaptive Behavior

### Sensory Feedback Integration

VLA systems must continuously monitor action execution and adapt based on sensory feedback:

- **Visual Feedback**: Confirming object positions, grasp success, and action outcomes
- **Proprioceptive Feedback**: Monitoring joint positions, forces, and robot state
- **Tactile Feedback**: Detecting contact, slip, and manipulation success
- **Audio Feedback**: Detecting environmental changes and human responses

### Closed-Loop Control

Closed-loop control ensures that actions adapt to changing conditions:

```python
import threading
import time
from collections import deque

class ClosedLoopController:
    def __init__(self, robot_controller, perception_system):
        self.robot_controller = robot_controller
        self.perception_system = perception_system
        self.control_thread = None
        self.running = False
        self.feedback_buffer = deque(maxlen=10)  # Store recent feedback

    def start_control_loop(self, action_plan):
        """Start closed-loop control for an action plan."""
        self.running = True
        self.control_thread = threading.Thread(
            target=self.control_loop,
            args=(action_plan,)
        )
        self.control_thread.start()

    def control_loop(self, action_plan):
        """Main control loop with feedback integration."""
        for action in action_plan:
            if not self.running:
                break

            # Execute action
            success = self.execute_action_with_monitoring(action)

            if not success:
                # Handle failure by replanning or recovery
                self.handle_action_failure(action)

            # Update perception for next iteration
            self.update_perception()

    def execute_action_with_monitoring(self, action):
        """Execute action while monitoring for success/failure."""
        start_time = time.time()

        # Start action execution
        self.robot_controller.execute_action(action)

        # Monitor execution with feedback
        while self.robot_controller.is_executing():
            if not self.running:
                return False

            # Get current state
            current_state = self.robot_controller.get_state()
            perceived_state = self.perception_system.get_perception()

            # Check for success conditions
            if self.check_success_conditions(action, current_state, perceived_state):
                return True

            # Check for failure conditions
            if self.check_failure_conditions(action, current_state, perceived_state):
                return False

            # Adjust action based on feedback
            self.adjust_action(action, current_state, perceived_state)

            time.sleep(0.1)  # Control loop frequency

        return True  # Assume success if execution completes

    def check_success_conditions(self, action, current_state, perceived_state):
        """Check if action has been successfully completed."""
        # Implementation depends on action type
        return False

    def check_failure_conditions(self, action, current_state, perceived_state):
        """Check if action has failed."""
        # Implementation depends on action type
        return False

    def adjust_action(self, action, current_state, perceived_state):
        """Adjust action based on current state and perception."""
        # Implementation depends on action type
        pass

    def handle_action_failure(self, failed_action):
        """Handle action failure with recovery strategies."""
        # Implement recovery logic
        pass

    def update_perception(self):
        """Update perception system for current state."""
        self.perception_system.update()
```

### Adaptive Behavior Systems

Adaptive behavior allows VLA systems to adjust their actions based on experience and environmental changes:

- **Learning from Experience**: Improving action selection based on past outcomes
- **Context Adaptation**: Modifying behavior based on environmental conditions
- **Human Preference Learning**: Adapting to individual user preferences
- **Safety Adaptation**: Adjusting behavior based on risk assessment

## Safety and Error Handling

### Safety Constraints Implementation

Safety is paramount in VLA systems, especially when controlling humanoid robots around humans:

```python
class SafetyManager:
    def __init__(self):
        self.safety_constraints = {
            'collision_avoidance': True,
            'force_limits': True,
            'workspace_bounds': True,
            'human_proximity': True
        }
        self.emergency_stop = False

    def validate_action(self, action):
        """Validate action against safety constraints."""
        if self.emergency_stop:
            return False, "Emergency stop active"

        # Check collision constraints
        if self.safety_constraints['collision_avoidance']:
            collision_risk = self.check_collision_risk(action)
            if collision_risk > 0.9:  # High risk threshold
                return False, "High collision risk"

        # Check force limits
        if self.safety_constraints['force_limits']:
            force_exceeded = self.check_force_limits(action)
            if force_exceeded:
                return False, "Force limits exceeded"

        # Check workspace bounds
        if self.safety_constraints['workspace_bounds']:
            out_of_bounds = self.check_workspace_bounds(action)
            if out_of_bounds:
                return False, "Action would exceed workspace bounds"

        # Check human proximity
        if self.safety_constraints['human_proximity']:
            human_too_close = self.check_human_proximity(action)
            if human_too_close:
                return False, "Action would bring robot too close to human"

        return True, "Action is safe"

    def check_collision_risk(self, action):
        """Check for potential collisions."""
        # Implementation would use motion planning and collision detection
        return 0.0

    def check_force_limits(self, action):
        """Check if action would exceed force limits."""
        # Implementation would check expected forces
        return False

    def check_workspace_bounds(self, action):
        """Check if action would exceed workspace bounds."""
        # Implementation would check joint limits and workspace
        return False

    def check_human_proximity(self, action):
        """Check if action would bring robot too close to humans."""
        # Implementation would use human detection and tracking
        return False

    def trigger_emergency_stop(self):
        """Trigger emergency stop for immediate safety."""
        self.emergency_stop = True
        # Implement immediate robot stop
        self.stop_robot_immediately()

    def stop_robot_immediately(self):
        """Stop all robot motion immediately."""
        # Implementation would send immediate stop commands
        pass
```

### Error Recovery Strategies

VLA systems must implement robust error recovery to handle failures gracefully:

- **Graceful Degradation**: Continuing operation with reduced capabilities
- **Fallback Behaviors**: Using alternative methods when primary approach fails
- **Human Intervention**: Requesting human assistance when needed
- **Safe State Recovery**: Returning to a safe state when errors occur

## Implementation Examples

### Complete VLA Integration Node

Here's an example of a complete node that integrates VLM perception with action execution:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import threading
import time

class VLAIntegratedNode(Node):
    def __init__(self):
        super().__init__('vla_integrated_node')
        self.bridge = CvBridge()

        # Initialize components
        self.vlm_model = None  # Initialize your VLM model
        self.action_mapper = ActionCommandMapper()
        self.action_server = VLAActionServer(self)  # Embedded action server
        self.safety_manager = SafetyManager()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(
            String, 'vla_command', self.command_callback, 10)
        self.vision_pub = self.create_publisher(
            String, 'vla_vision_output', 10)
        self.action_pub = self.create_publisher(
            String, 'vla_action_output', 10)

        # Internal state
        self.latest_image = None
        self.latest_command = None
        self.system_active = True
        self.processing_lock = threading.Lock()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('VLA Integrated Node initialized')

    def image_callback(self, msg):
        """Handle incoming image data."""
        with self.processing_lock:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image

    def command_callback(self, msg):
        """Handle incoming command data."""
        with self.processing_lock:
            self.latest_command = msg.data
            self.get_logger().info(f'Received command: {msg.data}')

    def process_loop(self):
        """Main processing loop for VLA integration."""
        while self.system_active:
            with self.processing_lock:
                if self.latest_image is not None and self.latest_command is not None:
                    # Process VLA pipeline
                    self.process_vla_input(
                        self.latest_image,
                        self.latest_command
                    )

                    # Reset processed inputs
                    self.latest_image = None
                    self.latest_command = None

            time.sleep(0.1)  # Processing frequency

    def process_vla_input(self, image, command):
        """Process combined vision-language input to generate action."""
        try:
            # Step 1: Process visual input with VLM
            vlm_output = self.process_visual_input(image)

            # Step 2: Parse command with action mapper
            parsed_command = self.action_mapper.parse_command(command)

            # Step 3: Refine with VLM context
            refined_command = self.action_mapper.refine_with_vlm_context(
                vlm_output, parsed_command
            )

            # Step 4: Validate with safety manager
            is_safe, safety_msg = self.safety_manager.validate_action(refined_command)
            if not is_safe:
                self.get_logger().warn(f'Safety check failed: {safety_msg}')
                return

            # Step 5: Execute action
            self.execute_action(refined_command)

            # Log the complete pipeline
            self.get_logger().info(
                f'VLA Pipeline: "{command}" -> {refined_command["action_type"]} '
                f'with confidence {refined_command["confidence"]:.2f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error in VLA processing: {e}')

    def process_visual_input(self, image):
        """Process visual input through VLM."""
        # Placeholder for VLM processing
        # In practice, this would run the image through your VLM model
        return {
            'objects': [{'name': 'test_object', 'position': [0, 0, 0]}],
            'scene_description': 'test scene'
        }

    def execute_action(self, action_command):
        """Execute the action command."""
        # Publish action for execution
        action_msg = String()
        action_msg.data = str(action_command)
        self.action_pub.publish(action_msg)

    def destroy_node(self):
        """Clean up before destroying node."""
        self.system_active = False
        if self.processing_thread.is_alive():
            self.processing_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    vla_node = VLAIntegratedNode()

    try:
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter explored the critical connection between Vision-Language Models and robot action execution in VLA systems. We examined decision-making frameworks, the mapping of VLM outputs to robot actions, and the integration with ROS 2 for reliable action execution. The chapter covered feedback loops, adaptive behavior systems, and essential safety considerations for deploying VLA systems with humanoid robots.

In the next chapter, we will focus on the integration of all VLA components in simulation environments, evaluation metrics, and the implementation of complete VLA systems for humanoid robots.