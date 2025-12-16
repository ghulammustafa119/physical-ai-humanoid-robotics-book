---
title: "Chapter 4: Bridging the AI Brain to ROS 2"
description: "Python AI agents, ROS 2 integration, and closed-loop feedback for humanoid robots"
sidebar_position: 4
---

# Chapter 4: Bridging the AI Brain to ROS 2

## Introduction

The final layer of the AI-robot brain is the bridge to ROS 2, where high-level decisions become concrete robot actions. This bridge translates the outputs of perception, planning, and decision-making systems into ROS 2 messages that control the physical robot. The bridge must handle the transition from abstract intelligence to concrete action while maintaining the closed-loop feedback that enables intelligent, adaptive behavior.

This chapter explores how Python AI agents function as ROS 2 nodes, how decisions become ROS 2 actions, and how closed-loop feedback enables intelligent robot behavior. We'll examine the patterns and practices for integrating AI intelligence with the ROS 2 communication framework.

## Python AI Agents as ROS 2 Nodes

AI agents in the robot brain are implemented as ROS 2 nodes, enabling them to participate in the ROS 2 communication ecosystem while maintaining their intelligent decision-making capabilities.

### AI Agent Architecture

Python AI agents follow a structured architecture that integrates intelligence with ROS 2 communication:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

class AIBrainNode(Node):
    """
    AI Brain node that bridges high-level intelligence to ROS 2 control.
    This node receives sensor data, makes decisions, and sends commands to the robot.
    """

    def __init__(self):
        super().__init__('ai_brain_node')

        # Initialize perception, planning, and decision-making components
        self.perception_system = PerceptionSystem()
        self.planning_system = PlanningSystem()
        self.decision_system = DecisionSystem()

        # Initialize ROS 2 communication components
        self.setup_publishers()
        self.setup_subscribers()

        # Initialize internal state
        self.robot_state = {}
        self.goals = []
        self.active_plan = None

        # Setup processing timer
        self.processing_timer = self.create_timer(
            0.1,  # Process at 10 Hz
            self.ai_processing_loop
        )

        self.get_logger().info('AI Brain node initialized')

    def setup_publishers(self):
        """Setup ROS 2 publishers for sending commands to the robot."""
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for navigation goals
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Publisher for robot status
        self.status_publisher = self.create_publisher(
            String,
            '/ai_brain/status',
            10
        )

        # Publisher for action commands
        self.action_publisher = self.create_publisher(
            String,
            '/ai_brain/action',
            10
        )

    def setup_subscribers(self):
        """Setup ROS 2 subscribers for receiving sensor data."""
        # Subscribe to sensor data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )

        # Subscribe to robot state updates
        self.robot_state_subscriber = self.create_subscription(
            String,
            '/robot/state',
            self.robot_state_callback,
            10
        )

        # Subscribe to goals from higher-level systems
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

    def lidar_callback(self, msg):
        """Process LiDAR sensor data."""
        self.robot_state['lidar'] = {
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def camera_callback(self, msg):
        """Process camera sensor data."""
        self.robot_state['camera'] = {
            'encoding': msg.encoding,
            'height': msg.height,
            'width': msg.width,
            'data': msg.data  # In practice, this would be processed further
        }

    def imu_callback(self, msg):
        """Process IMU sensor data."""
        self.robot_state['imu'] = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

    def odometry_callback(self, msg):
        """Process odometry data."""
        self.robot_state['odometry'] = {
            'pose': msg.pose.pose,
            'twist': msg.twist.twist
        }

    def robot_state_callback(self, msg):
        """Process robot state updates."""
        self.robot_state['status'] = msg.data

    def goal_callback(self, msg):
        """Process new goal requests."""
        new_goal = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
            'timestamp': msg.header.stamp
        }
        self.goals.append(new_goal)

    def ai_processing_loop(self):
        """Main AI processing loop that runs continuously."""
        if not self.robot_state:
            self.get_logger().debug('Waiting for sensor data...')
            return

        try:
            # 1. Update perception system with current sensor data
            world_model = self.perception_system.update(self.robot_state)

            # 2. Process goals and update planning system
            if self.goals:
                self.active_plan = self.planning_system.update_goals(self.goals, world_model)

            # 3. Make decisions based on current state and plan
            decision = self.decision_system.make_decision(
                world_model=world_model,
                active_plan=self.active_plan,
                robot_state=self.robot_state
            )

            # 4. Execute decision by sending commands to robot
            self.execute_decision(decision)

            # 5. Publish status updates
            self.publish_status(decision)

        except Exception as e:
            self.get_logger().error(f'Error in AI processing loop: {e}')
            self.publish_error_status(str(e))

    def execute_decision(self, decision):
        """Execute AI decision by sending appropriate ROS 2 commands."""
        if decision['type'] == 'navigation':
            self.send_navigation_command(decision['params'])
        elif decision['type'] == 'manipulation':
            self.send_manipulation_command(decision['params'])
        elif decision['type'] == 'interaction':
            self.send_interaction_command(decision['params'])
        elif decision['type'] == 'idle':
            self.send_idle_command()
        else:
            self.get_logger().warn(f'Unknown decision type: {decision["type"]}')

    def send_navigation_command(self, params):
        """Send navigation commands to robot."""
        cmd = Twist()
        cmd.linear.x = params.get('linear_velocity', 0.0)
        cmd.linear.y = params.get('linear_y_velocity', 0.0)
        cmd.linear.z = params.get('linear_z_velocity', 0.0)
        cmd.angular.x = params.get('angular_x_velocity', 0.0)
        cmd.angular.y = params.get('angular_y_velocity', 0.0)
        cmd.angular.z = params.get('angular_z_velocity', 0.0)

        self.cmd_vel_publisher.publish(cmd)

    def send_manipulation_command(self, params):
        """Send manipulation commands to robot."""
        # This would publish to manipulation-specific topics
        # For example, joint position commands or gripper control
        action_msg = String()
        action_msg.data = f'manipulation:{params}'
        self.action_publisher.publish(action_msg)

    def send_interaction_command(self, params):
        """Send interaction commands to robot."""
        # This could control speech, gestures, or other interactive behaviors
        action_msg = String()
        action_msg.data = f'interaction:{params}'
        self.action_publisher.publish(action_msg)

    def send_idle_command(self):
        """Send idle/stop commands to robot."""
        cmd = Twist()  # Zero velocities
        self.cmd_vel_publisher.publish(cmd)

    def publish_status(self, decision):
        """Publish AI brain status."""
        status_msg = String()
        status_msg.data = f"Active: {decision['type']}, Goals: {len(self.goals)}"
        self.status_publisher.publish(status_msg)

    def publish_error_status(self, error_msg):
        """Publish error status."""
        status_msg = String()
        status_msg.data = f"ERROR: {error_msg}"
        self.status_publisher.publish(status_msg)

    def cleanup(self):
        """Cleanup resources before shutdown."""
        self.get_logger().info('Shutting down AI Brain node')


class PerceptionSystem:
    """Handles perception and world modeling."""

    def __init__(self):
        self.world_model = {}

    def update(self, sensor_data):
        """Update world model based on sensor data."""
        # Process sensor data to create world understanding
        # This is where perception algorithms would run

        processed_data = {
            'objects': self.detect_objects(sensor_data),
            'obstacles': self.identify_obstacles(sensor_data),
            'free_space': self.map_free_space(sensor_data),
            'robot_pose': self.estimate_robot_pose(sensor_data)
        }

        self.world_model.update(processed_data)
        return self.world_model

    def detect_objects(self, sensor_data):
        """Detect objects in the environment."""
        # In practice, this would use computer vision algorithms
        return []

    def identify_obstacles(self, sensor_data):
        """Identify obstacles for navigation."""
        # Process LiDAR and other sensor data to find obstacles
        return []

    def map_free_space(self, sensor_data):
        """Map free navigable space."""
        # Create map of traversable areas
        return []

    def estimate_robot_pose(self, sensor_data):
        """Estimate robot's current position and orientation."""
        # Use odometry and other data to estimate pose
        return {'x': 0.0, 'y': 0.0, 'theta': 0.0}


class PlanningSystem:
    """Handles path and task planning."""

    def __init__(self):
        self.current_plan = None

    def update_goals(self, goals, world_model):
        """Update planning based on new goals and world model."""
        if not goals:
            return None

        # Select the most relevant goal
        active_goal = goals[0]  # Simple selection for example

        # Plan path to goal
        plan = self.create_plan(active_goal, world_model)

        # Update internal state
        self.current_plan = plan

        return plan

    def create_plan(self, goal, world_model):
        """Create a plan to achieve the goal."""
        # This would implement planning algorithms
        # For example, A* for path planning, STRIPS for task planning

        plan = {
            'goal': goal,
            'path': self.compute_path_to_goal(goal, world_model),
            'actions': self.determine_required_actions(goal, world_model),
            'constraints': self.identify_constraints(world_model)
        }

        return plan

    def compute_path_to_goal(self, goal, world_model):
        """Compute path to the goal."""
        # Implement path planning algorithm
        return []

    def determine_required_actions(self, goal, world_model):
        """Determine actions needed to achieve goal."""
        # Plan sequence of actions
        return []

    def identify_constraints(self, world_model):
        """Identify constraints for plan execution."""
        # Consider obstacles, safety, etc.
        return {}


class DecisionSystem:
    """Makes decisions based on planning and perception."""

    def __init__(self):
        self.decision_policy = None  # Could be learned policy or rule-based

    def make_decision(self, world_model, active_plan, robot_state):
        """Make decision based on current information."""
        if active_plan is None:
            return {
                'type': 'idle',
                'params': {},
                'confidence': 1.0
            }

        # Determine what action to take based on plan
        decision = self.select_action(world_model, active_plan, robot_state)

        return decision

    def select_action(self, world_model, active_plan, robot_state):
        """Select appropriate action based on current state."""
        # This could implement various decision-making approaches:
        # - Rule-based reasoning
        # - Utility-based decision making
        # - Reinforcement learning policy
        # - Hierarchical task network

        # For this example, simple navigation decision
        if 'path' in active_plan and active_plan['path']:
            # Navigate along planned path
            next_waypoint = active_plan['path'][0] if active_plan['path'] else None

            if next_waypoint:
                # Calculate navigation command
                nav_command = self.calculate_navigation_to_waypoint(next_waypoint, robot_state)

                return {
                    'type': 'navigation',
                    'params': nav_command,
                    'confidence': 0.9
                }

        # Default idle if no plan or waypoints
        return {
            'type': 'idle',
            'params': {},
            'confidence': 1.0
        }

    def calculate_navigation_to_waypoint(self, waypoint, robot_state):
        """Calculate navigation command to reach waypoint."""
        # Simple proportional navigation controller
        robot_pose = robot_state.get('odometry', {}).get('pose', {})

        # Calculate desired velocity to reach waypoint
        dx = waypoint['x'] - robot_pose.get('position', {}).get('x', 0)
        dy = waypoint['y'] - robot_pose.get('position', {}).get('y', 0)

        # Simple proportional control
        linear_vel = min(0.5, (dx**2 + dy**2)**0.5)  # Scale with distance
        angular_vel = 1.0 * (dy / (abs(dx) + 0.01))  # Simple heading correction

        return {
            'linear_velocity': linear_vel,
            'angular_z_velocity': angular_vel
        }


def main(args=None):
    """Main entry point for the AI Brain node."""
    rclpy.init(args=args)

    ai_brain_node = AIBrainNode()

    try:
        rclpy.spin(ai_brain_node)
    except KeyboardInterrupt:
        ai_brain_node.get_logger().info('Interrupted, shutting down...')
    finally:
        ai_brain_node.cleanup()
        ai_brain_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### AI Agent Responsibilities

AI agents in this architecture have several key responsibilities:

1. **Sensor Data Integration**: Collect and process data from multiple sensors
2. **World Modeling**: Create and maintain understanding of the environment
3. **Goal Processing**: Interpret and prioritize goals from various sources
4. **Decision Making**: Determine appropriate actions based on current state
5. **Command Execution**: Send commands to robot control systems
6. **Status Reporting**: Provide feedback on AI brain state and decisions

### Node Lifecycle Management

AI agents follow standard ROS 2 node lifecycles:

- **Initialization**: Setup publishers, subscribers, and internal state
- **Operation**: Continuous processing loop with sensor data and decision making
- **Shutdown**: Cleanup resources and ensure safe robot state

## How Decisions Become ROS 2 Actions

The transformation from AI decisions to ROS 2 actions involves several steps that ensure safe and appropriate robot behavior.

### Decision Processing Pipeline

```python
class DecisionProcessor:
    """Process AI decisions into ROS 2 commands."""

    def __init__(self, node):
        self.node = node
        self.safety_checker = SafetyChecker()
        self.command_generator = CommandGenerator()

    def process_decision(self, decision):
        """Process AI decision and convert to ROS 2 commands."""
        # 1. Validate decision safety
        if not self.safety_checker.is_safe(decision):
            self.node.get_logger().warn('Decision is unsafe, overriding...')
            return self.generate_safe_override()

        # 2. Validate decision feasibility
        if not self.is_feasible(decision):
            self.node.get_logger().warn('Decision is infeasible, selecting alternative...')
            return self.generate_alternative_action(decision)

        # 3. Generate appropriate ROS 2 commands
        ros_commands = self.command_generator.generate_commands(decision)

        # 4. Apply safety limits
        safe_commands = self.apply_safety_limits(ros_commands)

        # 5. Publish commands to robot
        self.publish_commands(safe_commands)

        return safe_commands

    def is_feasible(self, decision):
        """Check if decision is feasible."""
        # Check if robot has capability to execute decision
        # Check if environment allows execution
        # Check if current state permits action
        return True

    def generate_safe_override(self):
        """Generate safe override commands."""
        # Stop robot or return to safe state
        pass

    def generate_alternative_action(self, original_decision):
        """Generate alternative action when original is not feasible."""
        # Find safe alternative to original decision
        pass

    def apply_safety_limits(self, commands):
        """Apply safety limits to commands."""
        # Limit velocities, accelerations, forces
        # Ensure commands stay within safe operating ranges
        pass

    def publish_commands(self, commands):
        """Publish commands to appropriate ROS 2 topics."""
        # Send commands to robot control systems
        pass


class SafetyChecker:
    """Check if decisions are safe for robot and environment."""

    def __init__(self):
        self.safety_limits = {
            'velocity': {'linear': 1.0, 'angular': 1.0},  # m/s, rad/s
            'acceleration': {'linear': 2.0, 'angular': 2.0},  # m/s², rad/s²
            'force': 100.0,  # N
            'torque': 50.0  # Nm
        }

    def is_safe(self, decision):
        """Check if decision is safe."""
        # Check velocity limits
        if 'params' in decision and 'linear_velocity' in decision['params']:
            if abs(decision['params']['linear_velocity']) > self.safety_limits['velocity']['linear']:
                return False

        # Check for collision risks
        if self.would_cause_collision(decision):
            return False

        # Check for stability risks
        if self.would_compromise_stability(decision):
            return False

        return True

    def would_cause_collision(self, decision):
        """Check if decision would cause collision."""
        # Analyze decision against current world model
        # Check predicted robot path for obstacles
        return False

    def would_compromise_stability(self, decision):
        """Check if decision would compromise robot stability."""
        # Analyze decision against robot dynamics
        # Check center of mass, support polygon, etc.
        return False


class CommandGenerator:
    """Generate ROS 2 commands from AI decisions."""

    def __init__(self):
        pass

    def generate_commands(self, decision):
        """Generate appropriate ROS 2 commands for decision."""
        command_type = decision.get('type', 'unknown')

        if command_type == 'navigation':
            return self.generate_navigation_commands(decision)
        elif command_type == 'manipulation':
            return self.generate_manipulation_commands(decision)
        elif command_type == 'interaction':
            return self.generate_interaction_commands(decision)
        elif command_type == 'idle':
            return self.generate_idle_commands(decision)
        else:
            return self.generate_unknown_command(decision)

    def generate_navigation_commands(self, decision):
        """Generate navigation commands."""
        params = decision.get('params', {})

        cmd = Twist()
        cmd.linear.x = params.get('linear_velocity', 0.0)
        cmd.linear.y = params.get('linear_y_velocity', 0.0)
        cmd.linear.z = params.get('linear_z_velocity', 0.0)
        cmd.angular.x = params.get('angular_x_velocity', 0.0)
        cmd.angular.y = params.get('angular_y_velocity', 0.0)
        cmd.angular.z = params.get('angular_z_velocity', 0.0)

        return {'cmd_vel': cmd}

    def generate_manipulation_commands(self, decision):
        """Generate manipulation commands."""
        # This would generate joint position, velocity, or effort commands
        pass

    def generate_interaction_commands(self, decision):
        """Generate interaction commands."""
        # This would generate speech, gesture, or display commands
        pass

    def generate_idle_commands(self, decision):
        """Generate idle commands (stop robot)."""
        cmd = Twist()  # Zero all velocities
        return {'cmd_vel': cmd}

    def generate_unknown_command(self, decision):
        """Generate command for unknown decision type."""
        # Default to safe idle command
        return self.generate_idle_commands({'type': 'idle'})
```

### Command Types and Mapping

Different decision types map to different ROS 2 command types:

- **Navigation decisions** → `geometry_msgs/Twist` for movement control
- **Manipulation decisions** → Joint commands or action goals
- **Interaction decisions** → Speech, gesture, or UI commands
- **Monitoring decisions** → Sensor configuration or data requests

## Closed-Loop Feedback Explanation

Closed-loop feedback is essential for intelligent robot behavior, enabling the AI brain to adapt its decisions based on outcomes and environmental changes.

### Feedback Loop Architecture

```
Perception → Planning → Decision → Action → Robot → Environment → Perception
    ↑                                                                 ↓
    └──────────────────────── Feedback ──────────────────────────────┘
```

### State Estimation and Monitoring

The AI brain continuously monitors robot state and environment:

```python
class StateEstimator:
    """Estimate robot and environment state for feedback."""

    def __init__(self):
        self.robot_state_history = []
        self.environment_state = {}
        self.estimated_outcomes = {}

    def update_state_estimate(self, sensor_data, command_sent, time_stamp):
        """Update state estimate with new information."""
        # Update robot state based on sensor feedback
        self.update_robot_state(sensor_data, time_stamp)

        # Update environment state based on observations
        self.update_environment_state(sensor_data, time_stamp)

        # Estimate outcome of recent commands
        self.estimate_command_outcome(command_sent, sensor_data, time_stamp)

    def update_robot_state(self, sensor_data, time_stamp):
        """Update estimate of robot's current state."""
        new_state = {
            'position': self.estimate_position(sensor_data),
            'orientation': self.estimate_orientation(sensor_data),
            'velocity': self.estimate_velocity(sensor_data),
            'effort': self.estimate_effort(sensor_data),
            'timestamp': time_stamp
        }

        self.robot_state_history.append(new_state)

        # Keep only recent history to manage memory
        if len(self.robot_state_history) > 100:
            self.robot_state_history.pop(0)

    def update_environment_state(self, sensor_data, time_stamp):
        """Update estimate of environment state."""
        # Update map of environment
        # Update locations of objects
        # Update status of dynamic elements
        pass

    def estimate_command_outcome(self, command_sent, sensor_data, time_stamp):
        """Estimate whether command achieved desired outcome."""
        # Compare expected vs actual outcomes
        # Update success/failure statistics
        # Adjust future decision-making based on outcomes
        pass

    def get_current_state(self):
        """Get current estimated state."""
        if self.robot_state_history:
            return {
                'robot': self.robot_state_history[-1],
                'environment': self.environment_state,
                'performance': self.estimate_performance()
            }
        else:
            return None

    def estimate_performance(self):
        """Estimate performance based on recent outcomes."""
        # Calculate success rates
        # Identify patterns in failures
        # Assess efficiency of actions
        pass
```

### Adaptive Decision Making

The feedback loop enables adaptive decision making:

```python
class AdaptiveDecisionMaker:
    """Make decisions that adapt based on feedback."""

    def __init__(self):
        self.performance_history = []
        self.adaptation_rules = {}
        self.learning_enabled = True

    def make_adaptive_decision(self, current_state, original_plan, feedback):
        """Make decision that adapts based on feedback."""
        # Analyze recent performance
        performance_analysis = self.analyze_recent_performance(feedback)

        # Check if current approach is effective
        if not performance_analysis['effective']:
            # Adapt decision-making strategy
            adapted_plan = self.adapt_plan(original_plan, performance_analysis)
            decision = self.make_decision_with_plan(adapted_plan, current_state)
        else:
            # Continue with current approach
            decision = self.make_decision_with_plan(original_plan, current_state)

        # Record decision for future learning
        self.record_decision_outcome(decision, current_state)

        return decision

    def analyze_recent_performance(self, feedback):
        """Analyze recent performance based on feedback."""
        # Look for patterns in success/failure
        # Identify environmental factors affecting performance
        # Assess efficiency metrics
        return {
            'effective': True,  # Simplified for example
            'success_rate': 0.8,
            'patterns': [],
            'recommendations': []
        }

    def adapt_plan(self, original_plan, performance_analysis):
        """Adapt plan based on performance analysis."""
        # Modify plan based on identified issues
        # Adjust parameters for better performance
        # Consider alternative strategies
        adapted_plan = original_plan.copy()

        # Example: if navigation is too slow, increase velocity limits
        if performance_analysis.get('too_slow'):
            adapted_plan['velocity_limit'] *= 1.2

        return adapted_plan

    def make_decision_with_plan(self, plan, current_state):
        """Make decision following the given plan."""
        # Implement decision-making logic that follows plan
        pass

    def record_decision_outcome(self, decision, state):
        """Record decision and outcome for learning."""
        outcome_record = {
            'decision': decision,
            'state': state,
            'timestamp': self.get_current_time(),
            'expected_outcome': decision.get('expected_outcome'),
            'actual_outcome': None  # Will be updated when feedback arrives
        }

        self.performance_history.append(outcome_record)

        # Limit history size
        if len(self.performance_history) > 1000:
            self.performance_history.pop(0)

    def get_current_time(self):
        """Get current time for timestamping."""
        import time
        return time.time()

    def learn_from_experience(self):
        """Learn from recorded experiences."""
        if not self.learning_enabled:
            return

        # Analyze patterns in successful/unsuccessful decisions
        # Update decision-making parameters
        # Improve future performance
        pass
```

### Performance Monitoring and Adjustment

Continuous monitoring enables performance optimization:

```python
class PerformanceMonitor:
    """Monitor AI brain performance and enable adjustments."""

    def __init__(self):
        self.metrics = {
            'decision_rate': [],
            'success_rate': [],
            'computation_time': [],
            'resource_usage': []
        }
        self.thresholds = {
            'min_success_rate': 0.7,
            'max_computation_time': 0.1,  # seconds
            'max_cpu_usage': 80.0  # percent
        }
        self.adjustment_callbacks = []

    def record_metric(self, metric_name, value, timestamp=None):
        """Record performance metric."""
        if metric_name not in self.metrics:
            self.metrics[metric_name] = []

        metric_entry = {
            'value': value,
            'timestamp': timestamp or self.get_current_time()
        }

        self.metrics[metric_name].append(metric_entry)

        # Keep only recent metrics
        self.trim_metrics(metric_name)

        # Check if adjustment is needed
        self.check_adjustment_needed(metric_name, value)

    def trim_metrics(self, metric_name):
        """Trim metrics history to manage memory."""
        # Keep only last 1000 entries
        if len(self.metrics[metric_name]) > 1000:
            self.metrics[metric_name] = self.metrics[metric_name][-1000:]

    def check_adjustment_needed(self, metric_name, value):
        """Check if performance adjustment is needed."""
        if metric_name == 'success_rate':
            if value < self.thresholds['min_success_rate']:
                self.trigger_adjustment('performance_degradation')
        elif metric_name == 'computation_time':
            if value > self.thresholds['max_computation_time']:
                self.trigger_adjustment('computation_overload')

    def trigger_adjustment(self, adjustment_type):
        """Trigger performance adjustment."""
        for callback in self.adjustment_callbacks:
            callback(adjustment_type)

    def add_adjustment_callback(self, callback):
        """Add callback for performance adjustments."""
        self.adjustment_callbacks.append(callback)

    def get_performance_summary(self):
        """Get summary of recent performance."""
        summary = {}
        for metric_name, values in self.metrics.items():
            if values:
                recent_values = [v['value'] for v in values[-10:]]  # Last 10 values
                summary[metric_name] = {
                    'current': recent_values[-1] if recent_values else None,
                    'average': sum(recent_values) / len(recent_values) if recent_values else None,
                    'trend': self.calculate_trend(recent_values)
                }

        return summary

    def calculate_trend(self, values):
        """Calculate trend of recent values."""
        if len(values) < 2:
            return 'stable'

        # Simple trend calculation
        if values[-1] > values[0]:
            return 'increasing'
        elif values[-1] < values[0]:
            return 'decreasing'
        else:
            return 'stable'

    def get_current_time(self):
        """Get current time for timestamping."""
        import time
        return time.time()
```

## Integration Patterns

### Publisher-Subscriber Pattern

The AI brain uses ROS 2's publisher-subscriber pattern extensively:

- **Subscribers**: Listen to sensor data, robot state, and commands
- **Publishers**: Send robot commands, status updates, and AI brain state

### Service Calls

For synchronous operations, the AI brain can use services:

```python
class AIBrainWithServices(AIBrainNode):
    """AI Brain node that also uses services for synchronous operations."""

    def __init__(self):
        super().__init__()

        # Create service clients for synchronous operations
        self.planning_service_client = self.create_client(
            srv_type=GetPlan,  # Custom service type
            srv_name='/planning_server/get_plan'
        )

        self.kinematics_service_client = self.create_client(
            srv_type=GetPositionIK,  # Inverse kinematics service
            srv_name='/kinematics_service/get_ik'
        )

    def request_plan_synchronously(self, start_pose, goal_pose):
        """Request a plan synchronously using services."""
        if not self.planning_service_client.service_is_ready():
            self.get_logger().warn('Planning service not ready')
            return None

        request = GetPlan.Request()
        request.start = start_pose
        request.goal = goal_pose

        future = self.planning_service_client.call_async(request)

        # Wait for response with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            try:
                response = future.result()
                return response.plan
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return None
        else:
            self.get_logger().warn('Service call timed out')
            return None
```

### Action Interface Pattern

For long-running operations with feedback, the AI brain uses actions:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class AIBrainWithActions(AIBrainNode):
    """AI Brain node that uses actions for long-running operations."""

    def __init__(self):
        super().__init__()

        # Create action clients for long-running operations
        self.navigation_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_navigation_goal(self, goal_pose):
        """Send navigation goal using action interface."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Wait for action server
        if not self.navigation_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal asynchronously
        self.navigation_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback,
            goal_response_callback=self.navigation_goal_response_callback,
            result_callback=self.navigation_result_callback
        )

        return True

    def navigation_feedback_callback(self, feedback):
        """Handle navigation feedback."""
        self.get_logger().info(f'Navigation progress: {feedback.feedback.distance_remaining:.2f}m remaining')

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.get_logger().info(f'Navigation completed: {result}')
```

## Summary

This chapter has explored how the AI brain bridges to ROS 2, connecting high-level intelligence to physical robot control. We've examined:

- **Python AI agents** as ROS 2 nodes that integrate intelligence with communication
- **Decision-to-action transformation** that safely converts AI decisions to robot commands
- **Closed-loop feedback systems** that enable adaptive, intelligent behavior
- **Integration patterns** that connect AI intelligence with ROS 2 communication

The bridge between AI intelligence and ROS 2 control is critical for creating autonomous robots that can operate intelligently in complex environments. This integration enables the perception-planning-decision-action cycle that characterizes intelligent robot behavior.

The AI brain, perception systems, planning algorithms, and ROS 2 integration work together to create robots that can understand their environment, make intelligent decisions, and execute those decisions safely and effectively.

This completes Module 3: AI-Robot Brain (NVIDIA Isaac™), providing a foundation for understanding how artificial intelligence integrates with robotic control systems to create intelligent, autonomous humanoid robots.