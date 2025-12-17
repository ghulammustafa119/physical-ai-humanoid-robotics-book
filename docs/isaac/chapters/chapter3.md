---
title: "Chapter 3: Planning & Decision Making"
description: "Task planning, motion planning, and decision-making for humanoid robots"
sidebar_position: 11
---

# Chapter 3: Planning & Decision Making

## Introduction

Planning and decision-making represent the cognitive layer of the AI-robot brain. While perception systems understand the current state of the world, planning and decision-making systems determine what actions to take to achieve goals. This chapter explores how robots plan their behavior, make decisions under uncertainty, and handle the complexities of real-world environments.

The planning and decision-making layer bridges high-level goals with low-level control actions. It determines how to achieve objectives while respecting physical constraints, safety requirements, and environmental conditions. This layer must balance the need for intelligent, adaptive behavior with the requirements for reliable, safe operation.

## Types of Planning

Robots must plan at multiple levels, from high-level task sequences to low-level motion trajectories. Different types of planning address different aspects of robot behavior.

### Motion Planning

Motion planning focuses on how to move the robot's body to achieve specific goals while avoiding obstacles and respecting physical constraints:

- **Path planning**: Computing collision-free paths through the environment
- **Trajectory planning**: Creating time-parameterized paths with velocity and acceleration profiles
- **Manipulation planning**: Planning arm movements for grasping and manipulation tasks
- **Whole-body planning**: Coordinating multiple parts of the robot simultaneously

Motion planning algorithms must consider:
- **Collision avoidance**: Ensuring the robot doesn't collide with obstacles
- **Kinematic constraints**: Respecting joint limits and reachability
- **Dynamic constraints**: Considering balance and stability requirements
- **Optimization criteria**: Minimizing time, energy, or other objectives

```python
class MotionPlanner:
    """Plan robot motions for navigation and manipulation."""

    def __init__(self):
        self.environment_map = None
        self.robot_model = None
        self.planning_algorithms = {
            'path': 'RRT*',  # Rapidly-exploring Random Trees
            'trajectory': 'polynomial',  # Polynomial trajectory generation
            'optimization': 'gradient_descent'
        }

    def plan_navigation_path(self, start_pose, goal_pose):
        """Plan collision-free path from start to goal."""
        # Check if goal is reachable
        if not self.is_reachable(goal_pose):
            raise ValueError("Goal is not reachable")

        # Plan path using selected algorithm
        path = self.compute_path(start_pose, goal_pose)

        # Optimize path for smoothness and efficiency
        optimized_path = self.optimize_path(path)

        return optimized_path

    def plan_manipulation_trajectory(self, start_config, goal_config):
        """Plan joint-space trajectory for manipulation."""
        # Check joint limits
        if not self.valid_configuration(goal_config):
            raise ValueError("Goal configuration is invalid")

        # Plan trajectory in joint space
        trajectory = self.compute_joint_trajectory(start_config, goal_config)

        # Verify trajectory is collision-free
        collision_free = self.verify_trajectory_collisions(trajectory)

        if not collision_free:
            raise ValueError("Trajectory has collisions")

        return trajectory

    def compute_path(self, start_pose, goal_pose):
        """Compute path using motion planning algorithm."""
        # This would implement RRT*, A*, or similar algorithm
        # Return sequence of poses from start to goal
        pass

    def optimize_path(self, path):
        """Optimize path for smoothness and efficiency."""
        # Smooth path to reduce sharp turns
        # Shorten path while maintaining safety
        pass

    def compute_joint_trajectory(self, start_config, goal_config):
        """Compute joint-space trajectory."""
        # Generate smooth trajectory in joint space
        # Consider velocity and acceleration limits
        pass

    def verify_trajectory_collisions(self, trajectory):
        """Verify trajectory is collision-free."""
        # Check each point in trajectory against environment
        pass

    def is_reachable(self, pose):
        """Check if pose is reachable by robot."""
        # Use inverse kinematics to check reachability
        pass

    def valid_configuration(self, config):
        """Check if joint configuration is valid."""
        # Check joint limits, self-collisions, etc.
        pass
```

### Task Planning

Task planning focuses on what sequence of actions to perform to achieve high-level goals:

- **Goal decomposition**: Breaking complex goals into simpler subtasks
- **Action sequencing**: Determining the order of actions to achieve goals
- **Resource allocation**: Managing computational and physical resources
- **Contingency planning**: Preparing alternative plans for different scenarios

Task planning must consider:
- **Logical dependencies**: Some actions must precede others
- **Resource constraints**: Limited computational and physical resources
- **Temporal constraints**: Deadlines and timing requirements
- **Uncertainty handling**: Dealing with unknown or stochastic outcomes

```python
class TaskPlanner:
    """Plan high-level tasks and sequences of actions."""

    def __init__(self):
        self.knowledge_base = {}  # World knowledge and facts
        self.goal_library = {}    # Predefined goals and methods
        self.action_library = {}  # Available actions and effects

    def plan_task_sequence(self, high_level_goal):
        """Plan sequence of actions to achieve high-level goal."""
        # Decompose goal into subgoals
        subgoals = self.decompose_goal(high_level_goal)

        # Sequence actions for each subgoal
        action_sequence = self.sequence_actions(subgoals)

        # Optimize sequence for efficiency
        optimized_sequence = self.optimize_sequence(action_sequence)

        return optimized_sequence

    def decompose_goal(self, goal):
        """Decompose high-level goal into subgoals."""
        # Use predefined methods or learning-based decomposition
        # Return sequence of subgoals
        pass

    def sequence_actions(self, subgoals):
        """Sequence actions to achieve subgoals."""
        # Determine order of actions considering dependencies
        # Ensure logical and temporal constraints are satisfied
        pass

    def optimize_sequence(self, sequence):
        """Optimize action sequence for efficiency."""
        # Minimize resource usage, time, or other criteria
        # Consider parallelizable actions
        pass

    def handle_uncertainty(self, plan):
        """Add contingency plans for uncertainty."""
        # Identify potential failure points
        # Add monitoring and recovery actions
        # Create alternative execution paths
        pass

    def update_knowledge_base(self, new_information):
        """Update knowledge base with new information."""
        # Incorporate new facts about world state
        # Update beliefs about object locations, etc.
        pass
```

## Reactive vs Deliberative Behaviors

Robots must balance reactive responses to immediate stimuli with deliberate planning for long-term goals. Different types of behaviors serve different purposes in robot operation.

### Reactive Behaviors

Reactive behaviors provide immediate responses to environmental stimuli without complex reasoning:

- **Characteristics**: Fast, predictable, reliable
- **Applications**: Obstacle avoidance, reflexive responses, emergency reactions
- **Advantages**: Immediate response, guaranteed performance, simple implementation
- **Disadvantages**: Limited flexibility, cannot handle complex situations

Reactive behaviors are essential for safety and basic navigation, providing reliable responses to common situations.

```python
class ReactiveController:
    """Handle reactive behaviors for immediate responses."""

    def __init__(self):
        self.safety_thresholds = {
            'proximity': 0.5,  # meters
            'velocity': 1.0,   # m/s
            'acceleration': 2.0  # m/s²
        }

    def handle_proximity_avoidance(self, sensor_data):
        """Reactive obstacle avoidance."""
        closest_obstacle = self.find_closest_obstacle(sensor_data)

        if closest_obstacle < self.safety_thresholds['proximity']:
            # Immediate evasive action
            evasive_command = self.generate_evasive_action(closest_obstacle)
            return evasive_command

        return None  # No action needed

    def handle_balance_recovery(self, imu_data):
        """Reactive balance recovery."""
        tilt_angle = self.calculate_tilt_angle(imu_data)

        if abs(tilt_angle) > self.safety_thresholds['tilt']:
            # Immediate balance recovery action
            recovery_command = self.generate_balance_recovery(tilt_angle)
            return recovery_command

        return None  # Within safe limits

    def find_closest_obstacle(self, sensor_data):
        """Find closest obstacle from sensor data."""
        # Process sensor data to find nearest obstacle
        pass

    def generate_evasive_action(self, obstacle_info):
        """Generate evasive action based on obstacle."""
        # Return immediate command to avoid obstacle
        pass

    def calculate_tilt_angle(self, imu_data):
        """Calculate robot tilt angle from IMU."""
        # Use IMU data to determine tilt
        pass

    def generate_balance_recovery(self, tilt_angle):
        """Generate balance recovery action."""
        # Return command to restore balance
        pass
```

### Deliberative Behaviors

Deliberative behaviors involve complex reasoning and planning for long-term goals:

- **Characteristics**: Computationally intensive, flexible, goal-oriented
- **Applications**: Complex task execution, strategic planning, problem solving
- **Advantages**: Handles complex situations, achieves long-term goals, adaptable
- **Disadvantages**: Computationally expensive, slower response, may fail in emergencies

Deliberative behaviors enable robots to perform complex tasks that require reasoning and planning.

```python
class DeliberativePlanner:
    """Handle deliberative planning and reasoning."""

    def __init__(self):
        self.reasoning_engine = None
        self.long_term_memory = {}
        self.goal_hierarchy = []

    def execute_complex_task(self, complex_goal):
        """Execute complex task requiring deliberative planning."""
        # Analyze goal and decompose into subproblems
        plan = self.analyze_and_plan(complex_goal)

        # Execute plan with monitoring and adaptation
        execution_result = self.execute_with_monitoring(plan)

        return execution_result

    def analyze_and_plan(self, goal):
        """Analyze goal and create detailed plan."""
        # Use knowledge base to understand goal requirements
        # Consider multiple possible approaches
        # Select optimal strategy based on criteria
        pass

    def execute_with_monitoring(self, plan):
        """Execute plan with monitoring and adaptation."""
        # Execute plan step by step
        # Monitor progress and outcomes
        # Adapt plan as needed based on feedback
        pass

    def reason_about_context(self, situation):
        """Reason about current situation and context."""
        # Use knowledge base to understand situation
        # Consider relevant facts and constraints
        # Generate appropriate responses
        pass

    def learn_from_execution(self, results):
        """Learn from plan execution results."""
        # Update knowledge base with lessons learned
        # Improve future planning based on experience
        pass
```

## Learned Policies vs Rule-Based Logic

Modern AI-robot brains combine learned policies with rule-based logic to achieve both adaptability and reliability.

### Learned Policies

Learned policies use machine learning to determine appropriate actions:

- **Advantages**: Adaptive to complex situations, can learn from experience, handles uncertainty well
- **Applications**: Motor skill learning, adaptive behavior, complex control tasks
- **Characteristics**: Data-driven, improves with experience, may be interpretable or opaque

```python
class LearnedPolicy:
    """A learned policy for adaptive behavior."""

    def __init__(self, policy_model):
        self.model = policy_model
        self.state_representation = None
        self.action_space = None

    def select_action(self, current_state):
        """Select action based on learned policy."""
        # Convert state to policy input format
        policy_input = self.format_state_input(current_state)

        # Get action from policy model
        action = self.model.predict(policy_input)

        # Convert action to robot command
        robot_command = self.format_action_output(action)

        return robot_command

    def format_state_input(self, state):
        """Format state for policy model input."""
        # Extract relevant features from state
        # Normalize or scale as needed
        pass

    def format_action_output(self, action):
        """Format policy action for robot execution."""
        # Convert policy action to robot command
        # Apply safety limits and constraints
        pass

    def update_policy(self, experience_data):
        """Update policy based on new experience."""
        # Train model on new experience data
        # Validate policy performance
        pass
```

### Rule-Based Logic

Rule-based systems use explicit programming to determine actions:

- **Advantages**: Predictable, interpretable, reliable, safety-guaranteed
- **Applications**: Safety systems, critical operations, predictable scenarios
- **Characteristics**: Explicit rules, deterministic behavior, human-readable logic

```python
class RuleBasedSystem:
    """A rule-based system for predictable behavior."""

    def __init__(self):
        self.rule_base = [
            {'condition': 'obstacle_close', 'action': 'stop_and_avoid'},
            {'condition': 'battery_low', 'action': 'return_to_charger'},
            {'condition': 'goal_reached', 'action': 'report_success'},
            {'condition': 'emergency', 'action': 'activate_safety_protocol'}
        ]

    def select_action(self, current_state):
        """Select action based on rule evaluation."""
        for rule in self.rule_base:
            if self.evaluate_condition(rule['condition'], current_state):
                return self.execute_action(rule['action'], current_state)

        # Default action if no rules match
        return self.default_action(current_state)

    def evaluate_condition(self, condition, state):
        """Evaluate rule condition against current state."""
        # Check if condition is met based on state
        pass

    def execute_action(self, action, state):
        """Execute rule-based action."""
        # Perform action specified by rule
        pass

    def default_action(self, state):
        """Return default action when no rules match."""
        # Safe default behavior
        pass
```

### Hybrid Approaches

Modern systems often combine both approaches:

- **Architecture**: Rule-based safety systems with learned policies for flexibility
- **Benefits**: Safety guarantees with adaptive capabilities
- **Implementation**: Learned policies operate within rule-based safety constraints

```python
class HybridController:
    """Combine learned policies with rule-based safety."""

    def __init__(self):
        self.learned_policy = None  # Learned policy model
        self.rule_based_system = RuleBasedSystem()  # Safety rules
        self.arbitration_logic = None  # Decision logic

    def select_action(self, current_state):
        """Select action using hybrid approach."""
        # Get candidate action from learned policy
        policy_action = self.learned_policy.select_action(current_state)

        # Get safety action from rule-based system
        safety_action = self.rule_based_system.select_action(current_state)

        # Arbitrate between policy and safety actions
        final_action = self.arbitrate_actions(policy_action, safety_action, current_state)

        return final_action

    def arbitrate_actions(self, policy_action, safety_action, state):
        """Arbitrate between policy and safety actions."""
        # Prioritize safety actions over policy actions
        # Allow policy actions when safe
        pass
```

## Failure Handling and Replanning

Robots must handle failures gracefully and adapt their plans when situations change unexpectedly.

### Failure Detection

Robots need to detect when plans are failing or when unexpected situations arise:

- **Performance monitoring**: Track plan execution progress
- **Anomaly detection**: Identify deviations from expected behavior
- **Sensor validation**: Verify sensor data reliability
- **Goal assessment**: Determine if goals remain achievable

### Recovery Strategies

When failures occur, robots need appropriate recovery strategies:

- **Fallback behaviors**: Predefined safe responses to common failures
- **Replanning**: Generate new plans when current plans fail
- **Goal revision**: Modify goals when original goals become unachievable
- **Human intervention**: Request help when autonomous recovery fails

```python
class FailureHandler:
    """Handle failures and enable recovery."""

    def __init__(self):
        self.recovery_behaviors = {
            'navigation_failure': 'return_to_known_location',
            'manipulation_failure': 'release_and_retry',
            'communication_loss': 'wait_and_retry',
            'safety_violation': 'emergency_stop'
        }
        self.recovery_history = []

    def detect_failure(self, execution_state):
        """Detect when plan execution is failing."""
        # Monitor execution progress
        # Check for anomalies or deviations
        # Assess goal achievability
        pass

    def initiate_recovery(self, failure_type, context):
        """Initiate appropriate recovery behavior."""
        if failure_type in self.recovery_behaviors:
            recovery_action = self.recovery_behaviors[failure_type]
            return self.execute_recovery(recovery_action, context)

        # Default recovery if specific behavior not found
        return self.default_recovery(context)

    def execute_recovery(self, recovery_action, context):
        """Execute specific recovery action."""
        # Perform recovery action based on type
        # Monitor recovery progress
        # Update recovery history
        pass

    def replan_after_failure(self, original_goal, failure_context):
        """Generate new plan after failure."""
        # Analyze failure and context
        # Modify original goal if necessary
        # Generate new plan considering failure causes
        pass

    def default_recovery(self, context):
        """Default recovery behavior."""
        # Safe default action when specific recovery not known
        pass

    def learn_from_failures(self):
        """Learn from failure and recovery experiences."""
        # Update knowledge base with failure patterns
        # Improve future failure handling
        pass
```

## Planning Hierarchies

Robots operate at multiple planning levels simultaneously:

### Strategic Planning
- **Time horizon**: Minutes to hours
- **Scope**: Mission-level objectives
- **Frequency**: Occasional updates
- **Examples**: Daily schedules, long-term goals

### Tactical Planning
- **Time horizon**: Seconds to minutes
- **Scope**: Task-level activities
- **Frequency**: Periodic updates
- **Examples**: Task sequences, resource allocation

### Operational Planning
- **Time horizon**: Milliseconds to seconds
- **Scope**: Motion-level actions
- **Frequency**: Continuous updates
- **Examples**: Path following, obstacle avoidance

## Decision-Making Frameworks

### Utility-Based Decision Making

Robots can make decisions based on utility functions that quantify the desirability of different outcomes:

- **Utility calculation**: Assign values to different possible outcomes
- **Uncertainty handling**: Account for probabilistic outcomes
- **Multi-objective optimization**: Balance competing goals

### Behavior-Based Systems

Behavior-based systems use collections of simple behaviors that compete for control:

- **Behavior arbitration**: Determine which behavior should be active
- **Subsumption architecture**: Higher-level behaviors can inhibit lower-level ones
- **Emergent behavior**: Complex behavior emerges from simple interactions

## Real-World Humanoid Examples

### Navigation Tasks
- **Goal**: Reach destination while avoiding obstacles
- **Planning**: Path planning with dynamic obstacle avoidance
- **Decision-making**: Balance efficiency vs safety

### Manipulation Tasks
- **Goal**: Grasp and manipulate objects
- **Planning**: Reach planning, grasp planning, trajectory optimization
- **Decision-making**: Choose grasp type, handle failures

### Social Interaction
- **Goal**: Engage in natural human-robot interaction
- **Planning**: Dialogue management, gesture planning
- **Decision-making**: Choose appropriate responses

## Summary

This chapter explored planning and decision-making systems that determine robot behavior. We examined the differences between motion and task planning, reactive and deliberative behaviors, learned policies and rule-based logic, and failure handling strategies.

Planning and decision-making systems bridge the gap between high-level goals and low-level control, enabling robots to operate intelligently in complex environments. Modern systems often combine multiple approaches to achieve both adaptability and reliability.

The key insight is that different types of planning and decision-making are needed for different aspects of robot behavior, and these systems must work together to create coherent, intelligent robot operation.

In the next chapter, we'll explore how these planning and decision-making systems connect to ROS 2 for real robot control.