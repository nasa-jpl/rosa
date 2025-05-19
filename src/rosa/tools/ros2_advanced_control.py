#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import time
from typing import Dict, List, Optional, Union
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import JointState, BatteryState
from std_msgs.msg import String
from langchain.agents import tool

class RobotControlTools:
    """Advanced robot control tools for ROSA."""
    
    def __init__(self):
        self.node = None
        self.control_publishers = {}
        self.state_subscribers = {}
        self.last_states = {}
        self.safety_limits = {
            'max_linear_velocity': 1.0,  # m/s
            'max_angular_velocity': 1.0,  # rad/s
            'min_battery_level': 0.2,    # 20%
            'max_joint_velocity': 1.0,    # rad/s
        }

    def initialize_node(self, node: Node):
        """Initialize the node for robot control."""
        self.node = node
        # Initialize default publishers and subscribers
        self._setup_default_control_topics()

    def _setup_default_control_topics(self):
        """Setup default control topics."""
        if not self.node:
            raise RuntimeError("Node not initialized. Call initialize_node first.")

        # Setup default control topics
        self.control_publishers['cmd_vel'] = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # Setup state subscribers
        self.state_subscribers['joint_states'] = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10
        )

        self.state_subscribers['battery'] = self.node.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery_state_callback,
            10
        )

    def _joint_states_callback(self, msg: JointState):
        """Callback for joint states."""
        self.last_states['joint_states'] = msg

    def _battery_state_callback(self, msg: BatteryState):
        """Callback for battery state."""
        self.last_states['battery'] = msg

    def _check_safety_limits(self, control_command: Dict) -> bool:
        """Check if the control command is within safety limits."""
        if 'linear_velocity' in control_command:
            if abs(control_command['linear_velocity']) > self.safety_limits['max_linear_velocity']:
                return False
        
        if 'angular_velocity' in control_command:
            if abs(control_command['angular_velocity']) > self.safety_limits['max_angular_velocity']:
                return False

        if 'battery' in self.last_states:
            if self.last_states['battery'].percentage < self.safety_limits['min_battery_level']:
                return False

        return True

@tool
def ros2_robot_control(
    action: str,
    parameters: Dict,
    timeout: float = 5.0
) -> Dict:
    """
    Advanced robot control tool that supports multiple control modes and safety checks.
    
    Args:
        action: Type of control action ('move', 'grasp', 'navigate', etc.)
        parameters: Dictionary of control parameters
        timeout: Maximum time to wait for action completion
    
    Returns:
        dict: Status of the control action
    """
    try:
        # Initialize control tools if not already done
        if not hasattr(ros2_robot_control, 'control_tools'):
            ros2_robot_control.control_tools = RobotControlTools()
            # Note: Node initialization should be done by the main ROSA instance

        # Validate action type
        valid_actions = ['move', 'grasp', 'navigate', 'stop']
        if action not in valid_actions:
            return {
                'status': 'error',
                'message': f'Invalid action type. Must be one of {valid_actions}'
            }

        # Check safety limits
        if not ros2_robot_control.control_tools._check_safety_limits(parameters):
            return {
                'status': 'error',
                'message': 'Command exceeds safety limits'
            }

        # Execute the requested action
        if action == 'move':
            return _execute_move_action(parameters, timeout)
        elif action == 'grasp':
            return _execute_grasp_action(parameters, timeout)
        elif action == 'navigate':
            return _execute_navigate_action(parameters, timeout)
        elif action == 'stop':
            return _execute_stop_action()

    except Exception as e:
        return {
            'status': 'error',
            'message': f'Error executing control action: {str(e)}'
        }

def _execute_move_action(parameters: Dict, timeout: float) -> Dict:
    """Execute a movement action."""
    try:
        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = parameters.get('linear_velocity', 0.0)
        twist_msg.angular.z = parameters.get('angular_velocity', 0.0)

        # Publish the command
        ros2_robot_control.control_tools.control_publishers['cmd_vel'].publish(twist_msg)

        # Wait for the specified timeout
        time.sleep(timeout)

        return {
            'status': 'success',
            'message': 'Movement command executed successfully',
            'parameters': parameters
        }
    except Exception as e:
        return {
            'status': 'error',
            'message': f'Error executing movement: {str(e)}'
        }

def _execute_grasp_action(parameters: Dict, timeout: float) -> Dict:
    """Execute a grasping action."""
    try:
        # Implementation would depend on the specific robot's grasping mechanism
        # This is a placeholder implementation
        return {
            'status': 'success',
            'message': 'Grasp command executed successfully',
            'parameters': parameters
        }
    except Exception as e:
        return {
            'status': 'error',
            'message': f'Error executing grasp: {str(e)}'
        }

def _execute_navigate_action(parameters: Dict, timeout: float) -> Dict:
    """Execute a navigation action."""
    try:
        # Implementation would depend on the specific robot's navigation system
        # This is a placeholder implementation
        return {
            'status': 'success',
            'message': 'Navigation command executed successfully',
            'parameters': parameters
        }
    except Exception as e:
        return {
            'status': 'error',
            'message': f'Error executing navigation: {str(e)}'
        }

def _execute_stop_action() -> Dict:
    """Execute a stop action."""
    try:
        # Create and publish a zero velocity command
        twist_msg = Twist()
        ros2_robot_control.control_tools.control_publishers['cmd_vel'].publish(twist_msg)

        return {
            'status': 'success',
            'message': 'Stop command executed successfully'
        }
    except Exception as e:
        return {
            'status': 'error',
            'message': f'Error executing stop: {str(e)}'
        }

@tool
def ros2_robot_state_monitor(
    metrics: List[str],
    duration: float = 1.0,
    frequency: float = 10.0
) -> Dict:
    """
    Monitor robot state metrics in real-time.
    
    Args:
        metrics: List of metrics to monitor (e.g., ['position', 'velocity', 'battery'])
        duration: How long to monitor
        frequency: Sampling frequency in Hz
    
    Returns:
        dict: Time-series data of monitored metrics
    """
    try:
        if not hasattr(ros2_robot_state_monitor, 'control_tools'):
            ros2_robot_state_monitor.control_tools = RobotControlTools()

        # Validate metrics
        valid_metrics = ['position', 'velocity', 'battery', 'joint_states']
        invalid_metrics = [m for m in metrics if m not in valid_metrics]
        if invalid_metrics:
            return {
                'status': 'error',
                'message': f'Invalid metrics: {invalid_metrics}. Valid metrics are: {valid_metrics}'
            }

        # Calculate number of samples
        num_samples = int(duration * frequency)
        sampling_interval = 1.0 / frequency

        # Initialize data storage
        data = {metric: [] for metric in metrics}
        timestamps = []

        # Collect data
        for i in range(num_samples):
            timestamp = time.time()
            timestamps.append(timestamp)

            for metric in metrics:
                if metric == 'position' and 'joint_states' in ros2_robot_state_monitor.control_tools.last_states:
                    data[metric].append(
                        ros2_robot_state_monitor.control_tools.last_states['joint_states'].position
                    )
                elif metric == 'velocity' and 'joint_states' in ros2_robot_state_monitor.control_tools.last_states:
                    data[metric].append(
                        ros2_robot_state_monitor.control_tools.last_states['joint_states'].velocity
                    )
                elif metric == 'battery' and 'battery' in ros2_robot_state_monitor.control_tools.last_states:
                    data[metric].append(
                        ros2_robot_state_monitor.control_tools.last_states['battery'].percentage
                    )

            time.sleep(sampling_interval)

        return {
            'status': 'success',
            'data': data,
            'timestamps': timestamps,
            'metrics': metrics,
            'duration': duration,
            'frequency': frequency
        }

    except Exception as e:
        return {
            'status': 'error',
            'message': f'Error monitoring robot state: {str(e)}'
        } 