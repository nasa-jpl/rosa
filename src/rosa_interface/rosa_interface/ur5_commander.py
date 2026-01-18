#!/usr/bin/env python3
#  Copyright (c) 2024 Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
"""
UR5 Commander Node - ROSA interface for UR5 robot arm task control.

This node provides a natural language interface to control pick and place tasks
for a UR5e robot arm with Robotiq Hand-E gripper using ROSA (Robot Operating System Agent).

Note: This is an example package demonstrating how to integrate ROSA with a ROS 2 system.
The service dependencies (motion_planner) are specific to this example and should be
adapted to your own robot system when using this as a template.
"""

import sys
import rclpy
from rclpy.node import Node

try:
    from rosa.rosa import ROSA
    from rosa.prompts import RobotSystemPrompts
    from langchain.agents import tool
    ROSA_AVAILABLE = True
except ImportError:
    ROSA_AVAILABLE = False
    print("WARNING: ROSA is not available for natural language queries")


def get_llm():
    """Configure and return LLM for ROSA."""
    try:
        from langchain_openai import ChatOpenAI
        import os
        if not os.getenv("OPENAI_API_KEY"):
            print("WARNING: OPENAI_API_KEY not set, ROSA queries disabled")
            return None
        return ChatOpenAI(model="gpt-4o-mini", temperature=0)
    except ImportError:
        print("WARNING: langchain-openai not installed, ROSA queries disabled")
        return None


class UR5Commander(Node):
    """ROS 2 Node for commanding UR5 robot arm."""

    def __init__(self):
        super().__init__('ur5_commander')
        self.get_logger().info('UR5 Commander starting...')
        
        # Import the custom service type
        from motion_planner.srv import ExecuteTask
        
        # Create service clients for MTC tasks
        self.execute_task_client = self.create_client(ExecuteTask, 'execute_task')
        self.execute_return_task_client = self.create_client(ExecuteTask, 'execute_return_task')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for task services...')
        if self.execute_task_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('execute_task service is available')
        else:
            self.get_logger().warn('execute_task service not available yet')
            
        if self.execute_return_task_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('execute_return_task service is available')
        else:
            self.get_logger().warn('execute_return_task service not available yet')
        
        # Initialize ROSA if available
        self.rosa_agent = None
        if ROSA_AVAILABLE:
            llm = get_llm()
            if llm:
                try:
                    self.get_logger().info('Initializing ROSA with custom tools...')
                    
                    # Create tools for ROSA
                    tools = [
                        self.create_execute_task_tool(),
                        self.create_execute_return_task_tool()
                    ]
                    
                    # Create custom prompts
                    prompts = RobotSystemPrompts(
                        embodiment_and_persona=(
                            "You are an intelligent robot assistant controlling a UR5e robot arm with a Robotiq Hand-E gripper. "
                            "You can execute pick and place tasks for cylinders numbered 1-7. "
                            "When asked to pick or move a cylinder, use the execute_task tool with the appropriate cylinder_id. "
                            "When asked to return a cylinder, use the execute_return_task tool."
                        )
                    )
                    
                    self.rosa_agent = ROSA(ros_version=2, llm=llm, tools=tools, prompts=prompts)
                    self.get_logger().info('ROSA initialized successfully with custom tools!')
                except Exception as e:
                    self.get_logger().warn(f'Failed to initialize ROSA: {e}')
        
        self.get_logger().info('UR5 Commander ready!')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  - ask <question>')
        self.get_logger().info('  - help')
        self.get_logger().info('  - quit')

    def create_execute_task_tool(self):
        """Create a tool for executing pick and place task."""
        from langchain.agents import tool
        
        # Capture self in closure
        node = self
        
        @tool
        def execute_task(cylinder_id: int) -> str:
            """
            Execute a pick and place task for a specific cylinder.
            This will pick up the cylinder from its current position and place it in the container.
            
            :param cylinder_id: The ID of the cylinder to pick (1-7)
            :return: Status message indicating success or failure
            """
            from motion_planner.srv import ExecuteTask
            
            if cylinder_id < 1 or cylinder_id > 7:
                return f"Error: Invalid cylinder_id {cylinder_id}. Must be between 1 and 7."
            
            node.get_logger().info(f'Calling execute_task service for cylinder {cylinder_id}...')
            
            request = ExecuteTask.Request()
            request.cylinder_id = cylinder_id
            
            future = node.execute_task_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    node.get_logger().info(f'Task completed: {response.message}')
                    return f"Success! {response.message}"
                else:
                    node.get_logger().error(f'Task failed: {response.message}')
                    return f"Failed: {response.message}"
            else:
                node.get_logger().error('Service call failed')
                return "Error: Service call failed or timed out"
        
        return execute_task
    
    def create_execute_return_task_tool(self):
        """Create a tool for executing return task."""
        from langchain.agents import tool
        
        # Capture self in closure
        node = self
        
        @tool
        def execute_return_task(cylinder_id: int) -> str:
            """
            Execute a return task to bring a cylinder back from the container to its original position.
            
            :param cylinder_id: The ID of the cylinder to return (1-7)
            :return: Status message indicating success or failure
            """
            from motion_planner.srv import ExecuteTask
            
            if cylinder_id < 1 or cylinder_id > 7:
                return f"Error: Invalid cylinder_id {cylinder_id}. Must be between 1 and 7."
            
            node.get_logger().info(f'Calling execute_return_task service for cylinder {cylinder_id}...')
            
            request = ExecuteTask.Request()
            request.cylinder_id = cylinder_id
            
            future = node.execute_return_task_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    node.get_logger().info(f'Return task completed: {response.message}')
                    return f"Success! {response.message}"
                else:
                    node.get_logger().error(f'Return task failed: {response.message}')
                    return f"Failed: {response.message}"
            else:
                node.get_logger().error('Service call failed')
                return "Error: Service call failed or timed out"
        
        return execute_return_task

    def ask_rosa(self, question):
        """Ask ROSA a question about the robot system."""
        if not self.rosa_agent:
            self.get_logger().warn('ROSA is not available')
            return
        
        self.get_logger().info(f'Asking ROSA: {question}')
        try:
            response = self.rosa_agent.invoke(question)
            self.get_logger().info(f'ROSA Response:\n{response}')
        except Exception as e:
            self.get_logger().error(f'Error querying ROSA: {e}')

    def show_status(self):
        """Show current robot status using ROSA."""
        if self.rosa_agent:
            self.ask_rosa("What is the current state of the UR5 robot? Show joint positions and end-effector pose.")
        else:
            self.get_logger().info('ROSA not available. Use: ros2 topic echo /joint_states')

    def print_help(self):
        """Print help message."""
        help_text = """
╔══════════════════════════════════════════════════════════════════════╗
║                        UR5 Commander Help                            ║
╠══════════════════════════════════════════════════════════════════════╣
║ ROSA QUERIES (Natural Language):                                     ║
║   ask <question>                                                      ║
║       Ask ROSA about the robot system and control tasks               ║
║                                                                       ║
║   Example queries:                                                    ║
║     - ask pick cylinder 2                                             ║
║     - ask move cylinder 3 to the container                            ║
║     - ask return cylinder 1                                           ║
║     - ask bring back cylinder 2 from the container                    ║
║     - ask execute task for cylinder 5                                 ║
║     - ask what cylinders can you pick?                                ║
║                                                                       ║
║ OTHER:                                                                ║
║   help  - Show this help message                                      ║
║   quit  - Exit commander                                              ║
╚══════════════════════════════════════════════════════════════════════╝
"""
        print(help_text)

    def run_interactive(self):
        """Run interactive command loop."""
        self.print_help()
        
        while rclpy.ok():
            try:
                # Get user input
                command = input("\n🤖 UR5> ").strip()
                
                if not command:
                    continue
                
                parts = command.split()
                cmd = parts[0].lower()
                
                if cmd == 'quit' or cmd == 'exit':
                    self.get_logger().info('Shutting down...')
                    break
                
                elif cmd == 'help':
                    self.print_help()
                
                elif cmd == 'status':
                    self.show_status()
                
                elif cmd == 'ask':
                    if len(parts) < 2:
                        self.get_logger().error('Usage: ask <question>')
                        continue
                    question = ' '.join(parts[1:])
                    self.ask_rosa(question)
                
                else:
                    self.get_logger().warn(f'Unknown command: {cmd}. Type "help" for available commands.')
            
            except KeyboardInterrupt:
                self.get_logger().info('\nShutting down...')
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    commander = UR5Commander()
    
    # Run interactive mode
    try:
        commander.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
