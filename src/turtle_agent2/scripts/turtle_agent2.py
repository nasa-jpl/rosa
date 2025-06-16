#!/usr/bin/env python3
#  Copyright (c) 2025. Jet Propulsion Laboratory. All rights reserved.
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

import asyncio
import os
from datetime import datetime, timezone

import dotenv
import rclpy
from help import get_help
from langchain.agents import Tool, tool
from llm import get_llm
from prompts import get_prompts
from rclpy.node import Node
from rich.console import Console, Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from tools import turtle as turtle_tools

from rosa import ROSA


# Typical method for defining tools in ROSA
@tool
def cool_turtle_tool():
    """A cool turtle tool that doesn't really do anything."""
    return "This is a cool turtle tool! It doesn't do anything, but it's cool."


class TurtleAgent2(ROSA, Node):
    def __init__(self, streaming: bool = True, verbose: bool = True):
        # Initialize ROS2 node first
        Node.__init__(self, "rosa_turtle_agent2")

        # Declare parameters with defaults from constructor arguments
        self.declare_parameter("streaming", streaming)
        self.declare_parameter("verbose", verbose)

        # Use command line args passed to constructor (they already override ROS2 params)
        actual_streaming = True  # Force streaming mode
        actual_verbose = verbose

        print(
            f"ROSA TurtleAgent2 starting with streaming={actual_streaming}, verbose={actual_verbose}"
        )

        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=actual_streaming)

        # Another method for adding tools
        blast_off = Tool(
            name="blast_off",
            func=self.blast_off,
            description="Make the turtle blast off!",
        )

        ROSA.__init__(
            self,
            ros_version=2,  # ONLY CHANGE: 1 -> 2 for ROS2
            llm=self.__llm,
            tools=[cool_turtle_tool, blast_off],
            tool_packages=[turtle_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=actual_verbose,
            accumulate_chat_history=True,
            streaming=actual_streaming,
        )

        print(f"ROSA initialized. Internal streaming flag: {self._ROSA__streaming}")
        print(
            "ROSA initialized. Internal verbose from executor: verbose setting applied"
        )

        self.examples = [
            "Give me a ROS tutorial using the turtlesim.",
            "Show me how to move the turtle forward.",
            "Draw a 5-point star using the turtle.",
            "Teleport to (3, 3) and draw a small hexagon.",
            "Give me a list of nodes, topics, services, params, and log files.",
            "Change the background color to light blue and the pen color to red.",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
        }

    def blast_off(self, _user_input: str):
        return """
        Ok, we're blasting off at the speed of light!

        <ROSA_INSTRUCTIONS>
            You should now use your tools to make the turtle move around the screen at high speeds.
        </ROSA_INSTRUCTIONS>
        """

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-TurtleSim agent 🐢🤖. How can I help you today?\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit.",
            style="italic",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        print("\nChoose an example:")
        for i, example in enumerate(self.examples, 1):
            print(f"{i}. {example}")
        try:
            choice = input(f"\nEnter your choice (1-{len(self.examples)}): ") or "1"
            choice_num = int(choice)
            if 1 <= choice_num <= len(self.examples):
                return self.examples[choice_num - 1]
            else:
                return self.examples[0]
        except (ValueError, EOFError):
            return self.examples[0]

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        self.last_events = []
        self.command_handler.pop("info", None)
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        try:
            return input(prompt) or "help"
        except EOFError:
            # Handle non-interactive environments
            print("Non-interactive environment detected. Using default command 'help'")
            return "help"
        except Exception as e:
            print(f"Input error: {e}. Using default command 'help'")
            return "help"

    async def run(self):
        """
        Run the TurtleAgent2's main interaction loop.

        This method initializes the console interface and enters a continuous loop to handle user input.
        It processes various commands including 'help', 'examples', 'clear', and 'exit', as well as
        custom user queries. The method uses asynchronous operations to stream responses and maintain
        a responsive interface.

        The loop continues until the user inputs 'exit'.

        Returns:
            None

        Raises:
            Any exceptions that might occur during the execution of user commands or streaming responses.
        """
        await self.clear()
        console = Console()

        while True:
            console.print(self.greeting)
            user_input = self.get_input("> ")

            # Handle special commands
            if user_input == "exit":
                print("Goodbye! 🐢")
                break
            elif user_input in self.command_handler:
                await self.command_handler[user_input]()
            else:
                await self.submit(user_input)

    async def submit(self, query: str):
        try:
            # Always use streaming mode since we forced it to True
            await self.stream_response(query)
        except Exception:
            self.print_response(query)

    def print_response(self, query: str):
        """
        Submit the query to the agent and print the response to the console.

        Args:
            query (str): The input query to process.

        Returns:
            None
        """
        response = self.invoke(query)
        console = Console()
        content_panel = None

        with Live(
            console=console, auto_refresh=True, vertical_overflow="visible"
        ) as live:
            content_panel = Panel(
                Markdown(response), title="Final Response", border_style="green"
            )
            live.update(content_panel, refresh=True)

    async def stream_response(self, query: str):
        """
        Stream the agent's response with rich formatting.

        This method processes the agent's response in real-time, updating the console
        with formatted output for tokens and keeping track of events.

        Args:
            query (str): The input query to process.

        Returns:
            None

        Raises:
            Any exceptions raised during the streaming process.
        """
        console = Console()
        content = ""
        self.last_events = []

        panel = Panel("", title="Streaming Response", border_style="green")

        with Live(panel, console=console, auto_refresh=False) as live:
            try:
                async for event in self.astream(query):
                    event["timestamp"] = datetime.now(timezone.utc).strftime(
                        "%Y-%m-%d %H:%M:%S.%f"
                    )[:-3]
                    if event["type"] == "token":
                        content += event["content"]
                        panel.renderable = Markdown(content)
                        live.refresh()
                    elif event["type"] in ["tool_start", "tool_end", "error"]:
                        self.last_events.append(event)
                    elif event["type"] == "final":
                        content = event["content"]
                        panel.renderable = Markdown(content)
                        panel.title = "Final Response"
                        live.refresh()

            except Exception:
                # If streaming fails, fall back to non-streaming
                content = self.invoke(query)
                panel.renderable = Markdown(content)
                panel.title = "Final Response"
                live.refresh()

        # Always add info command after final response, regardless of events
        if self.last_events:
            panel.renderable = Markdown(
                content + "\n\nType 'info' for details on how I got my answer."
            )
            live.refresh()
            self.command_handler["info"] = self.show_event_details
        else:
            self.command_handler.pop("info", None)

    async def show_event_details(self):
        """
        Display detailed information about the events that occurred during the last query.
        """
        console = Console()

        if not self.last_events:
            console.print("[yellow]No events to display.[/yellow]")
            return
        else:
            console.print(Markdown("# Tool Usage and Events"))

        for event in self.last_events:
            timestamp = event["timestamp"]
            if event["type"] == "tool_start":
                console.print(
                    Panel(
                        Group(
                            Text(f"Input: {event.get('input', 'None')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"Tool Started: {event['name']}",
                        border_style="blue",
                    )
                )
            elif event["type"] == "tool_end":
                console.print(
                    Panel(
                        Group(
                            Text(f"Output: {event.get('output', 'N/A')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"Tool Completed: {event['name']}",
                        border_style="green",
                    )
                )
            elif event["type"] == "error":
                console.print(
                    Panel(
                        Group(
                            Text(f"Error: {event['content']}", style="bold red"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        border_style="red",
                    )
                )
            console.print()

        console.print("[bold]End of events[/bold]\n")


def main():
    dotenv.load_dotenv(dotenv.find_dotenv())
    turtle_agent = TurtleAgent2(
        verbose=True, streaming=True
    )  # Always use streaming and verbose

    try:
        asyncio.run(turtle_agent.run())
    except KeyboardInterrupt:
        print("\nShutting down ROSA TurtleAgent2...")
    finally:
        # Clean up the node
        try:
            turtle_agent.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")


if __name__ == "__main__":
    # Initialize ROS2 only if not already initialized
    if not rclpy.ok():
        rclpy.init()

    try:
        main()
    finally:
        # Shutdown ROS2
        if rclpy.ok():
            rclpy.shutdown()
