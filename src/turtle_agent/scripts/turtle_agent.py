#!/usr/bin/env python3.9
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

import asyncio
import os

import dotenv
import pyinputplus as pyip
import rospy
import tools.turtle as turtle_tools
from help import get_help
from langchain.agents import tool
from llm import get_llm
from prompts import get_prompts
from rich.console import Group  # Add this import
from rich.console import Console
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.prompt import Prompt
from rich.text import Text

from rosa import ROSA


@tool
def cool_turtle_tool():
    """A cool turtle tool."""
    return "This is a cool turtle tool! It doesn't do anything, but it's cool."


class TurtleAgent(ROSA):

    def __init__(self, streaming: bool = False, verbose: bool = True):
        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=streaming)
        self.__streaming = streaming

        super().__init__(
            ros_version=1,
            llm=self.__llm,
            tools=[cool_turtle_tool],
            tool_packages=[turtle_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
        )

        self.examples = [
            "Give me a ROS tutorial using the turtlesim.",
            "Show me how to move the turtle forward.",
            "Draw a 5-point star using the turtle.",
            "Teleport to (3, 3) and draw a small hexagon.",
            "Give me a list of ROS nodes and their topics.",
            "Change the background color to light blue and the pen color to red.",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear()
        }

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-TurtleSim agent 🐢🤖. How can I help you today?\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit.\n",
            style="underline",
        )
        greeting.append(
            f"Streaming: {self.__streaming}\n",
            style="green" if self.__streaming else "red",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        return pyip.inputMenu(
            self.examples,
            prompt="\nEnter your choice and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        return pyip.inputStr(prompt, default="help")

    async def run(self):
        """
        Run the TurtleAgent's main interaction loop.

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
        console = Console()

        while True:
            console.print(self.greeting)
            input = self.get_input("Turtle Chat: ")

            # Handle special commands
            if input == "exit":
                break
            elif input in self.command_handler:
                await self.command_handler[input]()
            else:
                await self.submit(input)

    async def submit(self, query: str):
        if self.__streaming:
            await self.stream_response(query)
        else:
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
        with formatted output for tokens, tool executions, and errors.

        Args:
            query (str): The input query to process.

        Returns:
            None

        Raises:
            Any exceptions raised during the streaming process.
        """
        console = Console()
        content = ""
        tool_panel = None
        content_panel = None

        with Live(
            console=console, auto_refresh=False, vertical_overflow="visible"
        ) as live:
            async for event in self.astream(query):
                # Accumulate and display token content
                if event["type"] == "token":
                    content += event["content"]
                    content_panel = Panel(
                        Markdown(content), title="Response", border_style="blue"
                    )
                    live.update(
                        (
                            Group(tool_panel, content_panel)
                            if tool_panel
                            else content_panel
                        ),
                        refresh=True,
                    )

                # Display panel for tool execution start
                elif event["type"] == "tool_start":
                    tool_input = event.get("input", "N/A")
                    tool_panel = Panel(
                        f"Using tool: {event['name']}\nInput: {tool_input}",
                        title="Tool Execution",
                        border_style="yellow",
                    )
                    live.update(
                        (
                            Group(tool_panel, content_panel)
                            if content_panel
                            else tool_panel
                        ),
                        refresh=True,
                    )

                # Update panel for tool execution completion
                elif event["type"] == "tool_end":
                    if tool_panel:
                        tool_panel.border_style = "green"
                        tool_panel.title = "Tool Execution Complete"
                        tool_input = event.get("input", "N/A")
                        tool_output = event.get("output", "N/A")
                        tool_panel.renderable = Text.from_markup(
                            f"Tool: [bold]{event['name']}[/bold]\n"
                            f"Input: {tool_input}\n"
                            f"Output: {tool_output}"
                        )
                        live.update(
                            (
                                Group(tool_panel, content_panel)
                                if content_panel
                                else tool_panel
                            ),
                            refresh=True,
                        )

                # Display final response
                elif event["type"] == "final":
                    content_panel = Panel(
                        Markdown(event["content"]),
                        title="Final Response",
                        border_style="green",
                    )
                    live.update(
                        (
                            Group(tool_panel, content_panel)
                            if tool_panel
                            else content_panel
                        ),
                        refresh=True,
                    )

                # Display any errors encountered
                elif event["type"] == "error":
                    error_panel = Panel(
                        f"[bold red]Error:[/bold red] {event['content']}",
                        title="Error",
                        border_style="red",
                    )
                    live.update(
                        (
                            Group(tool_panel, content_panel, error_panel)
                            if content_panel and tool_panel
                            else error_panel
                        ),
                        refresh=True,
                    )


def main():
    dotenv.load_dotenv(dotenv.find_dotenv())

    streaming = rospy.get_param("~streaming", True)
    turtle_agent = TurtleAgent(verbose=False, streaming=streaming)

    asyncio.run(turtle_agent.run())


if __name__ == "__main__":
    rospy.init_node("rosa", log_level=rospy.INFO)
    main()
