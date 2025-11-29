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
import signal
import sys
from datetime import datetime

import dotenv
import pyinputplus as pyip
import rospy
from langchain.agents import tool, Tool
# from langchain_ollama import ChatOllama
from rich.console import Console
from rich.console import Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA

import tools.turtle as turtle_tools
from help import get_help
from llm import get_llm
from prompts import get_prompts


class GracefulInterruptHandler:
    """Context manager to handle interrupts gracefully."""
    
    def __init__(self, verbose: bool = True):
        self.interrupted = False
        self.original_handler = None
        self.verbose = verbose
    
    def __enter__(self):
        self.interrupted = False
        self.original_handler = signal.signal(signal.SIGINT, self._handler)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        signal.signal(signal.SIGINT, self.original_handler)
        return False
    
    def _handler(self, signum, frame):
        self.interrupted = True
        if self.verbose:
            print("\n[Interrupt received - stopping current operation...]")
        # Raise KeyboardInterrupt to break out of loops
        raise KeyboardInterrupt


# Typical method for defining tools in ROSA
@tool
def cool_turtle_tool():
    """A cool turtle tool that doesn't really do anything."""
    return "This is a cool turtle tool! It doesn't do anything, but it's cool."


class TurtleAgent(ROSA):

    def __init__(self, streaming: bool = False, verbose: bool = True):
        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=streaming)

        # self.__llm = ChatOllama(
        #     base_url="host.docker.internal:11434",
        #     model="llama3.1",
        #     temperature=0,
        #     num_ctx=8192,
        # )

        self.__streaming = streaming

        # Another method for adding tools
        blast_off = Tool(
            name="blast_off",
            func=self.blast_off,
            description="Make the turtle blast off!",
        )

        super().__init__(
            ros_version=1,
            llm=self.__llm,
            tools=[cool_turtle_tool, blast_off],
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
            "Give me a list of nodes, topics, services, params, and log files.",
            "Change the background color to light blue and the pen color to red.",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
        }

    def blast_off(self, input: str):
        return f"""
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
        self.last_events = []
        self.command_handler.pop("info", None)
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
        await self.clear()
        console = Console()

        while True:
            try:
                console.print(self.greeting)
                input = self.get_input("> ")

                # Handle special commands
                if input == "exit":
                    break
                elif input in self.command_handler:
                    await self.command_handler[input]()
                else:
                    await self.submit(input)
            except KeyboardInterrupt:
                console.print("\n[yellow]Operation interrupted. Type 'exit' to quit or continue with a new query.[/yellow]")
                # Clear any partial state
                continue
            except Exception as e:
                console.print(f"[red]Error: {e}[/red]")
                continue

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
        console = Console()
        content_panel = None

        try:
            with GracefulInterruptHandler():
                response = self.invoke(query)
                with Live(
                    console=console, auto_refresh=True, vertical_overflow="visible"
                ) as live:
                    content_panel = Panel(
                        Markdown(response), title="Final Response", border_style="green"
                    )
                    live.update(content_panel, refresh=True)
        except KeyboardInterrupt:
            console.print("\n[yellow]Response interrupted.[/yellow]")
            raise

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

        try:
            with GracefulInterruptHandler() as handler:
                with Live(panel, console=console, auto_refresh=False) as live:
                    async for event in self.astream(query):
                        if handler.interrupted:
                            break
                        
                        event["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[
                            :-3
                        ]
                        if event["type"] == "token":
                            content += event["content"]
                            panel.renderable = Markdown(content)
                            live.refresh()
                        elif event["type"] in ["tool_start", "tool_end", "error"]:
                            self.last_events.append(event)
                        elif event["type"] == "final":
                            content = event["content"]
                            if self.last_events:
                                panel.renderable = Markdown(
                                    content
                                    + "\n\nType 'info' for details on how I got my answer."
                                )
                            else:
                                panel.renderable = Markdown(content)
                            panel.title = "Final Response"
                            live.refresh()

                if self.last_events:
                    self.command_handler["info"] = self.show_event_details
                else:
                    self.command_handler.pop("info", None)
        except KeyboardInterrupt:
            console.print("\n[yellow]Response interrupted.[/yellow]")
            raise

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

    streaming = rospy.get_param("~streaming", False)
    turtle_agent = TurtleAgent(verbose=False, streaming=streaming)

    try:
        asyncio.run(turtle_agent.run())
    except KeyboardInterrupt:
        print("\n[Shutdown complete]")
    except Exception as e:
        print(f"\n[Error: {e}]")
        sys.exit(1)


if __name__ == "__main__":
    rospy.init_node("rosa", log_level=rospy.INFO)
    main()
