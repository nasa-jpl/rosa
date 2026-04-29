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
from pathlib import Path
import signal
import sys
import threading
import time
import uuid
from datetime import datetime
import json
from typing import Any, Dict, List, Optional, Tuple

import dotenv
import pyinputplus as pyip
import rospy
import tools.obstacle as obstacle_tools
import tools.turtle as turtle_tools
from collision_event_sink import make_collision_event_sink
from collision_monitor import CollisionMonitor
from command_logger import CommandLogger
from help import get_help
from langchain.agents import Tool, tool

# from langchain_ollama import ChatOllama
from llm import get_llm
from memory_converter import MemoryConverter
from memory_prompting import build_memory_context, infer_query_context, load_long_term_records
from obstacle_store import ObstacleStore
from pose_hub import PoseHub
from pose_logger import (
    POSE_LOG_INTERVAL_SEC,
    CollisionJsonlWriter,
    PoseLogConsumer,
)
from prompts import get_prompts
from rich.console import Console, Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA
from ros_params import get_bool_param
from static_world import load_static_world


def _maybe_attach_debugpy() -> None:
    """Listen for debugpy when ROSA_DEBUGPY is enabled (Docker + host Cursor attach)."""
    flag = os.environ.get("ROSA_DEBUGPY", "").strip().lower()
    if flag not in ("1", "true", "yes"):
        return
    try:
        port = int(os.environ.get("ROSA_DEBUGPY_PORT", "5678").strip())
    except ValueError:
        port = 5678
    try:
        import debugpy
    except ImportError:
        print(
            "[ROSA_DEBUGPY] debugpy is not installed; rebuild the image or pip install debugpy.",
            file=sys.stderr,
        )
        raise
    debugpy.listen(("0.0.0.0", port))
    print(
        f"[ROSA_DEBUGPY] listening on 0.0.0.0:{port}; attach from host, then execution continues.",
        flush=True,
    )
    debugpy.wait_for_client()


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


def _normalize_tool_args(raw_input: Any) -> Dict[str, Any]:
    """LangChain AgentAction.tool_input 을 JSON 직렬화 가능한 dict 로 맞춥니다."""
    if isinstance(raw_input, dict):
        return dict(raw_input)
    if isinstance(raw_input, str):
        try:
            parsed = json.loads(raw_input)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass
        return {"tool_input": raw_input}
    if raw_input is None:
        return {}
    return {"tool_input": raw_input}


class TurtleAgent(ROSA):

    def __init__(
        self,
        streaming: bool = False,
        verbose: bool = True,
        obstacle_store: Optional[ObstacleStore] = None,
        command_logger: Optional[CommandLogger] = None,
        memory_converter: Optional[MemoryConverter] = None,
    ):
        self.__blacklist = ["master", "docker"]
        self._obstacle_store = obstacle_store or ObstacleStore()
        obstacle_tools.configure_obstacle_store(self._obstacle_store)
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=streaming)

        # self.__llm = ChatOllama(
        #     base_url="host.docker.internal:11434",
        #     model="llama3.1",
        #     temperature=0,
        #     num_ctx=8192,
        # )

        self.__streaming = streaming
        self._command_logger = command_logger or CommandLogger()
        self._memory_converter = memory_converter or MemoryConverter()
        self._turtle_id = rospy.get_param(
            "~turtle_id", os.environ.get("TURTLE_TURTLE_ID", "turtle1")
        )
        self._agent_mode = str(rospy.get_param("~agent_mode", "single"))
        self._memory_root = (Path(__file__).resolve().parent / "memory").resolve()

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
            tool_packages=[turtle_tools, obstacle_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
            return_intermediate_steps=True,
            on_intermediate_steps=self._record_agent_tool_steps,
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

    def _record_agent_tool_steps(self, intermediate_steps: List[Tuple[Any, Any]]) -> None:
        """AgentExecutor intermediate_steps 에서 실제 호출된 도구 이름·인자를 command 로그에 남깁니다."""
        if not intermediate_steps:
            return
        for pair in intermediate_steps:
            try:
                if not isinstance(pair, tuple) or len(pair) < 2:
                    continue
                action, observation = pair[0], pair[1]
            except Exception:
                continue
            tool_name = getattr(action, "tool", None) or getattr(action, "tool_name", None)
            if not tool_name:
                continue
            raw_input = getattr(action, "tool_input", None)
            args = _normalize_tool_args(raw_input)
            self._command_logger.log_skill(
                self._turtle_id,
                skill=str(tool_name),
                args=args,
                status="success",
                result=str(observation)[:4000],
            )

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
        no_clear = os.environ.get("ROSA_NO_CLEAR", "").strip().lower()
        if no_clear in ("1", "true", "yes", "on"):
            return
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
                console.print(
                    "\n[yellow]Operation interrupted. Type 'exit' to quit or continue with a new query.[/yellow]"
                )
                # Clear any partial state
                continue
            except Exception as e:
                console.print(f"[red]Error: {e}[/red]")
                continue

    async def submit(self, query: str):
        query_ctx = infer_query_context(query)
        memory_context = ""
        memory_hits = 0
        if self._agent_mode.strip().lower() == "single":
            long_records = load_long_term_records(self._memory_root, self._turtle_id)
            memory_context, memory_hits = build_memory_context(query, long_records, top_k=3)
        effective_query = (
            f"{memory_context}\n\nUser query:\n{query}" if memory_context else query
        )
        rospy.loginfo(
            "memory prompt: mode=%s hits=%s experience_key=%s",
            self._agent_mode,
            memory_hits,
            query_ctx.get("experience_key", ""),
        )
        self._command_logger.log_intent(
            self._turtle_id,
            {
                "task_family": str(query_ctx.get("task_family", "natural_language_query")),
                "slots": query_ctx.get("slots", {}),
                "natural_language": query,
                "query": query,
                "memory_hits": memory_hits,
                "experience_key": query_ctx.get("experience_key", ""),
            },
        )
        if self.__streaming:
            response = await self.stream_response(effective_query)
        else:
            response = self.print_response(effective_query)
        self._command_logger.log_skill(
            self._turtle_id,
            skill="rosa_response",
            args={
                "query": query,
                "memory_hits": memory_hits,
                "experience_key": query_ctx.get("experience_key", ""),
            },
            status="success",
            result=str(response),
        )
        try:
            conversion = self._memory_converter.convert_session(
                date_str=self._command_logger.date_str,
                session_id=self._command_logger.session_id,
                turtle_id=self._turtle_id,
                test_case_id=f"tc-{int(time.time() * 1000)}-{uuid.uuid4().hex[:6]}",
                write_long_term=False,
                mode=self._agent_mode,
            )
            rospy.loginfo(
                "memory conversion completed: short=%s",
                conversion.get("short_term_written", 0),
            )
        except Exception as e:
            rospy.logwarn("memory conversion skipped: %s", e)

    def print_response(self, query: str) -> str:
        """
        Submit the query to the agent and print the response to the console.

        Args:
            query (str): The input query to process.

        Returns:
            None
        """
        console = Console()
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
                return response
        except KeyboardInterrupt:
            console.print("\n[yellow]Response interrupted.[/yellow]")
            raise

    async def stream_response(self, query: str) -> str:
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

        stream_tool_inputs_queue: List[Any] = []

        try:
            with GracefulInterruptHandler() as handler:
                with Live(panel, console=console, auto_refresh=False) as live:
                    async for event in self.astream(query):
                        if handler.interrupted:
                            break

                        event["timestamp"] = datetime.now().strftime(
                            "%Y-%m-%d %H:%M:%S.%f"
                        )[:-3]
                        if event["type"] == "token":
                            content += event["content"]
                            panel.renderable = Markdown(content)
                            live.refresh()
                        elif event["type"] == "tool_start":
                            self.last_events.append(event)
                            stream_tool_inputs_queue.append(event.get("input"))
                        elif event["type"] == "tool_end":
                            self.last_events.append(event)
                            inp = stream_tool_inputs_queue.pop(0) if stream_tool_inputs_queue else None
                            args = _normalize_tool_args(inp)
                            self._command_logger.log_skill(
                                self._turtle_id,
                                skill=str(event.get("name", "unknown")),
                                args=args,
                                status="success",
                                result=str(event.get("output", ""))[:4000],
                            )
                        elif event["type"] == "error":
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
                return content
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


def main(
    obstacle_store: Optional[ObstacleStore] = None,
    *,
    load_static_world_once: bool = True,
) -> None:
    dotenv.load_dotenv(dotenv.find_dotenv())

    streaming = rospy.get_param("~streaming", False)
    turtle_id = rospy.get_param("~turtle_id", os.environ.get("TURTLE_TURTLE_ID", "turtle1"))
    if obstacle_store is None:
        obstacle_store = ObstacleStore()
    if load_static_world_once:
        load_static_world(obstacle_store)

    command_logger = CommandLogger(session_id=pose_log_consumer.session_id)
    memory_converter = MemoryConverter()
    turtle_agent = TurtleAgent(
        verbose=False,
        streaming=streaming,
        obstacle_store=obstacle_store,
        command_logger=command_logger,
        memory_converter=memory_converter,
    )

    try:
        asyncio.run(turtle_agent.run())
    except KeyboardInterrupt:
        print("\n[Shutdown complete]")
    except Exception as e:
        print(f"\n[Error: {e}]")
        sys.exit(1)
    finally:
        try:
            long_count = memory_converter.finalize_session(
                session_id=command_logger.session_id,
                turtle_id=str(turtle_id),
            )
            rospy.loginfo("long-term finalize completed: written=%s", long_count)
        except Exception as e:
            rospy.logwarn("long-term finalize skipped: %s", e)
        command_logger.close()


if __name__ == "__main__":
    _maybe_attach_debugpy()
    rospy.init_node("rosa", log_level=rospy.INFO)
    rospy.loginfo("runtime tools.turtle module: %s", getattr(turtle_tools, "__file__", "unknown"))

    # rospy.AsyncSpinner is missing in some stacks; a daemon spin thread is portable.
    def _ros_spin() -> None:
        rospy.spin()

    spin_thread = threading.Thread(target=_ros_spin, name="rospy_spin", daemon=True)
    spin_thread.start()

    obstacle_store = ObstacleStore()
    load_static_world(obstacle_store)
    pose_hub = PoseHub()
    pose_log_consumer = PoseLogConsumer(
        period=float(rospy.get_param("~pose_log_interval", POSE_LOG_INTERVAL_SEC))
    )
    collision_jsonl = CollisionJsonlWriter(pose_log_consumer.collision_log_path())
    if get_bool_param("~collision_log_to_console", False):
        rospy.loginfo("collision log: %s", collision_jsonl.path)
    collision_monitor = CollisionMonitor(
        obstacle_store,
        turtle_radius=float(rospy.get_param("~turtle_collision_radius", 0.5)),
        emit_stay=get_bool_param("~collision_emit_stay", False),
        event_sink=make_collision_event_sink(collision_jsonl),
    )
    pose_hub.register_consumer(pose_log_consumer.on_pose)
    pose_hub.register_consumer(collision_monitor.on_pose)
    registered_turtles = pose_hub.start_from_ros_graph_once()
    rospy.loginfo("PoseHub registered turtles: %s", ", ".join(registered_turtles))
    turtle_tools.configure_turtle_lifecycle_listener(pose_hub)
    try:
        main(obstacle_store=obstacle_store, load_static_world_once=False)
    finally:
        turtle_tools.configure_turtle_lifecycle_listener(None)
        pose_hub.stop()
        pose_log_consumer.close()
        collision_jsonl.close()
        rospy.signal_shutdown("turtle_agent exiting")
        spin_thread.join(timeout=5.0)
