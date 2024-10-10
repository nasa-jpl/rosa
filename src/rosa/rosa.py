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

from typing import Any, AsyncIterable, Dict, Literal, Optional, Union

from langchain.agents import AgentExecutor
from langchain.agents.format_scratchpad.openai_tools import (
    format_to_openai_tool_messages,
)
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.prompts import MessagesPlaceholder
from langchain_community.callbacks import get_openai_callback
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
from langchain_openai import AzureChatOpenAI, ChatOpenAI

from .prompts import RobotSystemPrompts, system_prompts
from .tools import ROSATools

ChatModel = Union[ChatOpenAI, AzureChatOpenAI, ChatOllama]


class ROSA:
    """ROSA (Robot Operating System Agent) is a class that encapsulates the logic for interacting with ROS systems
    using natural language.

    Args:
        ros_version (Literal[1, 2]): The version of ROS that the agent will interact with.
        llm (Union[AzureChatOpenAI, ChatOpenAI, ChatOllama]): The language model to use for generating responses.
        tools (Optional[list]): A list of additional LangChain tool functions to use with the agent.
        tool_packages (Optional[list]): A list of Python packages containing LangChain tool functions to use.
        prompts (Optional[RobotSystemPrompts]): Custom prompts to use with the agent.
        verbose (bool): Whether to print verbose output. Defaults to False.
        blacklist (Optional[list]): A list of ROS tools to exclude from the agent.
        accumulate_chat_history (bool): Whether to accumulate chat history. Defaults to True.
        show_token_usage (bool): Whether to show token usage. Does not work when streaming is enabled. Defaults to False.
        streaming (bool): Whether to stream the output of the agent. Defaults to True.

    Attributes:
        chat_history (list): A list of messages representing the chat history.

    Methods:
        clear_chat(): Clears the chat history.
        invoke(query: str) -> str: Processes a user query and returns the agent's response.
        astream(query: str) -> AsyncIterable[Dict[str, Any]]: Asynchronously streams the agent's response.

    Note:
        - The `tools` and `tool_packages` arguments allow for extending the agent's capabilities.
        - Custom `prompts` can be provided to tailor the agent's behavior for specific robots or use cases.
        - Token usage display is automatically disabled when streaming is enabled.
        - Use `invoke()` for non-streaming responses and `astream()` for streaming responses.
    """

    def __init__(
        self,
        ros_version: Literal[1, 2],
        llm: ChatModel,
        tools: Optional[list] = None,
        tool_packages: Optional[list] = None,
        prompts: Optional[RobotSystemPrompts] = None,
        verbose: bool = False,
        blacklist: Optional[list] = None,
        accumulate_chat_history: bool = True,
        show_token_usage: bool = False,
        streaming: bool = True,
    ):
        self.__chat_history = []
        self.__ros_version = ros_version
        self.__llm = llm.with_config({"streaming": streaming})
        self.__memory_key = "chat_history"
        self.__scratchpad = "agent_scratchpad"
        self.__blacklist = blacklist if blacklist else []
        self.__accumulate_chat_history = accumulate_chat_history
        self.__streaming = streaming
        self.__tools = self._get_tools(
            ros_version, packages=tool_packages, tools=tools, blacklist=self.__blacklist
        )
        self.__prompts = self._get_prompts(prompts)
        self.__llm_with_tools = self.__llm.bind_tools(self.__tools.get_tools())
        self.__agent = self._get_agent()
        self.__executor = self._get_executor(verbose=verbose)
        self.__show_token_usage = show_token_usage if not streaming else False

    @property
    def chat_history(self):
        """Get the chat history."""
        return self.__chat_history

    def clear_chat(self):
        """Clear the chat history."""
        self.__chat_history = []

    def invoke(self, query: str) -> str:
        """
        Invoke the agent with a user query and return the response.

        This method processes the user's query through the agent, handles token usage tracking,
        and updates the chat history.

        Args:
            query (str): The user's input query to be processed by the agent.

        Returns:
            str: The agent's response to the query. If an error occurs, it returns an error message.

        Raises:
            Any exceptions raised during the invocation process are caught and returned as error messages.

        Note:
            - This method uses OpenAI's callback to track token usage if enabled.
            - The chat history is updated with the query and response if successful.
            - Token usage is printed if the show_token_usage flag is set.
        """
        try:
            with get_openai_callback() as cb:
                result = self.__executor.invoke(
                    {"input": query, "chat_history": self.__chat_history}
                )
                self._print_usage(cb)
        except Exception as e:
            return f"An error occurred: {str(e)}"

        self._record_chat_history(query, result["output"])
        return result["output"]

    async def astream(self, query: str) -> AsyncIterable[Dict[str, Any]]:
        """
        Asynchronously stream the agent's response to a user query.

        This method processes the user's query and yields events as they occur,
        including token generation, tool usage, and final output. It's designed
        for use when streaming is enabled.

        Args:
            query (str): The user's input query.

        Returns:
            AsyncIterable[Dict[str, Any]]: An asynchronous iterable of dictionaries
            containing event information. Each dictionary has a 'type' key and
            additional keys depending on the event type:
            - 'token': Yields generated tokens with 'content'.
            - 'tool_start': Indicates the start of a tool execution with 'name' and 'input'.
            - 'tool_end': Indicates the end of a tool execution with 'name' and 'output'.
            - 'final': Provides the final output of the agent with 'content'.
            - 'error': Indicates an error occurred with 'content' describing the error.

        Raises:
            ValueError: If streaming is not enabled for this ROSA instance.
            Exception: If an error occurs during the streaming process.

        Note:
            This method updates the chat history with the final output if successful.
        """
        if not self.__streaming:
            raise ValueError(
                "Streaming is not enabled. Use 'invoke' method instead or initialize ROSA with streaming=True."
            )

        try:
            final_output = ""
            # Stream events from the agent's response
            async for event in self.__executor.astream_events(
                input={"input": query, "chat_history": self.__chat_history},
                config={"run_name": "Agent"},
                version="v2",
            ):
                # Extract the event type
                kind = event["event"]

                # Handle chat model stream events
                if kind == "on_chat_model_stream":
                    # Extract the content from the event and yield it
                    content = event["data"]["chunk"].content
                    if content:
                        final_output += f" {content}"
                        yield {"type": "token", "content": content}

                # Handle tool start events
                elif kind == "on_tool_start":
                    yield {
                        "type": "tool_start",
                        "name": event["name"],
                        "input": event["data"].get("input"),
                    }

                # Handle tool end events
                elif kind == "on_tool_end":
                    yield {
                        "type": "tool_end",
                        "name": event["name"],
                        "output": event["data"].get("output"),
                    }

                # Handle chain end events
                elif kind == "on_chain_end":
                    if event["name"] == "Agent":
                        chain_output = event["data"].get("output", {}).get("output")
                        if chain_output:
                            final_output = (
                                chain_output  # Override with final output if available
                            )
                            yield {"type": "final", "content": chain_output}

            if final_output:
                self._record_chat_history(query, final_output)
        except Exception as e:
            yield {"type": "error", "content": f"An error occurred: {e}"}

    def _get_executor(self, verbose: bool) -> AgentExecutor:
        """Create and return an executor for processing user inputs and generating responses."""
        executor = AgentExecutor(
            agent=self.__agent,
            tools=self.__tools.get_tools(),
            stream_runnable=self.__streaming,
            verbose=verbose,
        )
        return executor

    def _get_agent(self):
        """Create and return an agent for processing user inputs and generating responses."""
        agent = (
            {
                "input": lambda x: x["input"],
                "agent_scratchpad": lambda x: format_to_openai_tool_messages(
                    x["intermediate_steps"]
                ),
                "chat_history": lambda x: x["chat_history"],
            }
            | self.__prompts
            | self.__llm_with_tools
            | OpenAIToolsAgentOutputParser()
        )
        return agent

    def _get_tools(
        self,
        ros_version: Literal[1, 2],
        packages: Optional[list],
        tools: Optional[list],
        blacklist: Optional[list],
    ) -> ROSATools:
        """Create a ROSA tools object with the specified ROS version, tools, packages, and blacklist."""
        rosa_tools = ROSATools(ros_version, blacklist=blacklist)
        if tools:
            rosa_tools.add_tools(tools)
        if packages:
            rosa_tools.add_packages(packages, blacklist=blacklist)
        return rosa_tools

    def _get_prompts(
        self, robot_prompts: Optional[RobotSystemPrompts] = None
    ) -> ChatPromptTemplate:
        """Create a chat prompt template from the system prompts and robot-specific prompts."""
        # Start with default system prompts
        prompts = system_prompts

        # Add robot-specific prompts if provided
        if robot_prompts:
            prompts.append(robot_prompts.as_message())

        template = ChatPromptTemplate.from_messages(
            prompts
            + [
                MessagesPlaceholder(variable_name=self.__memory_key),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name=self.__scratchpad),
            ]
        )
        return template

    def _print_usage(self, cb):
        """Print the token usage if show_token_usage is enabled."""
        if cb and self.__show_token_usage:
            print(f"[bold]Prompt Tokens:[/bold] {cb.prompt_tokens}")
            print(f"[bold]Completion Tokens:[/bold] {cb.completion_tokens}")
            print(f"[bold]Total Cost (USD):[/bold] ${cb.total_cost}")

    def _record_chat_history(self, query: str, response: str):
        """Record the chat history if accumulation is enabled."""
        if self.__accumulate_chat_history:
            self.__chat_history.extend(
                [HumanMessage(content=query), AIMessage(content=response)]
            )
