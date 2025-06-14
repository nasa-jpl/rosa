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

import logging
from typing import Any, AsyncIterable, Dict, Literal, Optional, Union

from langchain.agents import AgentExecutor
from langchain.agents.format_scratchpad.openai_tools import (
    format_to_openai_tool_messages,
)
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.prompts import MessagesPlaceholder
from langchain_community.callbacks import get_openai_callback, get_anthropic_callback
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import Runnable
from langchain_openai import ChatOpenAI, AzureChatOpenAI
from langchain_ollama import ChatOllama
from langchain_community.chat_models import ChatAnthropic

from .prompts import RobotSystemPrompts, system_prompts
from .tools import ROSATools

ChatModel = Union[ChatOpenAI, AzureChatOpenAI, ChatOllama, ChatAnthropic]


class ROSAError(Exception):
    """Base exception class for ROSA-related errors."""
    pass


class ROSAConfigurationError(ROSAError):
    """Raised when ROSA is configured incorrectly."""
    pass


class ROSAExecutionError(ROSAError):
    """Raised when ROSA encounters an error during execution."""
    pass


class ROSA:
    """ROSA (Robot Operating System Agent) is a class that encapsulates the logic for interacting with ROS systems
    using natural language.

    Args:
        ros_version (Literal[1, 2]): The version of ROS that the agent will interact with.
        llm (Union[AzureChatOpenAI, ChatOpenAI, ChatOllama, ChatAnthropic]): The language model to use for generating responses.
        tools (Optional[list]): A list of additional LangChain tool functions to use with the agent.
        tool_packages (Optional[list]): A list of Python packages containing LangChain tool functions to use.
        prompts (Optional[RobotSystemPrompts]): Custom prompts to use with the agent.
        verbose (bool): Whether to print verbose output. Defaults to False.
        blacklist (Optional[list]): A list of ROS tools to exclude from the agent.
        accumulate_chat_history (bool): Whether to accumulate chat history. Defaults to True.
        show_token_usage (bool): Whether to show token usage. Does not work when streaming is enabled. Defaults to False.
        streaming (bool): Whether to stream the output of the agent. Defaults to True.
        max_history_length (Optional[int]): Maximum chat history messages. None for unlimited. Defaults to 50.

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
        - Chat history trimmed automatically when max_history_length exceeded.
    
    Raises:
        ROSAConfigurationError: If the agent is configured incorrectly.
        ROSAExecutionError: If an error occurs during agent execution.
    """
    
    # Class constants
    MEMORY_KEY = "chat_history"
    SCRATCHPAD_KEY = "agent_scratchpad"
    AGENT_RUN_NAME = "Agent"

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
        max_history_length: Optional[int] = 50,
    ):
        # Setup logging
        self.__logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        
        # Validate inputs
        self._validate_inputs(ros_version, llm, max_history_length)

        self.__logger.info(f"Initializing ROSA agent with ROS version {ros_version}")
        self.__logger.info(f"LLM: {llm}")
        self.__logger.info(f"Tools: {tools}")
        self.__logger.info(f"Tool packages: {tool_packages}")
        self.__logger.info(f"Prompts: {prompts}")
        self.__logger.info(f"Verbose: {verbose}")
        self.__logger.info(f"Blacklist: {blacklist}")
        
        # Initialize instance variables
        self.__chat_history = []
        self.__ros_version = ros_version
        self.__llm = llm.with_config({"streaming": streaming})
        self.__blacklist = blacklist if blacklist else []
        self.__accumulate_chat_history = accumulate_chat_history
        self.__streaming = streaming
        self.__show_token_usage = show_token_usage if not streaming else False
        self.__max_history_length = max_history_length
        
        # Initialize components
        try:
            self.__tools = self._get_tools(
                ros_version, packages=tool_packages, tools=tools, blacklist=self.__blacklist
            )
            self.__prompts = self._get_prompts(prompts)
            self.__llm_with_tools = self.__llm.bind_tools(self.__tools.get_tools())
            self.__agent = self._get_agent()
            self.__executor = self._get_executor(verbose=verbose)
            self.__logger.info(f"ROSA agent initialized with ROS version {ros_version}")
        except Exception as e:
            self.__logger.error(f"Failed to initialize ROSA agent: {e}")
            raise ROSAConfigurationError(f"Failed to initialize ROSA agent: {e}") from e
    
    def _validate_inputs(self, ros_version: Literal[1, 2], llm: ChatModel, max_history_length: Optional[int]) -> None:
        """Validate constructor inputs.
        
        Args:
            ros_version: The ROS version to validate.
            llm: The language model to validate.
            max_history_length: The maximum history length to validate.
            
        Raises:
            ROSAConfigurationError: If inputs are invalid.
        """
        if ros_version not in (1, 2):
            raise ROSAConfigurationError(f"Invalid ROS version: {ros_version}. Must be 1 or 2.")
            
        if not isinstance(llm, (ChatOpenAI, AzureChatOpenAI, ChatOllama, ChatAnthropic)):
            raise ROSAConfigurationError(
                f"Invalid LLM type: {type(llm)}. Must be ChatOpenAI, AzureChatOpenAI, ChatOllama, or ChatAnthropic."
            )
            
        if max_history_length is not None and (not isinstance(max_history_length, int) or max_history_length <= 0):
            raise ROSAConfigurationError(
                f"Invalid max_history_length: {max_history_length}. Must be a positive integer or None."
            )

    @property
    def chat_history(self):
        """Get the chat history."""
        return self.__chat_history

    def clear_chat(self, retain_system_messages: bool = False) -> None:
        """
        Clear the chat history.
        
        Args:
            retain_system_messages (bool): If True, keep system messages in the history.
                                         If False, clear all messages. Defaults to False.
        """
        if retain_system_messages:
            from langchain_core.messages import SystemMessage
            self.__chat_history = [msg for msg in self.__chat_history 
                                   if isinstance(msg, SystemMessage)]
        else:
            self.__chat_history = []

    def invoke(self, query: str) -> str:
        """
        Invoke the agent with a user query and return the response.

        This method processes the user's query through the agent, handles token usage tracking,
        and updates the chat history.

        Args:
            query (str): The user's input query to be processed by the agent.

        Returns:
            str: The agent's response to the query.

        Raises:
            ROSAExecutionError: If an error occurs during agent execution.
            ValueError: If the query is empty or invalid.

        Note:
            - This method uses OpenAI's callback to track token usage if enabled.
            - The chat history is updated with the query and response if successful.
            - Token usage is printed if the show_token_usage flag is set.
        """
        if not query or not query.strip():
            raise ValueError("Query cannot be empty")
            
        self.__logger.debug(f"Processing query: {query[:100]}{'...' if len(query) > 100 else ''}")
        
        try:
            with self._get_usage_callback() as cb:
                result = self.__executor.invoke(
                    {"input": query, self.MEMORY_KEY: self.__chat_history}
                )
                self._print_usage(cb)
        except Exception as e:
            self.__logger.error(f"Agent execution failed: {e}", exc_info=True)
            raise ROSAExecutionError(f"Agent execution failed: {e}") from e

        if "output" not in result:
            self.__logger.error("Agent result missing 'output' key")
            raise ROSAExecutionError("Agent result missing 'output' key")
            
        self._record_chat_history(query, result["output"])
        self.__logger.debug("Query processed successfully")
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
            ValueError: If streaming is not enabled or query is invalid.
            ROSAExecutionError: If an error occurs during the streaming process.

        Note:
            This method updates the chat history with the final output if successful.
        """
        if not self.__streaming:
            raise ValueError(
                "Streaming is not enabled. Use 'invoke' method instead or initialize ROSA with streaming=True."
            )
            
        if not query or not query.strip():
            raise ValueError("Query cannot be empty")
            
        self.__logger.debug(f"Streaming query: {query[:100]}{'...' if len(query) > 100 else ''}")

        try:
            final_output = ""
            # Stream events from the agent's response
            async for event in self.__executor.astream_events(
                input={"input": query, self.MEMORY_KEY: self.__chat_history},
                config={"run_name": self.AGENT_RUN_NAME},
                version="v2",
            ):
                # Extract the event type
                kind = event["event"]

                # Handle chat model stream events
                if kind == "on_chat_model_stream":
                    # Extract the content from the event and yield it
                    content = event["data"]["chunk"].content
                    if content:
                        final_output += content
                        yield {"type": "token", "content": content}

                # Handle tool start events
                elif kind == "on_tool_start":
                    self.__logger.debug(f"Tool started: {event['name']}")
                    yield {
                        "type": "tool_start",
                        "name": event["name"],
                        "input": event["data"].get("input"),
                    }

                # Handle tool end events
                elif kind == "on_tool_end":
                    self.__logger.debug(f"Tool ended: {event['name']}")
                    yield {
                        "type": "tool_end",
                        "name": event["name"],
                        "output": event["data"].get("output"),
                    }

                # Handle chain end events
                elif kind == "on_chain_end":
                    if event["name"] == self.AGENT_RUN_NAME:
                        chain_output = event["data"].get("output", {}).get("output")
                        if chain_output:
                            final_output = (
                                chain_output  # Override with final output if available
                            )
                            yield {"type": "final", "content": chain_output}

            if final_output:
                self._record_chat_history(query, final_output)
                self.__logger.debug("Streaming query processed successfully")
        except Exception as e:
            self.__logger.error(f"Streaming execution failed: {e}", exc_info=True)
            yield {"type": "error", "content": f"An error occurred: {e}"}

    def _get_executor(self, verbose: bool) -> AgentExecutor:
        """Create and return an executor for processing user inputs and generating responses.
        
        Args:
            verbose: Whether to enable verbose output.
            
        Returns:
            AgentExecutor: The configured executor.
        """
        executor = AgentExecutor(
            agent=self.__agent,
            tools=self.__tools.get_tools(),
            stream_runnable=self.__streaming,
            verbose=verbose,
            max_iterations=75,  # Allow complex multi-step tasks while preventing infinite loops
            max_execution_time=300,  # 5 minutes timeout
        )
        return executor

    def _get_agent(self) -> Runnable:
        """Create and return an agent for processing user inputs and generating responses.
        
        Returns:
            Runnable: The configured agent runnable chain.
        """
        agent = (
            {
                "input": lambda x: x["input"],
                self.SCRATCHPAD_KEY: lambda x: format_to_openai_tool_messages(
                    x["intermediate_steps"]
                ),
                self.MEMORY_KEY: lambda x: x[self.MEMORY_KEY],
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
        """Create a chat prompt template from the system prompts and robot-specific prompts.
        
        Args:
            robot_prompts: Optional robot-specific prompts to include.
            
        Returns:
            ChatPromptTemplate: The configured prompt template.
        """
        # Start with default system prompts
        prompts = system_prompts.copy()  # Make a copy to avoid modifying the original

        # Add robot-specific prompts if provided
        if robot_prompts:
            prompts.append(robot_prompts.as_message())

        template = ChatPromptTemplate.from_messages(
            prompts
            + [
                MessagesPlaceholder(variable_name=self.MEMORY_KEY),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name=self.SCRATCHPAD_KEY),
            ]
        )
        return template

    def _get_usage_callback(self):
        """Get the appropriate token usage callback based on LLM type.
        
        Returns:
            A context manager for tracking token usage, or a null callback
            for unsupported LLM types.
        """
        # Get the underlying LLM instance (handles RunnableBinding wrapper)
        llm_instance = self.__llm
        if hasattr(llm_instance, 'bound') and hasattr(llm_instance.bound, '__class__'):
            llm_instance = llm_instance.bound
        
        # Check if this is an OpenAI-compatible model
        if isinstance(llm_instance, (ChatOpenAI, AzureChatOpenAI)):
            return get_openai_callback()
        else:
            # For non-OpenAI models (Ollama, Anthropic, etc.), return a null callback
            return self._get_null_callback()
    
    def _get_null_callback(self):
        """Get a null callback that doesn't track usage for non-OpenAI models.
        
        Returns:
            A context manager that provides a dummy callback object.
        """
        from contextlib import contextmanager
        
        @contextmanager
        def null_callback():
            class NullUsageCallback:
                def __init__(self):
                    self.total_tokens = 0
                    self.prompt_tokens = 0
                    self.completion_tokens = 0
                    self.total_cost = 0.0
            
            yield NullUsageCallback()
        
        return null_callback()

    def _print_usage(self, cb) -> None:
        """Print the token usage if show_token_usage is enabled.
        
        Args:
            cb: The callback object containing usage information.
        """
        if cb and self.__show_token_usage:
            print(f"[bold]Prompt Tokens:[/bold] {cb.prompt_tokens}")
            print(f"[bold]Completion Tokens:[/bold] {cb.completion_tokens}")
            print(f"[bold]Total Cost (USD):[/bold] ${cb.total_cost}")

    def _record_chat_history(self, query: str, response: str) -> None:
        """Record the chat history if accumulation is enabled.
        
        This method atomically adds new Human/AI message pairs to the chat history
        and then applies trimming logic if the history exceeds the maximum length.
        
        Args:
            query: The user's query.
            response: The agent's response.
        """
        if not self.__accumulate_chat_history:
            return
            
        try:
            # Get original length for logging
            original_length = len(self.__chat_history)
            
            # Atomically add the new message pair
            new_messages = [HumanMessage(content=query), AIMessage(content=response)]
            self.__chat_history.extend(new_messages)
            
            # Apply trimming logic if needed
            if self.__max_history_length is not None:
                trimmed_history = self._trim_chat_history(self.__chat_history)
                
                # Check if trimming occurred
                if len(trimmed_history) < len(self.__chat_history):
                    trimmed_count = len(self.__chat_history) - len(trimmed_history)
                    self.__logger.debug(
                        f"Chat history trimmed: removed {trimmed_count} messages "
                        f"(from {len(self.__chat_history)} to {len(trimmed_history)})"
                    )
                    
                # Update history with trimmed version
                self.__chat_history = trimmed_history
                
        except Exception as e:
            self.__logger.error(
                f"Error occurred while recording chat history: {e}", 
                exc_info=True
            )
            # Don't re-raise - we want the main operation to continue even if 
            # history recording fails
    
    def _trim_chat_history(self, chat_history: list) -> list:
        """Trim chat history to keep only the most recent messages within the limit.
        
        Preserves conversation context by keeping Human/AI message pairs intact.
        Uses FIFO strategy - removes oldest pairs first when limit is exceeded.
        
        Args:
            chat_history: List of messages to trim.
            
        Returns:
            Trimmed list of messages preserving most recent pairs.
        """
        # No trimming needed if unlimited history or under limit
        if self.__max_history_length is None or len(chat_history) <= self.__max_history_length:
            return chat_history
        
        # Handle empty history
        if not chat_history:
            return []
        
        # Calculate how many messages to keep
        target_length = self.__max_history_length
        
        # If odd number of messages, preserve the incomplete pair at the end
        if len(chat_history) % 2 == 1:
            # Keep the last incomplete message plus as many complete pairs as possible
            incomplete_message = chat_history[-1:]
            complete_pairs_section = chat_history[:-1]
            
            # Calculate how many complete pair messages we can fit
            remaining_slots = target_length - 1  # Reserve 1 slot for incomplete message
            
            # Each pair takes 2 messages, so keep the most recent pairs
            if remaining_slots >= 2:
                num_pairs_to_keep = remaining_slots // 2
                start_index = len(complete_pairs_section) - (num_pairs_to_keep * 2)
                kept_pairs = complete_pairs_section[start_index:]
                return kept_pairs + incomplete_message
            else:
                # Not enough space for any complete pairs, just keep the incomplete message
                return incomplete_message
        else:
            # Even number of messages - all are complete pairs
            # Keep the most recent complete pairs
            num_pairs_to_keep = target_length // 2
            start_index = len(chat_history) - (num_pairs_to_keep * 2)
            return chat_history[start_index:]
    
    def get_history_length(self) -> int:
        """
        Get the current number of messages in chat history.
        
        Returns:
            int: The number of messages in the chat history.
        """
        return len(self.__chat_history)
    
    def trim_history(self, max_length: int) -> None:
        """
        Manually trim chat history to the specified maximum length.
        
        This method allows for manual trimming of the chat history, using the same
        logic as the automatic trimming system. Messages are preserved in pairs
        when possible to maintain conversation context.
        
        Args:
            max_length (int): Maximum number of messages to keep. Must be positive.
            
        Raises:
            ValueError: If max_length is not a positive integer.
        """
        if not isinstance(max_length, int) or max_length <= 0:
            raise ValueError("max_length must be a positive integer")
        
        # Temporarily set the max length to perform trimming
        original_max_length = self.__max_history_length
        self.__max_history_length = max_length
        
        try:
            self.__chat_history = self._trim_chat_history(self.__chat_history)
        finally:
            # Restore original max length setting
            self.__max_history_length = original_max_length
    
    def get_history_usage(self) -> dict:
        """
        Get information about chat history memory usage and token estimation.
        
        Returns:
            dict: A dictionary containing:
                - message_count (int): Number of messages in history
                - estimated_tokens (int): Rough estimate of token count
                - memory_bytes (int): Approximate memory usage in bytes
        """
        message_count = len(self.__chat_history)
        
        if message_count == 0:
            return {
                'message_count': 0,
                'estimated_tokens': 0,
                'memory_bytes': 0
            }
        
        # Calculate estimated tokens (rough approximation: 1 token ≈ 4 characters)
        total_chars = sum(len(str(msg.content)) for msg in self.__chat_history)
        estimated_tokens = total_chars // 4
        
        # Calculate approximate memory usage
        import sys
        memory_bytes = sum(sys.getsizeof(msg) + sys.getsizeof(msg.content) 
                          for msg in self.__chat_history)
        
        return {
            'message_count': message_count,
            'estimated_tokens': estimated_tokens,
            'memory_bytes': memory_bytes
        }
