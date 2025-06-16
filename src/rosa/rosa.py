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

import logging
from collections.abc import AsyncIterable
from typing import Any, Dict, Literal, Optional

from langchain_core.language_models.chat_models import BaseChatModel

from .exceptions import ROSAConfigurationError, ROSAError, ROSAExecutionError
from .helpers import ROSAChatManager, ROSAComponentBuilder, ROSAConfigValidator
from .prompts import RobotSystemPrompts

ChatModel = BaseChatModel


class ROSA:
    """ROSA (Robot Operating System Agent) is a class that encapsulates the logic for interacting with ROS systems
    using natural language.

    Args:
        ros_version (Literal[1, 2]): The version of ROS that the agent will interact with.
        llm (BaseChatModel): The language model to use for generating responses.
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
        cleanup(): Manually clean up resources and mark agent as cleaned up.

    Context Manager Support:
        ROSA can be used as a context manager for automatic resource cleanup:

        ```python
        with ROSA(ros_version=2, llm=llm) as agent:
            response = agent.invoke("list topics")
        # Resources automatically cleaned up here
        ```

    Note:
        - The `tools` and `tool_packages` arguments allow for extending the agent's capabilities.
        - Custom `prompts` can be provided to tailor the agent's behavior for specific robots or use cases.
        - Token usage display is automatically disabled when streaming is enabled.
        - Use `invoke()` for non-streaming responses and `astream()` for streaming responses.
        - Chat history trimmed automatically when max_history_length exceeded.
        - After calling `cleanup()` or exiting a context manager, the agent cannot be used for operations.

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
        self._initialize_logging()

        # Validate inputs
        self._validate_inputs(ros_version, llm, max_history_length)

        # Setup configuration
        self._setup_configuration(
            ros_version,
            llm,
            tools,
            tool_packages,
            prompts,
            verbose,
            blacklist,
            accumulate_chat_history,
            show_token_usage,
            streaming,
            max_history_length,
        )

        # Initialize components
        self._initialize_components(verbose)

    def _initialize_logging(self) -> None:
        """Initialize the logger for the ROSA instance.

        Sets up a logger with the format: module.class_name
        """
        self.__logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    def _setup_configuration(
        self,
        ros_version: Literal[1, 2],
        llm: ChatModel,
        tools: Optional[list],
        tool_packages: Optional[list],
        prompts: Optional[RobotSystemPrompts],
        verbose: bool,
        blacklist: Optional[list],
        accumulate_chat_history: bool,
        show_token_usage: bool,
        streaming: bool,
        max_history_length: Optional[int],
    ) -> None:
        """Setup instance configuration variables and helper classes.

        Args:
            ros_version: The ROS version to use.
            llm: The language model to use.
            tools: Additional tools to include.
            tool_packages: Tool packages to include.
            prompts: Custom prompts to use.
            verbose: Whether to enable verbose output.
            blacklist: Tools to exclude.
            accumulate_chat_history: Whether to accumulate chat history.
            show_token_usage: Whether to show token usage.
            streaming: Whether to enable streaming.
            max_history_length: Maximum chat history length.
        """
        # Initialize config validator first
        self.__config_validator = ROSAConfigValidator()

        # Use config validator to validate ALL parameters
        self.__config_validator.validate_all_inputs(
            ros_version, llm, max_history_length
        )
        self.__config_validator.validate_boolean_parameter(verbose, "verbose")
        self.__config_validator.validate_boolean_parameter(
            accumulate_chat_history, "accumulate_chat_history"
        )
        self.__config_validator.validate_boolean_parameter(
            show_token_usage, "show_token_usage"
        )
        self.__config_validator.validate_boolean_parameter(streaming, "streaming")
        self.__config_validator.validate_optional_list_parameter(tools, "tools")
        self.__config_validator.validate_optional_list_parameter(
            tool_packages, "tool_packages"
        )
        self.__config_validator.validate_optional_list_parameter(blacklist, "blacklist")
        self.__config_validator.validate_prompts_parameter(prompts)

        # Initialize instance variables
        self.__ros_version = ros_version
        self.__llm = llm.with_config({"streaming": streaming})
        self.__blacklist = blacklist if blacklist else []
        self.__accumulate_chat_history = accumulate_chat_history
        self.__streaming = streaming
        self.__show_token_usage = show_token_usage if not streaming else False
        self.__max_history_length = max_history_length

        # Store initialization parameters for component creation
        self.__init_tools = tools
        self.__init_tool_packages = tool_packages
        self.__init_prompts = prompts

        # Initialize helper classes (config validator already created above)
        self.__component_builder = ROSAComponentBuilder(
            ros_version=ros_version, llm=self.__llm, streaming=streaming
        )
        self.__chat_manager = ROSAChatManager(
            max_history_length=max_history_length,
            accumulate_chat_history=accumulate_chat_history,
            logger=self.__logger,
        )

        # Resource state tracking
        self._is_initialized = False
        self._is_cleaned_up = False

    def _initialize_components(self, verbose: bool) -> None:
        """Initialize all ROSA components.

        Args:
            verbose: Whether to enable verbose executor output.

        Raises:
            ROSAConfigurationError: If component initialization fails.
        """
        try:
            self.__tools = self.__component_builder.get_tools(
                packages=self.__init_tool_packages,
                tools=self.__init_tools,
                blacklist=self.__blacklist,
            )
            self.__prompts = self.__component_builder.get_prompts(self.__init_prompts)
            self.__llm_with_tools = self.__component_builder.bind_tools_to_llm(
                self.__tools
            )
            self.__agent = self.__component_builder.get_agent(
                self.__prompts, self.__llm_with_tools
            )
            self.__executor = self.__component_builder.get_executor(
                self.__agent, self.__tools, verbose=verbose
            )
            self._is_initialized = True
            self.__logger.info(
                f"ROSA agent initialized with ROS version {self.__ros_version}"
            )
        except Exception as e:
            self.__logger.error(f"Failed to initialize ROSA agent: {e}")
            raise ROSAConfigurationError(f"Failed to initialize ROSA agent: {e}") from e

    def _validate_inputs(
        self,
        ros_version: Literal[1, 2],
        llm: ChatModel,
        max_history_length: Optional[int],
    ) -> None:
        """Validate constructor inputs using the config validator.

        Args:
            ros_version: The ROS version to validate.
            llm: The language model to validate.
            max_history_length: The maximum history length to validate.

        Raises:
            ROSAConfigurationError: If inputs are invalid.
        """
        # Create a temporary validator for input validation
        # (the main validator is created later in _setup_configuration)
        validator = ROSAConfigValidator()
        validator.validate_all_inputs(ros_version, llm, max_history_length)

    @property
    def chat_history(self):
        """Get the chat history."""
        return self.__chat_manager.get_chat_history()

    def clear_chat(self, retain_system_messages: bool = False) -> None:
        """
        Clear the chat history.

        Args:
            retain_system_messages (bool): If True, keep system messages in the history.
                                         If False, clear all messages. Defaults to False.
        """
        self.__chat_manager.clear_chat_history(retain_system_messages)

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
            ROSAError: If the agent has been cleaned up or is not properly initialized.
            ValueError: If the query is empty or invalid.

        Note:
            - This method uses OpenAI's callback to track token usage if enabled.
            - The chat history is updated with the query and response if successful.
            - Token usage is printed if the show_token_usage flag is set.
        """
        self._check_initialized()

        if not query or not query.strip():
            raise ValueError("Query cannot be empty")

        self.__logger.debug(
            f"Processing query: {query[:100]}{'...' if len(query) > 100 else ''}"
        )

        try:
            with self.__component_builder.get_usage_callback() as cb:
                result = self.__executor.invoke(
                    {
                        "input": query,
                        self.MEMORY_KEY: self.__chat_manager.get_chat_history(),
                    }
                )
                self._print_usage(cb)
        except Exception as e:
            self.__logger.error(f"Agent execution failed: {e}", exc_info=True)
            raise ROSAExecutionError(f"Agent execution failed: {e}") from e

        if "output" not in result:
            self.__logger.error("Agent result missing 'output' key")
            raise ROSAExecutionError("Agent result missing 'output' key")

        self.__chat_manager.record_chat_history(query, result["output"])
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
            ROSAError: If the agent has been cleaned up or is not properly initialized.

        Note:
            This method updates the chat history with the final output if successful.
        """
        self._check_initialized()

        if not self.__streaming:
            raise ValueError(
                "Streaming is not enabled. Use 'invoke' method instead or initialize ROSA with streaming=True."
            )

        if not query or not query.strip():
            raise ValueError("Query cannot be empty")

        self.__logger.debug(
            f"Streaming query: {query[:100]}{'...' if len(query) > 100 else ''}"
        )

        try:
            final_output = ""
            # Stream events from the agent's response
            async for event in self.__executor.astream_events(
                input={
                    "input": query,
                    self.MEMORY_KEY: self.__chat_manager.get_chat_history(),
                },
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
                self.__chat_manager.record_chat_history(query, final_output)
                self.__logger.debug("Streaming query processed successfully")
        except Exception as e:
            self.__logger.error(f"Streaming execution failed: {e}", exc_info=True)
            yield {"type": "error", "content": f"An error occurred: {e}"}

    def _print_usage(self, cb) -> None:
        """Print the token usage if show_token_usage is enabled.

        Args:
            cb: The callback object containing usage information.
        """
        if cb and self.__show_token_usage:
            print(f"[bold]Prompt Tokens:[/bold] {cb.prompt_tokens}")
            print(f"[bold]Completion Tokens:[/bold] {cb.completion_tokens}")
            print(f"[bold]Total Cost (USD):[/bold] ${cb.total_cost}")

    def get_history_length(self) -> int:
        """
        Get the current number of messages in chat history.

        Returns:
            int: The number of messages in the chat history.
        """
        return self.__chat_manager.get_history_length()

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
        self.__chat_manager.trim_history(max_length)

    def get_history_usage(self) -> dict:
        """
        Get information about chat history memory usage and token estimation.

        Returns:
            dict: A dictionary containing:
                - message_count (int): Number of messages in history
                - estimated_tokens (int): Rough estimate of token count
                - memory_bytes (int): Approximate memory usage in bytes
        """
        return self.__chat_manager.get_history_usage()

    def _check_initialized(self) -> None:
        """Check if the agent is properly initialized and not cleaned up.

        Raises:
            ROSAError: If the agent has been cleaned up or is not initialized.
        """
        if self._is_cleaned_up:
            raise ROSAError("Agent has been cleaned up")
        if not self._is_initialized:
            raise ROSAError("Agent not properly initialized")

    def cleanup(self) -> None:
        """Clean up resources and connections.

        This method is idempotent and can be called multiple times safely.
        After calling cleanup(), the agent should not be used for any operations.

        Currently performs lightweight cleanup focused on state management.
        Future versions may add cleanup for persistent connections, threads, etc.
        """
        if self._is_cleaned_up:
            return

        self.__logger.info("Cleaning up ROSA agent resources")

        # Call cleanup methods in order: tools -> llm -> executor
        # Each method handles its own exceptions to ensure cleanup continues
        try:
            self._cleanup_tools()
        except Exception as e:
            self.__logger.warning(f"Error during tools cleanup: {e}")

        try:
            self._cleanup_llm()
        except Exception as e:
            self.__logger.warning(f"Error during LLM cleanup: {e}")

        try:
            self._cleanup_executor()
        except Exception as e:
            self.__logger.warning(f"Error during executor cleanup: {e}")

        self._is_cleaned_up = True
        self.__logger.info("ROSA agent cleanup completed")

    def _cleanup_executor(self) -> None:
        """Clean up executor resources.

        Currently performs minimal cleanup as AgentExecutor doesn't require
        explicit resource cleanup. Future versions may add cleanup for
        persistent connections or background tasks.
        """
        # Future: Add cleanup for executor-specific resources

    def _cleanup_llm(self) -> None:
        """Clean up LLM resources.

        Currently performs minimal cleanup as most LLM providers handle
        cleanup automatically. Future versions may add cleanup for
        persistent connections, custom clients, etc.
        """
        # Future: Add cleanup for LLM-specific resources like:
        # - Persistent HTTP connections
        # - Custom client connections
        # - Background token refresh threads

    def _cleanup_tools(self) -> None:
        """Clean up tool resources.

        Currently performs minimal cleanup as ROS tools are stateless.
        Future versions may add cleanup for persistent ROS connections,
        subscribers, publishers, service clients, etc.
        """
        # Future: Add cleanup for tool-specific resources like:
        # - Persistent ROS node connections
        # - Active subscribers/publishers
        # - Service clients
        # - Action clients

    def __enter__(self):
        """Enter the runtime context for context manager support.

        Returns:
            ROSA: The ROSA instance.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Exit the runtime context and clean up resources.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_val: Exception value if an exception occurred.
            exc_tb: Exception traceback if an exception occurred.

        Returns:
            None: Does not suppress exceptions.
        """
        try:
            self.cleanup()
        except Exception as e:
            self.__logger.warning(f"Error during context manager cleanup: {e}")
        # Return None to not suppress any exceptions that occurred in the with block
