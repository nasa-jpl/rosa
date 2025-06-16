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

"""Component builder for ROSA agent initialization."""

from contextlib import contextmanager
from typing import List, Literal, Optional

from langchain.agents import AgentExecutor
from langchain.agents.format_scratchpad.openai_tools import (
    format_to_openai_tool_messages,
)
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.prompts import MessagesPlaceholder
from langchain_community.callbacks import get_openai_callback
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import Runnable
from langchain_openai import AzureChatOpenAI, ChatOpenAI

from ..prompts import RobotSystemPrompts, system_prompts
from ..tools import ROSATools


class ROSAComponentBuilder:
    """Helper class for building ROSA components (agent, executor, tools, etc.).

    This class encapsulates the logic for creating and configuring all the
    components needed for a ROSA agent instance. It separates component
    creation concerns from the main ROSA class.

    Args:
        ros_version: The ROS version (1 or 2) for tool compatibility.
        llm: The language model to use for the agent.
        streaming: Whether streaming is enabled for the LLM.

    Attributes:
        MEMORY_KEY: Key used for chat history in prompts.
        SCRATCHPAD_KEY: Key used for agent scratchpad in prompts.
    """

    # Class constants
    MEMORY_KEY = "chat_history"
    SCRATCHPAD_KEY = "agent_scratchpad"

    def __init__(self, ros_version: Literal[1, 2], llm: BaseChatModel, streaming: bool):
        """Initialize the component builder.

        Args:
            ros_version: The ROS version to use for tools.
            llm: The language model instance.
            streaming: Whether streaming is enabled.
        """
        self._ros_version = ros_version
        self._llm = llm
        self._streaming = streaming

    def get_tools(
        self,
        packages: Optional[List[str]] = None,
        tools: Optional[List] = None,
        blacklist: Optional[List[str]] = None,
    ) -> ROSATools:
        """Create a ROSA tools object with the specified configuration.

        Args:
            packages: Python packages containing LangChain tool functions.
            tools: Additional LangChain tool functions to include.
            blacklist: ROS tools to exclude from the agent.

        Returns:
            Configured ROSATools instance.
        """
        rosa_tools = ROSATools(self._ros_version, blacklist=blacklist)
        if tools:
            rosa_tools.add_tools(tools)
        if packages:
            rosa_tools.add_packages(packages, blacklist=blacklist)
        return rosa_tools

    def get_prompts(
        self, robot_prompts: Optional[RobotSystemPrompts] = None
    ) -> ChatPromptTemplate:
        """Create a chat prompt template from system and robot-specific prompts.

        Args:
            robot_prompts: Optional robot-specific prompts to include.

        Returns:
            Configured chat prompt template.
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

    def get_agent(
        self, prompts: ChatPromptTemplate, llm_with_tools: BaseChatModel
    ) -> Runnable:
        """Create an agent runnable for processing inputs and generating responses.

        Args:
            prompts: The chat prompt template to use.
            llm_with_tools: The language model with tools bound.

        Returns:
            Configured agent runnable chain.
        """
        agent = (
            {
                "input": lambda x: x["input"],
                self.SCRATCHPAD_KEY: lambda x: format_to_openai_tool_messages(
                    x["intermediate_steps"]
                ),
                self.MEMORY_KEY: lambda x: x[self.MEMORY_KEY],
            }
            | prompts
            | llm_with_tools
            | OpenAIToolsAgentOutputParser()
        )
        return agent

    def get_executor(
        self, agent: Runnable, tools: ROSATools, verbose: bool
    ) -> AgentExecutor:
        """Create an executor for processing user inputs and generating responses.

        Args:
            agent: The agent runnable to execute.
            tools: The tools available to the agent.
            verbose: Whether to enable verbose output.

        Returns:
            Configured agent executor.
        """
        executor = AgentExecutor(
            agent=agent,
            tools=tools.get_tools(),
            stream_runnable=self._streaming,
            verbose=verbose,
            max_iterations=75,  # Allow complex multi-step tasks while preventing infinite loops
            max_execution_time=300,  # 5 minutes timeout
        )
        return executor

    def get_usage_callback(self):
        """Get the appropriate token usage callback based on LLM type.

        Returns:
            A context manager for tracking token usage, or a null callback
            for unsupported LLM types.
        """
        # Get the underlying LLM instance (handles RunnableBinding wrapper)
        llm_instance = self._llm
        if hasattr(llm_instance, "bound") and hasattr(llm_instance.bound, "__class__"):
            llm_instance = llm_instance.bound

        # Check if this is an OpenAI-compatible model
        if isinstance(llm_instance, (ChatOpenAI, AzureChatOpenAI)):
            return get_openai_callback()
        else:
            # For non-OpenAI models (Ollama, Anthropic, etc.), return a null callback
            return self.get_null_callback()

    def get_null_callback(self):
        """Get a null callback that doesn't track usage for non-OpenAI models.

        Returns:
            A context manager that provides a dummy callback object.
        """

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

    def bind_tools_to_llm(self, tools: ROSATools) -> BaseChatModel:
        """Bind tools to the language model.

        Args:
            tools: The tools to bind to the LLM.

        Returns:
            The LLM with tools bound.
        """
        return self._llm.bind_tools(tools.get_tools())
