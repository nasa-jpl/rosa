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

import os
from langchain.agents import AgentExecutor
from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.prompts import MessagesPlaceholder
from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import AzureChatOpenAI, ChatOpenAI
from langchain_community.callbacks import get_openai_callback
from rich import print
from typing import Literal, Union, Optional

try:
    from .prompts import system_prompts, RobotSystemPrompts
    from .tools import ROSATools
except ImportError:
    from prompts import system_prompts, RobotSystemPrompts
    from tools import ROSATools


class ROSA:
    """ ROSA (Robot OpenAI System Agent) is a class that encapsulates the logic for interacting with ROS systems
    using natural language.

    Args:
        ros_version: The version of ROS that the agent will interact with. This can be either 1 or 2.
        llm: The language model to use for generating responses. This can be either an instance of AzureChatOpenAI or ChatOpenAI.
        tools: A list of LangChain tool functions to use with the agent.
        tool_packages: A list of Python packages that contain LangChain tool functions to use with the agent.
        prompts: A list of prompts to use with the agent. This can be a list of prompts from the RobotSystemPrompts class.
        verbose: A boolean flag that indicates whether to print verbose output.
        blacklist: A list of ROS tools to exclude from the agent. This can be a list of ROS tools from the ROSATools class.
        accumulate_chat_history: A boolean flag that indicates whether to accumulate chat history.
        show_token_usage: A boolean flag that indicates whether to show token usage after each invocation.
    """

    def __init__(
            self,
            ros_version: Literal[1, 2],
            llm: Union[AzureChatOpenAI, ChatOpenAI],
            tools: Optional[list] = None,
            tool_packages: Optional[list] = None,
            prompts: Optional[RobotSystemPrompts] = None,
            verbose: bool = False,
            blacklist: Optional[list] = None,
            accumulate_chat_history: bool = True,
            show_token_usage: bool = False
    ):
        self.__chat_history = []
        self.__ros_version = ros_version
        self.__llm = llm
        self.__memory_key = "chat_history"
        self.__scratchpad = "agent_scratchpad"
        self.__show_token_usage = show_token_usage
        self.__blacklist = blacklist if blacklist else []
        self.__accumulate_chat_history = accumulate_chat_history
        self.__tools = self.__get_tools(ros_version, packages=tool_packages, tools=tools, blacklist=self.__blacklist)
        self.__prompts = self.__get_prompts(prompts)
        self.__llm_with_tools = llm.bind_tools(self.__tools.get_tools())
        self.__agent = self.__get_agent()
        self.__executor = self.__get_executor(verbose=verbose)

    def clear_chat(self):
        """Clear the chat history."""
        self.__chat_history = []
        os.system("clear")

    def invoke(self, query: str) -> str:
        """Invoke the agent with a user query."""
        try:
            with get_openai_callback() as cb:
                result = self.__executor.invoke({"input": query, "chat_history": self.__chat_history})
                if self.__show_token_usage:
                    print(f"[bold]Prompt Tokens:[/bold] {cb.prompt_tokens}")
                    print(f"[bold]Completion Tokens:[/bold] {cb.completion_tokens}")
                    print(f"[bold]Total Cost (USD):[/bold] ${cb.total_cost}")
        except Exception as e:
            if f"{e}".strip() == "":
                self.__record_chat_history(
                    query,
                    "An error with no description occurred. This is known to happen when multiple tools are used "
                    "concurrently. Please try again."
                )
                try:
                    result = self.__executor.invoke(
                        {
                            "input": "Please try again.",
                            "chat_history": self.__chat_history
                        }
                    )
                except Exception as e:
                    return "An error with no description occurred. This is known to happen when multiple tools are used concurrently. Please try again."
            else:
                return f"An error occurred: {e}"

        self.__record_chat_history(query, result["output"])
        return result["output"]

    def __get_executor(self, verbose: bool):
        executor = AgentExecutor(agent=self.__agent, tools=self.__tools.get_tools(), stream_runnable=False,
                                 verbose=verbose)
        return executor

    def __get_agent(self):
        agent = ({
                     "input": lambda x: x["input"],
                     "agent_scratchpad": lambda x: format_to_openai_tool_messages(x["intermediate_steps"]),
                     "chat_history": lambda x: x["chat_history"],
                 } | self.__prompts | self.__llm_with_tools | OpenAIToolsAgentOutputParser())
        return agent

    def __get_tools(self, ros_version: Literal[1, 2], packages: Optional[list], tools: Optional[list], blacklist: Optional[list]):
        rosa_tools = ROSATools(ros_version, blacklist=blacklist)
        if tools:
            rosa_tools.add_tools(tools)
        if packages:
            rosa_tools.add_packages(packages, blacklist=blacklist)
        return rosa_tools

    def __get_prompts(self, robot_prompts: Optional[RobotSystemPrompts] = None):
        prompts = system_prompts
        if robot_prompts:
            prompts.append(robot_prompts.as_message())
        template = ChatPromptTemplate.from_messages(
            prompts + [
                MessagesPlaceholder(variable_name=self.__memory_key),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name=self.__scratchpad)
            ]
        )
        return template

    def __record_chat_history(self, query: str, response: str):
        if self.__accumulate_chat_history:
            self.__chat_history.extend([
                HumanMessage(content=query),
                AIMessage(content=response)
            ])
