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

import inspect
from functools import wraps
from typing import Literal, Optional

from langchain.agents import Tool


def inject_blacklist(default_blacklist: list[str]):
    """
    Creates a decorator that automatically injects blacklist filtering into ROS tool functions.

    This decorator solves a critical problem: ensuring ROS tools consistently filter out unwanted
    entities (topics, nodes, services) without relying on the LLM to remember to pass the blacklist
    parameter correctly every time.

    **Why This Exists:**
    - LLMs can "forget" to pass blacklist parameters, leading to unwanted/sensitive data exposure
    - Manual blacklist handling is error-prone and inconsistent across tools
    - Provides automatic concatenation of system-level + user-provided blacklists

    **How It Works:**
    1. Wraps target @tool functions with a proxy that intercepts all calls
    2. Automatically detects if the function accepts a 'blacklist' parameter
    3. Injects/merges default_blacklist with any user-provided blacklist
    4. Preserves original function signature for LangChain compatibility

    **Parameter Injection Patterns Handled:**
    - Dict args: func({"param": "value", "blacklist": [...]}) → modifies dict in-place
    - Keyword args: func(param="value", blacklist=[...]) → merges with defaults
    - Missing blacklist: func(param="value") → injects defaults if signature has blacklist param

    **Signature Preservation:**
    The decorator rebuilds the function signature to ensure LangChain can properly
    inspect parameters and call the function. This is critical for LangChain's
    tool execution framework.

    Args:
        default_blacklist: List of regex patterns to automatically filter from all tool results

    Returns:
        Decorator function that wraps target functions with blacklist injection logic

    Example:
        @inject_blacklist(["system", "debug"])
        def my_tool(param1: str, blacklist: List[str] = None):
            # blacklist will always contain at least ["system", "debug"]
            # plus any user-provided patterns
            return filter_results(blacklist)
    """

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            # PATTERN 1: Dict-based arguments (common with LangChain tool calls)
            if args and isinstance(args[0], dict):
                if "blacklist" in args[0]:
                    # Merge: default blacklist + user blacklist
                    args[0]["blacklist"] = default_blacklist + args[0]["blacklist"]
                else:
                    # Inject: only default blacklist
                    args[0]["blacklist"] = default_blacklist

            # PATTERN 2: Keyword arguments with existing blacklist
            elif "blacklist" in kwargs:
                # Merge: default blacklist + user blacklist
                kwargs["blacklist"] = default_blacklist + kwargs["blacklist"]

            # PATTERN 3: No blacklist provided but function accepts one
            # Example: tool_func(param1="value") where signature has blacklist parameter
            else:
                params = inspect.signature(func).parameters
                if "blacklist" in params:
                    # Inject: only default blacklist
                    kwargs["blacklist"] = default_blacklist

            return func(*args, **kwargs)

        # CRITICAL: Rebuild function signature for LangChain compatibility
        # LangChain inspects function signatures to understand parameter types and defaults.
        # We must update the blacklist parameter's default value to reflect our injection.
        sig = inspect.signature(func)
        new_params = [
            (
                param.replace(default=default_blacklist)
                if param.name == "blacklist"
                else param
            )
            for param in sig.parameters.values()
        ]
        wrapper.__signature__ = sig.replace(parameters=new_params)
        return wrapper

    return decorator


class ROSATools:
    """
    Manages discovery, loading, and automatic blacklist injection for ROSA tool functions.

    This class serves as the central orchestrator for tool management, automatically:
    1. Discovering @tool decorated functions from specified packages
    2. Detecting which tools accept blacklist parameters
    3. Applying blacklist injection to appropriate tools
    4. Managing the final tool collection for LangChain agent use

    **Blacklist Injection Strategy:**
    - Tools with 'blacklist' parameter in their signature get automatic injection
    - Tools without 'blacklist' parameter remain unmodified
    - Only applied when ROSATools is initialized with a blacklist

    **Tool Loading Order:**
    1. Core tools: calculation, log, system (no blacklist injection needed)
    2. ROS-specific tools: ros1 OR ros2 (blacklist injection applied)

    Args:
        ros_version: ROS version (1 or 2) to determine which ROS tools to load
        blacklist: Optional list of regex patterns to automatically inject into ROS tools
    """

    def __init__(
        self, ros_version: Literal[1, 2], blacklist: Optional[list[str]] = None
    ):
        self.__tools: list = []
        self.__ros_version = ros_version
        self.__blacklist = blacklist

        # Load core tools (no blacklist injection needed - they don't filter ROS entities)
        from . import calculation, log, system

        self.__iterative_add(calculation)
        self.__iterative_add(log)
        self.__iterative_add(system)

        # Load ROS-specific tools (blacklist injection applied automatically)
        ros_version_1 = 1
        ros_version_2 = 2

        if self.__ros_version == ros_version_1:
            from . import ros1

            self.__iterative_add(ros1)
        elif self.__ros_version == ros_version_2:
            from . import ros2

            self.__iterative_add(ros2)
        else:
            msg = "Invalid ROS version. Must be either 1 or 2."
            raise ValueError(msg)

    def get_tools(self) -> list[Tool]:
        return self.__tools

    def __add_tool(self, tool):
        """
        Adds a single tool to the collection, applying blacklist injection if appropriate.

        **Blacklist Injection Decision Logic:**
        1. Check if we have a default blacklist configured
        2. Check if the tool's function accepts a 'blacklist' parameter
        3. If both true → wrap function with blacklist injection decorator
        4. If either false → add tool as-is without modification

        This ensures only relevant tools get blacklist functionality, avoiding
        unnecessary overhead for tools that don't need filtering.
        """
        if hasattr(tool, "name") and hasattr(tool, "func"):
            # Apply blacklist injection only if:
            # 1. We have a default blacklist configured AND
            # 2. The tool function accepts a blacklist parameter
            if self.__blacklist and "blacklist" in tool.func.__code__.co_varnames:
                # Wrap the tool's function with automatic blacklist injection
                tool.func = inject_blacklist(self.__blacklist)(tool.func)
            self.__tools.append(tool)

    def __iterative_add(self, package):
        """
        Discovers and adds all @tool decorated functions from a Python package.

        **Discovery Process:**
        1. Inspect all public attributes of the package (non-underscore prefixed)
        2. Extract each attribute and check if it's a valid LangChain tool
        3. Pass each discovered tool to __add_tool() for potential blacklist injection

        **Note:** The blacklist parameter is kept for API compatibility but is not used.
        Blacklist injection is handled automatically by __add_tool() based on the
        ROSATools instance's configured blacklist.

        Args:
            package: Python package/module containing @tool decorated functions
            blacklist: Legacy parameter, not used (kept for compatibility)
        """
        for tool_name in dir(package):
            # Skip private/internal attributes (Python convention)
            if not tool_name.startswith("_"):
                tool_candidate = getattr(package, tool_name)
                self.__add_tool(tool_candidate)

    def add_packages(self, tool_packages: list):
        """
        Add a list of tools to the Tools object by iterating through each package.

        :param tool_packages: A list of tool packages to add to the Tools object.
        """
        for pkg in tool_packages:
            self.__iterative_add(pkg)

    def add_tools(self, tools: list):
        """
        Add a single tool to the Tools object.

        :param tools: A list of tools to add
        """
        for tool in tools:
            self.__add_tool(tool)
