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


import inspect
from typing import Literal, List, Optional
from langchain.agents import Tool
from functools import wraps


def inject_blacklist(blacklist):
    """
    Inject a blacklist parameter into @tool functions that require it. Required because we do not
    want to rely on the LLM to manually use the blacklist, as it may "forget" to do so.

    Wraps all @tool functions with a new function that injects the blacklist parameter if it is not already present.
    Ensures the function signature and parameter list are maintained, such that LangChain can properly execute the
    function. Without this, LangChain would throw an error. Tried to use partial functions, but it did not work.
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if 'blacklist' in kwargs:
                kwargs['blacklist'] = blacklist
            else:
                params = inspect.signature(func).parameters
                if 'blacklist' in params:
                    kwargs['blacklist'] = blacklist
            return func(*args, **kwargs)

        # Rebuild the signature to include 'blacklist'
        sig = inspect.signature(func)
        new_params = [
            param.replace(default=blacklist) if param.name == 'blacklist' else param
            for param in sig.parameters.values()
        ]
        wrapper.__signature__ = sig.replace(parameters=new_params)
        return wrapper
    return decorator


class ROSATools:
    def __init__(self, ros_version: Literal[1, 2], blacklist: Optional[List[str]] = None):
        self.__tools: list = []
        self.__ros_version = ros_version

        # Add the default tools
        from . import calculation, log, ros1, ros2, system
        self.__iterative_add(calculation)
        self.__iterative_add(log)
        self.__iterative_add(system)

        if self.__ros_version == 1:
            try:
                from . import ros1
                self.__iterative_add(ros1, blacklist=blacklist)
            except Exception as e:
                print(e)
        elif self.__ros_version == 2:
            try:
                from . import ros2
                self.__iterative_add(ros2, blacklist=blacklist)
            except Exception as e:
                print(e)
        else:
            raise ValueError("Invalid ROS version. Must be either 1 or 2.")

    def get_tools(self) -> List[Tool]:
        return self.__tools

    def __iterative_add(self, package, blacklist: Optional[List[str]] = None):
        """
        Iterate through a package and add each @tool to the tools list.

        :param package: The package to iterate through.
        :param blacklist: A parameter used by some tools to filter out certain results.
        """
        for tool_name in dir(package):
            if not tool_name.startswith("_"):
                t = getattr(package, tool_name)
                if hasattr(t, 'name') and hasattr(t, 'func'):
                    if blacklist and 'blacklist' in t.func.__code__.co_varnames:
                        # Inject the blacklist into the tool function
                        t.func = inject_blacklist(blacklist)(t.func)
                    self.__tools.append(t)

    def add(self, tool_packages: List, blacklist: Optional[List[str]] = None):
        """
        Add a list of tools to the Tools object.

        :param tool_packages: A list of tool packages to add to the Tools object.
        """
        for pkg in tool_packages:
            self.__iterative_add(pkg, blacklist=blacklist)
