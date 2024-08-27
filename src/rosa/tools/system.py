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

import time

from langchain.agents import tool
from langchain.globals import set_debug, set_verbose


@tool
def set_verbosity(enable_verbose_messages: bool) -> str:
    """Sets the verbosity of the agent to enable or disable verbose messages.
    Set this to true to provide more detailed output for the user.

    :arg enable_verbose_messages: A boolean value to enable or disable verbose messages.
    """
    global VERBOSE
    VERBOSE = enable_verbose_messages
    set_verbose(VERBOSE)
    return f"Verbose messages are now {'enabled' if VERBOSE else 'disabled'}."


@tool
def set_debugging(enable_debug_messages: bool) -> str:
    """Sets the debug mode of the agent to enable or disable debug messages.
    Set this to true to provide debug output for the user. Debug output
    includes information about API calls, tool execution, and other.

    :arg enable_debug_messages: A boolean value to enable or disable debug messages.
    """
    global DEBUG
    DEBUG = enable_debug_messages
    set_debug(DEBUG)
    return f"Debug messages are now {'enabled' if DEBUG else 'disabled'}."


@tool
def wait(seconds: int) -> str:
    """Waits for the specified number of seconds before continuing.

    :arg seconds: The number of seconds to wait.
    """
    start = time.time()
    time.sleep(seconds)
    end = time.time()
    return f"Waited exactly {end - start} seconds."
