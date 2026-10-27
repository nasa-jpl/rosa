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
import re
import shlex
import subprocess
import time
from typing import List, Optional, Tuple, Union

from langchain.agents import tool
from rclpy.logging import get_logging_directory


def execute_ros_command(command: Union[str, List[str]]) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    The command is executed directly (never through a shell), so shell metacharacters in any
    argument are passed to ros2 verbatim rather than being interpreted.

    :param command: The ROS2 command to execute, either as a list of arguments or as a string
                    that will be split using shell-like syntax.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    try:
        cmd = shlex.split(command) if isinstance(command, str) else list(command)
    except ValueError as e:
        raise ValueError(f"'{command}' is not a valid ROS2 command: {e}")

    valid_ros2_commands = ["node", "topic", "service", "param", "doctor"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")
    if not all(isinstance(arg, str) for arg in cmd):
        raise ValueError(f"'{command}' is not a valid ROS2 command.")

    try:
        output = subprocess.check_output(cmd, shell=False).decode()
        return True, output
    except Exception as e:
        return False, str(e)


def validate_ros_name(name: str, kind: str = "name") -> None:
    """
    Ensure a ROS2 entity name cannot be interpreted as a command-line option.

    :param name: The name to validate (node, topic, service, parameter, type, etc.).
    :param kind: A human-readable description of the name, used in the error message.
    """
    if not isinstance(name, str) or not name.strip() or name.startswith("-"):
        raise ValueError(f"Invalid ROS2 {kind}: '{name}'.")


def get_entities(
    cmd: str,
    delimiter: str = "\n",
    pattern: str = None,
    blacklist: Optional[List[str]] = None,
) -> List[str]:
    """
    Get a list of ROS2 entities (nodes, topics, services, etc.).

    :param cmd: the ROS2 command to execute.
    :param delimiter: The delimiter to split the output by.
    :param pattern: A regular expression pattern to filter the list of entities.
    :return:
    """
    success, output = execute_ros_command(cmd)

    if not success:
        return [output]

    entities = output.split(delimiter)

    # Filter out blacklisted entities
    if blacklist:
        entities = list(
            filter(
                lambda x: not any(
                    re.match(f".*{pattern}.*", x) for pattern in blacklist
                ),
                entities,
            )
        )

    if pattern:
        entities = list(filter(lambda x: re.match(f".*{pattern}.*", x), entities))

    entities = [e for e in entities if e.strip() != ""]

    return entities


@tool
def ros2_node_list(pattern: Optional[str] = None, blacklist: Optional[List[str]] = None) -> dict:
    """
    Get a list of ROS2 nodes running on the system.

    :param pattern: A regular expression pattern to filter the list of nodes.
    """
    cmd = "ros2 node list"
    nodes = get_entities(cmd, pattern=pattern, blacklist=blacklist)
    return {"nodes": nodes}


@tool
def ros2_topic_list(pattern: Optional[str] = None, blacklist: Optional[List[str]] = None) -> dict:
    """
    Get a list of ROS2 topics.

    :param pattern: A regular expression pattern to filter the list of topics.
    """
    cmd = "ros2 topic list"
    topics = get_entities(cmd, pattern=pattern, blacklist=blacklist)
    return {"topics": topics}


@tool
def ros2_topic_echo(
    topic: str,
    count: int = 1,
    return_echoes: bool = False,
    delay: float = 1.0,
    timeout: float = 1.0,
) -> dict:
    """
    Echoes the contents of a specific ROS2 topic.

    :param topic: The name of the ROS topic to echo.
    :param count: The number of messages to echo. Valid range is 1-10.
    :param return_echoes: If True, return the messages as a list with the response.
    :param delay: Time to wait between each message in seconds.
    :param timeout: Max time to wait for a message before timing out.

    :note: Do not set return_echoes to True if the number of messages is large.
           This will cause the response to be too large and may cause the tool to fail.
    """
    if count < 1 or count > 10:
        return {"error": "Count must be between 1 and 10."}

    try:
        validate_ros_name(topic, "topic")
        timeout = float(timeout)
    except (TypeError, ValueError) as e:
        return {"error": str(e)}

    cmd = ["ros2", "topic", "echo", topic, "--once", "--spin-time", str(timeout)]

    echoes = []
    for i in range(count):
        success, output = execute_ros_command(cmd)

        if not success:
            return {"error": output}

        print(output)
        if return_echoes:
            echoes.append(output)

        time.sleep(delay)

    if return_echoes:
        return {"echoes": echoes}

    return {"success": True}


@tool
def ros2_service_list(
    pattern: Optional[str] = None, blacklist: Optional[List[str]] = None
) -> dict:
    """
    Get a list of ROS2 services.

    :param pattern: A regular expression pattern to filter the list of services.
    """
    cmd = "ros2 service list"
    services = get_entities(cmd, pattern=pattern, blacklist=blacklist)
    return {"services": services}


@tool
def ros2_node_info(nodes: List[str]) -> dict:
    """
    Get information about a ROS2 node.

    :param node_name: The name of the ROS2 node.
    """
    data = {}

    for node_name in nodes:

        try:
            validate_ros_name(node_name, "node name")
        except ValueError as e:
            data[node_name] = dict(error=str(e))
            continue

        cmd = ["ros2", "node", "info", node_name]
        success, output = execute_ros_command(cmd)
        if not success:
            data[node_name] = dict(error=output)
            continue
        data[node_name] = output

    return data


@tool
def ros2_topic_info(topics: List[str]) -> dict:
    """
    Get information about a ROS2 topic.

    :param topic_name: The name of the ROS2 topic.
    """
    data = {}

    for topic in topics:
        try:
            validate_ros_name(topic, "topic")
        except ValueError as e:
            data[topic] = dict(error=str(e))
            continue

        cmd = ["ros2", "topic", "info", topic, "--verbose"]
        success, output = execute_ros_command(cmd)
        if not success:
            topic_info = dict(error=output)
        else:
            topic_info = output

        data[topic] = topic_info

    return data


@tool
def ros2_param_list(
    node_name: Optional[str] = None,
    pattern: str = None,
    blacklist: Optional[List[str]] = None,
) -> dict:
    """
    Get a list of parameters for a ROS2 node.

    :param node_name: An optional ROS2 node name to get parameters for. If not provided, all parameters are listed.
    :param pattern: A regular expression pattern to filter the list of parameters.
    """
    if node_name:
        try:
            validate_ros_name(node_name, "node name")
        except ValueError as e:
            return {"error": str(e)}

        cmd = ["ros2", "param", "list", node_name]
        success, output = execute_ros_command(cmd)
        if not success:
            return {"error": output}

        params = [o for o in output.split("\n") if o]
        if pattern:
            params = [p for p in params if re.match(f".*{pattern}.*", p)]
        if blacklist:
            params = [
                p for p in params if not any(re.match(f".*{b}.*", p) for b in blacklist)
            ]
        return {node_name: params}
    else:
        cmd = f"ros2 param list"
        success, output = execute_ros_command(cmd)

        if not success:
            return {"error": output}

        # When we get a list of all nodes params, we have to parse it
        # The node name starts with a '/' and the params are indented
        lines = output.split("\n")
        data = {}
        current_node = None
        for line in lines:
            if line.startswith("/"):
                current_node = line
                data[current_node] = []
            elif line.strip() != "":
                data[current_node].append(line.strip())

        if pattern:
            data = {k: v for k, v in data.items() if re.match(f".*{pattern}.*", k)}
        if blacklist:
            data = {
                k: v
                for k, v in data.items()
                if not any(re.match(f".*{b}.*", k) for b in blacklist)
            }
        return data


@tool
def ros2_param_get(node_name: str, param_name: str) -> dict:
    """
    Get the value of a parameter for a ROS2 node.

    :param node_name: The name of the ROS2 node.
    :param param_name: The name of the parameter.
    """
    try:
        validate_ros_name(node_name, "node name")
        validate_ros_name(param_name, "parameter name")
    except ValueError as e:
        return {"error": str(e)}

    cmd = ["ros2", "param", "get", node_name, param_name]
    success, output = execute_ros_command(cmd)

    if not success:
        return {"error": output}

    return {param_name: output}


@tool
def ros2_param_set(node_name: str, param_name: str, param_value: str) -> dict:
    """
    Set the value of a parameter for a ROS2 node.

    :param node_name: The name of the ROS2 node.
    :param param_name: The name of the parameter.
    :param param_value: The value to set the parameter to.
    """
    try:
        validate_ros_name(node_name, "node name")
        validate_ros_name(param_name, "parameter name")
    except ValueError as e:
        return {"error": str(e)}

    cmd = ["ros2", "param", "set", node_name, param_name, str(param_value)]
    success, output = execute_ros_command(cmd)

    if not success:
        return {"error": output}

    return {param_name: output}


@tool
def ros2_service_info(services: List[str]) -> dict:
    """
    Get information about a ROS2 service.

    :param services: a list of ROS2 service names.
    """
    data = {}

    for service_name in services:
        try:
            validate_ros_name(service_name, "service name")
        except ValueError as e:
            data[service_name] = dict(error=str(e))
            continue

        cmd = ["ros2", "service", "type", service_name]
        success, output = execute_ros_command(cmd)

        if not success:
            data[service_name] = dict(error=output)
            continue

        data[service_name] = output

    return data


@tool
def ros2_service_call(service_name: str, srv_type: str, request: str) -> dict:
    """
    Call a ROS2 service.

    :param service_name: The name of the ROS2 service.
    :param srv_type: The type of the service (use ros2_service_info to verify).
    :param request: The request to send to the service.
    """
    try:
        validate_ros_name(service_name, "service name")
        validate_ros_name(srv_type, "service type")
    except ValueError as e:
        return {"error": str(e)}

    cmd = ["ros2", "service", "call", service_name, srv_type, str(request)]
    success, output = execute_ros_command(cmd)
    if not success:
        return {"error": output}
    return {"response": output}


@tool
def ros2_doctor() -> dict:
    """
    Check ROS setup and other potential issues.
    """
    cmd = "ros2 doctor"
    success, output = execute_ros_command(cmd)
    if not success:
        return {"error": output}
    return {"results": output}


def ros2_log_directories():
    """Get any available ROS2 log directories."""
    log_dir = get_logging_directory()
    print(f"ROS 2 logs are stored in: {log_dir}")

    return {"default": f"{log_dir}"}


@tool
def roslog_list(min_size: int = 2048, blacklist: Optional[List[str]] = None) -> dict:
    """
    Returns a list of ROS log files.

    :param min_size: The minimum size of the log file in bytes to include in the list.
    """

    logs = []
    log_dirs = ros2_log_directories()

    for _, log_dir in log_dirs.items():
        if not log_dir:
            continue

        # Get all .log files in the directory
        log_files = [
            os.path.join(log_dir, f)
            for f in os.listdir(log_dir)
            if os.path.isfile(os.path.join(log_dir, f)) and f.endswith(".log")
        ]

        print(f"Log files: {log_files}")

        # Filter out blacklisted files
        if blacklist:
            log_files = list(
                filter(
                    lambda x: not any(
                        re.match(f".*{pattern}.*", x) for pattern in blacklist
                    ),
                    log_files,
                )
            )

        # Filter out files that are too small
        log_files = list(filter(lambda x: os.path.getsize(x) > min_size, log_files))

        # Get the size of each log file in KB or MB if it's larger than 1 MB
        log_files = [
            {
                f.replace(log_dir, ""): (
                    f"{round(os.path.getsize(f) / 1024, 2)} KB"
                    if os.path.getsize(f) < 1024 * 1024
                    else f"{round(os.path.getsize(f) / (1024 * 1024), 2)} MB"
                ),
            }
            for f in log_files
        ]

        if len(log_files) > 0:
            logs.append(
                {
                    "directory": log_dir,
                    "total": len(log_files),
                    "files": log_files,
                }
            )

    return dict(
        total=len(logs),
        logs=logs,
    )
