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
import subprocess
from typing import List, Optional

from langchain.agents import tool


def execute_ros_command(command: str, regex_pattern: str = None) -> str:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: The output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")
    valid_ros2_commands = ["node", "topic", "service", "param", "doctor"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
    except Exception as e:
        return f"Error executing command '{command}': {e}"

    if regex_pattern:
        output = subprocess.check_output(
            f"echo '{output}' | grep -E '{regex_pattern}'", shell=True
        ).decode()

    return output


@tool
def ros2_node_list(
    regex_pattern: str = None, blacklist: Optional[List[str]] = None
) -> List[str]:
    """
    Get a list of ROS2 nodes running on the system.

    :param regex_pattern: A regular expression pattern to filter the list of nodes.
    """
    cmd = "ros2 node list"
    output = execute_ros_command(cmd, regex_pattern)
    nodes = output.split("\n")
    return [node for node in nodes if node]


@tool
def ros2_topic_list(regex_pattern: str = None) -> List[str]:
    """
    Get a list of ROS2 topics.

    :param regex_pattern: A regular expression pattern to filter the list of topics.
    """
    cmd = "ros2 topic list"
    output = execute_ros_command(cmd, regex_pattern)
    topics = output.split("\n")
    return [topic for topic in topics if topic]


@tool
def ros2_service_list(regex_pattern: str = None) -> List[str]:
    """
    Get a list of ROS2 services.

    :param regex_pattern: A regular expression pattern to filter the list of services.
    """
    cmd = "ros2 service list"
    output = execute_ros_command(cmd, regex_pattern)
    services = output.split("\n")
    return [service for service in services if service]


@tool
def ros2_node_info(nodes: List[str]) -> dict:
    """
    Get information about a ROS2 node.

    :param node_name: The name of the ROS2 node.
    """
    data = {}

    for node_name in nodes:

        cmd = f"ros2 node info {node_name}"

        try:
            output = execute_ros_command(cmd)
        except subprocess.CalledProcessError as e:
            print(f"Error getting info for node '{node_name}': {e}")
            data[node_name] = dict(error=str(e))
            continue

        data[node_name] = dict(
            name=node_name,
            subscribers=[],
            publishers=[],
            service_servers=[],
            service_clients=[],
            action_servers=[],
            action_clients=[],
        )

        lines = output.split("\n")
        # Find indices for each section
        subscriber_idx = lines.index("  Subscribers:")
        publisher_idx = lines.index("  Publishers:")
        service_server_idx = lines.index("  Service Servers:")
        service_client_idx = lines.index("  Service Clients:")
        action_server_idx = lines.index("  Action Servers:")
        action_client_idx = lines.index("  Action Clients:")

        # Get subscribers
        for i in range(subscriber_idx + 1, publisher_idx):
            data[node_name]["subscribers"].append(lines[i].strip())

        # Get publishers
        for i in range(publisher_idx + 1, service_server_idx):
            data[node_name]["publishers"].append(lines[i].strip())

        # Get service servers
        for i in range(service_server_idx + 1, service_client_idx):
            data[node_name]["service_servers"].append(lines[i].strip())

        # Get service clients
        for i in range(service_client_idx + 1, action_server_idx):
            data[node_name]["service_clients"].append(lines[i].strip())

        # Get action servers
        for i in range(action_server_idx + 1, action_client_idx):
            data[node_name]["action_servers"].append(lines[i].strip())

        # Get action clients
        for i in range(action_client_idx + 1, len(lines)):
            data[node_name]["action_clients"].append(lines[i].strip())

    return data


def parse_ros2_topic_info(output):
    topic_info = {"name": "", "type": "", "publishers": [], "subscribers": []}

    lines = output.split("\n")

    # Extract the topic name
    for line in lines:
        if line.startswith("ros2 topic info"):
            topic_info["name"] = line.split(" ")[3]

    # Extract the Type
    for line in lines:
        if line.startswith("Type:"):
            topic_info["type"] = line.split(": ")[1]

    # Extract publisher and subscriber sections
    publisher_section = ""
    subscriber_section = ""
    collecting_publishers = False
    collecting_subscribers = False

    for line in lines:
        if line.startswith("Publisher count:"):
            collecting_publishers = True
            collecting_subscribers = False
        elif line.startswith("Subscription count:"):
            collecting_publishers = False
            collecting_subscribers = True

        if collecting_publishers:
            publisher_section += line + "\n"
        if collecting_subscribers:
            subscriber_section += line + "\n"

    # Extract node names for publishers
    publisher_lines = publisher_section.split("\n")
    for line in publisher_lines:
        if line.startswith("Node name:"):
            node_name = line.split(": ")[1]
            topic_info["publishers"].append(node_name)

    # Extract node names for subscribers
    subscriber_lines = subscriber_section.split("\n")
    for line in subscriber_lines:
        if line.startswith("Node name:"):
            node_name = line.split(": ")[1]
            topic_info["subscribers"].append(node_name)

    return topic_info


@tool
def ros2_topic_info(topics: List[str]) -> dict:
    """
    Get information about a ROS2 topic.

    :param topic_name: The name of the ROS2 topic.
    """
    data = {}

    for topic in topics:
        try:
            cmd = f"ros2 topic info {topic} --verbose"
            output = execute_ros_command(cmd)
            topic_info = parse_ros2_topic_info(output)
        except subprocess.CalledProcessError as e:
            topic_info = dict(error=str(e))
        data[topic] = topic_info

    return data


@tool
def ros2_param_list(node_name: Optional[str]) -> dict:
    """
    Get a list of parameters for a ROS2 node.

    :param node_name: An optional ROS2 node name to get parameters for. If not provided, all parameters are listed.
    """
    if node_name:
        cmd = f"ros2 param list {node_name}"
        output = execute_ros_command(cmd)

        # Trim all whitespace and split by newline
        params = output.strip().split("\n")
        params = [param.strip() for param in params if param]
        return {node_name: params}
    else:
        cmd = f"ros2 param list"
        output = execute_ros_command(cmd)

        # When we get a list of all nodes params, we have to parse it
        # The node name starts with a '/' and the params are indented
        lines = output.split("\n")
        data = {}
        current_node = None
        for line in lines:
            if line.startswith("/"):
                current_node = line
                data[current_node] = []
            else:
                data[current_node].append(line.strip())
        return data


@tool
def ros2_param_get(node_name: str, param_name: str) -> str:
    """
    Get the value of a parameter for a ROS2 node.

    :param node_name: The name of the ROS2 node.
    :param param_name: The name of the parameter.
    """
    cmd = f"ros2 param get {node_name} {param_name}"
    output = execute_ros_command(cmd)
    return output


@tool
def ros2_param_set(node_name: str, param_name: str, param_value: str) -> str:
    """
    Set the value of a parameter for a ROS2 node.

    :param node_name: The name of the ROS2 node.
    :param param_name: The name of the parameter.
    :param param_value: The value to set the parameter to.
    """
    cmd = f"ros2 param set {node_name} {param_name} {param_value}"
    output = execute_ros_command(cmd)
    return output


@tool
def ros2_service_info(services: List[str]) -> dict:
    """
    Get information about a ROS2 service.

    :param service_name: The name of the ROS2 service.
    """
    data = {}

    for service_name in services:
        cmd = f"ros2 service info {service_name}"
        try:
            output = execute_ros_command(cmd)
            data[service_name] = output
        except subprocess.CalledProcessError as e:
            data[service_name] = dict(error=str(e))

    return data


@tool
def ros2_service_info(services: List[str]) -> dict:
    """
    Get information about a ROS2 service.

    :param service_name: The name of the ROS2 service.
    """
    data = {}

    for service_name in services:
        cmd = f"ros2 service info {service_name}"
        try:
            output = execute_ros_command(cmd)
            data[service_name] = output
        except subprocess.CalledProcessError as e:
            data[service_name] = dict(error=str(e))

    return data


@tool
def ros2_service_call(service_name: str, srv_type: str, request: str) -> str:
    """
    Call a ROS2 service.

    :param service_name: The name of the ROS2 service.
    :param srv_type: The type of the service (use ros2_service_info to verify).
    :param request: The request to send to the service.
    """
    cmd = f'ros2 service call {service_name} {srv_type} "{request}"'
    try:
        output = execute_ros_command(cmd)
    except Exception as e:
        output = f"Error calling '{service_name}'. Command that was run: {cmd}. Error message: {e}"
    return output


@tool
def ros2_doctor() -> str:
    """
    Check ROS setup and other potential issues.
    """
    cmd = "ros2 doctor"
    output = execute_ros_command(cmd)
    return output


def get_ros2_log_root() -> str:
    """
    Get the root directory for ROS2 log files.
    """
    ros2_log_dir = os.environ.get("ROS_LOG_DIR", None)
    ros_home = os.environ.get("ROS_HOME", None)

    if not ros2_log_dir and ros_home:
        ros2_log_dir = os.path.join(ros_home, "log")
    elif not ros2_log_dir:
        ros2_log_dir = os.path.join(os.path.expanduser("~"), ".ros/log")

    return ros2_log_dir


@tool
def ros2_log_list(ros_log_dir: Optional[str]) -> dict:
    """Returns a list of ROS2 log files.

    :param ros_log_dir: The directory where ROS2 log files are stored. If not provided, the default ROS2 log directory is used.
    """

    # The log files will either be in $ROS_LOG_DIR (if it exists) or $ROS_HOME/log
    # First check if either of those env variables are set, starting with ROS_LOG_DIR
    ros_log_dir = ros_log_dir or get_ros2_log_root()

    if not os.path.exists(ros_log_dir):
        return dict(error=f"ROS log directory '{ros_log_dir}' does not exist.")

    log_files = [f for f in os.listdir(ros_log_dir) if f.endswith(".log")]

    # Get metadata for each file
    log_files_with_metadata = []
    for log_file in log_files:
        log_file_path = os.path.join(ros_log_dir, log_file)
        log_file_size = os.path.getsize(log_file_path)

        log_lines = []
        with open(log_file_path, "r") as f:
            log_lines = f.readlines()

        debug = 0
        info = 0
        warnings = 0
        errors = 0
        for line in log_lines:
            if line.startswith("[WARN]"):
                warnings += 1
            elif line.startswith("[ERROR]"):
                errors += 1
            elif line.startswith("[INFO]"):
                info += 1
            elif line.startswith("[DEBUG]"):
                debug += 1

        log_file_lines = len(log_lines)
        log_files_with_metadata.append(
            dict(
                name=log_file,
                bytes=log_file_size,
                lines=log_file_lines,
                debug=debug,
                info=info,
                warnings=warnings,
                errors=errors,
            )
        )

    return dict(log_file_directory=ros_log_dir, log_files=log_files_with_metadata)


@tool
def ros2_read_log(log_file_name: str, level: Optional[str]) -> dict:
    """Read a ROS2 log file.

    :param log_file_name: The name of the log file to read.
    :param level: (optional) The log level to filter by. If not provided, all log messages are returned.
    """
    ros_log_dir = get_ros2_log_root()
    log_file_path = os.path.join(ros_log_dir, log_file_name)

    if not os.path.exists(log_file_path):
        return dict(error=f"Log file '{log_file_name}' does not exist.")

    log_lines = []
    with open(log_file_path, "r") as f:
        log_lines = f.readlines()

    res = dict(log_file=log_file_name, log_dir=ros_log_dir, lines=[])

    for line in log_lines:
        if level and not line.startswith(f"[{level.upper()}]"):
            continue
        res["lines"].append(line.strip())

    return res
