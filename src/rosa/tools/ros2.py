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
import threading
import time
from typing import List, Optional, Tuple, Union

import yaml
from langchain.agents import tool

# rclpy is optional: when it is available the tools query the ROS2 graph through a shared,
# long-lived node instead of spawning a ros2 CLI process (and a fresh DDS participant) per call.
try:
    import rclpy
    from rclpy.logging import get_logging_directory
    from rclpy.node import Node

    HAS_RCLPY = True
except ImportError:
    rclpy = None
    get_logging_directory = None
    Node = object
    HAS_RCLPY = False

try:
    from rclpy.action.graph import (
        get_action_client_names_and_types_by_node,
        get_action_server_names_and_types_by_node,
    )
except ImportError:
    get_action_client_names_and_types_by_node = None
    get_action_server_names_and_types_by_node = None

# Maximum time to wait for DDS discovery to populate the graph when the shared node is created.
DISCOVERY_TIMEOUT = float(os.environ.get("ROSA_ROS2_DISCOVERY_TIMEOUT", "2.0"))


class ROSANode(Node):
    """Singleton ROS2 node shared across all native tool invocations."""

    NODE_NAME = "_rosa_agent"

    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> Optional["ROSANode"]:
        """
        Get the shared node, creating it on first use.

        :return: The shared node, or None if rclpy is unavailable or the node cannot be created,
                 in which case callers should fall back to the ros2 CLI.
        """
        if not HAS_RCLPY:
            return None

        with cls._lock:
            if cls._instance is None:
                try:
                    if not rclpy.ok():
                        rclpy.init()
                    node = cls(cls.NODE_NAME, start_parameter_services=False)
                except Exception:
                    return None
                node.wait_for_discovery(DISCOVERY_TIMEOUT)
                cls._instance = node
            return cls._instance

    def wait_for_discovery(self, timeout: float) -> None:
        """
        Block until another node has been discovered or the timeout elapses.

        DDS discovery is asynchronous, so a freshly created node's graph cache can be empty or
        incomplete. Graph queries do not fail in that case, so wait briefly before trusting them.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(
                name != self.get_name() or ns != self.get_namespace()
                for name, ns in self.get_node_names_and_namespaces()
            ):
                # Give endpoints of the discovered nodes a moment to arrive as well.
                time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
                return
            time.sleep(0.05)


def is_hidden_name(name: str) -> bool:
    """Return True if any token of a ROS2 name is hidden (starts with an underscore)."""
    return any(token.startswith("_") for token in name.split("/"))


def filter_entities(
    entities: List[str],
    pattern: Optional[str] = None,
    blacklist: Optional[List[str]] = None,
) -> List[str]:
    """Apply the blacklist and pattern filters used by the list tools."""
    if blacklist:
        entities = [
            e for e in entities if not any(re.match(f".*{b}.*", e) for b in blacklist)
        ]
    if pattern:
        entities = [e for e in entities if re.match(f".*{pattern}.*", e)]
    return entities


def split_node_name(node_name: str) -> Tuple[str, str]:
    """Split a fully qualified node name into (name, namespace)."""
    ns, _, name = node_name.rpartition("/")
    return name, ns or "/"


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
    node = ROSANode.get_instance()
    if node is not None:
        try:
            nodes = [
                f"{ns.rstrip('/')}/{name}"
                for name, ns in node.get_node_names_and_namespaces()
                if not is_hidden_name(name)
            ]
            return {"nodes": filter_entities(sorted(nodes), pattern, blacklist)}
        except Exception:
            pass

    cmd = "ros2 node list"
    nodes = get_entities(cmd, pattern=pattern, blacklist=blacklist)
    return {"nodes": nodes}


@tool
def ros2_topic_list(pattern: Optional[str] = None, blacklist: Optional[List[str]] = None) -> dict:
    """
    Get a list of ROS2 topics.

    :param pattern: A regular expression pattern to filter the list of topics.
    """
    node = ROSANode.get_instance()
    if node is not None:
        try:
            topics = [
                name
                for name, _ in node.get_topic_names_and_types()
                if not is_hidden_name(name)
            ]
            return {"topics": filter_entities(topics, pattern, blacklist)}
        except Exception:
            pass

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
    node = ROSANode.get_instance()
    if node is not None:
        try:
            services = [
                name
                for name, _ in node.get_service_names_and_types()
                if not is_hidden_name(name)
            ]
            return {"services": filter_entities(services, pattern, blacklist)}
        except Exception:
            pass

    cmd = "ros2 service list"
    services = get_entities(cmd, pattern=pattern, blacklist=blacklist)
    return {"services": services}


def native_node_info(node: "ROSANode", node_name: str) -> str:
    """Describe a node using the graph API, in the same layout as `ros2 node info`."""
    name, ns = split_node_name(node_name)

    sections = [
        ("Subscribers", node.get_subscriber_names_and_types_by_node(name, ns)),
        ("Publishers", node.get_publisher_names_and_types_by_node(name, ns)),
        ("Service Servers", node.get_service_names_and_types_by_node(name, ns)),
        ("Service Clients", node.get_client_names_and_types_by_node(name, ns)),
    ]
    if get_action_server_names_and_types_by_node is not None:
        sections += [
            ("Action Servers", get_action_server_names_and_types_by_node(node, name, ns)),
            ("Action Clients", get_action_client_names_and_types_by_node(node, name, ns)),
        ]

    lines = [node_name]
    for title, entries in sections:
        lines.append(f"  {title}:")
        for entry_name, entry_types in entries:
            if not is_hidden_name(entry_name):
                lines.append(f"    {entry_name}: {', '.join(entry_types)}")
    return "\n".join(lines) + "\n"


@tool
def ros2_node_info(nodes: List[str]) -> dict:
    """
    Get information about a ROS2 node.

    :param nodes: A list of ROS2 node names.
    """
    node = ROSANode.get_instance()
    data = {}

    for node_name in nodes:

        try:
            validate_ros_name(node_name, "node name")
        except ValueError as e:
            data[node_name] = dict(error=str(e))
            continue

        if node is not None:
            try:
                data[node_name] = native_node_info(node, node_name)
                continue
            except Exception:
                pass

        cmd = ["ros2", "node", "info", node_name]
        success, output = execute_ros_command(cmd)
        if not success:
            data[node_name] = dict(error=output)
            continue
        data[node_name] = output

    return data


def native_topic_info(node: "ROSANode", topic: str) -> Optional[str]:
    """
    Describe a topic using the graph API, in the same layout as `ros2 topic info --verbose`.

    :return: The description, or None if the topic is not known to the graph.
    """
    types = dict(node.get_topic_names_and_types()).get(topic)
    if types is None:
        return None

    publishers = node.get_publishers_info_by_topic(topic)
    subscriptions = node.get_subscriptions_info_by_topic(topic)

    lines = [f"Type: {', '.join(types)}", ""]
    for title, endpoints in (("Publisher", publishers), ("Subscription", subscriptions)):
        lines += [f"{title} count: {len(endpoints)}", ""]
        for endpoint in endpoints:
            lines += [str(endpoint), ""]
    return "\n".join(lines)


@tool
def ros2_topic_info(topics: List[str]) -> dict:
    """
    Get information about a ROS2 topic.

    :param topics: A list of ROS2 topic names.
    """
    node = ROSANode.get_instance()
    data = {}

    for topic in topics:
        try:
            validate_ros_name(topic, "topic")
        except ValueError as e:
            data[topic] = dict(error=str(e))
            continue

        if node is not None:
            try:
                info = native_topic_info(node, topic)
                if info is not None:
                    data[topic] = info
                    continue
            except Exception:
                pass

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
    node = ROSANode.get_instance()
    data = {}

    for service_name in services:
        try:
            validate_ros_name(service_name, "service name")
        except ValueError as e:
            data[service_name] = dict(error=str(e))
            continue

        if node is not None:
            try:
                types = dict(node.get_service_names_and_types()).get(service_name)
                if types:
                    data[service_name] = "\n".join(types) + "\n"
                    continue
            except Exception:
                pass

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
def ros2_topic_pub(
    topic: str,
    msg_type: str,
    message: str,
    rate: float = 10.0,
    duration: float = 1.0,
    wait_timeout: float = 2.0,
) -> dict:
    """
    Publish a message to a ROS2 topic, repeatedly at the given rate for the given duration.

    :param topic: The name of the topic to publish to.
    :param msg_type: The message type (e.g. 'geometry_msgs/msg/Twist').
    :param message: The message content in YAML format (e.g. '{linear: {x: 1.0}}').
    :param rate: The frequency (Hz) at which to publish the message.
    :param duration: The total time (seconds) to keep publishing the message.
    :param wait_timeout: Max time (seconds) to wait for a subscriber before giving up.
    """
    try:
        validate_ros_name(topic, "topic")
        validate_ros_name(msg_type, "message type")
        rate = float(rate)
        duration = float(duration)
        wait_timeout = float(wait_timeout)
    except (TypeError, ValueError) as e:
        return {"error": str(e)}

    if rate <= 0 or duration <= 0:
        return {"error": "Rate and duration must be greater than zero."}

    times = max(1, round(duration * rate))

    node = ROSANode.get_instance()
    if node is None:
        cmd = ["ros2", "topic", "pub", "--rate", str(rate), "--times", str(times)]
        cmd += [topic, msg_type, str(message)]
        success, output = execute_ros_command(cmd)
        if not success:
            return {"error": output}
        return {"success": True, "output": output}

    try:
        from rosidl_runtime_py.set_message import set_message_fields
        from rosidl_runtime_py.utilities import get_message

        msg_class = get_message(msg_type)
        msg = msg_class()
        values = yaml.safe_load(str(message))
        if values:
            set_message_fields(msg, values)
    except Exception as e:
        return {"error": f"Failed to build {msg_type} message: {e}"}

    publisher = node.create_publisher(msg_class, topic, 10)
    try:
        # A volatile publisher drops messages sent before a subscriber is matched.
        deadline = time.monotonic() + wait_timeout
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                return {
                    "error": f"No subscribers matched on {topic} within {wait_timeout}s; "
                    "nothing was published."
                }
            time.sleep(0.05)

        period = 1.0 / rate
        for i in range(times):
            publisher.publish(msg)
            if i < times - 1:
                time.sleep(period)
    finally:
        node.destroy_publisher(publisher)

    return {"success": True, "published": times, "topic": topic, "rate": rate}


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
    if get_logging_directory is not None:
        log_dir = get_logging_directory()
    else:
        # Mirror rcl_logging's resolution: $ROS_LOG_DIR, then $ROS_HOME/log, then ~/.ros/log.
        ros_home = os.environ.get("ROS_HOME") or os.path.join(os.path.expanduser("~"), ".ros")
        log_dir = os.environ.get("ROS_LOG_DIR") or os.path.join(ros_home, "log")
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
        if not log_dir or not os.path.isdir(log_dir):
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
