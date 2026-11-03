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
import unittest
from unittest.mock import MagicMock, patch

try:
    from src.rosa.tools.ros2 import (
        ROSANode,
        execute_ros_command,
        ros2_node_list,
        ros2_topic_list,
        ros2_topic_echo,
        ros2_service_list,
        ros2_node_info,
        ros2_topic_info,
        ros2_service_info,
        ros2_topic_pub,
        ros2_param_list,
        ros2_param_get,
        ros2_param_set,
    )
except ModuleNotFoundError:
    pass


@unittest.skipIf(
    os.environ.get("ROS_VERSION") == "1",
    "Skipping ROS2 tests because ROS_VERSION is set to 1",
)
class TestROS2Tools(unittest.TestCase):
    """Tests for the ros2 CLI path, used when no native rclpy node is available."""

    def setUp(self):
        patcher = patch.object(ROSANode, "get_instance", return_value=None)
        patcher.start()
        self.addCleanup(patcher.stop)

    @patch("src.rosa.tools.ros2.subprocess.check_output")
    def test_execute_valid_ros2_command(self, mock_check_output):
        mock_check_output.return_value = b"Node /example_node\n"
        success, output = execute_ros_command("ros2 node list")
        self.assertTrue(success)
        self.assertEqual(output, "Node /example_node\n")

    @patch("src.rosa.tools.ros2.subprocess.check_output")
    def test_execute_invalid_ros2_command(self, mock_check_output):
        mock_check_output.side_effect = subprocess.CalledProcessError(
            1, "ros2 node list"
        )
        success, output = execute_ros_command("ros2 node list")
        self.assertFalse(success)
        self.assertIn(
            "Command 'ros2 node list' returned non-zero exit status 1.", output
        )

    @patch("src.rosa.tools.ros2.subprocess.check_output")
    def test_execute_does_not_use_shell(self, mock_check_output):
        mock_check_output.return_value = b""
        execute_ros_command("ros2 node list; touch /tmp/pwned")
        args, kwargs = mock_check_output.call_args
        self.assertEqual(args[0], ["ros2", "node", "list;", "touch", "/tmp/pwned"])
        self.assertFalse(kwargs.get("shell", False))

    @patch("src.rosa.tools.ros2.subprocess.check_output")
    def test_tool_arguments_are_not_shell_interpreted(self, mock_check_output):
        mock_check_output.return_value = b"42\n"
        ros2_param_get.invoke({"node_name": "/node; id", "param_name": "$(id)"})
        args, kwargs = mock_check_output.call_args
        self.assertEqual(args[0], ["ros2", "param", "get", "/node; id", "$(id)"])
        self.assertFalse(kwargs.get("shell", False))

    @patch("src.rosa.tools.ros2.subprocess.check_output")
    def test_option_injection_in_names_is_rejected(self, mock_check_output):
        result = ros2_param_get.invoke({"node_name": "--help", "param_name": "p"})
        self.assertIn("error", result)
        mock_check_output.assert_not_called()

    def test_execute_command_with_invalid_prefix(self):
        with self.assertRaises(ValueError):
            execute_ros_command("invalid node list")

    def test_execute_command_with_invalid_subcommand(self):
        with self.assertRaises(ValueError):
            execute_ros_command("ros2 invalid_subcommand")

    def test_execute_command_with_insufficient_arguments(self):
        with self.assertRaises(ValueError):
            execute_ros_command("ros2")

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_list_returns_nodes(self, mock_execute):
        mock_execute.return_value = (True, "/node1\n/node2\n")
        result = ros2_node_list.invoke({"pattern": None})
        self.assertEqual(result, {"nodes": ["/node1", "/node2"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_list_with_pattern(self, mock_execute):
        mock_execute.return_value = (True, "/node1\n/node2\n")
        result = ros2_node_list.invoke({"pattern": "node1"})
        self.assertEqual(result, {"nodes": ["/node1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_list_with_blacklist(self, mock_execute):
        mock_execute.return_value = (True, "/node1\n/node2\n")
        result = ros2_node_list.invoke({"blacklist": ["node2"]})
        self.assertEqual(result, {"nodes": ["/node1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_list_invalid_command(self, mock_execute):
        mock_execute.return_value = (False, "Invalid command")
        result = ros2_node_list.invoke({"pattern": None})
        self.assertEqual(result, {"nodes": ["Invalid command"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_list_returns_topics(self, mock_execute):
        mock_execute.return_value = (True, "/topic1\n/topic2\n")
        result = ros2_topic_list.invoke({"pattern": None})
        self.assertEqual(result, {"topics": ["/topic1", "/topic2"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_list_with_pattern(self, mock_execute):
        mock_execute.return_value = (True, "/topic1\n/topic2\n")
        result = ros2_topic_list.invoke({"pattern": "topic1"})
        self.assertEqual(result, {"topics": ["/topic1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_list_with_blacklist(self, mock_execute):
        mock_execute.return_value = (True, "/topic1\n/topic2\n")
        result = ros2_topic_list.invoke({"blacklist": ["topic2"]})
        self.assertEqual(result, {"topics": ["/topic1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_list_invalid_command(self, mock_execute):
        mock_execute.return_value = (False, "Invalid command")
        result = ros2_topic_list.invoke({"pattern": None})
        self.assertEqual(result, {"topics": ["Invalid command"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_echo_success(self, mock_execute):
        mock_execute.return_value = (True, "Message 1\n")
        result = ros2_topic_echo.invoke(
            {"topic": "/example_topic", "count": 1, "return_echoes": True}
        )
        self.assertEqual(result, {"echoes": ["Message 1\n"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_echo_multiple_messages(self, mock_execute):
        mock_execute.return_value = (True, "Message 1\n")
        result = ros2_topic_echo.invoke(
            {"topic": "/example_topic", "count": 3, "return_echoes": True}
        )
        self.assertEqual(
            result, {"echoes": ["Message 1\n", "Message 1\n", "Message 1\n"]}
        )

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_echo_invalid_topic(self, mock_execute):
        mock_execute.return_value = (False, "Invalid topic")
        result = ros2_topic_echo.invoke({"topic": "/invalid_topic", "count": 1})
        self.assertEqual(result, {"error": "Invalid topic"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_echo_invalid_count(self, mock_execute):
        result = ros2_topic_echo.invoke({"topic": "/example_topic", "count": 11})
        self.assertEqual(result, {"error": "Count must be between 1 and 10."})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_topic_echo_command_failure(self, mock_execute):
        mock_execute.return_value = (False, "Command failed")
        result = ros2_topic_echo.invoke({"topic": "/example_topic", "count": 1})
        self.assertEqual(result, {"error": "Command failed"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_service_list_returns_services(self, mock_execute):
        mock_execute.return_value = (True, "/service1\n/service2\n")
        result = ros2_service_list.invoke({"pattern": None})
        self.assertEqual(result, {"services": ["/service1", "/service2"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_service_list_with_pattern(self, mock_execute):
        mock_execute.return_value = (True, "/service1\n/service2\n")
        result = ros2_service_list.invoke({"pattern": "service1"})
        self.assertEqual(result, {"services": ["/service1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_service_list_with_blacklist(self, mock_execute):
        mock_execute.return_value = (True, "/service1\n/service2\n")
        result = ros2_service_list.invoke({"blacklist": ["service2"]})
        self.assertEqual(result, {"services": ["/service1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_service_list_invalid_command(self, mock_execute):
        mock_execute.return_value = (False, "Invalid command")
        result = ros2_service_list.invoke({"pattern": None})
        self.assertEqual(result, {"services": ["Invalid command"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_info_success(self, mock_execute):
        mock_execute.return_value = (True, "Node info for /node1")
        result = ros2_node_info.invoke({"nodes": ["/node1"]})
        self.assertEqual(result, {"/node1": "Node info for /node1"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_info_multiple_nodes(self, mock_execute):
        mock_execute.side_effect = [
            (True, "Node info for /node1"),
            (True, "Node info for /node2"),
        ]
        result = ros2_node_info.invoke({"nodes": ["/node1", "/node2"]})
        self.assertEqual(
            result, {"/node1": "Node info for /node1", "/node2": "Node info for /node2"}
        )

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_info_invalid_node(self, mock_execute):
        mock_execute.return_value = (False, "Invalid node")
        result = ros2_node_info.invoke({"nodes": ["/invalid_node"]})
        self.assertEqual(result, {"/invalid_node": {"error": "Invalid node"}})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_node_info_command_failure(self, mock_execute):
        mock_execute.return_value = (False, "Command failed")
        result = ros2_node_info.invoke({"nodes": ["/node1"]})
        self.assertEqual(result, {"/node1": {"error": "Command failed"}})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_list_returns_params_for_node(self, mock_execute):
        mock_execute.return_value = (True, "param1\nparam2\n")
        result = ros2_param_list.invoke({"node_name": "/example_node"})
        self.assertEqual(result, {"/example_node": ["param1", "param2"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_list_returns_all_params(self, mock_execute):
        mock_execute.return_value = (
            True,
            "/node1\n  param1\n  param2\n/node2\n  param3\n",
        )
        result = ros2_param_list.invoke({})
        self.assertEqual(result, {"/node1": ["param1", "param2"], "/node2": ["param3"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_list_with_pattern(self, mock_execute):
        mock_execute.return_value = (True, "param1\nparam2\n")
        result = ros2_param_list.invoke(
            {"node_name": "/example_node", "pattern": "param1"}
        )
        self.assertEqual(result, {"/example_node": ["param1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_list_with_blacklist(self, mock_execute):
        mock_execute.return_value = (True, "param1\nparam2\n")
        result = ros2_param_list.invoke(
            {"node_name": "/example_node", "blacklist": ["param2"]}
        )
        self.assertEqual(result, {"/example_node": ["param1"]})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_list_invalid_command(self, mock_execute):
        mock_execute.return_value = (False, "Invalid command")
        result = ros2_param_list.invoke({"node_name": "/example_node"})
        self.assertEqual(result, {"error": "Invalid command"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_get_success(self, mock_execute):
        mock_execute.return_value = (True, "value1")
        result = ros2_param_get.invoke(
            {"node_name": "/example_node", "param_name": "param1"}
        )
        self.assertEqual(result, {"param1": "value1"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_get_invalid_command(self, mock_execute):
        mock_execute.return_value = (False, "Invalid command")
        result = ros2_param_get.invoke(
            {"node_name": "/example_node", "param_name": "param1"}
        )
        self.assertEqual(result, {"error": "Invalid command"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_set_success(self, mock_execute):
        mock_execute.return_value = (True, "value1")
        result = ros2_param_set.invoke(
            {
                "node_name": "/example_node",
                "param_name": "param1",
                "param_value": "value1",
            }
        )
        self.assertEqual(result, {"param1": "value1"})

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_ros2_param_set_invalid_command(self, mock_execute):
        mock_execute.return_value = (False, "Invalid command")
        result = ros2_param_set.invoke(
            {
                "node_name": "/example_node",
                "param_name": "param1",
                "param_value": "value1",
            }
        )
        self.assertEqual(result, {"error": "Invalid command"})


@unittest.skipIf(
    os.environ.get("ROS_VERSION") == "1",
    "Skipping ROS2 tests because ROS_VERSION is set to 1",
)
class TestROS2NativeTools(unittest.TestCase):
    """Tests for the native rclpy path, using a mocked shared node."""

    def setUp(self):
        self.node = MagicMock()
        patcher = patch.object(ROSANode, "get_instance", return_value=self.node)
        patcher.start()
        self.addCleanup(patcher.stop)
        execute_patcher = patch("src.rosa.tools.ros2.execute_ros_command")
        self.mock_execute = execute_patcher.start()
        self.addCleanup(execute_patcher.stop)

    def test_node_list_hides_hidden_nodes_and_filters(self):
        self.node.get_node_names_and_namespaces.return_value = [
            ("talker", "/"),
            ("listener", "/demo"),
            ("_rosa_agent", "/"),
        ]
        self.assertEqual(
            ros2_node_list.invoke({}), {"nodes": ["/demo/listener", "/talker"]}
        )
        self.assertEqual(
            ros2_node_list.invoke({"blacklist": ["demo"]}), {"nodes": ["/talker"]}
        )
        self.mock_execute.assert_not_called()

    def test_topic_list(self):
        self.node.get_topic_names_and_types.return_value = [
            ("/chatter", ["std_msgs/msg/String"]),
            ("/turtle1/_action/status", ["action_msgs/msg/GoalStatusArray"]),
        ]
        self.assertEqual(ros2_topic_list.invoke({}), {"topics": ["/chatter"]})
        self.mock_execute.assert_not_called()

    def test_service_list_with_pattern(self):
        self.node.get_service_names_and_types.return_value = [
            ("/spawn", ["turtlesim/srv/Spawn"]),
            ("/kill", ["turtlesim/srv/Kill"]),
        ]
        self.assertEqual(
            ros2_service_list.invoke({"pattern": "spawn"}), {"services": ["/spawn"]}
        )

    def test_native_error_falls_back_to_cli(self):
        self.node.get_topic_names_and_types.side_effect = RuntimeError("context invalid")
        self.mock_execute.return_value = (True, "/chatter\n")
        self.assertEqual(ros2_topic_list.invoke({}), {"topics": ["/chatter"]})

    def test_node_info_includes_clients_and_actions(self):
        self.node.get_subscriber_names_and_types_by_node.return_value = [
            ("/cmd_vel", ["geometry_msgs/msg/Twist"])
        ]
        self.node.get_publisher_names_and_types_by_node.return_value = []
        self.node.get_service_names_and_types_by_node.return_value = [
            ("/spawn", ["turtlesim/srv/Spawn"]),
            ("/turtle1/rotate_absolute/_action/send_goal", ["x/srv/SendGoal"]),
        ]
        self.node.get_client_names_and_types_by_node.return_value = [
            ("/clear", ["std_srvs/srv/Empty"])
        ]
        servers = MagicMock(
            return_value=[("/turtle1/rotate_absolute", ["turtlesim/action/RotateAbsolute"])]
        )
        clients = MagicMock(return_value=[])
        with patch(
            "src.rosa.tools.ros2.get_action_server_names_and_types_by_node", servers
        ), patch("src.rosa.tools.ros2.get_action_client_names_and_types_by_node", clients):
            result = ros2_node_info.invoke({"nodes": ["/demo/turtlesim"]})

        info = result["/demo/turtlesim"]
        self.node.get_client_names_and_types_by_node.assert_called_with("turtlesim", "/demo")
        servers.assert_called_with(self.node, "turtlesim", "/demo")
        self.assertIn("Service Clients:\n    /clear: std_srvs/srv/Empty", info)
        self.assertIn(
            "Action Servers:\n    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute",
            info,
        )
        self.assertNotIn("_action", info)
        self.mock_execute.assert_not_called()

    def test_topic_info_includes_type_and_endpoints(self):
        self.node.get_topic_names_and_types.return_value = [
            ("/chatter", ["std_msgs/msg/String"])
        ]
        endpoint = MagicMock()
        endpoint.__str__.return_value = "Node name: talker\nNode namespace: /"
        self.node.get_publishers_info_by_topic.return_value = [endpoint]
        self.node.get_subscriptions_info_by_topic.return_value = []

        info = ros2_topic_info.invoke({"topics": ["/chatter"]})["/chatter"]
        self.assertIn("Type: std_msgs/msg/String", info)
        self.assertIn("Publisher count: 1", info)
        self.assertIn("Node name: talker", info)
        self.assertIn("Subscription count: 0", info)
        self.mock_execute.assert_not_called()

    def test_unknown_topic_info_falls_back_to_cli(self):
        self.node.get_topic_names_and_types.return_value = []
        self.mock_execute.return_value = (False, "Unknown topic '/missing'")
        result = ros2_topic_info.invoke({"topics": ["/missing"]})
        self.assertEqual(result, {"/missing": {"error": "Unknown topic '/missing'"}})

    def test_service_info(self):
        self.node.get_service_names_and_types.return_value = [
            ("/spawn", ["turtlesim/srv/Spawn"])
        ]
        result = ros2_service_info.invoke({"services": ["/spawn"]})
        self.assertEqual(result, {"/spawn": "turtlesim/srv/Spawn\n"})
        self.mock_execute.assert_not_called()


@unittest.skipIf(
    os.environ.get("ROS_VERSION") == "1",
    "Skipping ROS2 tests because ROS_VERSION is set to 1",
)
class TestROS2TopicPub(unittest.TestCase):

    @patch("src.rosa.tools.ros2.execute_ros_command")
    @patch.object(ROSANode, "get_instance", return_value=None)
    def test_cli_fallback_passes_message_verbatim(self, _, mock_execute):
        mock_execute.return_value = (True, "publishing #1")
        result = ros2_topic_pub.invoke(
            {
                "topic": "/chatter",
                "msg_type": "std_msgs/msg/String",
                "message": "{data: hello $HOME!}",
                "rate": 2.0,
                "duration": 1.5,
            }
        )
        self.assertTrue(result["success"])
        mock_execute.assert_called_once_with(
            ["ros2", "topic", "pub", "--rate", "2.0", "--times", "3",
             "/chatter", "std_msgs/msg/String", "{data: hello $HOME!}"]
        )

    @patch("src.rosa.tools.ros2.execute_ros_command")
    def test_rejects_invalid_arguments(self, mock_execute):
        for args in (
            {"topic": "--help", "msg_type": "std_msgs/msg/String", "message": "{}"},
            {"topic": "/t", "msg_type": "std_msgs/msg/String", "message": "{}", "rate": 0},
        ):
            self.assertIn("error", ros2_topic_pub.invoke(args))
        mock_execute.assert_not_called()

    def _invoke_native(self, node, **kwargs):
        msg_class = MagicMock()
        set_fields = MagicMock()
        with patch.object(ROSANode, "get_instance", return_value=node), patch(
            "rosidl_runtime_py.utilities.get_message", return_value=msg_class, create=True
        ), patch(
            "rosidl_runtime_py.set_message.set_message_fields", set_fields, create=True
        ), patch("src.rosa.tools.ros2.time.sleep"):
            args = {"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist",
                    "message": "{linear: {x: 1.0}}"}
            args.update(kwargs)
            result = ros2_topic_pub.invoke(args)
        return result, msg_class, set_fields

    @unittest.skipUnless(
        __import__("importlib").util.find_spec("rosidl_runtime_py"),
        "rosidl_runtime_py is not installed",
    )
    def test_native_publishes_after_subscriber_matches(self):
        node = MagicMock()
        publisher = node.create_publisher.return_value
        publisher.get_subscription_count.side_effect = [0, 1]

        result, msg_class, set_fields = self._invoke_native(node, rate=10.0, duration=0.5)

        self.assertEqual(result["published"], 5)
        self.assertEqual(publisher.publish.call_count, 5)
        set_fields.assert_called_once_with(msg_class.return_value, {"linear": {"x": 1.0}})
        node.destroy_publisher.assert_called_once_with(publisher)

    @unittest.skipUnless(
        __import__("importlib").util.find_spec("rosidl_runtime_py"),
        "rosidl_runtime_py is not installed",
    )
    def test_native_times_out_without_subscribers(self):
        node = MagicMock()
        publisher = node.create_publisher.return_value
        publisher.get_subscription_count.return_value = 0

        result, _, _ = self._invoke_native(node, wait_timeout=0.0)

        self.assertIn("No subscribers matched", result["error"])
        publisher.publish.assert_not_called()
        node.destroy_publisher.assert_called_once_with(publisher)


if __name__ == "__main__":
    import os

    if os.environ.get("ROS_VERSION") == 2:
        unittest.main()
