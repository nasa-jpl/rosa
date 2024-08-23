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
import unittest
from unittest.mock import patch, MagicMock

try:
    from src.rosa.tools.ros1 import (
        get_entities,
        rosgraph_get,
        rostopic_list,
        rostopic_info,
        rostopic_echo,
        rosnode_list,
        rosnode_info,
        rosservice_list,
        rosservice_info,
        rosservice_call,
        rosmsg_info,
        rossrv_info,
        rosparam_list,
        rosparam_get,
        rosparam_set,
        rospkg_list,
        rospkg_roots,
        roslog_list,
    )
except ModuleNotFoundError:
    pass


@unittest.skipIf(
    os.environ.get("ROS_VERSION") == "2",
    "Skipping ROS1 tests because ROS_VERSION is set to 2",
)
class TestROS1Tools(unittest.TestCase):

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    def test_get_entities_topics(self, mock_get_topic_list):
        mock_get_topic_list.return_value = (
            [("/turtle1/cmd_vel", "std_msgs/Empty")],
            [("/turtle1/pose", "std_msgs/Empty")],
        )
        total, in_namespace, match_pattern, entities = get_entities("topic", None, None)
        self.assertEqual(total, 2)
        self.assertEqual(in_namespace, 2)
        self.assertEqual(match_pattern, 2)
        self.assertIn("/turtle1/cmd_vel", entities)
        self.assertIn("/turtle1/pose", entities)

    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_nodes(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/turtlesim"]
        total, in_namespace, match_pattern, entities = get_entities("node", None, None)
        self.assertEqual(total, 1)
        self.assertEqual(in_namespace, 1)
        self.assertEqual(match_pattern, 1)
        self.assertIn("/turtlesim", entities)

    @patch("src.rosa.tools.ros1.rosgraph.masterapi.Master.getSystemState")
    def test_rosgraph_get_returns_graph(self, mock_get_system_state):
        mock_get_system_state.return_value = (
            [("/topic1", ["/node1"]), ("/topic2", ["/node2"])],
            [("/topic1", ["/node3"]), ("/topic2", ["/node4"])],
            [],
        )
        result = rosgraph_get.invoke(
            {
                "node_pattern": ".*",
                "topic_pattern": ".*",
                "blacklist": [],
                "exclude_self_connections": True,
            }
        )
        self.assertIn("graph", result)
        self.assertEqual(len(result["graph"]), 2)

    @patch("src.rosa.tools.ros1.rosgraph.masterapi.Master.getSystemState")
    def test_rosgraph_get_handles_empty_graph(self, mock_get_system_state):
        mock_get_system_state.return_value = ([], [], [])
        result = rosgraph_get.invoke(
            {
                "node_pattern": ".*",
                "topic_pattern": ".*",
                "blacklist": [],
                "exclude_self_connections": True,
            }
        )
        self.assertIn("error", result)
        self.assertEqual(
            result["error"],
            "No results found for the specified parameters. Note that the following have been excluded: []",
        )

    @patch("src.rosa.tools.ros1.rosgraph.masterapi.Master.getSystemState")
    def test_rosgraph_get_excludes_blacklisted_nodes(self, mock_get_system_state):
        mock_get_system_state.return_value = (
            [("/topic1", ["/node1"]), ("/topic2", ["/node2"])],
            [("/topic1", ["/node3"]), ("/topic2", ["/node4"])],
            [],
        )
        result = rosgraph_get.invoke(
            {
                "node_pattern": ".*",
                "topic_pattern": ".*",
                "blacklist": ["node1"],
                "exclude_self_connections": True,
            }
        )
        self.assertIn("graph", result)
        self.assertEqual(len(result["graph"]), 1)
        self.assertNotIn("/node1", result["graph"][0])

    @patch("src.rosa.tools.ros1.rosgraph.masterapi.Master.getSystemState")
    def test_rosgraph_get_excludes_self_connections(self, mock_get_system_state):
        mock_get_system_state.return_value = (
            [("/topic1", ["/node1"])],
            [("/topic1", ["/node1"])],
            [],
        )
        result = rosgraph_get.invoke(
            {
                "node_pattern": ".*",
                "topic_pattern": ".*",
                "blacklist": [],
                "exclude_self_connections": True,
            }
        )
        self.assertIn("error", result)
        self.assertEqual(
            result["error"],
            "No results found for the specified parameters. Note that the following have been excluded: []",
        )

    @patch("src.rosa.tools.ros1.rostopic.get_info_text")
    def test_rostopic_info(self, mock_get_info_text):
        mock_get_info_text.return_value = (
            "Type: std_msgs/String\nPublishers:\n* /turtlesim\nSubscribers:\n* /rosout"
        )
        result = rostopic_info.invoke({"topics": ["/turtle1/cmd_vel"]})
        self.assertIn("/turtle1/cmd_vel", result)
        self.assertEqual(result["/turtle1/cmd_vel"]["type"], "std_msgs/String")
        self.assertIn("/turtlesim", result["/turtle1/cmd_vel"]["publishers"])
        self.assertIn("/rosout", result["/turtle1/cmd_vel"]["subscribers"])

    @patch("src.rosa.tools.ros1.rospy.wait_for_message")
    @patch("src.rosa.tools.ros1.rostopic.get_topic_class")
    def test_rostopic_echo(self, mock_get_topic_class, mock_wait_for_message):
        mock_get_topic_class.return_value = (MagicMock(), None, None)
        mock_wait_for_message.return_value = MagicMock()
        result = rostopic_echo.invoke(
            {"topic": "/turtle1/cmd_vel", "count": 1, "return_echoes": True}
        )
        self.assertEqual(result["requested_count"], 1)
        self.assertEqual(result["actual_count"], 1)
        self.assertIn("echoes", result)

    @patch("rosnode.get_node_names")
    def test_rosnode_list_returns_all_nodes(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/node1", "/node2", "/node3"]
        result = rosnode_list.invoke({})
        self.assertEqual(result["total"], 3)
        self.assertEqual(result["in_namespace"], 3)
        self.assertEqual(result["match_pattern"], 3)
        self.assertEqual(result["nodes"], ["/node1", "/node2", "/node3"])

    @patch("rosnode.get_node_names")
    def test_rosnode_list_filters_by_namespace(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/namespace1/node1", "/namespace2/node2"]
        result = rosnode_list.invoke({"namespace": "/namespace1"})
        self.assertEqual(result["total"], 2)
        self.assertEqual(result["in_namespace"], 1)
        self.assertEqual(result["match_pattern"], 1)
        self.assertEqual(result["nodes"], ["/namespace1/node1"])

    @patch("rosnode.get_node_names")
    def test_rosnode_list_filters_by_pattern(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/node1", "/node2", "/node3"]
        result = rosnode_list.invoke({"pattern": "node1"})
        self.assertEqual(result["total"], 3)
        self.assertEqual(result["in_namespace"], 3)
        self.assertEqual(result["match_pattern"], 1)
        self.assertEqual(result["nodes"], ["/node1"])

    @patch("rosnode.get_node_names")
    def test_rosnode_list_handles_no_nodes(self, mock_get_node_names):
        mock_get_node_names.return_value = []
        result = rosnode_list.invoke({})
        self.assertEqual(result["total"], 0)
        self.assertEqual(result["in_namespace"], 0)
        self.assertEqual(result["match_pattern"], 0)
        self.assertEqual(
            result["nodes"], ["There are currently no nodes available in the system."]
        )

    @patch("rosnode.get_node_names")
    def test_rosnode_list_handles_no_nodes_in_namespace(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/node1", "/node2"]
        result = rosnode_list.invoke({"namespace": "/namespace1"})
        self.assertEqual(result["total"], 2)
        self.assertEqual(result["in_namespace"], 0)
        self.assertEqual(result["match_pattern"], 0)
        self.assertEqual(
            result["nodes"],
            [
                "There are currently no nodes available using the '/namespace1' namespace."
            ],
        )

    @patch("rosnode.get_node_names")
    def test_rosnode_list_handles_no_nodes_matching_pattern(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/node1", "/node2"]
        result = rosnode_list.invoke({"pattern": "node3"})
        self.assertEqual(result["total"], 2)
        self.assertEqual(result["in_namespace"], 2)
        self.assertEqual(result["match_pattern"], 0)
        self.assertEqual(
            result["nodes"],
            ["There are currently no nodes available matching the specified pattern."],
        )

    @patch("rosnode.get_node_names")
    def test_rosnode_list_filters_by_blacklist(self, mock_get_node_names):
        mock_get_node_names.return_value = ["/node1", "/node2", "/node3"]
        result = rosnode_list.invoke({"blacklist": ["node2"]})
        self.assertEqual(result["total"], 3)
        self.assertEqual(result["in_namespace"], 3)
        self.assertEqual(result["match_pattern"], 3)
        self.assertEqual(result["nodes"], ["/node1", "/node3"])

    @patch("src.rosa.tools.ros1.rosnode.get_node_info_description")
    def test_rosnode_info(self, mock_get_node_info_description):
        mock_get_node_info_description.return_value = (
            "Node: /turtlesim\nPublications: /turtle1/cmd_vel"
        )
        result = rosnode_info.invoke({"nodes": ["/turtlesim"]})
        self.assertIn("/turtlesim", result)
        self.assertIn("Node: /turtlesim", result["/turtlesim"])

    @patch("src.rosa.tools.ros1.rosservice.get_service_list")
    def test_rosservice_list(self, mock_get_service_list):
        mock_get_service_list.return_value = ["/clear", "/reset"]
        result = rosservice_list.invoke({})
        self.assertIn("/clear", result)
        self.assertIn("/reset", result)

    @patch("src.rosa.tools.ros1.rosservice.get_service_headers")
    @patch("src.rosa.tools.ros1.rosservice.get_service_uri")
    def test_rosservice_info(self, mock_get_service_uri, mock_get_service_headers):
        mock_get_service_uri.return_value = "rosrpc://localhost:12345"
        mock_get_service_headers.return_value = {"callerid": "/turtlesim"}
        result = rosservice_info.invoke({"services": ["/clear"]})
        self.assertIn("/clear", result)
        self.assertIn("callerid", result["/clear"])

    @patch("src.rosa.tools.ros1.rosservice.call_service")
    def test_rosservice_call(self, mock_call_service):
        mock_call_service.return_value = "success"
        result = rosservice_call.invoke({"service": "/clear", "args": []})
        self.assertEqual(result, "success")

    @patch("src.rosa.tools.ros1.rosmsg.get_msg_text")
    def test_rosmsg_info(self, mock_get_msg_text):
        mock_get_msg_text.return_value = "string data"
        result = rosmsg_info.invoke({"msg_type": ["std_msgs/String"]})
        self.assertIn("std_msgs/String", result)
        self.assertEqual(result["std_msgs/String"], "string data")

    @patch("src.rosa.tools.ros1.rosmsg.get_srv_text")
    def test_rossrv_info(self, mock_get_srv_text):
        mock_get_srv_text.return_value = "string data"
        result = rossrv_info.invoke({"srv_type": ["std_srvs/Empty"]})
        self.assertIn("std_srvs/Empty", result)
        self.assertEqual(result["std_srvs/Empty"], "string data")

    @patch("src.rosa.tools.ros1.rosparam.list_params")
    def test_rosparam_list(self, mock_list_params):
        mock_list_params.return_value = [
            "/turtlesim/background_r",
            "/turtlesim/background_g",
        ]
        result = rosparam_list.invoke({})
        self.assertIn("/turtlesim/background_r", result["ros_params"])
        self.assertIn("/turtlesim/background_g", result["ros_params"])

    @patch("src.rosa.tools.ros1.rosparam.get_param")
    def test_rosparam_get(self, mock_get_param):
        mock_get_param.return_value = 255
        result = rosparam_get.invoke({"params": ["/turtlesim/background_r"]})
        self.assertIn("/turtlesim/background_r", result)
        self.assertEqual(result["/turtlesim/background_r"], 255)

    @patch("src.rosa.tools.ros1.rosparam.set_param")
    def test_rosparam_set(self, mock_set_param):
        result = rosparam_set.invoke(
            {"param": "/turtlesim/background_r", "value": "255", "is_rosa_param": False}
        )
        self.assertEqual(result, "Set parameter '/turtlesim/background_r' to '255'.")

    @patch("src.rosa.tools.ros1.rospkg.RosPack.list")
    def test_rospkg_list(self, mock_list):
        mock_list.return_value = ["turtlesim", "std_msgs"]
        result = rospkg_list.invoke({"ignore_msgs": True})
        self.assertIn("turtlesim", result["packages"])
        self.assertNotIn("std_msgs", result["packages"])

        result = rospkg_list.invoke({"ignore_msgs": False})
        self.assertIn("turtlesim", result["packages"])
        self.assertIn("std_msgs", result["packages"])

    @patch("src.rosa.tools.ros1.rospkg.get_ros_package_path")
    def test_rospkg_roots(self, mock_get_ros_package_path):
        mock_get_ros_package_path.return_value = ["/opt/ros/noetic/share"]
        result = rospkg_roots.invoke({})
        self.assertIn("/opt/ros/noetic/share", result)

    @patch("src.rosa.tools.ros1.get_roslog_directories")
    @patch("os.listdir")
    @patch("os.path.isfile")
    @patch("os.path.getsize")
    def test_roslog_list_with_min_size(
        self, mock_getsize, mock_isfile, mock_listdir, mock_get_roslog_directories
    ):
        mock_get_roslog_directories.return_value = {"default": "/mock/log/dir"}
        mock_listdir.return_value = ["log1.log", "log2.log", "log3.log"]
        mock_isfile.side_effect = lambda x: x.endswith(".log")
        mock_getsize.side_effect = lambda x: 3000 if "log1.log" in x else 1000

        result = roslog_list.invoke({"min_size": 2048})

        self.assertEqual(result["total"], 1)
        self.assertEqual(len(result["logs"]), 1)
        self.assertEqual(result["logs"][0]["total"], 1)
        self.assertIn("/log1.log", result["logs"][0]["files"][0])

    @patch("src.rosa.tools.ros1.get_roslog_directories")
    @patch("os.listdir")
    @patch("os.path.isfile")
    @patch("os.path.getsize")
    def test_roslog_list_with_blacklist(
        self, mock_getsize, mock_isfile, mock_listdir, mock_get_roslog_directories
    ):
        mock_get_roslog_directories.return_value = {"default": "/mock/log/dir"}
        mock_listdir.return_value = ["log1.log", "log2.log", "log3.log"]
        mock_isfile.side_effect = lambda x: x.endswith(".log")
        mock_getsize.side_effect = lambda x: 3000

        result = roslog_list.invoke({"blacklist": ["log2"]})

        self.assertEqual(result["total"], 1)
        self.assertEqual(len(result["logs"]), 1)
        self.assertEqual(result["logs"][0]["total"], 2)
        self.assertNotIn("log2.log", result["logs"][0]["files"][0])

    @patch("src.rosa.tools.ros1.get_roslog_directories")
    @patch("os.listdir")
    @patch("os.path.isfile")
    @patch("os.path.getsize")
    def test_roslog_list_no_logs(
        self, mock_getsize, mock_isfile, mock_listdir, mock_get_roslog_directories
    ):
        mock_get_roslog_directories.return_value = {"default": "/mock/log/dir"}
        mock_listdir.return_value = []
        mock_isfile.side_effect = lambda x: x.endswith(".log")
        mock_getsize.side_effect = lambda x: 3000

        result = roslog_list.invoke({})

        self.assertEqual(result["total"], 0)
        self.assertEqual(len(result["logs"]), 0)

    @patch("src.rosa.tools.ros1.get_roslog_directories")
    @patch("os.listdir")
    @patch("os.path.isfile")
    @patch("os.path.getsize")
    def test_roslog_list_with_multiple_directories(
        self, mock_getsize, mock_isfile, mock_listdir, mock_get_roslog_directories
    ):
        mock_get_roslog_directories.return_value = {
            "default": "/mock/log/dir1",
            "latest": "/mock/log/dir2",
        }
        mock_listdir.side_effect = lambda x: (
            ["log1.log", "log2.log"] if "dir1" in x else ["log3.log", "log4.log"]
        )
        mock_isfile.side_effect = lambda x: x.endswith(".log")
        mock_getsize.side_effect = lambda x: 3000

        result = roslog_list.invoke({})

        self.assertEqual(result["total"], 2)
        self.assertEqual(len(result["logs"]), 2)
        self.assertEqual(result["logs"][0]["total"], 2)
        self.assertEqual(result["logs"][1]["total"], 2)

    @patch("rospy.loginfo")
    @patch("src.rosa.tools.ros1.get_entities")
    def test_rostopic_list_returns_all_topics(self, mock_get_entities, mock_loginfo):
        mock_get_entities.return_value = (10, 10, 10, ["topic1", "topic2"])
        result = rostopic_list.invoke({})
        self.assertEqual(result["total"], 10)
        self.assertEqual(result["in_namespace"], 10)
        self.assertEqual(result["match_pattern"], 10)
        self.assertEqual(result["topics"], ["topic1", "topic2"])

    @patch("rospy.loginfo")
    @patch("src.rosa.tools.ros1.get_entities")
    def test_rostopic_list_with_pattern(self, mock_get_entities, mock_loginfo):
        mock_get_entities.return_value = (10, 10, 2, ["topic1", "topic2"])
        result = rostopic_list.invoke({"pattern": "topic"})
        self.assertEqual(result["match_pattern"], 2)
        self.assertEqual(result["topics"], ["topic1", "topic2"])

    @patch("rospy.loginfo")
    @patch("src.rosa.tools.ros1.get_entities")
    def test_rostopic_list_with_namespace(self, mock_get_entities, mock_loginfo):
        mock_get_entities.return_value = (
            10,
            5,
            5,
            ["namespace/topic1", "namespace/topic2"],
        )
        result = rostopic_list.invoke({"namespace": "namespace"})
        self.assertEqual(result["in_namespace"], 5)
        self.assertEqual(result["topics"], ["namespace/topic1", "namespace/topic2"])

    @patch("rospy.loginfo")
    @patch("src.rosa.tools.ros1.get_entities")
    def test_rostopic_list_with_blacklist(self, mock_get_entities, mock_loginfo):
        mock_get_entities.return_value = (2, 2, 2, ["topic1", "topic2"])
        result = rostopic_list.invoke({"blacklist": ["topic2"]})
        self.assertEqual(result["topics"], ["topic1"])

    @patch("rospy.loginfo")
    @patch("src.rosa.tools.ros1.get_entities")
    def test_rostopic_list_no_topics_available(self, mock_get_entities, mock_loginfo):
        mock_get_entities.return_value = (
            0,
            0,
            0,
            ["There are currently no topics available in the system."],
        )
        result = rostopic_list.invoke({})
        self.assertEqual(result["total"], 0)
        self.assertEqual(
            result["topics"], ["There are currently no topics available in the system."]
        )

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_topics(self, mock_get_node_names, mock_get_topic_list):
        mock_get_topic_list.return_value = (
            [(f"/topic{i}", "type") for i in range(5)],
            [(f"/topic{i}", "type") for i in range(5, 10)],
        )
        total, in_namespace, match_pattern, entities = get_entities("topic", None, None)
        self.assertEqual(total, 10)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 10)
        self.assertEqual(len(entities), 10)

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_nodes(self, mock_get_node_names, mock_get_topic_list):
        mock_get_node_names.return_value = [f"/node{i}" for i in range(10)]
        total, in_namespace, match_pattern, entities = get_entities("node", None, None)
        self.assertEqual(total, 10)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 10)
        self.assertEqual(len(entities), 10)

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_with_namespace(
        self, mock_get_node_names, mock_get_topic_list
    ):
        mock_get_topic_list.return_value = (
            [(f"/namespace/topic{i}", "type") for i in range(5)]
            + [(f"/topic{i}", "type") for i in range(5)],
            [(f"/namespace/topic{i}", "type") for i in range(5, 10)],
        )

        mock_get_node_names.return_value = [
            f"/namespace/node{i}" for i in range(10)
        ] + [f"/node{i}" for i in range(10)]

        total, in_namespace, match_pattern, entities = get_entities(
            "topic", None, "/namespace"
        )
        self.assertEqual(total, 15)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 10)
        self.assertEqual(len(entities), 10)

        total, in_namespace, match_pattern, entities = get_entities(
            "node", None, "/namespace"
        )
        self.assertEqual(total, 20)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 10)
        self.assertEqual(len(entities), 10)

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_with_pattern(self, mock_get_node_names, mock_get_topic_list):
        mock_get_topic_list.return_value = (
            [(f"/topic{i}", "type") for i in range(5)],
            [(f"/topic{i}", "type") for i in range(5, 10)],
        )
        total, in_namespace, match_pattern, entities = get_entities(
            "topic", "topic[0-4]", None
        )
        self.assertEqual(total, 10)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 5)
        self.assertEqual(len(entities), 5)

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_with_blacklist(
        self, mock_get_node_names, mock_get_topic_list
    ):
        mock_get_topic_list.return_value = (
            [(f"/topic{i}", "type") for i in range(5)],
            [(f"/topic{i}", "type") for i in range(5, 10)],
        )
        total, in_namespace, match_pattern, entities = get_entities(
            "topic", None, None, blacklist=["/topic0", "/topic1"]
        )
        self.assertEqual(total, 10)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 10)
        self.assertEqual(len(entities), 8)

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_no_entities(self, mock_get_node_names, mock_get_topic_list):
        mock_get_topic_list.return_value = ([], [])
        total, in_namespace, match_pattern, entities = get_entities("topic", None, None)
        self.assertEqual(total, 0)
        self.assertEqual(in_namespace, 0)
        self.assertEqual(match_pattern, 0)
        self.assertEqual(
            entities, ["There are currently no topics available in the system."]
        )

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_no_namespace_entities(
        self, mock_get_node_names, mock_get_topic_list
    ):
        mock_get_topic_list.return_value = (
            [(f"/topic{i}", "type") for i in range(5)],
            [(f"/topic{i}", "type") for i in range(5, 10)],
        )
        total, in_namespace, match_pattern, entities = get_entities(
            "topic", None, "/nonexistent"
        )
        self.assertEqual(total, 10)
        self.assertEqual(in_namespace, 0)
        self.assertEqual(match_pattern, 0)
        self.assertEqual(
            entities,
            [
                "There are currently no topics available using the '/nonexistent' namespace."
            ],
        )

    @patch("src.rosa.tools.ros1.rostopic.get_topic_list")
    @patch("src.rosa.tools.ros1.rosnode.get_node_names")
    def test_get_entities_no_pattern_entities(
        self, mock_get_node_names, mock_get_topic_list
    ):
        mock_get_topic_list.return_value = (
            [(f"/topic{i}", "type") for i in range(5)],
            [(f"/topic{i}", "type") for i in range(5, 10)],
        )
        total, in_namespace, match_pattern, entities = get_entities(
            "topic", "nonexistent", None
        )
        self.assertEqual(total, 10)
        self.assertEqual(in_namespace, 10)
        self.assertEqual(match_pattern, 0)
        self.assertEqual(
            entities,
            ["There are currently no topics available matching the specified pattern."],
        )


if __name__ == "__main__":
    unittest.main()
