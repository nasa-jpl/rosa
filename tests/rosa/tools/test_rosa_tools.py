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
from unittest.mock import patch

from langchain.agents import tool

from src.rosa.tools import ROSATools, inject_blacklist


@tool
def sample_tool(blacklist=None):
    """A sample tool that returns the blacklist."""
    return blacklist


class TestROSATools(unittest.TestCase):
    def setUp(self):
        self.ros_version = int(os.getenv("ROS_VERSION", 1))

    def test_initializes_with_ros_version_1(self):
        tools = ROSATools(ros_version=1)
        self.assertEqual(tools._ROSATools__ros_version, 1)

    def test_initializes_with_ros_version_2(self):
        with self.assertRaises(ModuleNotFoundError):
            tools = ROSATools(ros_version=2)
            self.assertEqual(tools._ROSATools__ros_version, 2)

    def test_raises_value_error_for_invalid_ros_version(self):
        if self.ros_version == 1:
            with self.assertRaises(ModuleNotFoundError):
                ROSATools(ros_version=2)
        else:
            with self.assertRaises(ModuleNotFoundError):
                ROSATools(ros_version=1)

    @patch("src.rosa.tools.calculation")
    @patch("src.rosa.tools.log")
    @patch("src.rosa.tools.system")
    def test_adds_default_tools(self, mock_system, mock_log, mock_calculation):
        tools = ROSATools(ros_version=1)
        self.assertIn(mock_calculation.return_value, tools.get_tools())
        self.assertIn(mock_log.return_value, tools.get_tools())
        self.assertIn(mock_system.return_value, tools.get_tools())

    @patch("src.rosa.tools.ros1")
    def test_adds_ros1_tools(self, mock_ros1):
        tools = ROSATools(ros_version=1)
        if self.ros_version == 1:
            self.assertIn(mock_ros1.return_value, tools.get_tools())
        else:
            self.assertNotIn(mock_ros1.return_value, tools.get_tools())

    def test_adds_ros2_tools(self):
        if self.ros_version == 2:
            try:
                from src.rosa.tools import ros2

                with patch("src.rosa.tools.ros2") as mock_ros2:
                    tools = ROSATools(ros_version=2)
                    self.assertIn(mock_ros2, tools.get_tools())
            except ImportError:
                self.fail("ros2 module should be available for ROS version 2")
        else:
            self.skipTest("Skipping ROS2 tools test for ROS version 1")

    def test_injects_blacklist_into_tool_function(self):
        def sample_tool(blacklist=None):
            return blacklist

        decorated_tool = inject_blacklist(["item1", "item2"])(sample_tool)
        self.assertEqual(decorated_tool(), ["item1", "item2"])

    def test_blacklist_gets_concatenated(self):
        decorated_tool = inject_blacklist(["item1", "item2"])(sample_tool)
        self.assertEqual(
            decorated_tool({"blacklist": ["item3"]}),
            ["item1", "item2", "item3"],
        )


if __name__ == "__main__":
    unittest.main()
