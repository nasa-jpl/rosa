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
        if self.ros_version == 1:
            tools = ROSATools(ros_version=1)
            self.assertEqual(tools._ROSATools__ros_version, 1)
        else:
            with self.assertRaises(ModuleNotFoundError):
                tools = ROSATools(ros_version=1)
                self.assertEqual(tools._ROSATools__ros_version, 1)

    def test_initializes_with_ros_version_2(self):
        if self.ros_version == 2:
            tools = ROSATools(ros_version=2)
            self.assertEqual(tools._ROSATools__ros_version, 2)
        else:
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
        if self.ros_version == 1:
            tools = ROSATools(ros_version=1)
        else:
            tools = ROSATools(ros_version=2)
        self.assertIn(mock_calculation.return_value, tools.get_tools())
        self.assertIn(mock_log.return_value, tools.get_tools())
        self.assertIn(mock_system.return_value, tools.get_tools())

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


@unittest.skipIf(os.environ.get("ROS_VERSION") == "2", "Skipping ROS 1 tests")
class TestROSA1Tools(unittest.TestCase):
    @patch("src.rosa.tools.ros1")
    def test_ros1_tools(self, mock_ros1):
        tools = ROSATools(ros_version=1)
        self.assertIn(mock_ros1.return_value, tools.get_tools())
        with self.assertRaises(ModuleNotFoundError):
            tools = ROSATools(ros_version=2)
            self.assertIn(mock_ros1.return_value, tools.get_tools())


@unittest.skipIf(os.environ.get("ROS_VERSION") == "1", "Skipping ROS 2 tests")
class TestROSA2Tools(unittest.TestCase):
    @patch("src.rosa.tools.ros2")
    def test_ros2_tools(self, mock_ros2):
        tools = ROSATools(ros_version=2)
        self.assertIn(mock_ros2.return_value, tools.get_tools())
        with self.assertRaises(ModuleNotFoundError):
            tools = ROSATools(ros_version=1)
            self.assertIn(mock_ros2.return_value, tools.get_tools())

class TestROSAToolModules(unittest.TestCase):
    """Tests for the tool_modules parameter introduced to allow selective
    loading of built-in ROSA tool modules."""

    def test_empty_tool_modules_loads_no_builtin_tools(self):
        """Passing tool_modules=set() should result in zero built-in tools."""
        ros_version = int(os.getenv("ROS_VERSION", 1))
        tools = ROSATools(ros_version=ros_version, tool_modules=set())
        self.assertEqual(len(tools.get_tools()), 0)

    def test_none_tool_modules_loads_all_builtin_tools(self):
        """Passing tool_modules=None should load all built-in modules (default behaviour)."""
        from src.rosa.tools import DEFAULT_MODULES, ALL_MODULES
        self.assertEqual(DEFAULT_MODULES, ALL_MODULES)

    def test_selective_tool_modules_loads_only_specified(self):
        """Passing tool_modules={'calculation'} should load only the specified modules
        and not load others."""
        ros_version = int(os.getenv("ROS_VERSION", 1))
        # With only calculation, we get some tools
        tools_calc = ROSATools(ros_version=ros_version, tool_modules={"calculation"})
        count_calc = len(tools_calc.get_tools())
        # With empty set, we get zero tools
        tools_empty = ROSATools(ros_version=ros_version, tool_modules=set())
        count_empty = len(tools_empty.get_tools())
        # With all modules, we get more tools than with just calculation
        tools_all = ROSATools(ros_version=ros_version, tool_modules=None)
        count_all = len(tools_all.get_tools())
        # Selective loading should give more than empty but less than all
        self.assertEqual(count_empty, 0)
        self.assertGreater(count_calc, count_empty)
        self.assertLess(count_calc, count_all)

    def test_unknown_module_raises_warning(self):
        """Passing an unknown module name should raise a UserWarning."""
        ros_version = int(os.getenv("ROS_VERSION", 1))
        with self.assertWarns(UserWarning):
            ROSATools(ros_version=ros_version, tool_modules={"unknown_module"})

    def test_tool_modules_defaults_to_all(self):
        """Not passing tool_modules should default to all built-in modules."""
        from src.rosa.tools import DEFAULT_MODULES, ALL_MODULES
        ros_version = int(os.getenv("ROS_VERSION", 1))
        tools = ROSATools(ros_version=ros_version, tool_modules=None)
        self.assertEqual(tools._ROSATools__tool_modules, ALL_MODULES)

    def test_custom_tools_work_with_empty_tool_modules(self):
        """Custom tools should still be added when tool_modules=set()."""
        @tool
        def my_custom_tool() -> str:
            """A custom tool."""
            return "custom"

        ros_version = int(os.getenv("ROS_VERSION", 1))
        rosa_tools = ROSATools(ros_version=ros_version, tool_modules=set())
        rosa_tools.add_tools([my_custom_tool])
        self.assertEqual(len(rosa_tools.get_tools()), 1)
        self.assertEqual(rosa_tools.get_tools()[0].name, "my_custom_tool")

if __name__ == "__main__":
    unittest.main()
