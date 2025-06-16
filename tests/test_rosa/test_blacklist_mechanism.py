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
import unittest
from unittest.mock import MagicMock

from src.rosa.tools import ROSATools, inject_blacklist


class TestBlacklistInjection(unittest.TestCase):
    """Comprehensive tests for the blacklist injection mechanism."""

    def test_inject_blacklist_with_no_parameters(self):
        """Test that blacklist is injected when function is called with no parameters."""

        def test_func(blacklist=None):
            return blacklist

        decorated = inject_blacklist(["default1", "default2"])(test_func)
        result = decorated()

        self.assertEqual(result, ["default1", "default2"])

    def test_inject_blacklist_with_dict_args_no_blacklist(self):
        """Test blacklist injection when called with dict args without existing blacklist."""

        def test_func(params):
            return params.get("blacklist")

        decorated = inject_blacklist(["default1"])(test_func)
        result = decorated({"other_param": "value"})

        self.assertEqual(result, ["default1"])

    def test_inject_blacklist_with_dict_args_existing_blacklist(self):
        """Test blacklist concatenation when dict args contain existing blacklist."""

        def test_func(params):
            return params.get("blacklist")

        decorated = inject_blacklist(["default1"])(test_func)
        result = decorated({"blacklist": ["user1", "user2"]})

        self.assertEqual(result, ["default1", "user1", "user2"])

    def test_inject_blacklist_with_kwargs_no_blacklist(self):
        """Test blacklist injection with keyword arguments without existing blacklist."""

        def test_func(_param1=None, blacklist=None):
            return blacklist

        decorated = inject_blacklist(["default1"])(test_func)
        result = decorated(_param1="value")

        self.assertEqual(result, ["default1"])

    def test_inject_blacklist_with_kwargs_existing_blacklist(self):
        """Test blacklist concatenation with existing blacklist in kwargs."""

        def test_func(_param1=None, blacklist=None):
            return blacklist

        decorated = inject_blacklist(["default1"])(test_func)
        result = decorated(_param1="value", blacklist=["user1"])

        self.assertEqual(result, ["default1", "user1"])

    def test_inject_blacklist_preserves_signature(self):
        """Test that function signature is preserved after decoration."""

        def test_func(_param1: str, _param2: int = 10, blacklist=None):
            return blacklist

        decorated = inject_blacklist(["default"])(test_func)
        original_sig = inspect.signature(test_func)
        decorated_sig = inspect.signature(decorated)

        # Check that parameter names and types are preserved
        self.assertEqual(
            list(original_sig.parameters.keys()), list(decorated_sig.parameters.keys())
        )

        # Check that blacklist parameter default is updated
        blacklist_param = decorated_sig.parameters["blacklist"]
        self.assertEqual(blacklist_param.default, ["default"])

    def test_inject_blacklist_function_without_blacklist_parameter(self):
        """Test that functions without blacklist parameter are not affected."""

        def test_func(_param1: str):
            return _param1

        decorated = inject_blacklist(["default"])(test_func)
        result = decorated("test_value")

        self.assertEqual(result, "test_value")

    def test_inject_blacklist_with_multiple_defaults(self):
        """Test injection with multiple default blacklist items."""

        def test_func(blacklist=None):
            return blacklist

        decorated = inject_blacklist(["item1", "item2", "item3"])(test_func)
        result = decorated()

        self.assertEqual(result, ["item1", "item2", "item3"])

    def test_inject_blacklist_preserves_function_metadata(self):
        """Test that function metadata like __name__ and __doc__ are preserved."""

        def test_func(blacklist=None):
            """Test function docstring."""
            return blacklist

        decorated = inject_blacklist(["default"])(test_func)

        self.assertEqual(decorated.__name__, "test_func")
        self.assertEqual(decorated.__doc__, "Test function docstring.")

    def test_inject_blacklist_with_empty_default_list(self):
        """Test injection with empty default blacklist."""

        def test_func(blacklist=None):
            return blacklist

        decorated = inject_blacklist([])(test_func)
        result = decorated()

        self.assertEqual(result, [])

    def test_inject_blacklist_concatenation_order(self):
        """Test that default blacklist comes before user blacklist."""

        def test_func(blacklist=None):
            return blacklist

        decorated = inject_blacklist(["default1", "default2"])(test_func)
        result = decorated(blacklist=["user1", "user2"])

        self.assertEqual(result, ["default1", "default2", "user1", "user2"])


class TestROSAToolsBlacklistIntegration(unittest.TestCase):
    """Test blacklist integration within ROSATools."""

    def test_rosa_tools_blacklist_parameter_detection_with_blacklist(self):
        """Test that __add_tool correctly detects and wraps tools with blacklist parameters."""
        # Create a simple ROSATools instance to test the add_tool method
        rosa_tools = ROSATools.__new__(ROSATools)  # Create without calling __init__
        rosa_tools._ROSATools__tools = []
        rosa_tools._ROSATools__blacklist = ["default_item"]

        # Create a mock tool with blacklist parameter
        mock_tool = MagicMock()
        mock_tool.name = "test_tool"
        mock_tool.func = MagicMock()
        mock_tool.func.__code__ = MagicMock()
        mock_tool.func.__code__.co_varnames = ("_param1", "blacklist")

        original_func = mock_tool.func

        # Add the tool
        rosa_tools._ROSATools__add_tool(mock_tool)

        # Verify the function was wrapped (should be different object)
        self.assertNotEqual(mock_tool.func, original_func)
        self.assertEqual(len(rosa_tools._ROSATools__tools), 1)

    def test_rosa_tools_no_blacklist_injection_without_parameter(self):
        """Test that tools without blacklist parameter are not modified."""
        # Create a simple ROSATools instance to test the add_tool method
        rosa_tools = ROSATools.__new__(ROSATools)  # Create without calling __init__
        rosa_tools._ROSATools__tools = []
        rosa_tools._ROSATools__blacklist = ["default_item"]

        # Create a mock tool without blacklist parameter
        mock_tool = MagicMock()
        mock_tool.name = "test_tool"
        mock_tool.func = MagicMock()
        mock_tool.func.__code__ = MagicMock()
        mock_tool.func.__code__.co_varnames = ("_param1", "_param2")

        original_func = mock_tool.func

        # Add the tool
        rosa_tools._ROSATools__add_tool(mock_tool)

        # Verify the function was NOT wrapped (should be same object)
        self.assertEqual(mock_tool.func, original_func)
        self.assertEqual(len(rosa_tools._ROSATools__tools), 1)

    def test_rosa_tools_no_blacklist_injection_when_no_default_blacklist(self):
        """Test that no injection occurs when ROSATools has no default blacklist."""
        # Create a simple ROSATools instance to test the add_tool method
        rosa_tools = ROSATools.__new__(ROSATools)  # Create without calling __init__
        rosa_tools._ROSATools__tools = []
        rosa_tools._ROSATools__blacklist = None  # No blacklist

        # Create a mock tool with blacklist parameter
        mock_tool = MagicMock()
        mock_tool.name = "test_tool"
        mock_tool.func = MagicMock()
        mock_tool.func.__code__ = MagicMock()
        mock_tool.func.__code__.co_varnames = ("_param1", "blacklist")

        original_func = mock_tool.func

        # Add the tool
        rosa_tools._ROSATools__add_tool(mock_tool)

        # Verify the function was NOT wrapped since no default blacklist
        self.assertEqual(mock_tool.func, original_func)
        self.assertEqual(len(rosa_tools._ROSATools__tools), 1)


class TestBlacklistFunctionalBehavior(unittest.TestCase):
    """Test the functional behavior of blacklist filtering."""

    def test_blacklist_filtering_behavior(self):
        """Test that blacklist actually filters content as expected."""

        def mock_ros_tool_func(topics: list, blacklist=None):
            """Mock ROS tool that filters topics by blacklist."""
            if not blacklist:
                return topics

            # Simulate regex filtering like in actual ROS tools
            import re

            filtered = []
            for topic in topics:
                should_exclude = False
                for bl_item in blacklist:
                    if re.search(f".*{bl_item}.*", topic):
                        should_exclude = True
                        break
                if not should_exclude:
                    filtered.append(topic)
            return filtered

        # Test without blacklist injection
        topics = ["/turtle1/cmd_vel", "/turtle1/pose", "/rosout", "/debug_info"]
        result = mock_ros_tool_func(topics)
        self.assertEqual(len(result), 4)  # All topics present

        # Test with blacklist injection
        decorated_tool = inject_blacklist(["debug", "rosout"])(mock_ros_tool_func)
        result = decorated_tool(topics)

        # Should filter out topics containing "debug" or "rosout"
        self.assertEqual(len(result), 2)
        self.assertIn("/turtle1/cmd_vel", result)
        self.assertIn("/turtle1/pose", result)
        self.assertNotIn("/rosout", result)
        self.assertNotIn("/debug_info", result)

    def test_blacklist_user_override_and_concatenation(self):
        """Test that user-provided blacklist is concatenated with defaults."""

        def mock_filter_tool_func(items: list, blacklist=None):
            """Mock tool that filters items."""
            if not blacklist:
                return items
            return [item for item in items if not any(bl in item for bl in blacklist)]

        decorated_tool = inject_blacklist(["system"])(mock_filter_tool_func)

        items = ["user_item", "system_item", "debug_item", "normal_item"]

        # Test with additional user blacklist
        result = decorated_tool(items, blacklist=["debug"])

        # Should filter both "system" (default) and "debug" (user-provided)
        expected = ["user_item", "normal_item"]
        self.assertEqual(result, expected)


if __name__ == "__main__":
    unittest.main()
