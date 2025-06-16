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

"""
Tests for the improved context-based blacklist implementation.

This test suite validates the context-based approach as a cleaner alternative
to the current decorator-based blacklist injection system.
"""

import threading
import time
import unittest
import unittest.mock
from unittest.mock import patch

from src.rosa.tools.blacklist_context import (
    BlacklistContext,
    blacklist_context,
    filter_entities_with_blacklist,
    get_current_blacklist,
    merge_blacklists,
)
from src.rosa.tools.improved_ros_tools_example import (
    improved_ros_node_list,
    improved_ros_service_list,
    improved_ros_topic_list,
)
from src.rosa.tools.improved_rosa_tools import ImprovedROSATools


class TestBlacklistContext(unittest.TestCase):
    """Test the BlacklistContext implementation."""

    def setUp(self):
        self.context = BlacklistContext()

    def test_context_blacklist_scope(self):
        """Test basic blacklist context functionality."""
        # Initially no blacklist
        self.assertIsNone(self.context.get_current_blacklist())

        # Set blacklist in scope
        test_blacklist = ["debug", "system"]
        with self.context.blacklist_scope(test_blacklist):
            self.assertEqual(self.context.get_current_blacklist(), test_blacklist)

        # Blacklist cleared after scope
        self.assertIsNone(self.context.get_current_blacklist())

    def test_nested_blacklist_scopes(self):
        """Test nested blacklist contexts."""
        outer_blacklist = ["outer"]
        inner_blacklist = ["inner"]

        with self.context.blacklist_scope(outer_blacklist):
            self.assertEqual(self.context.get_current_blacklist(), outer_blacklist)

            with self.context.blacklist_scope(inner_blacklist):
                self.assertEqual(self.context.get_current_blacklist(), inner_blacklist)

            # Restored to outer scope
            self.assertEqual(self.context.get_current_blacklist(), outer_blacklist)

        # Fully restored
        self.assertIsNone(self.context.get_current_blacklist())

    def test_thread_safety(self):
        """Test that blacklist context is thread-local."""
        results = {}

        def thread_function(thread_id, blacklist):
            with self.context.blacklist_scope(blacklist):
                # Store result from this thread
                results[thread_id] = self.context.get_current_blacklist()
                # Sleep to ensure threads overlap
                time.sleep(0.1)
                # Verify blacklist is still correct after sleep
                results[f"{thread_id}_after_sleep"] = (
                    self.context.get_current_blacklist()
                )

        # Start multiple threads with different blacklists
        threads = []
        for i in range(3):
            blacklist = [f"thread_{i}"]
            thread = threading.Thread(target=thread_function, args=(i, blacklist))
            threads.append(thread)
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

        # Verify each thread had its own blacklist
        for i in range(3):
            expected = [f"thread_{i}"]
            self.assertEqual(results[i], expected)
            self.assertEqual(results[f"{i}_after_sleep"], expected)

    def test_merge_blacklists(self):
        """Test blacklist merging functionality."""
        context_blacklist = ["context1", "context2"]
        user_blacklist = ["user1", "user2"]

        with self.context.blacklist_scope(context_blacklist):
            merged = self.context.merge_blacklists(user_blacklist)
            expected = context_blacklist + user_blacklist
            self.assertEqual(merged, expected)

    def test_merge_blacklists_with_none(self):
        """Test blacklist merging with None values."""
        context_blacklist = ["context1"]

        with self.context.blacklist_scope(context_blacklist):
            # None user blacklist
            merged = self.context.merge_blacklists(None)
            self.assertEqual(merged, context_blacklist)

        # No context blacklist
        merged = self.context.merge_blacklists(["user1"])
        self.assertEqual(merged, ["user1"])

        # Both None
        merged = self.context.merge_blacklists(None)
        self.assertEqual(merged, [])


class TestGlobalContextFunctions(unittest.TestCase):
    """Test the global convenience functions."""

    def test_get_current_blacklist(self):
        """Test global get_current_blacklist function."""
        test_blacklist = ["global_test"]

        # No blacklist initially
        self.assertIsNone(get_current_blacklist())

        # With blacklist context
        with blacklist_context.blacklist_scope(test_blacklist):
            self.assertEqual(get_current_blacklist(), test_blacklist)

        # Cleared after context
        self.assertIsNone(get_current_blacklist())

    def test_merge_blacklists_global(self):
        """Test global merge_blacklists function."""
        context_bl = ["context"]
        user_bl = ["user"]

        with blacklist_context.blacklist_scope(context_bl):
            merged = merge_blacklists(user_bl)
            self.assertEqual(merged, context_bl + user_bl)

    def test_filter_entities_with_blacklist(self):
        """Test the utility filtering function."""
        entities = ["topic1", "debug_topic", "system_info", "user_data"]
        context_bl = ["debug"]
        user_bl = ["system"]

        # Test with context blacklist only
        with blacklist_context.blacklist_scope(context_bl):
            filtered = filter_entities_with_blacklist(entities)
            expected = ["topic1", "system_info", "user_data"]  # debug_topic filtered
            self.assertEqual(filtered, expected)

        # Test with both context and user blacklist
        with blacklist_context.blacklist_scope(context_bl):
            filtered = filter_entities_with_blacklist(entities, user_bl)
            expected = [
                "topic1",
                "user_data",
            ]  # both debug_topic and system_info filtered
            self.assertEqual(filtered, expected)

    def test_filter_entities_no_blacklist(self):
        """Test filtering with no blacklist context."""
        entities = ["topic1", "debug_topic", "system_info"]

        # No context, no user blacklist
        filtered = filter_entities_with_blacklist(entities)
        self.assertEqual(filtered, entities)  # No filtering applied

        # No context, with user blacklist
        filtered = filter_entities_with_blacklist(entities, ["debug"])
        expected = ["topic1", "system_info"]
        self.assertEqual(filtered, expected)


class TestImprovedROSATools(unittest.TestCase):
    """Test the improved ROSATools implementation."""

    def test_improved_tools_initialization_simple(self):
        """Test ImprovedROSATools can be created with simple mocking."""
        # Test basic initialization without complex ROS dependencies
        with (
            patch(
                "src.rosa.tools.improved_rosa_tools.ImprovedROSATools._ImprovedROSATools__load_core_tools"
            ),
            patch(
                "src.rosa.tools.improved_rosa_tools.ImprovedROSATools._ImprovedROSATools__load_ros_tools"
            ),
        ):
            blacklist = ["debug", "system"]
            tools = ImprovedROSATools(ros_version=1, blacklist=blacklist)

            # Basic initialization should work
            self.assertEqual(tools._ImprovedROSATools__blacklist, blacklist)
            self.assertEqual(tools._ImprovedROSATools__ros_version, 1)

    def test_execute_with_blacklist_context(self):
        """Test execution within blacklist context."""
        blacklist = ["test_blacklist"]

        with (
            patch(
                "src.rosa.tools.improved_rosa_tools.ImprovedROSATools._ImprovedROSATools__load_core_tools"
            ),
            patch(
                "src.rosa.tools.improved_rosa_tools.ImprovedROSATools._ImprovedROSATools__load_ros_tools"
            ),
        ):
            tools = ImprovedROSATools(ros_version=1, blacklist=blacklist)

            # Track if context was set correctly during execution
            context_during_execution = None

            def mock_execution():
                nonlocal context_during_execution
                context_during_execution = get_current_blacklist()
                return "execution_result"

            # Execute within context
            result = tools.execute_with_blacklist_context(mock_execution)

            # Verify context was set during execution
            self.assertEqual(context_during_execution, blacklist)
            self.assertEqual(result, "execution_result")

            # Verify context is cleared after execution
            self.assertIsNone(get_current_blacklist())


class TestImprovedToolExamples(unittest.TestCase):
    """Test the example improved tool implementations."""

    def test_improved_ros_topic_list(self):
        """Test improved topic list tool with context blacklist."""
        blacklist = ["debug", "rosout"]

        # Without blacklist context
        result = improved_ros_topic_list.func()
        original_count = result["total_found"]
        self.assertEqual(result["after_filtering"], original_count)  # No filtering

        # With blacklist context
        with blacklist_context.blacklist_scope(blacklist):
            result = improved_ros_topic_list.func()

        # Should have filtered out debug and rosout topics
        self.assertLess(result["after_filtering"], result["total_found"])
        for topic in result["topics"]:
            self.assertNotIn("debug", topic)
            self.assertNotIn("rosout", topic)

    def test_improved_ros_node_list_with_user_blacklist(self):
        """Test improved node list with both context and user blacklists."""
        context_bl = ["debug"]
        user_bl = ["system"]

        with blacklist_context.blacklist_scope(context_bl):
            result = improved_ros_node_list.func(blacklist=user_bl)

        # Verify both blacklists were applied
        self.assertEqual(result["context_blacklist"], context_bl)
        self.assertEqual(result["user_blacklist"], user_bl)
        self.assertLess(result["after_filtering"], result["total_found"])

        # Verify filtered results don't contain blacklisted patterns
        for node in result["nodes"]:
            self.assertNotIn("debug", node)
            self.assertNotIn("system", node)

    def test_improved_ros_service_list_utility_approach(self):
        """Test service list tool using utility function approach."""
        blacklist = ["debug", "system"]

        with blacklist_context.blacklist_scope(blacklist):
            result = improved_ros_service_list.func()

        # Verify filtering was applied
        self.assertLess(result["after_filtering"], result["total_found"])

        # Verify no blacklisted services in results
        for service in result["services"]:
            self.assertNotIn("debug", service)
            self.assertNotIn("system", service)


class TestMigrationBenefits(unittest.TestCase):
    """Demonstrate benefits of the improved approach."""

    def test_no_function_wrapping(self):
        """Verify that tools are not wrapped/modified at runtime."""
        with (
            patch(
                "src.rosa.tools.improved_rosa_tools.ImprovedROSATools._ImprovedROSATools__load_core_tools"
            ),
            patch(
                "src.rosa.tools.improved_rosa_tools.ImprovedROSATools._ImprovedROSATools__load_ros_tools"
            ),
        ):
            tools = ImprovedROSATools(ros_version=1, blacklist=["test"])

            # Tools should maintain their original function references
            for tool in tools.get_tools():
                if hasattr(tool, "func"):
                    # Function should not be wrapped (would have different __name__ if wrapped)
                    self.assertFalse(hasattr(tool.func, "__wrapped__"))

    def test_easy_debugging(self):
        """Demonstrate that debugging is easier without function wrapping."""
        # Tools maintain original signatures and stack traces
        import inspect

        # Original function signature is preserved
        sig = inspect.signature(improved_ros_topic_list.func)
        self.assertIn("pattern", sig.parameters)
        self.assertNotIn(
            "blacklist", sig.parameters
        )  # Not forced to have blacklist param

    def test_flexible_blacklist_usage(self):
        """Demonstrate flexibility in how tools can use blacklist context."""
        blacklist = ["flexible_test"]

        # Tool can choose whether to use context blacklist
        def flexible_tool_with_context():
            return get_current_blacklist()

        def flexible_tool_without_context():
            return "no_blacklist_needed"

        with blacklist_context.blacklist_scope(blacklist):
            # Tool that uses context gets blacklist
            result1 = flexible_tool_with_context()
            self.assertEqual(result1, blacklist)

            # Tool that doesn't use context works fine
            result2 = flexible_tool_without_context()
            self.assertEqual(result2, "no_blacklist_needed")


if __name__ == "__main__":
    unittest.main()
