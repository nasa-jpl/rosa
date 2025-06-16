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

import logging
import unittest
from unittest.mock import MagicMock, patch

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from rosa.helpers.chat_manager import ROSAChatManager


class TestROSAChatManager(unittest.TestCase):
    """Test ROSAChatManager class."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_logger = MagicMock(spec=logging.Logger)

        # Create chat manager instance
        self.manager = ROSAChatManager(
            max_history_length=10, accumulate_chat_history=True, logger=self.mock_logger
        )

    def test_chat_manager_initialization(self):
        """Test that ROSAChatManager initializes correctly."""
        self.assertEqual(self.manager._max_history_length, 10)
        self.assertTrue(self.manager._accumulate_chat_history)
        self.assertEqual(self.manager._logger, self.mock_logger)
        self.assertEqual(self.manager._chat_history, [])

    def test_chat_manager_initialization_with_defaults(self):
        """Test chat manager initialization with default values."""
        manager = ROSAChatManager()

        self.assertIsNone(manager._max_history_length)
        self.assertTrue(manager._accumulate_chat_history)
        self.assertIsNotNone(manager._logger)
        self.assertEqual(manager._chat_history, [])

    def test_get_chat_history(self):
        """Test that get_chat_history returns the current history."""
        test_history = [HumanMessage(content="test")]
        self.manager._chat_history = test_history

        result = self.manager.get_chat_history()
        self.assertEqual(result, test_history)

    def test_clear_chat_history_all_messages(self):
        """Test clearing all chat history."""
        self.manager._chat_history = [
            HumanMessage(content="test query"),
            AIMessage(content="test response"),
            SystemMessage(content="system message"),
        ]

        self.manager.clear_chat_history(retain_system_messages=False)

        self.assertEqual(len(self.manager._chat_history), 0)

    def test_clear_chat_history_retain_system_messages(self):
        """Test clearing chat history while retaining system messages."""
        system_msg = SystemMessage(content="system message")
        self.manager._chat_history = [
            HumanMessage(content="test query"),
            AIMessage(content="test response"),
            system_msg,
        ]

        self.manager.clear_chat_history(retain_system_messages=True)

        self.assertEqual(len(self.manager._chat_history), 1)
        self.assertEqual(self.manager._chat_history[0], system_msg)

    def test_record_chat_history_accumulation_enabled(self):
        """Test recording chat history when accumulation is enabled."""
        result = self.manager.record_chat_history("test query", "test response")

        self.assertTrue(result)  # Should return True for successful recording
        self.assertEqual(len(self.manager._chat_history), 2)
        self.assertIsInstance(self.manager._chat_history[0], HumanMessage)
        self.assertEqual(self.manager._chat_history[0].content, "test query")
        self.assertIsInstance(self.manager._chat_history[1], AIMessage)
        self.assertEqual(self.manager._chat_history[1].content, "test response")

    def test_record_chat_history_accumulation_disabled(self):
        """Test recording chat history when accumulation is disabled."""
        self.manager._accumulate_chat_history = False

        result = self.manager.record_chat_history("test query", "test response")

        self.assertFalse(result)  # Should return False when accumulation disabled
        self.assertEqual(len(self.manager._chat_history), 0)

    def test_record_chat_history_triggers_trimming(self):
        """Test that recording chat history triggers trimming when limit exceeded."""
        # Set up manager with small limit
        self.manager._max_history_length = 4

        # Add initial messages to approach the limit
        self.manager._chat_history = [
            HumanMessage(content="Query 1"),
            AIMessage(content="Response 1"),
        ]

        # Record new message that should trigger trimming
        result = self.manager.record_chat_history("Query 2", "Response 2")

        self.assertTrue(result)
        self.assertEqual(len(self.manager._chat_history), 4)  # Should be at the limit

        # Add one more to trigger trimming
        result = self.manager.record_chat_history("Query 3", "Response 3")

        self.assertTrue(result)
        self.assertEqual(len(self.manager._chat_history), 4)  # Should still be at limit
        # Should contain the most recent messages
        self.assertEqual(self.manager._chat_history[-1].content, "Response 3")
        self.assertEqual(self.manager._chat_history[-2].content, "Query 3")

    def test_record_chat_history_error_handling(self):
        """Test that record_chat_history handles errors gracefully."""
        # Mock trim_chat_history to raise an exception
        with patch.object(
            self.manager, "trim_chat_history", side_effect=Exception("Trimming failed")
        ):
            result = self.manager.record_chat_history("Query", "Response")

            # Should return False on error
            self.assertFalse(result)
            # Should log the error
            self.mock_logger.error.assert_called()

    def test_trim_chat_history_even_messages_basic(self):
        """Test trimming chat history with even number of messages."""
        chat_history = [
            HumanMessage(content="Query 1"),
            AIMessage(content="Response 1"),
            HumanMessage(content="Query 2"),
            AIMessage(content="Response 2"),
            HumanMessage(content="Query 3"),
            AIMessage(content="Response 3"),
        ]

        self.manager._max_history_length = 4
        trimmed = self.manager.trim_chat_history(chat_history)

        # Should keep the last 2 pairs (4 messages total)
        self.assertEqual(len(trimmed), 4)
        self.assertEqual(trimmed[0].content, "Query 2")
        self.assertEqual(trimmed[1].content, "Response 2")
        self.assertEqual(trimmed[2].content, "Query 3")
        self.assertEqual(trimmed[3].content, "Response 3")

    def test_trim_chat_history_odd_messages(self):
        """Test trimming chat history with odd number of messages."""
        chat_history = [
            HumanMessage(content="Query 1"),
            AIMessage(content="Response 1"),
            HumanMessage(content="Query 2"),
            AIMessage(content="Response 2"),
            HumanMessage(content="Query 3"),  # Incomplete pair
        ]

        self.manager._max_history_length = 4
        trimmed = self.manager.trim_chat_history(chat_history)

        # Should keep the incomplete message plus one complete pair (3 messages total)
        self.assertEqual(len(trimmed), 3)
        self.assertEqual(trimmed[0].content, "Query 2")
        self.assertEqual(trimmed[1].content, "Response 2")
        self.assertEqual(trimmed[2].content, "Query 3")

    def test_trim_chat_history_no_trimming_needed(self):
        """Test trimming when no trimming is needed."""
        chat_history = [
            HumanMessage(content="Query 1"),
            AIMessage(content="Response 1"),
        ]

        self.manager._max_history_length = 10
        trimmed = self.manager.trim_chat_history(chat_history)

        # Should return the same list when under the limit
        self.assertEqual(trimmed, chat_history)

    def test_trim_chat_history_unlimited_length(self):
        """Test trimming when max_history_length is None (unlimited)."""
        chat_history = [
            HumanMessage(content="Query 1"),
            AIMessage(content="Response 1"),
            HumanMessage(content="Query 2"),
            AIMessage(content="Response 2"),
        ]

        self.manager._max_history_length = None
        trimmed = self.manager.trim_chat_history(chat_history)

        # Should return the same list when unlimited
        self.assertEqual(trimmed, chat_history)

    def test_trim_chat_history_empty_list(self):
        """Test trimming an empty chat history."""
        trimmed = self.manager.trim_chat_history([])

        self.assertEqual(trimmed, [])

    def test_trim_chat_history_preserves_message_types(self):
        """Test that trimming preserves message types and content."""
        chat_history = [
            HumanMessage(content="Human message"),
            AIMessage(content="AI message"),
            HumanMessage(content="Another human message"),
            AIMessage(content="Another AI message"),
        ]

        self.manager._max_history_length = 2
        trimmed = self.manager.trim_chat_history(chat_history)

        self.assertEqual(len(trimmed), 2)
        self.assertIsInstance(trimmed[0], HumanMessage)
        self.assertIsInstance(trimmed[1], AIMessage)
        self.assertEqual(trimmed[0].content, "Another human message")
        self.assertEqual(trimmed[1].content, "Another AI message")

    def test_get_history_length(self):
        """Test getting the current history length."""
        self.assertEqual(self.manager.get_history_length(), 0)

        self.manager._chat_history = [
            HumanMessage(content="test"),
            AIMessage(content="response"),
        ]

        self.assertEqual(self.manager.get_history_length(), 2)

    def test_trim_history_public_method(self):
        """Test the public trim_history method."""
        self.manager._chat_history = [
            HumanMessage(content="Query 1"),
            AIMessage(content="Response 1"),
            HumanMessage(content="Query 2"),
            AIMessage(content="Response 2"),
            HumanMessage(content="Query 3"),
            AIMessage(content="Response 3"),
        ]

        self.manager.trim_history(4)

        # Should trim the internal history
        self.assertEqual(len(self.manager._chat_history), 4)
        self.assertEqual(self.manager._chat_history[0].content, "Query 2")

    def test_trim_history_invalid_length(self):
        """Test trim_history with invalid length parameters."""
        with self.assertRaises(ValueError):
            self.manager.trim_history(0)

        with self.assertRaises(ValueError):
            self.manager.trim_history(-1)

        with self.assertRaises(ValueError):
            self.manager.trim_history("invalid")

    def test_get_history_usage(self):
        """Test getting history usage information."""
        # Test with empty history
        usage = self.manager.get_history_usage()
        expected = {"message_count": 0, "estimated_tokens": 0, "memory_bytes": 0}
        self.assertEqual(usage, expected)

        # Test with some messages
        self.manager._chat_history = [
            HumanMessage(content="Hello"),
            AIMessage(content="Hi there!"),
        ]

        usage = self.manager.get_history_usage()

        self.assertEqual(usage["message_count"], 2)
        self.assertGreater(usage["estimated_tokens"], 0)
        self.assertGreater(usage["memory_bytes"], 0)
        self.assertIsInstance(usage["estimated_tokens"], int)
        self.assertIsInstance(usage["memory_bytes"], int)


class TestROSAChatManagerIntegration(unittest.TestCase):
    """Test ROSAChatManager integration scenarios."""

    def test_full_chat_workflow(self):
        """Test complete chat workflow with multiple operations."""
        manager = ROSAChatManager(max_history_length=6, accumulate_chat_history=True)

        # Add several conversations
        for i in range(5):
            manager.record_chat_history(f"Query {i}", f"Response {i}")

        # Should have triggered trimming
        self.assertEqual(manager.get_history_length(), 6)

        # Check usage information
        usage = manager.get_history_usage()
        self.assertEqual(usage["message_count"], 6)
        self.assertGreater(usage["estimated_tokens"], 0)

        # Manually trim to smaller size
        manager.trim_history(4)
        self.assertEqual(manager.get_history_length(), 4)

        # Clear history
        manager.clear_chat_history()
        self.assertEqual(manager.get_history_length(), 0)

    def test_concurrent_operations_simulation(self):
        """Test simulated concurrent operations on chat history."""
        manager = ROSAChatManager(max_history_length=8)

        # Simulate multiple rapid operations
        for i in range(10):
            manager.record_chat_history(
                f"Concurrent query {i}", f"Concurrent response {i}"
            )
            if i % 3 == 0:
                usage = manager.get_history_usage()
                self.assertIsInstance(usage, dict)

        # Should maintain consistency
        self.assertLessEqual(manager.get_history_length(), 8)

    def test_edge_case_small_history_limit(self):
        """Test edge case with very small history limit."""
        manager = ROSAChatManager(max_history_length=1)

        # Should handle single message limit
        manager.record_chat_history("Query", "Response")
        self.assertEqual(manager.get_history_length(), 1)

        # Should trim when adding another
        manager.record_chat_history("New Query", "New Response")
        self.assertEqual(manager.get_history_length(), 1)
        self.assertEqual(manager._chat_history[0].content, "New Response")


if __name__ == "__main__":
    unittest.main()
