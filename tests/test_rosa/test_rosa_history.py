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

import unittest
from unittest.mock import MagicMock, patch

from langchain_core.messages import AIMessage, HumanMessage
from langchain_openai import ChatOpenAI

from rosa import ROSA
from rosa.exceptions import ROSAConfigurationError


class TestROSAHistoryLength(unittest.TestCase):
    """Test max_history_length parameter functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_history_length_parameter_acceptance(self):
        """Test that max_history_length parameter is accepted in constructor."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Test with positive integer
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=100)
            self.assertEqual(rosa._ROSA__max_history_length, 100)

            # Test with None (unlimited)
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=None)
            self.assertIsNone(rosa._ROSA__max_history_length)

            # Test with default value (50)
            rosa = ROSA(ros_version=1, llm=self.mock_llm)
            self.assertEqual(rosa._ROSA__max_history_length, 50)

    def test_history_length_validation_positive_integer(self):
        """Test that max_history_length must be positive integer or None."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Valid positive integers should work
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=1)
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=50)
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=1000)

    def test_history_length_validation_none(self):
        """Test that max_history_length=None is accepted for unlimited history."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # None should work for unlimited history
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=None)
            self.assertIsNone(rosa._ROSA__max_history_length)

    def test_history_length_validation_invalid_zero(self):
        """Test that max_history_length=0 raises ROSAConfigurationError."""
        with self.assertRaisesRegex(
            ROSAConfigurationError,
            "Invalid max_history_length: 0. Must be a positive integer or None.",
        ):
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=0)

    def test_history_length_validation_invalid_negative(self):
        """Test that negative max_history_length raises ROSAConfigurationError."""
        with self.assertRaisesRegex(
            ROSAConfigurationError,
            "Invalid max_history_length: -1. Must be a positive integer or None.",
        ):
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=-1)

        with self.assertRaisesRegex(
            ROSAConfigurationError,
            "Invalid max_history_length: -100. Must be a positive integer or None.",
        ):
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=-100)

    def test_history_length_validation_invalid_type_string(self):
        """Test that string max_history_length raises ROSAConfigurationError."""
        with self.assertRaisesRegex(
            ROSAConfigurationError,
            "Invalid max_history_length: invalid. Must be a positive integer or None.",
        ):
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length="invalid")

    def test_history_length_validation_invalid_type_float(self):
        """Test that float max_history_length raises ROSAConfigurationError."""
        with self.assertRaisesRegex(
            ROSAConfigurationError,
            "Invalid max_history_length: 10.5. Must be a positive integer or None.",
        ):
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=10.5)

    def test_history_length_validation_invalid_type_list(self):
        """Test that list max_history_length raises ROSAConfigurationError."""
        with self.assertRaisesRegex(
            ROSAConfigurationError,
            r"Invalid max_history_length: \[1, 2, 3\]. Must be a positive integer or None.",
        ):
            ROSA(ros_version=1, llm=self.mock_llm, max_history_length=[1, 2, 3])


class TestROSAHistoryTrimming(unittest.TestCase):
    """Test _trim_chat_history() method functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_trim_chat_history_even_messages_basic(self):
        """Test trimming with even number of messages keeps most recent pairs."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=4)

            # Create chat history with 6 messages (3 pairs)
            chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
                HumanMessage(content="Query 3"),
                AIMessage(content="Response 3"),
            ]

            # Should keep only the most recent 4 messages (2 pairs)
            trimmed = rosa._ROSA__chat_manager.trim_chat_history(chat_history)

            self.assertEqual(len(trimmed), 4)
            self.assertEqual(trimmed[0].content, "Query 2")
            self.assertEqual(trimmed[1].content, "Response 2")
            self.assertEqual(trimmed[2].content, "Query 3")
            self.assertEqual(trimmed[3].content, "Response 3")

    def test_trim_chat_history_odd_messages(self):
        """Test trimming with odd number of messages handles incomplete pairs."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=4)

            # Create chat history with 5 messages (2 pairs + 1 incomplete)
            chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
                HumanMessage(content="Query 3"),  # Incomplete pair
            ]

            # Should keep the incomplete pair + most recent complete pair = 3 messages
            trimmed = rosa._ROSA__chat_manager.trim_chat_history(chat_history)

            self.assertEqual(len(trimmed), 3)
            self.assertEqual(trimmed[0].content, "Query 2")
            self.assertEqual(trimmed[1].content, "Response 2")
            self.assertEqual(trimmed[2].content, "Query 3")

    def test_trim_chat_history_no_trimming_needed(self):
        """Test that no trimming occurs when under the limit."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=10)

            # Create chat history with 4 messages (under limit)
            chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]

            # Should return original list unchanged
            trimmed = rosa._ROSA__chat_manager.trim_chat_history(chat_history)

            self.assertEqual(len(trimmed), 4)
            self.assertEqual(trimmed, chat_history)

    def test_trim_chat_history_unlimited_length(self):
        """Test that no trimming occurs when max_history_length is None."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=None)

            # Create large chat history
            chat_history = []
            for i in range(100):
                chat_history.append(HumanMessage(content=f"Query {i}"))
                chat_history.append(AIMessage(content=f"Response {i}"))

            # Should return original list unchanged
            trimmed = rosa._ROSA__chat_manager.trim_chat_history(chat_history)

            self.assertEqual(len(trimmed), 200)
            self.assertEqual(trimmed, chat_history)


class TestROSAHistoryManagementMethods(unittest.TestCase):
    """Test history management methods: clear_chat, get_history_length, trim_history, get_history_usage."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_clear_chat_basic(self):
        """Test that clear_chat() clears all chat history by default."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Add some chat history
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]

            # Clear chat history
            rosa.clear_chat()

            # Should be empty
            self.assertEqual(len(rosa.chat_history), 0)

    def test_clear_chat_retain_system_messages_false(self):
        """Test clear_chat() with retain_system_messages=False (default behavior)."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Add mixed chat history including system messages
            from langchain_core.messages import SystemMessage

            rosa._ROSA__chat_manager._chat_history = [
                SystemMessage(content="System prompt"),
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                SystemMessage(content="Another system message"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]

            # Clear chat history without retaining system messages
            rosa.clear_chat(retain_system_messages=False)

            # Should be completely empty
            self.assertEqual(len(rosa.chat_history), 0)

    def test_clear_chat_retain_system_messages_true(self):
        """Test clear_chat() with retain_system_messages=True."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Add mixed chat history including system messages
            from langchain_core.messages import SystemMessage

            system_msg1 = SystemMessage(content="System prompt")
            system_msg2 = SystemMessage(content="Another system message")
            rosa._ROSA__chat_manager._chat_history = [
                system_msg1,
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                system_msg2,
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]

            # Clear chat history while retaining system messages
            rosa.clear_chat(retain_system_messages=True)

            # Should only have system messages
            self.assertEqual(len(rosa.chat_history), 2)
            self.assertEqual(rosa.chat_history[0], system_msg1)
            self.assertEqual(rosa.chat_history[1], system_msg2)

    def test_clear_chat_no_system_messages(self):
        """Test clear_chat() when there are no system messages."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Add chat history without system messages
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]

            # Clear chat history while trying to retain system messages
            rosa.clear_chat(retain_system_messages=True)

            # Should be empty since there were no system messages
            self.assertEqual(len(rosa.chat_history), 0)

    def test_get_history_length_empty(self):
        """Test get_history_length() with empty chat history."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Empty history
            rosa._ROSA__chat_manager._chat_history = []

            # Should return 0
            self.assertEqual(rosa.get_history_length(), 0)

    def test_get_history_length_with_messages(self):
        """Test get_history_length() with various numbers of messages."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Test with different numbers of messages
            for i in range(1, 11):
                rosa._ROSA__chat_manager._chat_history = []
                for j in range(i):
                    rosa.chat_history.append(HumanMessage(content=f"Query {j}"))
                    rosa.chat_history.append(AIMessage(content=f"Response {j}"))

                expected_length = i * 2
                self.assertEqual(rosa.get_history_length(), expected_length)

    def test_trim_history_public_method(self):
        """Test trim_history() public method."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Create large chat history
            rosa._ROSA__chat_manager._chat_history = []
            for i in range(10):
                rosa.chat_history.append(HumanMessage(content=f"Query {i}"))
                rosa.chat_history.append(AIMessage(content=f"Response {i}"))

            # Trim to 6 messages
            rosa.trim_history(6)

            # Should have 6 messages (3 pairs)
            self.assertEqual(len(rosa.chat_history), 6)

            # Should be the most recent messages
            self.assertEqual(rosa.chat_history[0].content, "Query 7")
            self.assertEqual(rosa.chat_history[1].content, "Response 7")
            self.assertEqual(rosa.chat_history[4].content, "Query 9")
            self.assertEqual(rosa.chat_history[5].content, "Response 9")

    def test_trim_history_invalid_length(self):
        """Test trim_history() with invalid length parameters."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Test with negative length
            with self.assertRaises(ValueError):
                rosa.trim_history(-1)

            # Test with zero length
            with self.assertRaises(ValueError):
                rosa.trim_history(0)

    def test_trim_history_larger_than_current(self):
        """Test trim_history() when requested length is larger than current history."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Create small chat history
            original_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]
            rosa._ROSA__chat_manager._chat_history = original_history.copy()

            # Trim to larger size
            rosa.trim_history(10)

            # Should remain unchanged
            self.assertEqual(len(rosa.chat_history), 4)
            self.assertEqual(rosa.chat_history, original_history)

    def test_get_history_usage_empty(self):
        """Test get_history_usage() with empty chat history."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Empty history
            rosa._ROSA__chat_manager._chat_history = []

            usage = rosa.get_history_usage()

            # Should return zero values
            self.assertEqual(usage["message_count"], 0)
            self.assertEqual(usage["estimated_tokens"], 0)
            self.assertEqual(usage["memory_bytes"], 0)

    def test_get_history_usage_with_messages(self):
        """Test get_history_usage() with actual messages."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Add chat history
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="This is a test query with multiple words"),
                AIMessage(content="This is a test response with multiple words"),
                HumanMessage(content="Another query"),
                AIMessage(content="Another response"),
            ]

            usage = rosa.get_history_usage()

            # Should return proper counts
            self.assertEqual(usage["message_count"], 4)
            self.assertGreater(usage["estimated_tokens"], 0)
            self.assertGreater(usage["memory_bytes"], 0)

            # Basic sanity checks
            self.assertIsInstance(usage["estimated_tokens"], int)
            self.assertIsInstance(usage["memory_bytes"], int)

    def test_get_history_usage_return_format(self):
        """Test that get_history_usage() returns the expected dictionary format."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Add some chat history
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Test"),
                AIMessage(content="Response"),
            ]

            usage = rosa.get_history_usage()

            # Check that all expected keys are present
            required_keys = ["message_count", "estimated_tokens", "memory_bytes"]
            for key in required_keys:
                self.assertIn(key, usage)
                self.assertIsInstance(usage[key], int)


class TestROSAHistoryRecordingIntegration(unittest.TestCase):
    """Test _record_chat_history method integration with trimming logic."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_record_chat_history_triggers_trimming(self):
        """Test that _record_chat_history triggers trimming when history exceeds limit."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Create ROSA with small history limit
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=4)

            # Pre-populate with 2 messages (at limit)
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
            ]

            # Record new conversation (should trigger trimming)
            rosa._ROSA__chat_manager.record_chat_history("Query 2", "Response 2")

            # Should have exactly 4 messages (limit reached, but no trimming yet)
            self.assertEqual(len(rosa.chat_history), 4)

            # Record another conversation (should trigger trimming)
            rosa._ROSA__chat_manager.record_chat_history("Query 3", "Response 3")

            # Should still have 4 messages (oldest pair removed)
            self.assertEqual(len(rosa.chat_history), 4)

            # Should be the most recent messages
            self.assertEqual(rosa.chat_history[0].content, "Query 2")
            self.assertEqual(rosa.chat_history[1].content, "Response 2")
            self.assertEqual(rosa.chat_history[2].content, "Query 3")
            self.assertEqual(rosa.chat_history[3].content, "Response 3")

    def test_record_chat_history_disabled_accumulation(self):
        """Test that _record_chat_history doesn't add or trim when accumulation is disabled."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Create ROSA with accumulation disabled
            rosa = ROSA(
                ros_version=1,
                llm=self.mock_llm,
                accumulate_chat_history=False,
                max_history_length=4,
            )

            # Pre-populate history manually
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
            ]

            # Record new conversation (should not add anything)
            rosa._ROSA__chat_manager.record_chat_history("Query 2", "Response 2")

            # Should still have original 2 messages
            self.assertEqual(len(rosa.chat_history), 2)
            self.assertEqual(rosa.chat_history[0].content, "Query 1")
            self.assertEqual(rosa.chat_history[1].content, "Response 1")

    def test_record_chat_history_unlimited_length(self):
        """Test that _record_chat_history doesn't trim when max_history_length is None."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Create ROSA with unlimited history
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=None)

            # Record many conversations
            for i in range(10):
                rosa._ROSA__chat_manager.record_chat_history(
                    f"Query {i}", f"Response {i}"
                )

            # Should have all 20 messages
            self.assertEqual(len(rosa.chat_history), 20)

            # Verify all messages are present
            for i in range(10):
                self.assertEqual(rosa.chat_history[i * 2].content, f"Query {i}")
                self.assertEqual(rosa.chat_history[i * 2 + 1].content, f"Response {i}")

    def test_record_chat_history_atomic_operations(self):
        """Test that _record_chat_history maintains message pairs atomically."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Create ROSA with history limit
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=6)

            # Pre-populate with some messages
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
                HumanMessage(content="Query 2"),
                AIMessage(content="Response 2"),
            ]

            # Record new conversation
            rosa._ROSA__chat_manager.record_chat_history("Query 3", "Response 3")

            # Should have 6 messages total
            self.assertEqual(len(rosa.chat_history), 6)

            # All messages should be in pairs (even count)
            self.assertEqual(len(rosa.chat_history) % 2, 0)

            # Verify message types alternate correctly
            for i in range(0, len(rosa.chat_history), 2):
                self.assertIsInstance(rosa.chat_history[i], HumanMessage)
                self.assertIsInstance(rosa.chat_history[i + 1], AIMessage)

    def test_record_chat_history_error_handling(self):
        """Test that _record_chat_history handles errors gracefully."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=4)

            # Mock the trim_chat_history method on chat manager to raise an exception
            original_trim = rosa._ROSA__chat_manager.trim_chat_history

            def failing_trim(history):
                literal_two = 2
                if len(history) > literal_two:  # Fail when trying to trim
                    msg = "Trimming failed"
                    raise RuntimeError(msg)
                return original_trim(history)

            with patch.object(
                rosa._ROSA__chat_manager, "trim_chat_history", side_effect=failing_trim
            ):
                # This should not raise an exception, but should log the error
                rosa._ROSA__chat_manager.record_chat_history(
                    "Query 1", "Response 1"
                )  # Should work

                # This should handle the trimming error gracefully
                with patch.object(rosa._ROSA__chat_manager, "_logger") as mock_logger:
                    rosa._ROSA__chat_manager.record_chat_history(
                        "Query 2", "Response 2"
                    )  # Should trigger error

                    # Should have logged an error
                    mock_logger.error.assert_called()

                    # History should still be updated (messages added before trimming fails)
                    self.assertEqual(len(rosa.chat_history), 4)

    def test_record_chat_history_logging_when_trimming_occurs(self):
        """Test that _record_chat_history logs when trimming occurs."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=4)

            # Pre-populate to near limit
            rosa._ROSA__chat_manager._chat_history = [
                HumanMessage(content="Query 1"),
                AIMessage(content="Response 1"),
            ]

            with patch.object(rosa._ROSA__chat_manager, "_logger") as mock_logger:
                # This should not trigger trimming (at limit but not over)
                rosa._ROSA__chat_manager.record_chat_history("Query 2", "Response 2")

                # Should not have logged trimming
                mock_logger.debug.assert_not_called()

                # This should trigger trimming
                rosa._ROSA__chat_manager.record_chat_history("Query 3", "Response 3")

                # Should have logged trimming operation
                mock_logger.debug.assert_called()


class TestROSAHistoryPerformanceAndStress(unittest.TestCase):
    """Test performance and stress scenarios for chat history management."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_performance_with_large_history(self):
        """Test performance with large chat history (1000+ messages)."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Create ROSA with large history limit
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=1000)

            import time

            # Test 1: Performance of adding many messages
            start_time = time.time()
            for i in range(500):  # Add 1000 messages total
                rosa._ROSA__chat_manager.record_chat_history(
                    f"Query {i}", f"Response {i}"
                )
            add_time = time.time() - start_time

            # Should complete in reasonable time (< 5 seconds)
            self.assertLess(
                add_time,
                5.0,
                f"Adding 1000 messages took {add_time:.2f}s, should be < 5s",
            )

            # Verify we have exactly 1000 messages
            self.assertEqual(len(rosa.chat_history), 1000)

            # Test 2: Performance of trimming large history
            start_time = time.time()
            rosa.trim_history(100)  # Trim to 100 messages
            trim_time = time.time() - start_time

            # Trimming should be fast (< 1 second)
            self.assertLess(
                trim_time,
                1.0,
                f"Trimming 1000->100 messages took {trim_time:.2f}s, should be < 1s",
            )

            # Verify trimming worked correctly
            self.assertEqual(len(rosa.chat_history), 100)

            # Test 3: Performance of get_history_usage() with large history
            # First add back more messages
            for i in range(500, 750):  # Add 500 more messages
                rosa._ROSA__chat_manager.record_chat_history(
                    f"Query {i}", f"Response {i}"
                )

            start_time = time.time()
            usage = rosa.get_history_usage()
            usage_time = time.time() - start_time

            # Usage calculation should be fast (< 0.5 seconds)
            self.assertLess(
                usage_time,
                0.5,
                f"get_history_usage() took {usage_time:.2f}s, should be < 0.5s",
            )

            # Verify usage stats are reasonable
            self.assertEqual(usage["message_count"], 600)  # 100 + 500 new messages
            self.assertGreater(usage["estimated_tokens"], 0)
            self.assertGreater(usage["memory_bytes"], 0)

    def test_memory_leak_detection(self):
        """Test for memory leaks in chat history operations."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            import gc

            # Create ROSA with small history limit to force frequent trimming
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=10)

            # Force garbage collection and measure initial memory
            gc.collect()
            initial_objects = len(gc.get_objects())

            # Perform many operations that should trigger trimming
            for cycle in range(100):  # 100 cycles of operations
                # Add messages to trigger trimming
                for i in range(10):
                    rosa._ROSA__chat_manager.record_chat_history(
                        f"Cycle {cycle} Query {i}", f"Cycle {cycle} Response {i}"
                    )

                # Clear and reload history
                rosa.clear_chat()
                for i in range(5):
                    rosa._ROSA__chat_manager.record_chat_history(
                        f"Reload {cycle}.{i}", f"Response {cycle}.{i}"
                    )

                # Manual trimming operations
                rosa.trim_history(8)
                rosa.trim_history(6)
                rosa.trim_history(4)

                # Usage calculations - ensure memory usage stays reasonable
                usage = rosa.get_history_usage()
                self.assertLess(
                    usage["memory_bytes"],
                    50000,  # 50KB reasonable threshold for chat history memory usage
                    f"Memory usage {usage['memory_bytes']} bytes exceeded threshold during stress test cycle {cycle}",
                )
                self.assertLess(
                    usage["estimated_tokens"],
                    10000,  # Token count threshold
                    f"Token count {usage['estimated_tokens']} exceeded threshold during stress test cycle {cycle}",
                )

                # Periodic garbage collection
                if cycle % 20 == 0:
                    gc.collect()

            # Final garbage collection and memory measurement
            gc.collect()
            final_objects = len(gc.get_objects())

            # Memory should not have grown excessively
            # Allow for some growth due to test infrastructure, but not excessive
            memory_growth = final_objects - initial_objects
            self.assertLess(
                memory_growth,
                1000,
                f"Memory grew by {memory_growth} objects, possible memory leak",
            )

            # History should be properly maintained
            self.assertEqual(len(rosa.chat_history), 4)  # Last trim_history(4)

            # Verify no circular references by checking that objects can be garbage collected
            rosa_ref = id(rosa)
            del rosa
            gc.collect()

            # Check that the ROSA object was properly garbage collected
            # (This is a basic check - in a real scenario you'd use memory profiling tools)
            current_objects = gc.get_objects()
            rosa_still_exists = any(
                id(obj) == rosa_ref
                for obj in current_objects
                if hasattr(obj, "__class__")
            )
            # Note: This check might not always work due to test framework references
            self.assertFalse(
                rosa_still_exists, "ROSA object may not have been garbage collected"
            )

    def test_concurrent_access_chat_history(self):
        """Test concurrent access to chat history operations."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            import threading
            import time

            # Create ROSA with moderate history limit
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=50)

            errors = []
            results = {}

            def worker_record_history(worker_id, iterations):
                """Worker function to record chat history."""
                try:
                    for i in range(iterations):
                        rosa._ROSA__chat_manager.record_chat_history(
                            f"Worker {worker_id} Query {i}",
                            f"Worker {worker_id} Response {i}",
                        )
                        time.sleep(
                            0.001
                        )  # Small delay to increase chance of race conditions
                    results[f"worker_{worker_id}"] = "completed"
                except Exception as e:
                    errors.append(f"Worker {worker_id}: {e}")

            def worker_read_operations(worker_id, iterations):
                """Worker function to perform read operations."""
                threshold = 100000
                try:
                    for _ in range(iterations):
                        # Various read operations
                        length = rosa.get_history_length()
                        usage = rosa.get_history_usage()
                        history_copy = rosa.chat_history.copy()

                        # Validate memory usage during concurrent access
                        if (
                            usage["memory_bytes"] > threshold
                        ):  # 100KB threshold for concurrent operations
                            errors.append(
                                f"Reader {worker_id}: Memory usage {usage['memory_bytes']} bytes exceeded threshold"
                            )

                        # Validate consistency
                        if len(history_copy) != length:
                            errors.append(f"Reader {worker_id}: Inconsistent length")

                        time.sleep(0.001)
                    results[f"reader_{worker_id}"] = "completed"
                except Exception as e:
                    errors.append(f"Reader {worker_id}: {e}")

            def worker_management_operations(worker_id, iterations):
                """Worker function to perform management operations."""
                try:
                    for i in range(iterations):
                        if i % 5 == 0:
                            rosa.trim_history(30)
                        if i % 10 == 0:
                            rosa.clear_chat(retain_system_messages=False)
                        time.sleep(0.002)
                    results[f"manager_{worker_id}"] = "completed"
                except Exception as e:
                    errors.append(f"Manager {worker_id}: {e}")

            # Create multiple threads for concurrent access
            threads = []

            # 2 workers recording history
            for i in range(2):
                t = threading.Thread(target=worker_record_history, args=(i, 20))
                threads.append(t)

            # 2 workers reading history
            for i in range(2):
                t = threading.Thread(target=worker_read_operations, args=(i, 50))
                threads.append(t)

            # 1 worker doing management operations
            t = threading.Thread(target=worker_management_operations, args=(0, 10))
            threads.append(t)

            # Start all threads
            start_time = time.time()
            for t in threads:
                t.start()

            # Wait for all threads to complete
            for t in threads:
                t.join(timeout=10.0)  # 10 second timeout
                if t.is_alive():
                    errors.append(f"Thread {t.name} did not complete in time")

            end_time = time.time()

            # Check for errors
            if errors:
                self.fail(f"Concurrent access errors: {errors}")

            # Verify all workers completed
            expected_results = [
                "worker_0",
                "worker_1",
                "reader_0",
                "reader_1",
                "manager_0",
            ]
            for worker in expected_results:
                self.assertIn(worker, results, f"Worker {worker} did not complete")
                self.assertEqual(results[worker], "completed")

            # Test should complete in reasonable time
            self.assertLess(end_time - start_time, 8.0, "Concurrent test took too long")

            # Final state should be valid
            final_length = rosa.get_history_length()
            final_usage = rosa.get_history_usage()
            self.assertEqual(final_usage["message_count"], final_length)

            # History should be consistent (even number of messages for pairs)
            if final_length > 0:
                self.assertEqual(
                    final_length % 2, 0, "History should contain complete message pairs"
                )

    def test_stress_rapid_operations(self):
        """Test rapid succession of various history operations."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Create ROSA with small history limit for frequent trimming
            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=20)

            import time

            start_time = time.time()

            # Rapid operations for stress testing
            for i in range(200):
                # Record history (triggers automatic trimming)
                rosa._ROSA__chat_manager.record_chat_history(
                    f"Rapid query {i}", f"Rapid response {i}"
                )

                # Every few iterations, perform various operations
                if i % 5 == 0:
                    length = rosa.get_history_length()
                    self.assertLessEqual(
                        length, 20, f"History exceeded limit: {length}"
                    )

                if i % 10 == 0:
                    usage = rosa.get_history_usage()
                    self.assertGreater(usage["message_count"], 0)

                if i % 15 == 0:
                    rosa.trim_history(10)
                    self.assertLessEqual(rosa.get_history_length(), 10)

                if i % 25 == 0:
                    rosa.clear_chat()
                    self.assertEqual(rosa.get_history_length(), 0)

            end_time = time.time()

            # Should complete in reasonable time
            total_time = end_time - start_time
            self.assertLess(
                total_time, 3.0, f"Stress test took {total_time:.2f}s, should be < 3s"
            )

            # Final state should be valid
            self.assertLessEqual(rosa.get_history_length(), 20)

            # Verify history consistency
            history = rosa.chat_history
            if len(history) > 0:
                self.assertEqual(
                    len(history) % 2, 0, "History should contain complete pairs"
                )

                # Verify message types alternate
                for i in range(0, len(history), 2):
                    self.assertIsInstance(history[i], HumanMessage)
                    self.assertIsInstance(history[i + 1], AIMessage)

    def test_trim_chat_history_empty_list(self):
        """Test trimming with empty chat history."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=4)

            trimmed = rosa._ROSA__chat_manager.trim_chat_history([])

            self.assertEqual(trimmed, [])

    def test_trim_chat_history_preserves_message_types(self):
        """Test that message types and content are preserved after trimming."""
        with (
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
            ) as mock_get_tools,
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
            patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"),
            patch(
                "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
            ) as mock_bind_tools,
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(ros_version=1, llm=self.mock_llm, max_history_length=2)

            # Create mixed message types
            chat_history = [
                HumanMessage(content="First human message"),
                AIMessage(content="First AI response"),
                HumanMessage(content="Second human message"),
                AIMessage(content="Second AI response"),
            ]

            trimmed = rosa._ROSA__chat_manager.trim_chat_history(chat_history)

            self.assertEqual(len(trimmed), 2)
            self.assertIsInstance(trimmed[0], HumanMessage)
            self.assertIsInstance(trimmed[1], AIMessage)
            self.assertEqual(trimmed[0].content, "Second human message")
            self.assertEqual(trimmed[1].content, "Second AI response")


if __name__ == "__main__":
    unittest.main()
