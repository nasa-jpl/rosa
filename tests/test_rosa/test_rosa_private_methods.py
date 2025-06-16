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
import sys
import unittest
from io import StringIO
from unittest.mock import MagicMock, patch

from langchain_openai import ChatOpenAI

from rosa import ROSA, RobotSystemPrompts


class TestROSAPrivateComponentMethods(unittest.TestCase):
    """Test ROSA private methods that still exist in the main class."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

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
            self.rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

    def test_print_usage_with_callback_and_flag_enabled(self):
        """Test that _print_usage prints when show_token_usage is enabled."""
        self.rosa._ROSA__show_token_usage = True

        # Mock callback with usage data
        mock_cb = MagicMock()
        mock_cb.prompt_tokens = 50
        mock_cb.completion_tokens = 25
        mock_cb.total_cost = 0.001

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        try:
            self.rosa._print_usage(mock_cb)
            output = captured_output.getvalue()

            self.assertIn("Prompt Tokens", output)
            self.assertIn("50", output)
            self.assertIn("Completion Tokens", output)
            self.assertIn("25", output)
            self.assertIn("Total Cost", output)
            self.assertIn("0.001", output)
        finally:
            sys.stdout = sys.__stdout__

    def test_print_usage_with_flag_disabled(self):
        """Test that _print_usage doesn't print when show_token_usage is disabled."""
        self.rosa._ROSA__show_token_usage = False

        # Mock callback
        mock_cb = MagicMock()
        mock_cb.prompt_tokens = 50
        mock_cb.completion_tokens = 25
        mock_cb.total_cost = 0.001

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        try:
            self.rosa._print_usage(mock_cb)
            output = captured_output.getvalue()

            # Should be empty since flag is disabled
            self.assertEqual(output.strip(), "")
        finally:
            sys.stdout = sys.__stdout__

    def test_print_usage_with_none_callback(self):
        """Test that _print_usage handles None callback gracefully."""
        self.rosa._ROSA__show_token_usage = True

        # Capture stdout
        captured_output = StringIO()
        sys.stdout = captured_output

        try:
            self.rosa._print_usage(None)
            output = captured_output.getvalue()

            # Should be empty since callback is None
            self.assertEqual(output.strip(), "")
        finally:
            sys.stdout = sys.__stdout__


class TestROSAPrivateInitializationMethods(unittest.TestCase):
    """Test ROSA private initialization methods."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_initialize_logging_creates_logger(self):
        """Test that _initialize_logging creates logger with correct name."""
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

            # Check that logger was created with correct name
            logger = rosa._ROSA__logger
            self.assertIsInstance(logger, logging.Logger)
            self.assertEqual(logger.name, "rosa.rosa.ROSA")

    def test_setup_configuration_sets_instance_variables(self):
        """Test that _setup_configuration sets all instance variables correctly."""
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

            # Create instance to test configuration setup
            rosa = ROSA(
                ros_version=2,
                llm=self.mock_llm,
                tools=["custom_tool"],
                tool_packages=["custom_package"],
                prompts=RobotSystemPrompts(embodiment_and_persona="Test"),
                verbose=True,
                blacklist=["excluded_tool"],
                accumulate_chat_history=False,
                show_token_usage=True,
                streaming=False,
                max_history_length=100,
            )

            # Verify all configuration was set correctly
            self.assertEqual(rosa._ROSA__ros_version, 2)
            self.assertEqual(rosa._ROSA__blacklist, ["excluded_tool"])
            self.assertFalse(rosa._ROSA__accumulate_chat_history)
            self.assertFalse(rosa._ROSA__streaming)
            self.assertTrue(rosa._ROSA__show_token_usage)
            self.assertEqual(rosa._ROSA__max_history_length, 100)
            self.assertEqual(rosa._ROSA__init_tools, ["custom_tool"])
            self.assertEqual(rosa._ROSA__init_tool_packages, ["custom_package"])
            self.assertIsNotNone(rosa._ROSA__init_prompts)

    def test_setup_configuration_streaming_disables_token_usage(self):
        """Test that _setup_configuration disables token usage when streaming is enabled."""
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

            # Create instance with streaming enabled and token usage requested
            rosa = ROSA(
                ros_version=1,
                llm=self.mock_llm,
                streaming=True,
                show_token_usage=True,  # This should be disabled due to streaming
            )

            # Verify that token usage was disabled due to streaming
            self.assertFalse(rosa._ROSA__show_token_usage)
            self.assertTrue(rosa._ROSA__streaming)


if __name__ == "__main__":
    unittest.main()
