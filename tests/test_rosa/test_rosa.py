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

from langchain_core.messages import AIMessage, HumanMessage
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI

from rosa import ROSA, RobotSystemPrompts
from rosa.exceptions import ROSAConfigurationError, ROSAError, ROSAExecutionError


class TestROSAExceptions(unittest.TestCase):
    """Test custom exception classes."""

    def test_rosa_error_inheritance(self):
        """Test that ROSAError inherits from Exception."""
        error = ROSAError("test message")
        self.assertIsInstance(error, Exception)
        self.assertEqual(str(error), "test message")

    def test_rosa_configuration_error_inheritance(self):
        """Test that ROSAConfigurationError inherits from ROSAError."""
        error = ROSAConfigurationError("config error")
        self.assertIsInstance(error, ROSAError)
        self.assertIsInstance(error, Exception)
        self.assertEqual(str(error), "config error")

    def test_rosa_execution_error_inheritance(self):
        """Test that ROSAExecutionError inherits from ROSAError."""
        error = ROSAExecutionError("execution error")
        self.assertIsInstance(error, ROSAError)
        self.assertIsInstance(error, Exception)
        self.assertEqual(str(error), "execution error")


class TestROSAValidation(unittest.TestCase):
    """Test input validation functionality."""

    def test_invalid_ros_version_raises_error(self):
        """Test that invalid ROS version raises ROSAConfigurationError."""
        mock_llm = MagicMock(spec=ChatOpenAI)
        mock_llm.with_config.return_value = mock_llm

        with self.assertRaisesRegex(ROSAConfigurationError, "Invalid ROS version: 3"):
            ROSA(ros_version=3, llm=mock_llm)

    def test_invalid_llm_type_raises_error(self):
        """Test that invalid LLM type raises ROSAConfigurationError."""
        invalid_llm = "not_an_llm"

        with self.assertRaisesRegex(ROSAConfigurationError, "Invalid LLM type"):
            ROSA(ros_version=1, llm=invalid_llm)

    def test_valid_ros_versions_accepted(self):
        """Test that valid ROS versions (1, 2) are accepted."""
        mock_llm = MagicMock(spec=ChatOpenAI)
        mock_llm.with_config.return_value = mock_llm

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

            # Should not raise for valid versions
            ROSA(ros_version=1, llm=mock_llm)
            ROSA(ros_version=2, llm=mock_llm)


class TestROSAConstants(unittest.TestCase):
    """Test ROSA class constants."""

    def test_class_constants_defined(self):
        """Test that class constants are properly defined."""
        self.assertEqual(ROSA.MEMORY_KEY, "chat_history")
        self.assertEqual(ROSA.SCRATCHPAD_KEY, "agent_scratchpad")
        self.assertEqual(ROSA.AGENT_RUN_NAME, "Agent")


class TestROSAInitialization(unittest.TestCase):
    """Test ROSA class initialization."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_initialization_success(self):
        """Test successful ROSA initialization."""
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

            self.assertEqual(rosa._ROSA__ros_version, 1)
            self.assertTrue(rosa._ROSA__streaming)  # default
            self.assertTrue(rosa._ROSA__accumulate_chat_history)  # default
            self.assertEqual(rosa._ROSA__blacklist, [])
            self.assertFalse(
                rosa._ROSA__show_token_usage
            )  # disabled when streaming=True

    def test_initialization_with_custom_params(self):
        """Test ROSA initialization with custom parameters."""
        custom_blacklist = ["tool1", "tool2"]
        custom_prompts = RobotSystemPrompts(embodiment_and_persona="Test robot")

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

            rosa = ROSA(
                ros_version=2,
                llm=self.mock_llm,
                blacklist=custom_blacklist,
                prompts=custom_prompts,
                streaming=False,
                accumulate_chat_history=False,
                show_token_usage=True,
            )

            self.assertEqual(rosa._ROSA__ros_version, 2)
            self.assertFalse(rosa._ROSA__streaming)
            self.assertFalse(rosa._ROSA__accumulate_chat_history)
            self.assertEqual(rosa._ROSA__blacklist, custom_blacklist)
            self.assertTrue(rosa._ROSA__show_token_usage)

    def test_logger_setup(self):
        """Test that logger is properly set up."""
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

            logger = rosa._ROSA__logger
            self.assertIsInstance(logger, logging.Logger)
            self.assertEqual(logger.name, "rosa.rosa.ROSA")


class TestROSAMethods(unittest.TestCase):
    """Test ROSA class methods."""

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

            self.rosa = ROSA(ros_version=1, llm=self.mock_llm)

    def test_clear_chat(self):
        """Test clear_chat method."""
        # Add some chat history
        self.rosa._ROSA__chat_manager._chat_history = [
            HumanMessage(content="test query"),
            AIMessage(content="test response"),
        ]

        # Clear chat history
        self.rosa.clear_chat()

        self.assertEqual(len(self.rosa.chat_history), 0)

    def test_chat_history_property(self):
        """Test chat_history property."""
        test_history = [HumanMessage(content="test")]
        self.rosa._ROSA__chat_manager._chat_history = test_history

        self.assertEqual(self.rosa.chat_history, test_history)

    def test_invoke_empty_query_raises_error(self):
        """Test that empty query raises ValueError."""
        with self.assertRaisesRegex(ValueError, "Query cannot be empty"):
            self.rosa.invoke("")

        with self.assertRaisesRegex(ValueError, "Query cannot be empty"):
            self.rosa.invoke("   ")  # whitespace only

    def test_invoke_success(self):
        """Test successful invoke call."""
        mock_executor = MagicMock()
        mock_executor.invoke.return_value = {"output": "test response"}
        self.rosa._ROSA__executor = mock_executor

        with patch.object(self.rosa, "_ROSA__component_builder") as mock_builder:
            mock_cb = MagicMock()
            mock_builder.get_usage_callback.return_value.__enter__.return_value = (
                mock_cb
            )

            result = self.rosa.invoke("test query")

            self.assertEqual(result, "test response")
            mock_executor.invoke.assert_called_once_with(
                {"input": "test query", ROSA.MEMORY_KEY: self.rosa.chat_history}
            )

    def test_invoke_executor_failure_raises_execution_error(self):
        """Test that executor failure raises ROSAExecutionError."""
        mock_executor = MagicMock()
        mock_executor.invoke.side_effect = Exception("Executor failed")
        self.rosa._ROSA__executor = mock_executor

        with patch.object(self.rosa, "_ROSA__component_builder") as mock_builder:
            mock_builder.get_usage_callback.return_value.__enter__.return_value = (
                MagicMock()
            )
            # Suppress logging during this test to avoid confusing error messages
            with (
                patch.object(self.rosa._ROSA__logger, "error"),
                self.assertRaisesRegex(ROSAExecutionError, "Agent execution failed"),
            ):
                self.rosa.invoke("test query")

    def test_invoke_missing_output_raises_execution_error(self):
        """Test that missing output key raises ROSAExecutionError."""
        mock_executor = MagicMock()
        mock_executor.invoke.return_value = {}  # Missing 'output' key
        self.rosa._ROSA__executor = mock_executor

        with patch.object(self.rosa, "_ROSA__component_builder") as mock_builder:
            mock_builder.get_usage_callback.return_value.__enter__.return_value = (
                MagicMock()
            )
            # Suppress logging during this test to avoid confusing error messages
            with (
                patch.object(self.rosa._ROSA__logger, "error"),
                self.assertRaisesRegex(
                    ROSAExecutionError, "Agent result missing 'output' key"
                ),
            ):
                self.rosa.invoke("test query")

    def test_invoke_updates_chat_history(self):
        """Test that successful invoke updates chat history."""
        self.rosa._ROSA__accumulate_chat_history = True
        mock_executor = MagicMock()
        mock_executor.invoke.return_value = {"output": "test response"}
        self.rosa._ROSA__executor = mock_executor

        with patch.object(self.rosa, "_ROSA__component_builder") as mock_builder:
            mock_builder.get_usage_callback.return_value.__enter__.return_value = (
                MagicMock()
            )
            self.rosa.invoke("test query")

            history = self.rosa.chat_history
            self.assertEqual(len(history), 2)
            self.assertIsInstance(history[0], HumanMessage)
            self.assertEqual(history[0].content, "test query")
            self.assertIsInstance(history[1], AIMessage)
            self.assertEqual(history[1].content, "test response")


class TestROSAPrivateMethods(unittest.TestCase):
    """Test ROSA private methods."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_validate_inputs_valid(self):
        """Test _validate_inputs with valid inputs."""
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

            # Should not raise for valid inputs
            rosa._validate_inputs(1, self.mock_llm, 50)
            rosa._validate_inputs(2, self.mock_llm, None)

    def test_validate_inputs_invalid_ros_version(self):
        """Test _validate_inputs with invalid ROS version."""
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

            with self.assertRaisesRegex(
                ROSAConfigurationError, "Invalid ROS version: 3"
            ):
                rosa._validate_inputs(3, self.mock_llm, 50)

    def test_record_chat_history_enabled(self):
        """Test _record_chat_history when accumulation is enabled."""
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

            rosa = ROSA(ros_version=1, llm=self.mock_llm, accumulate_chat_history=True)

            rosa._ROSA__chat_manager.record_chat_history("test query", "test response")

            history = rosa.chat_history
            self.assertEqual(len(history), 2)
            self.assertIsInstance(history[0], HumanMessage)
            self.assertEqual(history[0].content, "test query")
            self.assertIsInstance(history[1], AIMessage)
            self.assertEqual(history[1].content, "test response")

    def test_record_chat_history_disabled(self):
        """Test _record_chat_history when accumulation is disabled."""
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

            rosa = ROSA(ros_version=1, llm=self.mock_llm, accumulate_chat_history=False)

            rosa._ROSA__chat_manager.record_chat_history("test query", "test response")

            self.assertEqual(len(rosa.chat_history), 0)


class TestROSAStreaming(unittest.TestCase):
    """Test ROSA streaming functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock()

    def test_astream_final_output_no_extra_spaces(self):
        """Test that final_output in chat history has proper spacing without leading spaces."""
        import asyncio

        async def run_test():
            with (
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
                ) as mock_get_tools,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"
                ),
                patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"
                ) as mock_get_executor,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
                ) as mock_bind_tools,
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
                ) as mock_validate,
            ):
                # Skip validation for this test
                mock_validate.return_value = None

                # Setup mocks
                mock_tools_instance = MagicMock()
                mock_tools_instance.get_tools.return_value = []
                mock_get_tools.return_value = mock_tools_instance
                mock_bind_tools.return_value = MagicMock()

                mock_executor = MagicMock()
                mock_get_executor.return_value = mock_executor

                # Mock streaming events WITHOUT on_chain_end to test proper concatenation
                # Typical LLM streaming includes spaces in tokens
                mock_events = [
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="Hello")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" World")},
                    },
                ]

                # Mock the async iterator
                async def mock_astream_events(*args, **kwargs):  # noqa: ARG001
                    for event in mock_events:
                        yield event

                mock_executor.astream_events = mock_astream_events

                # Create ROSA instance with streaming enabled and chat history
                rosa = ROSA(
                    ros_version=1,
                    llm=self.mock_llm,
                    streaming=True,
                    accumulate_chat_history=True,
                )

                # Execute streaming
                chunks = []
                async for chunk in rosa.astream("test query"):
                    chunks.append(chunk)

                # Check that the chat history contains the buggy final_output with extra spaces
                # The bug is: final_output += f" {content}" creates " Hello World" instead of "HelloWorld"
                self.assertEqual(len(rosa.chat_history), 2)  # HumanMessage + AIMessage
                ai_message_content = rosa.chat_history[-1].content

                # After the fix: final_output should be "Hello World" (no leading space)
                self.assertEqual(
                    ai_message_content,
                    "Hello World",
                    "Chat history should not have extra leading spaces from streaming concatenation",
                )

        # Run the async test - this test should PASS after the fix
        asyncio.run(run_test())

    def test_astream_single_word_response(self):
        """Test streaming with single word responses."""
        import asyncio

        async def run_test():
            with (
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
                ) as mock_get_tools,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"
                ),
                patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"
                ) as mock_get_executor,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
                ) as mock_bind_tools,
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_boolean_parameter"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_optional_list_parameter"
                ),
            ):
                mock_tools_instance = MagicMock()
                mock_tools_instance.get_tools.return_value = []
                mock_get_tools.return_value = mock_tools_instance
                mock_bind_tools.return_value = MagicMock()

                mock_executor = MagicMock()
                mock_get_executor.return_value = mock_executor

                # Single word response
                mock_events = [
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="Yes")},
                    },
                    {
                        "event": "on_chain_end",
                        "name": "Agent",
                        "data": {"output": {"output": "Yes"}},
                    },
                ]

                async def mock_astream_events(*args, **kwargs):  # noqa: ARG001
                    for event in mock_events:
                        yield event

                mock_executor.astream_events = mock_astream_events
                rosa = ROSA(
                    ros_version=1,
                    llm=self.mock_llm,
                    streaming=True,
                    accumulate_chat_history=True,
                )

                chunks = []
                async for chunk in rosa.astream("test query"):
                    chunks.append(chunk)

                # Should have exactly one token
                self.assertEqual(len([c for c in chunks if c["type"] == "token"]), 1)
                ai_message_content = rosa.chat_history[-1].content
                self.assertEqual(ai_message_content, "Yes")

        asyncio.run(run_test())

    def test_astream_multi_paragraph_response(self):
        """Test streaming with multi-paragraph responses."""
        import asyncio

        async def run_test():
            with (
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
                ) as mock_get_tools,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"
                ),
                patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"
                ) as mock_get_executor,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
                ) as mock_bind_tools,
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_boolean_parameter"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_optional_list_parameter"
                ),
            ):
                mock_tools_instance = MagicMock()
                mock_tools_instance.get_tools.return_value = []
                mock_get_tools.return_value = mock_tools_instance
                mock_bind_tools.return_value = MagicMock()

                mock_executor = MagicMock()
                mock_get_executor.return_value = mock_executor

                # Multi-paragraph response with newlines and spaces
                mock_events = [
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="First")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" paragraph.")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="\n\nSecond")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" paragraph.")},
                    },
                    {
                        "event": "on_chain_end",
                        "name": "Agent",
                        "data": {
                            "output": {
                                "output": "First paragraph.\n\nSecond paragraph."
                            }
                        },
                    },
                ]

                async def mock_astream_events(*args, **kwargs):  # noqa: ARG001
                    for event in mock_events:
                        yield event

                mock_executor.astream_events = mock_astream_events
                rosa = ROSA(
                    ros_version=1,
                    llm=self.mock_llm,
                    streaming=True,
                    accumulate_chat_history=True,
                )

                chunks = []
                async for chunk in rosa.astream("test query"):
                    chunks.append(chunk)

                ai_message_content = rosa.chat_history[-1].content
                expected = "First paragraph.\n\nSecond paragraph."
                self.assertEqual(ai_message_content, expected)

        asyncio.run(run_test())

    def test_astream_code_block_with_indentation(self):
        """Test streaming with code blocks containing indentation."""
        import asyncio

        async def run_test():
            with (
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
                ) as mock_get_tools,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"
                ),
                patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"
                ) as mock_get_executor,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
                ) as mock_bind_tools,
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_boolean_parameter"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_optional_list_parameter"
                ),
            ):
                mock_tools_instance = MagicMock()
                mock_tools_instance.get_tools.return_value = []
                mock_get_tools.return_value = mock_tools_instance
                mock_bind_tools.return_value = MagicMock()

                mock_executor = MagicMock()
                mock_get_executor.return_value = mock_executor

                # Code block with indentation
                mock_events = [
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="```python")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="\ndef")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" hello():")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="\n    print")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="('Hello')")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="\n```")},
                    },
                    {
                        "event": "on_chain_end",
                        "name": "Agent",
                        "data": {
                            "output": {
                                "output": "```python\ndef hello():\n    print('Hello')\n```"
                            }
                        },
                    },
                ]

                async def mock_astream_events(*args, **kwargs):  # noqa: ARG001
                    for event in mock_events:
                        yield event

                mock_executor.astream_events = mock_astream_events
                rosa = ROSA(
                    ros_version=1,
                    llm=self.mock_llm,
                    streaming=True,
                    accumulate_chat_history=True,
                )

                chunks = []
                async for chunk in rosa.astream("test query"):
                    chunks.append(chunk)

                ai_message_content = rosa.chat_history[-1].content
                expected = "```python\ndef hello():\n    print('Hello')\n```"
                self.assertEqual(ai_message_content, expected)

        asyncio.run(run_test())

    def test_astream_mixed_content(self):
        """Test streaming with mixed content (text + code)."""
        import asyncio

        async def run_test():
            with (
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_tools"
                ) as mock_get_tools,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_prompts"
                ),
                patch("rosa.helpers.component_builder.ROSAComponentBuilder.get_agent"),
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.get_executor"
                ) as mock_get_executor,
                patch(
                    "rosa.helpers.component_builder.ROSAComponentBuilder.bind_tools_to_llm"
                ) as mock_bind_tools,
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_boolean_parameter"
                ),
                patch(
                    "rosa.helpers.config_validator.ROSAConfigValidator.validate_optional_list_parameter"
                ),
            ):
                mock_tools_instance = MagicMock()
                mock_tools_instance.get_tools.return_value = []
                mock_get_tools.return_value = mock_tools_instance
                mock_bind_tools.return_value = MagicMock()

                mock_executor = MagicMock()
                mock_get_executor.return_value = mock_executor

                # Mixed content: text + code + text
                mock_events = [
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="Here's")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" the")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" code:")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" `print")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content="('hi')`")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" works")},
                    },
                    {
                        "event": "on_chat_model_stream",
                        "data": {"chunk": MagicMock(content=" well.")},
                    },
                    {
                        "event": "on_chain_end",
                        "name": "Agent",
                        "data": {
                            "output": {
                                "output": "Here's the code: `print('hi')` works well."
                            }
                        },
                    },
                ]

                async def mock_astream_events(*args, **kwargs):  # noqa: ARG001
                    for event in mock_events:
                        yield event

                mock_executor.astream_events = mock_astream_events
                rosa = ROSA(
                    ros_version=1,
                    llm=self.mock_llm,
                    streaming=True,
                    accumulate_chat_history=True,
                )

                chunks = []
                async for chunk in rosa.astream("test query"):
                    chunks.append(chunk)

                ai_message_content = rosa.chat_history[-1].content
                expected = "Here's the code: `print('hi')` works well."
                self.assertEqual(ai_message_content, expected)

        asyncio.run(run_test())


class TestROSATokenUsageCallbacks(unittest.TestCase):
    """Test token usage callback support for different LLM types."""

    def test_openai_llm_token_usage_callback(self):
        """Test that OpenAI LLMs use get_openai_callback for token usage."""
        # Use real ChatOpenAI instance from the correct package
        real_openai_llm = ChatOpenAI(api_key="fake-api-key", model="gpt-3.5-turbo")

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
            patch(
                "rosa.helpers.component_builder.get_openai_callback"
            ) as mock_callback,
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
            ),
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_boolean_parameter"
            ),
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_optional_list_parameter"
            ),
        ):
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            mock_cb = MagicMock()
            mock_callback.return_value.__enter__.return_value = mock_cb
            mock_cb.total_tokens = 100
            mock_cb.prompt_tokens = 60
            mock_cb.completion_tokens = 40
            mock_cb.total_cost = 0.002

            rosa = ROSA(
                ros_version=1,
                llm=real_openai_llm,
                show_token_usage=True,
                streaming=False,
            )

            mock_executor = MagicMock()
            mock_executor.invoke.return_value = {"output": "test response"}
            rosa._ROSA__executor = mock_executor

            # Test that OpenAI callback is used for OpenAI models
            result = rosa.invoke("test query")

            mock_callback.assert_called_once()
            self.assertEqual(result, "test response")

    def test_ollama_llm_token_usage_callback_not_supported(self):
        """Test that Ollama LLMs correctly use null callback instead of OpenAI callback."""
        mock_ollama_llm = MagicMock(spec=ChatOllama)
        mock_ollama_llm.with_config.return_value = mock_ollama_llm
        mock_ollama_llm.bind_tools.return_value = mock_ollama_llm

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
            patch(
                "rosa.helpers.component_builder.get_openai_callback"
            ) as mock_openai_callback,
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
            ),
        ):  # Skip validation for mock objects
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(
                ros_version=1,
                llm=mock_ollama_llm,
                show_token_usage=True,
                streaming=False,
            )

            mock_executor = MagicMock()
            mock_executor.invoke.return_value = {"output": "test response"}
            rosa._ROSA__executor = mock_executor

            # After the fix: Ollama models should NOT use get_openai_callback
            result = rosa.invoke("test query")

            # OpenAI callback should NOT be called for Ollama models
            mock_openai_callback.assert_not_called()
            self.assertEqual(result, "test response")

    def test_generic_llm_token_usage_callback_not_supported(self):
        """Test that generic LLMs correctly use null callback instead of OpenAI callback."""
        mock_generic_llm = MagicMock()  # Generic LLM without specific type
        mock_generic_llm.with_config.return_value = mock_generic_llm
        mock_generic_llm.bind_tools.return_value = mock_generic_llm

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
            patch(
                "rosa.helpers.component_builder.get_openai_callback"
            ) as mock_openai_callback,
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
            ),
        ):  # Skip validation for mock objects
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            rosa = ROSA(
                ros_version=1,
                llm=mock_generic_llm,
                show_token_usage=True,
                streaming=False,
            )

            mock_executor = MagicMock()
            mock_executor.invoke.return_value = {"output": "test response"}
            rosa._ROSA__executor = mock_executor

            # After the fix: Generic LLMs should NOT use get_openai_callback
            result = rosa.invoke("test query")

            # OpenAI callback should NOT be called for generic LLMs
            mock_openai_callback.assert_not_called()
            self.assertEqual(result, "test response")

    def test_streaming_token_usage_disabled_by_default(self):
        """Test that token usage is disabled when streaming is enabled."""
        mock_openai_llm = MagicMock(spec=ChatOpenAI)
        mock_openai_llm.with_config.return_value = mock_openai_llm
        mock_openai_llm.bind_tools.return_value = mock_openai_llm

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
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
            ),
        ):  # Skip validation for mock objects
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Streaming enabled (default), show_token_usage should be False
            rosa = ROSA(ros_version=1, llm=mock_openai_llm, streaming=True)

            self.assertFalse(rosa._ROSA__show_token_usage)

    def test_can_enable_token_usage_with_non_streaming(self):
        """Test that token usage can be enabled when streaming is disabled."""
        mock_openai_llm = MagicMock(spec=ChatOpenAI)
        mock_openai_llm.with_config.return_value = mock_openai_llm
        mock_openai_llm.bind_tools.return_value = mock_openai_llm

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
            patch(
                "rosa.helpers.config_validator.ROSAConfigValidator.validate_all_inputs"
            ),
        ):  # Skip validation for mock objects
            mock_tools_instance = MagicMock()
            mock_tools_instance.get_tools.return_value = []
            mock_get_tools.return_value = mock_tools_instance
            mock_bind_tools.return_value = MagicMock()

            # Streaming disabled, show_token_usage explicitly enabled
            rosa = ROSA(
                ros_version=1,
                llm=mock_openai_llm,
                streaming=False,
                show_token_usage=True,
            )

            self.assertTrue(rosa._ROSA__show_token_usage)


if __name__ == "__main__":
    unittest.main()
