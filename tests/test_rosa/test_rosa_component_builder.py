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

from langchain.agents import AgentExecutor
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.runnables import Runnable
from langchain_ollama import ChatOllama
from langchain_openai import AzureChatOpenAI, ChatOpenAI

from rosa import RobotSystemPrompts
from rosa.helpers.component_builder import ROSAComponentBuilder
from rosa.tools import ROSATools


class TestROSAComponentBuilder(unittest.TestCase):
    """Test ROSAComponentBuilder class."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

        # Create component builder instance
        self.builder = ROSAComponentBuilder(
            ros_version=1, llm=self.mock_llm, streaming=True
        )

    def test_component_builder_initialization(self):
        """Test that ROSAComponentBuilder initializes correctly."""
        self.assertEqual(self.builder._ros_version, 1)
        self.assertEqual(self.builder._llm, self.mock_llm)
        self.assertTrue(self.builder._streaming)

    def test_get_tools_creates_rosa_tools(self):
        """Test that get_tools creates ROSATools instance correctly."""
        with patch("rosa.helpers.component_builder.ROSATools") as mock_rosa_tools_class:
            mock_rosa_tools_instance = MagicMock()
            mock_rosa_tools_class.return_value = mock_rosa_tools_instance

            # Test without additional tools or packages
            result = self.builder.get_tools(
                packages=None, tools=None, blacklist=["tool1"]
            )

            # Verify ROSATools was created with correct parameters
            mock_rosa_tools_class.assert_called_once_with(1, blacklist=["tool1"])
            self.assertEqual(result, mock_rosa_tools_instance)

    def test_get_tools_with_additional_tools(self):
        """Test that get_tools adds additional tools correctly."""
        with patch("rosa.helpers.component_builder.ROSATools") as mock_rosa_tools_class:
            mock_rosa_tools_instance = MagicMock()
            mock_rosa_tools_class.return_value = mock_rosa_tools_instance

            custom_tools = ["tool1", "tool2"]

            result = self.builder.get_tools(
                packages=None, tools=custom_tools, blacklist=None
            )

            # Verify tools were added
            mock_rosa_tools_instance.add_tools.assert_called_once_with(custom_tools)
            self.assertEqual(result, mock_rosa_tools_instance)

    def test_get_tools_with_packages(self):
        """Test that get_tools adds packages correctly."""
        with patch("rosa.helpers.component_builder.ROSATools") as mock_rosa_tools_class:
            mock_rosa_tools_instance = MagicMock()
            mock_rosa_tools_class.return_value = mock_rosa_tools_instance

            custom_packages = ["package1", "package2"]
            blacklist = ["excluded_tool"]

            result = self.builder.get_tools(
                packages=custom_packages, tools=None, blacklist=blacklist
            )

            # Verify packages were added with blacklist
            mock_rosa_tools_instance.add_packages.assert_called_once_with(
                custom_packages, blacklist=blacklist
            )
            self.assertEqual(result, mock_rosa_tools_instance)

    def test_get_prompts_default_prompts(self):
        """Test that get_prompts creates template with default prompts."""
        template = self.builder.get_prompts()

        self.assertIsInstance(template, ChatPromptTemplate)
        # Verify it has the expected structure
        self.assertTrue(hasattr(template, "messages"))

    def test_get_prompts_with_robot_prompts(self):
        """Test that get_prompts includes robot-specific prompts."""
        robot_prompts = RobotSystemPrompts(embodiment_and_persona="Test robot persona")

        with patch(
            "rosa.helpers.component_builder.system_prompts", []
        ):  # Mock empty default prompts
            template = self.builder.get_prompts(robot_prompts)

            self.assertIsInstance(template, ChatPromptTemplate)

    def test_get_agent_returns_runnable(self):
        """Test that get_agent returns properly configured agent runnable."""
        # Mock prompts and llm with tools
        mock_prompts = MagicMock(spec=ChatPromptTemplate)
        mock_llm_with_tools = MagicMock()

        agent = self.builder.get_agent(mock_prompts, mock_llm_with_tools)

        # The agent is a composition of runnables, so it should be a Runnable
        self.assertTrue(hasattr(agent, "__or__"))  # Runnable protocol
        self.assertTrue(hasattr(agent, "invoke"))  # Runnable protocol

    def test_get_executor_returns_agent_executor(self):
        """Test that get_executor returns properly configured AgentExecutor."""
        # Mock the agent and tools
        mock_agent = MagicMock(spec=Runnable)
        mock_tools = MagicMock(spec=ROSATools)
        mock_tools.get_tools.return_value = []

        # Test executor creation
        executor = self.builder.get_executor(
            agent=mock_agent, tools=mock_tools, verbose=False
        )

        self.assertIsInstance(executor, AgentExecutor)
        # AgentExecutor wraps the agent in a RunnableAgent, so check the underlying runnable
        self.assertEqual(executor.agent.runnable, mock_agent)
        self.assertEqual(executor.tools, [])
        self.assertFalse(executor.verbose)
        self.assertEqual(executor.max_iterations, 75)
        self.assertEqual(executor.max_execution_time, 300)

    def test_get_executor_with_verbose(self):
        """Test that get_executor respects verbose parameter."""
        mock_agent = MagicMock(spec=Runnable)
        mock_tools = MagicMock(spec=ROSATools)
        mock_tools.get_tools.return_value = []

        executor = self.builder.get_executor(
            agent=mock_agent, tools=mock_tools, verbose=True
        )

        self.assertTrue(executor.verbose)

    def test_get_usage_callback_openai_llm(self):
        """Test that get_usage_callback returns OpenAI callback for OpenAI models."""
        # Set up a real OpenAI LLM instance
        openai_llm = ChatOpenAI(api_key="fake-key", model="gpt-3.5-turbo")
        builder = ROSAComponentBuilder(ros_version=1, llm=openai_llm, streaming=False)

        with patch(
            "rosa.helpers.component_builder.get_openai_callback"
        ) as mock_get_openai_callback:
            mock_callback = MagicMock()
            mock_get_openai_callback.return_value = mock_callback

            result = builder.get_usage_callback()

            mock_get_openai_callback.assert_called_once()
            self.assertEqual(result, mock_callback)

    def test_get_usage_callback_azure_openai_llm(self):
        """Test that get_usage_callback returns OpenAI callback for Azure OpenAI models."""
        # Mock Azure OpenAI LLM
        azure_llm = MagicMock(spec=AzureChatOpenAI)
        builder = ROSAComponentBuilder(ros_version=1, llm=azure_llm, streaming=False)

        with patch(
            "rosa.helpers.component_builder.get_openai_callback"
        ) as mock_get_openai_callback:
            mock_callback = MagicMock()
            mock_get_openai_callback.return_value = mock_callback

            result = builder.get_usage_callback()

            mock_get_openai_callback.assert_called_once()
            self.assertEqual(result, mock_callback)

    def test_get_usage_callback_ollama_llm(self):
        """Test that get_usage_callback returns null callback for Ollama models."""
        # Mock Ollama LLM
        ollama_llm = MagicMock(spec=ChatOllama)
        builder = ROSAComponentBuilder(ros_version=1, llm=ollama_llm, streaming=False)

        with patch.object(builder, "get_null_callback") as mock_get_null_callback:
            mock_null_callback = MagicMock()
            mock_get_null_callback.return_value = mock_null_callback

            result = builder.get_usage_callback()

            mock_get_null_callback.assert_called_once()
            self.assertEqual(result, mock_null_callback)

    def test_get_usage_callback_generic_llm(self):
        """Test that get_usage_callback returns null callback for generic models."""
        # Mock generic LLM
        generic_llm = MagicMock()
        builder = ROSAComponentBuilder(ros_version=1, llm=generic_llm, streaming=False)

        with patch.object(builder, "get_null_callback") as mock_get_null_callback:
            mock_null_callback = MagicMock()
            mock_get_null_callback.return_value = mock_null_callback

            result = builder.get_usage_callback()

            mock_get_null_callback.assert_called_once()
            self.assertEqual(result, mock_null_callback)

    def test_get_null_callback_returns_context_manager(self):
        """Test that get_null_callback returns a working context manager."""
        null_callback = self.builder.get_null_callback()

        # Test that it can be used as a context manager
        with null_callback as cb:
            self.assertEqual(cb.total_tokens, 0)
            self.assertEqual(cb.prompt_tokens, 0)
            self.assertEqual(cb.completion_tokens, 0)
            self.assertEqual(cb.total_cost, 0.0)

    def test_bind_tools_to_llm(self):
        """Test that bind_tools_to_llm returns LLM with tools bound."""
        mock_tools = MagicMock(spec=ROSATools)
        mock_tools.get_tools.return_value = ["tool1", "tool2"]

        result = self.builder.bind_tools_to_llm(mock_tools)

        # Verify bind_tools was called on the LLM
        self.mock_llm.bind_tools.assert_called_once_with(["tool1", "tool2"])
        self.assertEqual(result, self.mock_llm)


class TestROSAComponentBuilderIntegration(unittest.TestCase):
    """Test ROSAComponentBuilder integration scenarios."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

        self.builder = ROSAComponentBuilder(
            ros_version=2, llm=self.mock_llm, streaming=False
        )

    def test_full_component_creation_workflow(self):
        """Test complete workflow of creating all components."""
        with patch("rosa.helpers.component_builder.ROSATools") as mock_rosa_tools_class:
            # Setup mock tools
            mock_rosa_tools_instance = MagicMock()
            mock_rosa_tools_class.return_value = mock_rosa_tools_instance
            mock_rosa_tools_instance.get_tools.return_value = []

            # Step 1: Create tools
            tools = self.builder.get_tools(
                packages=["custom_package"],
                tools=["custom_tool"],
                blacklist=["excluded_tool"],
            )

            # Step 2: Create prompts
            prompts = self.builder.get_prompts()

            # Step 3: Bind tools to LLM
            llm_with_tools = self.builder.bind_tools_to_llm(tools)

            # Step 4: Create agent
            agent = self.builder.get_agent(prompts, llm_with_tools)

            # Step 5: Create executor
            executor = self.builder.get_executor(agent, tools, verbose=True)

            # Verify all components were created
            self.assertIsNotNone(tools)
            self.assertIsNotNone(prompts)
            self.assertIsNotNone(llm_with_tools)
            self.assertIsNotNone(agent)
            self.assertIsInstance(executor, AgentExecutor)
            self.assertTrue(executor.verbose)

    def test_component_builder_with_different_ros_versions(self):
        """Test that component builder works with different ROS versions."""
        # Test ROS1
        builder_ros1 = ROSAComponentBuilder(
            ros_version=1, llm=self.mock_llm, streaming=True
        )
        self.assertEqual(builder_ros1._ros_version, 1)

        # Test ROS2
        builder_ros2 = ROSAComponentBuilder(
            ros_version=2, llm=self.mock_llm, streaming=False
        )
        self.assertEqual(builder_ros2._ros_version, 2)

    def test_component_builder_preserves_llm_configuration(self):
        """Test that component builder preserves LLM streaming configuration."""
        # Test with streaming enabled
        streaming_builder = ROSAComponentBuilder(
            ros_version=1, llm=self.mock_llm, streaming=True
        )
        self.assertTrue(streaming_builder._streaming)

        # Test with streaming disabled
        non_streaming_builder = ROSAComponentBuilder(
            ros_version=1, llm=self.mock_llm, streaming=False
        )
        self.assertFalse(non_streaming_builder._streaming)


if __name__ == "__main__":
    unittest.main()
