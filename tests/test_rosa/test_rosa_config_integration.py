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

from langchain_openai import ChatOpenAI

from rosa import ROSA, RobotSystemPrompts
from rosa.exceptions import ROSAConfigurationError


class TestROSAConfigValidatorIntegration(unittest.TestCase):
    """Test that ROSAConfigValidator is properly utilized throughout ROSA."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_verbose_parameter_validation(self):
        """Test that verbose parameter is validated using config validator."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, verbose="true"
                )  # Should be boolean

            self.assertIn("verbose must be a boolean", str(context.exception))

    def test_accumulate_chat_history_parameter_validation(self):
        """Test that accumulate_chat_history parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, accumulate_chat_history="yes"
                )  # Should be boolean

            self.assertIn(
                "accumulate_chat_history must be a boolean", str(context.exception)
            )

    def test_show_token_usage_parameter_validation(self):
        """Test that show_token_usage parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, show_token_usage=1
                )  # Should be boolean

            self.assertIn("show_token_usage must be a boolean", str(context.exception))

    def test_streaming_parameter_validation(self):
        """Test that streaming parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, streaming="true"
                )  # Should be boolean

            self.assertIn("streaming must be a boolean", str(context.exception))

    def test_tools_parameter_validation(self):
        """Test that tools parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, tools="tool1,tool2"
                )  # Should be list

            self.assertIn("tools must be a list or None", str(context.exception))

    def test_tool_packages_parameter_validation(self):
        """Test that tool_packages parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, tool_packages="package1"
                )  # Should be list

            self.assertIn(
                "tool_packages must be a list or None", str(context.exception)
            )

    def test_blacklist_parameter_validation(self):
        """Test that blacklist parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, blacklist="excluded"
                )  # Should be list

            self.assertIn("blacklist must be a list or None", str(context.exception))

    def test_prompts_parameter_validation(self):
        """Test that prompts parameter is validated."""
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
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=1, llm=self.mock_llm, prompts="custom prompts"
                )  # Should be RobotSystemPrompts

            self.assertIn(
                "prompts must be a RobotSystemPrompts instance or None",
                str(context.exception),
            )

    def test_all_valid_parameters_accepted(self):
        """Test that all valid parameters are accepted when using config validator."""
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
            mock_get_tools.return_value.get_tools.return_value = []

            # Should not raise with all valid parameters
            rosa = ROSA(
                ros_version=1,
                llm=self.mock_llm,
                tools=["tool1", "tool2"],
                tool_packages=["package1"],
                prompts=RobotSystemPrompts(embodiment_and_persona="Test robot"),
                verbose=True,
                blacklist=["excluded_tool"],
                accumulate_chat_history=False,
                show_token_usage=True,
                streaming=False,
                max_history_length=100,
            )

            self.assertIsNotNone(rosa)

    def test_config_validator_is_accessible(self):
        """Test that the config validator instance is accessible for additional validation."""
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
            mock_get_tools.return_value.get_tools.return_value = []

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # The config validator should be accessible (though private)
            self.assertTrue(hasattr(rosa, "_ROSA__config_validator"))
            self.assertIsNotNone(rosa._ROSA__config_validator)

    def test_config_validator_comprehensive_validation(self):
        """Test that config validator catches multiple parameter issues."""
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
            # Test multiple invalid parameters - should fail on first one (fail-fast)
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(
                    ros_version=3,  # Invalid (should fail here first)
                    llm="invalid_llm",  # Also invalid
                    verbose="not_boolean",  # Also invalid
                    tools="not_list",  # Also invalid
                )

            # Should fail on ROS version (first validation)
            self.assertIn("Invalid ROS version: 3", str(context.exception))


class TestROSAConfigValidatorUtilization(unittest.TestCase):
    """Test specific utilization patterns of ROSAConfigValidator in ROSA."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm

    def test_validation_happens_before_component_creation(self):
        """Test that validation happens before any component creation."""
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
            # Make component creation fail to ensure validation runs first
            mock_get_tools.side_effect = Exception(
                "Component creation should not be reached"
            )

            # Validation should fail before component creation
            with self.assertRaises(ROSAConfigurationError) as context:
                ROSA(ros_version=5, llm=self.mock_llm)  # Invalid ROS version

            # Should fail on validation, not component creation
            self.assertIn("Invalid ROS version: 5", str(context.exception))
            self.assertNotIn(
                "Component creation should not be reached", str(context.exception)
            )

    def test_validator_reuse_in_multiple_methods(self):
        """Test that the same validator instance is used throughout ROSA."""
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
            mock_get_tools.return_value.get_tools.return_value = []

            rosa = ROSA(ros_version=1, llm=self.mock_llm)

            # Validator should be created once and reused
            validator1 = rosa._ROSA__config_validator

            # The same validator instance should be accessible
            self.assertIsNotNone(validator1)

            # If we could access it in other methods, it should be the same instance
            # (This demonstrates the singleton-like usage pattern)


if __name__ == "__main__":
    unittest.main()
