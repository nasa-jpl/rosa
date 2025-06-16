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
from unittest.mock import MagicMock

from langchain_openai import ChatOpenAI

from rosa.exceptions import ROSAConfigurationError
from rosa.helpers.config_validator import ROSAConfigValidator


class TestROSAConfigValidator(unittest.TestCase):
    """Test ROSAConfigValidator class."""

    def setUp(self):
        """Set up test fixtures."""
        self.validator = ROSAConfigValidator()
        self.mock_llm = MagicMock(spec=ChatOpenAI)

    def test_config_validator_initialization(self):
        """Test that ROSAConfigValidator initializes correctly."""
        self.assertIsNotNone(self.validator)

    def test_validate_ros_version_valid(self):
        """Test validation of valid ROS versions."""
        # Should not raise for valid versions
        self.validator.validate_ros_version(1)
        self.validator.validate_ros_version(2)

    def test_validate_ros_version_invalid(self):
        """Test validation of invalid ROS versions."""
        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_ros_version(0)
        self.assertIn("Invalid ROS version: 0", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_ros_version(3)
        self.assertIn("Invalid ROS version: 3", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_ros_version("2")
        self.assertIn("Invalid ROS version: 2", str(context.exception))

    def test_validate_llm_valid(self):
        """Test validation of valid LLM instances."""
        # Should not raise for valid LLM
        real_llm = ChatOpenAI(api_key="fake-key", model="gpt-3.5-turbo")
        self.validator.validate_llm(real_llm)
        self.validator.validate_llm(self.mock_llm)

    def test_validate_llm_invalid(self):
        """Test validation of invalid LLM instances."""
        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_llm("not_an_llm")
        self.assertIn("Invalid LLM type", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_llm(None)
        self.assertIn("Invalid LLM type", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_llm(123)
        self.assertIn("Invalid LLM type", str(context.exception))

    def test_validate_max_history_length_valid(self):
        """Test validation of valid max_history_length values."""
        # Should not raise for valid values
        self.validator.validate_max_history_length(None)  # Unlimited
        self.validator.validate_max_history_length(1)  # Minimum positive
        self.validator.validate_max_history_length(50)  # Typical value
        self.validator.validate_max_history_length(1000)  # Large value

    def test_validate_max_history_length_invalid(self):
        """Test validation of invalid max_history_length values."""
        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_max_history_length(0)
        self.assertIn("Invalid max_history_length: 0", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_max_history_length(-1)
        self.assertIn("Invalid max_history_length: -1", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_max_history_length(3.5)
        self.assertIn("Invalid max_history_length: 3.5", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_max_history_length("50")
        self.assertIn("Invalid max_history_length: 50", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_max_history_length([])
        self.assertIn("Invalid max_history_length: []", str(context.exception))

    def test_validate_all_inputs_valid(self):
        """Test validation of all valid inputs together."""
        # Should not raise for all valid inputs
        self.validator.validate_all_inputs(
            ros_version=1, llm=self.mock_llm, max_history_length=100
        )

        self.validator.validate_all_inputs(
            ros_version=2, llm=self.mock_llm, max_history_length=None
        )

    def test_validate_all_inputs_invalid_ros_version(self):
        """Test validation fails with invalid ROS version."""
        with self.assertRaises(ROSAConfigurationError):
            self.validator.validate_all_inputs(
                ros_version=3, llm=self.mock_llm, max_history_length=50
            )

    def test_validate_all_inputs_invalid_llm(self):
        """Test validation fails with invalid LLM."""
        with self.assertRaises(ROSAConfigurationError):
            self.validator.validate_all_inputs(
                ros_version=1, llm="invalid_llm", max_history_length=50
            )

    def test_validate_all_inputs_invalid_max_history_length(self):
        """Test validation fails with invalid max_history_length."""
        with self.assertRaises(ROSAConfigurationError):
            self.validator.validate_all_inputs(
                ros_version=1, llm=self.mock_llm, max_history_length=-5
            )

    def test_validate_boolean_parameter_valid(self):
        """Test validation of valid boolean parameters."""
        # Should not raise for valid boolean values
        self.validator.validate_boolean_parameter(True, "test_param")
        self.validator.validate_boolean_parameter(False, "test_param")

    def test_validate_boolean_parameter_invalid(self):
        """Test validation of invalid boolean parameters."""
        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_boolean_parameter("true", "test_param")
        self.assertIn("test_param must be a boolean", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_boolean_parameter(1, "test_param")
        self.assertIn("test_param must be a boolean", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_boolean_parameter(None, "test_param")
        self.assertIn("test_param must be a boolean", str(context.exception))

    def test_validate_optional_list_parameter_valid(self):
        """Test validation of valid optional list parameters."""
        # Should not raise for valid values
        self.validator.validate_optional_list_parameter(None, "test_param")
        self.validator.validate_optional_list_parameter([], "test_param")
        self.validator.validate_optional_list_parameter(
            ["item1", "item2"], "test_param"
        )
        self.validator.validate_optional_list_parameter([1, 2, 3], "test_param")

    def test_validate_optional_list_parameter_invalid(self):
        """Test validation of invalid optional list parameters."""
        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_optional_list_parameter("not_a_list", "test_param")
        self.assertIn("test_param must be a list or None", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_optional_list_parameter(123, "test_param")
        self.assertIn("test_param must be a list or None", str(context.exception))

        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_optional_list_parameter({}, "test_param")
        self.assertIn("test_param must be a list or None", str(context.exception))


class TestROSAConfigValidatorIntegration(unittest.TestCase):
    """Test ROSAConfigValidator integration scenarios."""

    def setUp(self):
        """Set up test fixtures."""
        self.validator = ROSAConfigValidator()
        self.mock_llm = MagicMock(spec=ChatOpenAI)

    def test_comprehensive_validation_scenario(self):
        """Test comprehensive validation of multiple parameters."""
        # Test all valid parameters
        self.validator.validate_all_inputs(
            ros_version=1, llm=self.mock_llm, max_history_length=100
        )

        # Test with additional validations
        self.validator.validate_boolean_parameter(True, "streaming")
        self.validator.validate_boolean_parameter(False, "verbose")
        self.validator.validate_optional_list_parameter(["tool1"], "tools")
        self.validator.validate_optional_list_parameter(None, "blacklist")

    def test_validation_error_messages_are_descriptive(self):
        """Test that validation error messages are descriptive and helpful."""
        # ROS version error
        try:
            self.validator.validate_ros_version(5)
        except ROSAConfigurationError as e:
            self.assertIn("Invalid ROS version: 5", str(e))
            self.assertIn("Must be 1 or 2", str(e))

        # LLM type error
        try:
            self.validator.validate_llm("string_llm")
        except ROSAConfigurationError as e:
            self.assertIn("Invalid LLM type", str(e))
            self.assertIn("BaseChatModel", str(e))

        # Max history length error
        try:
            self.validator.validate_max_history_length(-10)
        except ROSAConfigurationError as e:
            self.assertIn("Invalid max_history_length: -10", str(e))
            self.assertIn("positive integer or None", str(e))

    def test_edge_cases_and_boundary_values(self):
        """Test validation with edge cases and boundary values."""
        # Boundary values for max_history_length
        self.validator.validate_max_history_length(1)  # Minimum valid

        with self.assertRaises(ROSAConfigurationError):
            self.validator.validate_max_history_length(0)  # Just below minimum

        # Edge case: very large history length
        self.validator.validate_max_history_length(999999)

        # Edge case: empty lists
        self.validator.validate_optional_list_parameter([], "empty_list")

    def test_multiple_validation_failures(self):
        """Test behavior when multiple validations would fail."""
        # Only the first validation should raise (fail fast)
        with self.assertRaises(ROSAConfigurationError) as context:
            self.validator.validate_all_inputs(
                ros_version=3,  # Invalid
                llm="invalid",  # Also invalid
                max_history_length=-1,  # Also invalid
            )

        # Should fail on the first invalid parameter (ros_version)
        self.assertIn("Invalid ROS version: 3", str(context.exception))


if __name__ == "__main__":
    unittest.main()
