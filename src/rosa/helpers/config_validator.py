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

"""Configuration validator for ROSA initialization."""

from typing import Any

from langchain_core.language_models.chat_models import BaseChatModel

from ..exceptions import ROSAConfigurationError


class ROSAConfigValidator:
    """Helper class for validating ROSA configuration parameters.

    This class encapsulates all input validation logic for ROSA initialization,
    separating validation concerns from the main ROSA class and making validation
    logic reusable and testable.

    Methods:
        validate_ros_version: Validate ROS version parameter.
        validate_llm: Validate language model parameter.
        validate_max_history_length: Validate maximum history length parameter.
        validate_boolean_parameter: Validate boolean parameters.
        validate_optional_list_parameter: Validate optional list parameters.
        validate_all_inputs: Validate all core input parameters together.
    """

    def validate_ros_version(self, ros_version: Any) -> None:
        """Validate ROS version parameter.

        Args:
            ros_version: The ROS version to validate.

        Raises:
            ROSAConfigurationError: If ros_version is invalid.
        """
        if ros_version not in (1, 2):
            raise ROSAConfigurationError(
                f"Invalid ROS version: {ros_version}. Must be 1 or 2."
            )

    def validate_llm(self, llm: Any) -> None:
        """Validate language model parameter.

        Args:
            llm: The language model to validate.

        Raises:
            ROSAConfigurationError: If llm is invalid.
        """
        if not isinstance(llm, BaseChatModel):
            raise ROSAConfigurationError(
                f"Invalid LLM type: {type(llm)}. Must be a LangChain BaseChatModel instance."
            )

    def validate_max_history_length(self, max_history_length: Any) -> None:
        """Validate maximum history length parameter.

        Args:
            max_history_length: The maximum history length to validate.

        Raises:
            ROSAConfigurationError: If max_history_length is invalid.
        """
        if max_history_length is not None and (
            not isinstance(max_history_length, int) or max_history_length <= 0
        ):
            raise ROSAConfigurationError(
                f"Invalid max_history_length: {max_history_length}. Must be a positive integer or None."
            )

    def validate_boolean_parameter(self, value: Any, parameter_name: str) -> None:
        """Validate a boolean parameter.

        Args:
            value: The value to validate.
            parameter_name: Name of the parameter for error messages.

        Raises:
            ROSAConfigurationError: If value is not a boolean.
        """
        if not isinstance(value, bool):
            raise ROSAConfigurationError(
                f"{parameter_name} must be a boolean, got {type(value).__name__}: {value}"
            )

    def validate_optional_list_parameter(self, value: Any, parameter_name: str) -> None:
        """Validate an optional list parameter.

        Args:
            value: The value to validate.
            parameter_name: Name of the parameter for error messages.

        Raises:
            ROSAConfigurationError: If value is not a list or None.
        """
        if value is not None and not isinstance(value, list):
            raise ROSAConfigurationError(
                f"{parameter_name} must be a list or None, got {type(value).__name__}: {value}"
            )

    def validate_prompts_parameter(self, prompts: Any) -> None:
        """Validate prompts parameter.

        Args:
            prompts: The prompts parameter to validate.

        Raises:
            ROSAConfigurationError: If prompts is not a RobotSystemPrompts instance or None.
        """
        from ..prompts import RobotSystemPrompts

        if prompts is not None and not isinstance(prompts, RobotSystemPrompts):
            raise ROSAConfigurationError(
                f"prompts must be a RobotSystemPrompts instance or None, got {type(prompts).__name__}: {prompts}"
            )

    def validate_all_inputs(
        self, ros_version: Any, llm: Any, max_history_length: Any
    ) -> None:
        """Validate all core ROSA input parameters.

        This method validates the three core parameters required for ROSA
        initialization. It uses a fail-fast approach, raising an exception
        on the first invalid parameter encountered.

        Args:
            ros_version: The ROS version to validate.
            llm: The language model to validate.
            max_history_length: The maximum history length to validate.

        Raises:
            ROSAConfigurationError: If any parameter is invalid.
        """
        # Validate in order of importance
        self.validate_ros_version(ros_version)
        self.validate_llm(llm)
        self.validate_max_history_length(max_history_length)
