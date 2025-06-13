#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
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
from langchain_openai import ChatOpenAI

from rosa import ROSA, RobotSystemPrompts
from rosa.rosa import ROSAError, ROSAConfigurationError, ROSAExecutionError


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
        
        with patch('rosa.rosa.ROSATools'), \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
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
        with patch('rosa.rosa.ROSATools') as mock_tools, \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            mock_tools_instance = MagicMock()
            mock_tools.return_value = mock_tools_instance
            mock_tools_instance.get_tools.return_value = []
            
            rosa = ROSA(ros_version=1, llm=self.mock_llm)
            
            self.assertEqual(rosa._ROSA__ros_version, 1)
            self.assertTrue(rosa._ROSA__streaming)  # default
            self.assertTrue(rosa._ROSA__accumulate_chat_history)  # default
            self.assertEqual(rosa._ROSA__blacklist, [])
            self.assertFalse(rosa._ROSA__show_token_usage)  # disabled when streaming=True
    
    def test_initialization_with_custom_params(self):
        """Test ROSA initialization with custom parameters."""
        custom_blacklist = ["tool1", "tool2"]
        custom_prompts = RobotSystemPrompts(embodiment_and_persona="Test robot")
        
        with patch('rosa.rosa.ROSATools') as mock_tools, \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            mock_tools_instance = MagicMock()
            mock_tools.return_value = mock_tools_instance
            mock_tools_instance.get_tools.return_value = []
            
            rosa = ROSA(
                ros_version=2,
                llm=self.mock_llm,
                blacklist=custom_blacklist,
                prompts=custom_prompts,
                streaming=False,
                accumulate_chat_history=False,
                show_token_usage=True
            )
            
            self.assertEqual(rosa._ROSA__ros_version, 2)
            self.assertFalse(rosa._ROSA__streaming)
            self.assertFalse(rosa._ROSA__accumulate_chat_history)
            self.assertEqual(rosa._ROSA__blacklist, custom_blacklist)
            self.assertTrue(rosa._ROSA__show_token_usage)
    
    def test_logger_setup(self):
        """Test that logger is properly set up."""
        with patch('rosa.rosa.ROSATools') as mock_tools, \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            mock_tools_instance = MagicMock()
            mock_tools.return_value = mock_tools_instance
            mock_tools_instance.get_tools.return_value = []
            
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
        
        with patch('rosa.rosa.ROSATools') as mock_tools, \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            mock_tools_instance = MagicMock()
            mock_tools.return_value = mock_tools_instance
            mock_tools_instance.get_tools.return_value = []
            
            self.rosa = ROSA(ros_version=1, llm=self.mock_llm)
    
    def test_clear_chat(self):
        """Test clear_chat method."""
        # Add some chat history
        self.rosa._ROSA__chat_history = [
            HumanMessage(content="test query"),
            AIMessage(content="test response")
        ]
        
        # Clear chat history
        self.rosa.clear_chat()
        
        self.assertEqual(len(self.rosa._ROSA__chat_history), 0)
    
    def test_chat_history_property(self):
        """Test chat_history property."""
        test_history = [HumanMessage(content="test")]
        self.rosa._ROSA__chat_history = test_history
        
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
        
        with patch('rosa.rosa.get_openai_callback') as mock_callback:
            mock_cb = MagicMock()
            mock_callback.return_value.__enter__.return_value = mock_cb
            
            result = self.rosa.invoke("test query")
            
            self.assertEqual(result, "test response")
            mock_executor.invoke.assert_called_once_with({
                "input": "test query",
                ROSA.MEMORY_KEY: self.rosa._ROSA__chat_history
            })
    
    def test_invoke_executor_failure_raises_execution_error(self):
        """Test that executor failure raises ROSAExecutionError."""
        mock_executor = MagicMock()
        mock_executor.invoke.side_effect = Exception("Executor failed")
        self.rosa._ROSA__executor = mock_executor
        
        with patch('rosa.rosa.get_openai_callback'):
            # Suppress logging during this test to avoid confusing error messages
            with patch.object(self.rosa._ROSA__logger, 'error'):
                with self.assertRaisesRegex(ROSAExecutionError, "Agent execution failed"):
                    self.rosa.invoke("test query")
    
    def test_invoke_missing_output_raises_execution_error(self):
        """Test that missing output key raises ROSAExecutionError."""
        mock_executor = MagicMock()
        mock_executor.invoke.return_value = {}  # Missing 'output' key
        self.rosa._ROSA__executor = mock_executor
        
        with patch('rosa.rosa.get_openai_callback'):
            # Suppress logging during this test to avoid confusing error messages
            with patch.object(self.rosa._ROSA__logger, 'error'):
                with self.assertRaisesRegex(ROSAExecutionError, "Agent result missing 'output' key"):
                    self.rosa.invoke("test query")
    
    def test_invoke_updates_chat_history(self):
        """Test that successful invoke updates chat history."""
        self.rosa._ROSA__accumulate_chat_history = True
        mock_executor = MagicMock()
        mock_executor.invoke.return_value = {"output": "test response"}
        self.rosa._ROSA__executor = mock_executor
        
        with patch('rosa.rosa.get_openai_callback'):
            self.rosa.invoke("test query")
            
            history = self.rosa._ROSA__chat_history
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
        with patch('rosa.rosa.ROSATools'), \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            rosa = ROSA(ros_version=1, llm=self.mock_llm)
            
            # Should not raise for valid inputs
            rosa._validate_inputs(1, self.mock_llm)
            rosa._validate_inputs(2, self.mock_llm)
    
    def test_validate_inputs_invalid_ros_version(self):
        """Test _validate_inputs with invalid ROS version."""
        with patch('rosa.rosa.ROSATools'), \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            rosa = ROSA(ros_version=1, llm=self.mock_llm)
            
            with self.assertRaisesRegex(ROSAConfigurationError, "Invalid ROS version: 3"):
                rosa._validate_inputs(3, self.mock_llm)
    
    def test_record_chat_history_enabled(self):
        """Test _record_chat_history when accumulation is enabled."""
        with patch('rosa.rosa.ROSATools'), \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            rosa = ROSA(ros_version=1, llm=self.mock_llm, accumulate_chat_history=True)
            
            rosa._record_chat_history("test query", "test response")
            
            history = rosa._ROSA__chat_history
            self.assertEqual(len(history), 2)
            self.assertIsInstance(history[0], HumanMessage)
            self.assertEqual(history[0].content, "test query")
            self.assertIsInstance(history[1], AIMessage)
            self.assertEqual(history[1].content, "test response")
    
    def test_record_chat_history_disabled(self):
        """Test _record_chat_history when accumulation is disabled."""
        with patch('rosa.rosa.ROSATools'), \
             patch.object(ROSA, '_get_prompts'), \
             patch.object(ROSA, '_get_agent'), \
             patch.object(ROSA, '_get_executor'):
            
            rosa = ROSA(ros_version=1, llm=self.mock_llm, accumulate_chat_history=False)
            
            rosa._record_chat_history("test query", "test response")
            
            self.assertEqual(len(rosa._ROSA__chat_history), 0)


if __name__ == '__main__':
    unittest.main()