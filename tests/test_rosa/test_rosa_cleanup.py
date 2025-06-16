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
import unittest.mock
from unittest.mock import MagicMock, patch

from langchain_openai import ChatOpenAI

from rosa import ROSA
from rosa.exceptions import ROSAError


class TestROSACleanup(unittest.TestCase):
    """Test ROSA resource cleanup functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

        # Mock the tools to avoid ROS dependencies in tests
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
            self.rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

    def test_cleanup_method_exists(self):
        """Test that cleanup method exists."""
        self.assertTrue(hasattr(self.rosa, "cleanup"))
        self.assertTrue(callable(self.rosa.cleanup))

    def test_cleanup_method_signature(self):
        """Test that cleanup method has correct signature."""
        import inspect

        sig = inspect.signature(self.rosa.cleanup)
        self.assertEqual(len(sig.parameters), 0, "cleanup() should take no parameters")
        self.assertEqual(sig.return_annotation, None, "cleanup() should return None")

    def test_cleanup_idempotent(self):
        """Test that cleanup can be called multiple times safely."""
        # First cleanup should not raise
        self.rosa.cleanup()

        # Second cleanup should also not raise
        self.rosa.cleanup()

        # Third cleanup should also not raise
        self.rosa.cleanup()

    def test_cleanup_sets_cleaned_up_flag(self):
        """Test that cleanup sets internal _is_cleaned_up flag."""
        # Initially should not be cleaned up
        self.assertFalse(getattr(self.rosa, "_is_cleaned_up", False))

        # After cleanup, should be marked as cleaned up
        self.rosa.cleanup()
        self.assertTrue(getattr(self.rosa, "_is_cleaned_up", False))

    def test_private_cleanup_methods_exist(self):
        """Test that private cleanup methods exist."""
        self.assertTrue(hasattr(self.rosa, "_cleanup_executor"))
        self.assertTrue(hasattr(self.rosa, "_cleanup_llm"))
        self.assertTrue(hasattr(self.rosa, "_cleanup_tools"))

        self.assertTrue(callable(self.rosa._cleanup_executor))
        self.assertTrue(callable(self.rosa._cleanup_llm))
        self.assertTrue(callable(self.rosa._cleanup_tools))

    def test_cleanup_calls_private_methods(self):
        """Test that cleanup calls all private cleanup methods."""
        with (
            patch.object(self.rosa, "_cleanup_executor") as mock_cleanup_executor,
            patch.object(self.rosa, "_cleanup_llm") as mock_cleanup_llm,
            patch.object(self.rosa, "_cleanup_tools") as mock_cleanup_tools,
        ):
            self.rosa.cleanup()

            mock_cleanup_executor.assert_called_once()
            mock_cleanup_llm.assert_called_once()
            mock_cleanup_tools.assert_called_once()

    def test_cleanup_method_order(self):
        """Test that cleanup methods are called in correct order (tools -> llm -> executor)."""
        call_order = []

        def track_cleanup_executor():
            call_order.append("executor")

        def track_cleanup_llm():
            call_order.append("llm")

        def track_cleanup_tools():
            call_order.append("tools")

        with (
            patch.object(
                self.rosa, "_cleanup_executor", side_effect=track_cleanup_executor
            ),
            patch.object(self.rosa, "_cleanup_llm", side_effect=track_cleanup_llm),
            patch.object(self.rosa, "_cleanup_tools", side_effect=track_cleanup_tools),
        ):
            self.rosa.cleanup()

            self.assertEqual(call_order, ["tools", "llm", "executor"])

    def test_cleanup_handles_exceptions_in_private_methods(self):
        """Test that cleanup handles exceptions in private methods gracefully."""
        with (
            patch.object(
                self.rosa, "_cleanup_executor", side_effect=Exception("executor error")
            ),
            patch.object(self.rosa, "_cleanup_llm", side_effect=Exception("llm error")),
            patch.object(
                self.rosa, "_cleanup_tools", side_effect=Exception("tools error")
            ),
            patch.object(self.rosa, "_ROSA__logger") as mock_logger,
        ):
            # Cleanup should not raise exceptions even if private methods fail
            try:
                self.rosa.cleanup()
            except Exception:
                self.fail(
                    "cleanup() should not raise exceptions from private cleanup methods"
                )

            # Verify that warnings were logged for each exception
            expected_warning_calls = [
                unittest.mock.call.warning("Error during tools cleanup: tools error"),
                unittest.mock.call.warning("Error during LLM cleanup: llm error"),
                unittest.mock.call.warning(
                    "Error during executor cleanup: executor error"
                ),
            ]
            mock_logger.assert_has_calls(expected_warning_calls, any_order=False)

            # Should still mark as cleaned up even if some cleanup methods failed
            self.assertTrue(getattr(self.rosa, "_is_cleaned_up", False))


class TestROSAContextManager(unittest.TestCase):
    """Test ROSA context manager functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_llm = MagicMock(spec=ChatOpenAI)
        self.mock_llm.with_config.return_value = self.mock_llm
        self.mock_llm.bind_tools.return_value = self.mock_llm

    def test_context_manager_methods_exist(self):
        """Test that context manager methods exist."""
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
            rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

            self.assertTrue(hasattr(rosa, "__enter__"))
            self.assertTrue(hasattr(rosa, "__exit__"))
            self.assertTrue(callable(rosa.__enter__))
            self.assertTrue(callable(rosa.__exit__))

    def test_enter_returns_self(self):
        """Test that __enter__ returns self."""
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
            rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

            self.assertEqual(rosa.__enter__(), rosa)

    def test_exit_calls_cleanup(self):
        """Test that __exit__ calls cleanup."""
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
            rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

            with patch.object(rosa, "cleanup") as mock_cleanup:
                rosa.__exit__(None, None, None)
                mock_cleanup.assert_called_once()

    def test_context_manager_usage(self):
        """Test using ROSA as context manager."""
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

            with ROSA(ros_version=1, llm=self.mock_llm, streaming=False) as rosa:
                self.assertIsInstance(rosa, ROSA)
                # Should not be cleaned up while in context
                self.assertFalse(getattr(rosa, "_is_cleaned_up", False))

            # Should be cleaned up after exiting context
            self.assertTrue(getattr(rosa, "_is_cleaned_up", False))

    def test_context_manager_with_exception(self):
        """Test context manager cleanup when exception occurs."""
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

            try:
                with ROSA(ros_version=1, llm=self.mock_llm, streaming=False) as rosa:
                    msg = "test exception"
                    raise ValueError(msg)  # noqa: TRY301
            except ValueError:
                pass  # Expected exception

            # Should still be cleaned up even when exception occurs
            self.assertTrue(getattr(rosa, "_is_cleaned_up", False))

    def test_exit_handles_cleanup_exceptions(self):
        """Test that __exit__ handles cleanup exceptions gracefully."""
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
            rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

            with (
                patch.object(rosa, "cleanup", side_effect=Exception("cleanup error")),
                patch.object(rosa, "_ROSA__logger") as mock_logger,
            ):
                # __exit__ should not re-raise cleanup exceptions
                result = rosa.__exit__(None, None, None)
                self.assertIsNone(result)  # Should not suppress original exceptions

                # Verify that the cleanup exception was logged as a warning
                mock_logger.warning.assert_called_once_with(
                    "Error during context manager cleanup: cleanup error"
                )


class TestROSAResourceStateTracking(unittest.TestCase):
    """Test ROSA resource state tracking functionality."""

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
            mock_get_tools.return_value.get_tools.return_value = []
            self.rosa = ROSA(ros_version=1, llm=self.mock_llm, streaming=False)

    def test_is_initialized_flag_exists(self):
        """Test that _is_initialized flag exists and is set."""
        self.assertTrue(hasattr(self.rosa, "_is_initialized"))
        self.assertTrue(self.rosa._is_initialized)

    def test_check_initialized_method_exists(self):
        """Test that _check_initialized method exists."""
        self.assertTrue(hasattr(self.rosa, "_check_initialized"))
        self.assertTrue(callable(self.rosa._check_initialized))

    def test_check_initialized_passes_when_active(self):
        """Test that _check_initialized passes when agent is active."""
        # Should not raise when agent is properly initialized and not cleaned up
        try:
            self.rosa._check_initialized()
        except Exception:
            self.fail("_check_initialized() should not raise for active agent")

    def test_check_initialized_fails_after_cleanup(self):
        """Test that _check_initialized fails after cleanup."""
        self.rosa.cleanup()

        with self.assertRaises(ROSAError) as context:
            self.rosa._check_initialized()

        self.assertIn("cleaned up", str(context.exception).lower())

    def test_invoke_fails_after_cleanup(self):
        """Test that invoke fails after cleanup."""
        self.rosa.cleanup()

        with self.assertRaises(ROSAError) as context:
            self.rosa.invoke("test query")

        self.assertIn("cleaned up", str(context.exception).lower())

    def test_astream_fails_after_cleanup(self):
        """Test that astream fails after cleanup."""
        self.rosa.cleanup()

        # astream should raise immediately when called on a cleaned up agent
        # since _check_initialized() is called at the start of the method
        with self.assertRaises(ROSAError) as context:
            # This should trigger the exception immediately, not return a generator
            import asyncio

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                coro = self.rosa.astream("test query")
                loop.run_until_complete(coro.__anext__())
            finally:
                loop.close()

        self.assertIn("cleaned up", str(context.exception).lower())

    def test_operations_work_before_cleanup(self):
        """Test that operations work normally before cleanup."""
        # Mock the executor to avoid complex setup
        with patch.object(self.rosa, "_ROSA__executor") as mock_executor:
            mock_executor.invoke.return_value = {"output": "test response"}

            # Should work normally before cleanup
            result = self.rosa.invoke("test query")
            self.assertIsInstance(result, str)

    def test_multiple_cleanups_dont_interfere_with_state_check(self):
        """Test that multiple cleanups don't interfere with state checking."""
        # Multiple cleanups
        self.rosa.cleanup()
        self.rosa.cleanup()
        self.rosa.cleanup()

        # State check should still work correctly
        with self.assertRaises(ROSAError):
            self.rosa._check_initialized()


if __name__ == "__main__":
    unittest.main()
