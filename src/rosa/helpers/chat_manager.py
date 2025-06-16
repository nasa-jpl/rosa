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

"""Chat manager for ROSA chat history handling."""

import logging
import sys
from typing import Any, Optional

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage


class ROSAChatManager:
    """Helper class for managing ROSA chat history.

    This class encapsulates all chat history management functionality,
    including trimming, recording, and usage tracking. It separates
    chat management concerns from the main ROSA class.

    Args:
        max_history_length: Maximum number of messages to keep in history.
                          None for unlimited. Defaults to None.
        accumulate_chat_history: Whether to accumulate chat history.
                               Defaults to True.
        logger: Logger instance to use. If None, creates a default logger.

    Attributes:
        _chat_history: List of chat messages.
        _max_history_length: Maximum history length setting.
        _accumulate_chat_history: Whether accumulation is enabled.
        _logger: Logger instance for this manager.
    """

    def __init__(
        self,
        max_history_length: Optional[int] = None,
        accumulate_chat_history: bool = True,
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize the chat manager.

        Args:
            max_history_length: Maximum chat history messages. None for unlimited.
            accumulate_chat_history: Whether to accumulate chat history.
            logger: Logger instance. If None, creates a default logger.
        """
        self._max_history_length = max_history_length
        self._accumulate_chat_history = accumulate_chat_history
        self._chat_history: list = []

        # Set up logger
        if logger is None:
            self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        else:
            self._logger = logger

    def get_chat_history(self) -> list:
        """Get the current chat history.

        Returns:
            List of chat messages.
        """
        return self._chat_history

    def clear_chat_history(self, retain_system_messages: bool = False) -> None:
        """Clear the chat history.

        Args:
            retain_system_messages: If True, keep system messages in the history.
                                  If False, clear all messages.
        """
        if retain_system_messages:
            self._chat_history = [
                msg for msg in self._chat_history if isinstance(msg, SystemMessage)
            ]
        else:
            self._chat_history = []

    def record_chat_history(self, query: str, response: str) -> bool:
        """Record a chat interaction in the history.

        This method atomically adds new Human/AI message pairs to the chat history
        and then applies trimming logic if the history exceeds the maximum length.

        Args:
            query: The user's query.
            response: The agent's response.

        Returns:
            True if recording was successful, False otherwise.
        """
        if not self._accumulate_chat_history:
            return False

        try:
            # Get original length for logging
            original_length = len(self._chat_history)

            # Atomically add the new message pair
            new_messages = [HumanMessage(content=query), AIMessage(content=response)]
            self._chat_history.extend(new_messages)

            # Apply trimming logic if needed
            if self._max_history_length is not None:
                trimmed_history = self.trim_chat_history(self._chat_history)

                # Check if trimming occurred
                if len(trimmed_history) < len(self._chat_history):
                    trimmed_count = len(self._chat_history) - len(trimmed_history)
                    self._logger.debug(
                        f"Chat history trimmed: removed {trimmed_count} messages "
                        f"(from {original_length} → {original_length + 2} → {len(trimmed_history)})"
                    )

                # Update history with trimmed version
                self._chat_history = trimmed_history

        except Exception as e:
            self._logger.error(
                f"Error occurred while recording chat history: {e}", exc_info=True
            )
            return False
        else:
            return True

    def trim_chat_history(self, chat_history: list) -> list:
        """Trim chat history to keep only the most recent messages within the limit.

        Preserves conversation context by keeping Human/AI message pairs intact.
        Uses FIFO strategy - removes oldest pairs first when limit is exceeded.

        Args:
            chat_history: List of messages to trim.

        Returns:
            Trimmed list of messages preserving most recent pairs.
        """
        # No trimming needed if unlimited history or under limit
        if (
            self._max_history_length is None
            or len(chat_history) <= self._max_history_length
        ):
            return chat_history

        # Handle empty history
        if not chat_history:
            return []

        # Calculate how many messages to keep
        target_length = self._max_history_length

        # If odd number of messages, preserve the incomplete pair at the end
        if len(chat_history) % 2 == 1:
            # Keep the last incomplete message plus as many complete pairs as possible
            incomplete_message = chat_history[-1:]
            complete_pairs_section = chat_history[:-1]

            # Calculate how many complete pair messages we can fit
            remaining_slots = target_length - 1  # Reserve 1 slot for incomplete message

            # Each pair takes 2 messages, so keep the most recent pairs
            messages_per_pair = 2
            if (
                remaining_slots >= messages_per_pair
                and len(complete_pairs_section) >= messages_per_pair
            ):
                num_pairs_to_keep = min(
                    remaining_slots // messages_per_pair,
                    len(complete_pairs_section) // messages_per_pair,
                )
                start_index = len(complete_pairs_section) - (
                    num_pairs_to_keep * messages_per_pair
                )
                kept_pairs = complete_pairs_section[start_index:]
                return kept_pairs + incomplete_message
            else:
                # Not enough space for any complete pairs, just keep the incomplete message
                return incomplete_message
        else:
            # Even number of messages - all are complete pairs
            # Keep the most recent complete pairs
            num_pairs_to_keep = target_length // 2
            if num_pairs_to_keep == 0 and target_length > 0:
                # Edge case: we can't keep any complete pairs, but we can keep some messages
                # Keep the most recent messages up to the target length
                return chat_history[-target_length:]
            else:
                start_index = len(chat_history) - (num_pairs_to_keep * 2)
                return chat_history[start_index:]

    def get_history_length(self) -> int:
        """Get the current number of messages in chat history.

        Returns:
            The number of messages in the chat history.
        """
        return len(self._chat_history)

    def trim_history(self, max_length: int) -> None:
        """Manually trim chat history to the specified maximum length.

        This method allows for manual trimming of the chat history, using the same
        logic as the automatic trimming system. Messages are preserved in pairs
        when possible to maintain conversation context.

        Args:
            max_length: Maximum number of messages to keep. Must be positive.

        Raises:
            ValueError: If max_length is not a positive integer.
        """
        if not isinstance(max_length, int) or max_length <= 0:
            msg = "max_length must be a positive integer"
            raise ValueError(msg)

        # Temporarily set the max length to perform trimming
        original_max_length = self._max_history_length
        self._max_history_length = max_length

        try:
            self._chat_history = self.trim_chat_history(self._chat_history)
        finally:
            # Restore original max length setting
            self._max_history_length = original_max_length

    def get_history_usage(self) -> dict[str, Any]:
        """Get information about chat history memory usage and token estimation.

        Returns:
            Dictionary containing:
                - message_count (int): Number of messages in history
                - estimated_tokens (int): Rough estimate of token count
                - memory_bytes (int): Approximate memory usage in bytes
        """
        message_count = len(self._chat_history)

        if message_count == 0:
            return {"message_count": 0, "estimated_tokens": 0, "memory_bytes": 0}

        # Calculate estimated tokens (rough approximation: 1 token ≈ 4 characters)
        total_chars = sum(len(str(msg.content)) for msg in self._chat_history)
        estimated_tokens = total_chars // 4

        # Calculate approximate memory usage
        memory_bytes = sum(
            sys.getsizeof(msg) + sys.getsizeof(msg.content)
            for msg in self._chat_history
        )

        return {
            "message_count": message_count,
            "estimated_tokens": estimated_tokens,
            "memory_bytes": memory_bytes,
        }
