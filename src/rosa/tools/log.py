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

import os
from typing import Optional, Literal

from langchain.agents import tool


@tool
def read_log(
    log_file_directory: str,
    log_filename: str,
    level_filter: Optional[
        Literal["ERROR", "INFO", "DEBUG", "WARNING", "CRITICAL", "FATAL", "TRACE"]
    ] = None,
    num_lines: Optional[int] = None,
) -> dict:
    """
    Read a log file and return the log lines that match the level filter and line range.

    :param log_file_directory: The directory containing the log file to read (use your tools to get it)
    :param log_filename: The path to the log file to read
    :param level_filter: Only show log lines that contain this level (e.g. "ERROR", "INFO", "DEBUG", etc.)
    :param num_lines: The number of most recent lines to return from the log file
    """
    if num_lines is not None and num_lines < 1:
        return {"error": "Invalid `num_lines` argument. It must be a positive integer."}

    if not os.path.exists(log_file_directory):
        return {
            "error": f"The log directory '{log_file_directory}' does not exist. You should first use your tools to "
            f"get the correct log directory."
        }

    full_log_path = os.path.join(log_file_directory, log_filename)

    if not os.path.exists(full_log_path):
        return {
            "error": f"The log file '{log_filename}' does not exist in the log directory '{log_file_directory}'."
        }

    if not os.path.isfile(full_log_path):
        return {"error": f"The path '{full_log_path}' is not a file."}

    with open(full_log_path, "r") as f:
        log_lines = f.readlines()

    total_lines = len(log_lines)

    for i in range(len(log_lines)):
        log_lines[i] = f"line {i+1}: " + log_lines[i].strip()

    if num_lines is not None:
        # Get the most recent num_lines from the log file
        log_lines = log_lines[-num_lines:]

    # If there are more than 200 lines, return a message to use the line_range argument
    if len(log_lines) > 200:
        return {
            "error": f"The log file '{log_filename}' has more than 200 lines. Please use the `num_lines` argument to "
            f"read a subset of the log file at a time."
        }

    if level_filter is not None:
        log_lines = [line for line in log_lines if level_filter.lower() in line.lower()]

    result = {
        "log_filename": log_filename,
        "log_file_directory": log_file_directory,
        "level_filter": level_filter,
        "requested_num_lines": num_lines,
        "total_lines": total_lines,
        "lines_returned": len(log_lines),
        "lines": log_lines,
    }

    return result
