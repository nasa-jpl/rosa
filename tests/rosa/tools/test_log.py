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

import unittest
from unittest.mock import patch, mock_open

from src.rosa.tools.log import read_log


class TestReadLog(unittest.TestCase):

    @patch("os.path.exists")
    def test_log_directory_does_not_exist(self, mock_exists):
        mock_exists.return_value = False
        result = read_log.invoke(
            {
                "log_file_directory": "/invalid/directory",
                "log_filename": "logfile.log",
            }
        )
        self.assertEqual(
            result["error"],
            "The log directory '/invalid/directory' does not exist. You should first use your tools to get the "
            "correct log directory.",
        )

    @patch("os.path.exists")
    def test_log_path_is_not_a_file(self, mock_exists):
        mock_exists.side_effect = [True, True]
        with patch("os.path.isfile", return_value=False):
            result = read_log.invoke(
                {
                    "log_file_directory": "/valid/directory",
                    "log_filename": "logfile.log",
                }
            )
            self.assertEqual(
                result["error"],
                "The path '/valid/directory/logfile.log' is not a file.",
            )

    @patch(
        "builtins.open",
        new_callable=mock_open,
        read_data="INFO: line 1\nERROR: line 2\nDEBUG: line 3\n",
    )
    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_read_log_with_level_filter(self, mock_exists, mock_isfile, mock_file):
        result = read_log.invoke(
            {
                "log_file_directory": "/valid/directory",
                "log_filename": "logfile.log",
                "level_filter": "ERROR",
            }
        )
        self.assertEqual(result["lines"], ["line 2: ERROR: line 2"])

    @patch(
        "builtins.open",
        new_callable=mock_open,
        read_data="INFO: line 1\nERROR: line 2\nDEBUG: line 3\n",
    )
    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_read_log_with_line_range(self, mock_exists, mock_isfile, mock_file):
        result = read_log.invoke(
            {
                "log_file_directory": "/valid/directory",
                "log_filename": "logfile.log",
                "num_lines": 2,
            }
        )
        self.assertEqual(
            result["lines"], ["line 2: ERROR: line 2", "line 3: DEBUG: line 3"]
        )

    @patch("builtins.open", new_callable=mock_open, read_data="INFO: line 1\n" * 202)
    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_log_file_exceeds_200_lines(self, mock_exists, mock_isfile, mock_file):
        result = read_log.invoke(
            {
                "log_file_directory": "/valid/directory",
                "log_filename": "logfile.log",
                "num_lines": 203,
            }
        )
        self.assertEqual(
            result["error"],
            "The log file 'logfile.log' has more than 200 lines. Please use the `num_lines` argument to read a subset "
            "of the log file at a time.",
        )

    @patch(
        "builtins.open",
        new_callable=mock_open,
        read_data="INFO: line 1\nERROR: line 2\nDEBUG: line 3\n",
    )
    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_read_log_happy_path(self, mock_exists, mock_isfile, mock_file):
        result = read_log.invoke(
            {
                "log_file_directory": "/valid/directory",
                "log_filename": "logfile.log",
            }
        )
        self.assertEqual(
            result["lines"],
            ["line 1: INFO: line 1", "line 2: ERROR: line 2", "line 3: DEBUG: line 3"],
        )

    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_invalid_num_lines_argument(self, mock_exists, mock_isfile):
        with patch(
            "builtins.open",
            new_callable=mock_open,
            read_data="INFO: line 1\nERROR: line 2\n",
        ):
            result = read_log.invoke(
                {
                    "log_file_directory": "/valid/directory",
                    "log_filename": "logfile.log",
                    "num_lines": -1,
                }
            )
            self.assertEqual(
                result["error"],
                "Invalid `num_lines` argument. It must be a positive integer.",
            )

    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_empty_log_file(self, mock_exists, mock_isfile):
        with patch("builtins.open", new_callable=mock_open, read_data=""):
            result = read_log.invoke(
                {
                    "log_file_directory": "/valid/directory",
                    "log_filename": "logfile.log",
                }
            )
            self.assertEqual(result["lines"], [])

    @patch("os.path.exists", return_value=True)
    @patch("os.path.isfile", return_value=True)
    def test_specific_log_level_not_present(self, mock_exists, mock_isfile):
        with patch(
            "builtins.open",
            new_callable=mock_open,
            read_data="INFO: line 1\nDEBUG: line 2\n",
        ):
            result = read_log.invoke(
                {
                    "log_file_directory": "/valid/directory",
                    "log_filename": "logfile.log",
                    "level_filter": "ERROR",
                }
            )
            self.assertEqual(result["lines"], [])


if __name__ == "__main__":
    unittest.main()
