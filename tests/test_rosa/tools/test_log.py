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
import tempfile
import unittest
from unittest.mock import patch, mock_open

from src.rosa.tools.log import read_log


class TestReadLog(unittest.TestCase):

    def setUp(self):
        patcher = patch(
            "src.rosa.tools.log.get_allowed_log_directories",
            return_value=["/valid/directory", "/invalid/directory"],
        )
        self.mock_allowed_dirs = patcher.start()
        self.addCleanup(patcher.stop)

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


    def test_path_traversal_in_filename_is_rejected(self):
        with patch("builtins.open", new_callable=mock_open, read_data="secret\n") as m:
            result = read_log.invoke(
                {
                    "log_file_directory": "/valid/directory",
                    "log_filename": "../../etc/passwd",
                }
            )
        self.assertIn("Access denied", result["error"])
        m.assert_not_called()

    def test_absolute_filename_is_rejected(self):
        with patch("builtins.open", new_callable=mock_open, read_data="secret\n") as m:
            result = read_log.invoke(
                {
                    "log_file_directory": "/valid/directory",
                    "log_filename": "/etc/passwd",
                }
            )
        self.assertIn("Access denied", result["error"])
        m.assert_not_called()

    def test_directory_outside_allowed_roots_is_rejected(self):
        with patch("builtins.open", new_callable=mock_open, read_data="secret\n") as m:
            result = read_log.invoke(
                {
                    "log_file_directory": "/etc",
                    "log_filename": "passwd",
                }
            )
        self.assertIn("Access denied", result["error"])
        m.assert_not_called()

    def test_symlink_escaping_allowed_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as log_dir, tempfile.TemporaryDirectory() as other_dir:
            log_dir = os.path.realpath(log_dir)
            secret = os.path.join(other_dir, "secret.txt")
            with open(secret, "w") as f:
                f.write("secret\n")
            os.symlink(secret, os.path.join(log_dir, "link.log"))
            self.mock_allowed_dirs.return_value = [log_dir]

            result = read_log.invoke(
                {"log_file_directory": log_dir, "log_filename": "link.log"}
            )
        self.assertIn("Access denied", result["error"])

    def test_real_file_in_allowed_directory_is_read(self):
        with tempfile.TemporaryDirectory() as log_dir:
            log_dir = os.path.realpath(log_dir)
            with open(os.path.join(log_dir, "node.log"), "w") as f:
                f.write("INFO: hello\n")
            self.mock_allowed_dirs.return_value = [log_dir]

            result = read_log.invoke(
                {"log_file_directory": log_dir, "log_filename": "node.log"}
            )
        self.assertEqual(result["lines"], ["line 1: INFO: hello"])


if __name__ == "__main__":
    unittest.main()
