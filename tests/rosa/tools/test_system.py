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

import time
import unittest

from langchain.globals import get_debug, get_verbose, set_debug

from src.rosa.tools.system import set_verbosity, set_debugging, wait


class TestSystemTools(unittest.TestCase):

    def test_sets_verbosity_to_true(self):
        result = set_verbosity.invoke({"enable_verbose_messages": True})
        self.assertEqual(result, "Verbose messages are now enabled.")
        self.assertTrue(get_verbose())
        result = set_verbosity.invoke({"enable_verbose_messages": False})
        self.assertEqual(result, "Verbose messages are now disabled.")
        self.assertFalse(get_verbose())

    def test_sets_debug_to_true(self):
        result = set_debugging.invoke({"enable_debug_messages": True})
        self.assertEqual(result, "Debug messages are now enabled.")
        self.assertTrue(get_debug())
        set_debug(False)
        result = set_debugging.invoke({"enable_debug_messages": False})
        self.assertEqual(result, "Debug messages are now disabled.")
        self.assertFalse(get_debug())

    def test_waits_for_specified_seconds(self):
        start = time.time()
        result = wait.invoke({"seconds": 1.0})
        end = time.time()

        self.assertTrue(result.startswith("Waited exactly"))
        self.assertAlmostEqual(end - start, 1.0, places=1)

    def test_waits_for_zero_seconds(self):
        start = time.time()
        result = wait.invoke({"seconds": 0})
        end = time.time()

        self.assertTrue(result.startswith("Waited exactly"))
        self.assertAlmostEqual(end - start, 0.0, places=1)


if __name__ == "__main__":
    unittest.main()
