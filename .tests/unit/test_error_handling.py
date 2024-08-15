import unittest
from unittest.mock import MagicMock
from rosa import ROSA

class TestROSAErrorHandling(unittest.TestCase):
    def test_invalid_command(self):
        llm = MagicMock()
        rosa = ROSA(ros_version=1, llm=llm)
        with self.assertRaises(Exception):  # Replace 'Exception' with a specific exception if possible
            rosa.invoke("This command does not exist")
