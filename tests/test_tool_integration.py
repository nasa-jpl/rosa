import unittest
from unittest.mock import MagicMock
from rosa import ROSA, tool

@tool
def mock_tool():
    return "Tool executed"

class TestROSAIntegration(unittest.TestCase):
    def test_custom_tool_integration(self):
        llm = MagicMock()
        rosa = ROSA(ros_version=1, llm=llm, tools=[mock_tool])
        result = rosa.invoke("Run the mock tool")
        self.assertEqual(result, "Tool executed")
