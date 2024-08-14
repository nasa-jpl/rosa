import unittest
from unittest.mock import MagicMock
from rosa import ROSA

class TestROSCommands(unittest.TestCase):
    def test_ros_command_execution(self):
        llm = MagicMock()
        rosa = ROSA(ros_version=2, llm=llm)
        llm.query.return_value = "rosnode list: [node1, node2]"
        result = rosa.invoke("List all running ROS nodes")
        self.assertIn("node1", result)
        self.assertIn("node2", result)
