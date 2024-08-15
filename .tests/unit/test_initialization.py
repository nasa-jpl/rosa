import unittest
from rosa import ROSA

class TestROSAInitialization(unittest.TestCase):
    def test_initialization_ros1(self):
        llm = get_mock_llm()  # Mock LLM
        rosa = ROSA(ros_version=1, llm=llm)
        self.assertIsNotNone(rosa)

    def test_initialization_ros2(self):
        llm = get_mock_llm()  # Mock LLM
        rosa = ROSA(ros_version=2, llm=llm)
        self.assertIsNotNone(rosa)
