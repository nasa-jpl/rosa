import unittest
from unittest.mock import MagicMock
from rosa import ROSA

class TestROSACommands(unittest.TestCase):
    def test_list_topics(self):
        llm = MagicMock()
        rosa = ROSA(ros_version=1, llm=llm)
        llm.query.return_value = "topics: [topic1, topic2]"
        result = rosa.invoke("Show me a list of topics that have publishers but no subscribers")
        self.assertIn("topics", result)
