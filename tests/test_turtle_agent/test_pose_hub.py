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

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from pose_hub import PoseHub, extract_turtle_name_from_pose_topic  # noqa: E402


class FakeSubscriber:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class TestExtractTurtleName(unittest.TestCase):
    def test_accepts_top_level_turtlesim_pose_topic(self):
        self.assertEqual(
            extract_turtle_name_from_pose_topic("/turtle1/pose", "turtlesim/Pose"),
            "turtle1",
        )

    def test_rejects_non_pose_topics(self):
        self.assertIsNone(
            extract_turtle_name_from_pose_topic(
                "/turtle1/cmd_vel", "geometry_msgs/Twist"
            )
        )
        self.assertIsNone(
            extract_turtle_name_from_pose_topic("/ns/turtle1/pose", "turtlesim/Pose")
        )


class TestPoseHub(unittest.TestCase):
    def setUp(self):
        self.subscribers = {}

        def subscriber_factory(topic, callback):
            sub = FakeSubscriber(topic, callback)
            self.subscribers[topic] = sub
            return sub

        self.hub = PoseHub(
            subscriber_factory=subscriber_factory,
            time_source=lambda: {"secs": 1, "nsecs": 2},
        )

    def test_register_turtle_is_idempotent(self):
        self.assertTrue(self.hub.register_turtle("turtle1"))
        self.assertFalse(self.hub.register_turtle("turtle1"))
        self.assertEqual(list(self.subscribers), ["/turtle1/pose"])

    def test_start_from_ros_graph_once_filters_topics(self):
        hub = PoseHub(
            subscriber_factory=lambda topic, callback: FakeSubscriber(topic, callback),
            topic_provider=lambda: [
                ("/turtle1/pose", "turtlesim/Pose"),
                ("/turtle2/pose", "turtlesim/msg/Pose"),
                ("/turtle1/cmd_vel", "geometry_msgs/Twist"),
                ("/ns/turtle3/pose", "turtlesim/Pose"),
            ],
            time_source=lambda: {"secs": 1, "nsecs": 0},
        )
        self.assertEqual(hub.start_from_ros_graph_once(), ("turtle1", "turtle2"))

    def test_callback_updates_snapshot_and_consumers(self):
        events = []
        self.hub.register_consumer(
            lambda name, pose, stamp: events.append((name, pose, stamp))
        )
        self.hub.register_turtle("turtle1")

        pose = SimpleNamespace(x=1.0)
        self.subscribers["/turtle1/pose"].callback(pose)

        snapshot = self.hub.snapshot()
        self.assertIs(snapshot["turtle1"][0], pose)
        self.assertEqual(snapshot["turtle1"][1], {"secs": 1, "nsecs": 2})
        self.assertEqual(events, [("turtle1", pose, {"secs": 1, "nsecs": 2})])

    def test_unregister_cleans_up(self):
        self.hub.register_turtle("turtle1")
        sub = self.subscribers["/turtle1/pose"]
        self.assertTrue(self.hub.unregister_turtle("turtle1"))
        self.assertTrue(sub.unregistered)
        self.assertEqual(self.hub.snapshot(), {})


if __name__ == "__main__":
    unittest.main()
