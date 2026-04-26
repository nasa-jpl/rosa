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

"""Shared turtlesim pose subscriptions with fan-out to in-process consumers."""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

POSE_TOPIC_TYPE = "turtlesim/Pose"

PoseConsumer = Callable[[str, Any, Any], None]
SubscriberFactory = Callable[[str, Callable[[Any], None]], Any]
TopicProvider = Callable[[], Iterable[Tuple[str, str]]]
TimeSource = Callable[[], Any]


def normalize_turtle_name(name: str) -> str:
    """Return a ROS namespace-friendly turtle name without slashes."""
    return str(name).strip().replace("/", "")


def extract_turtle_name_from_pose_topic(topic: str, topic_type: str) -> Optional[str]:
    """Extract ``name`` from a top-level ``/name/pose`` turtlesim Pose topic."""
    if topic_type not in (POSE_TOPIC_TYPE, "turtlesim/msg/Pose"):
        return None
    parts = str(topic).strip().split("/")
    if len(parts) != 3 or parts[0] != "" or parts[2] != "pose":
        return None
    name = normalize_turtle_name(parts[1])
    if not name:
        return None
    return name


class PoseHub:
    """Manage one ``/{name}/pose`` subscriber per turtle and notify consumers."""

    def __init__(
        self,
        *,
        subscriber_factory: Optional[SubscriberFactory] = None,
        topic_provider: Optional[TopicProvider] = None,
        time_source: Optional[TimeSource] = None,
    ) -> None:
        self._lock = threading.RLock()
        self._subscribers: Dict[str, Any] = {}
        self._latest_poses: Dict[str, Tuple[Any, Any]] = {}
        self._consumers: List[PoseConsumer] = []
        self._subscriber_factory = subscriber_factory
        self._topic_provider = topic_provider
        self._time_source = time_source
        self._stopped = False

    def register_consumer(self, consumer: PoseConsumer) -> None:
        """Register a callback receiving ``(turtle_name, pose, stamp)``."""
        with self._lock:
            if consumer not in self._consumers:
                self._consumers.append(consumer)

    def unregister_consumer(self, consumer: PoseConsumer) -> None:
        with self._lock:
            self._consumers = [c for c in self._consumers if c != consumer]

    def start_from_ros_graph_once(self) -> Tuple[str, ...]:
        """Scan current ROS topics once and register all turtlesim pose topics."""
        registered = []
        for topic, topic_type in self._get_published_topics():
            name = extract_turtle_name_from_pose_topic(topic, topic_type)
            if name and self.register_turtle(name):
                registered.append(name)
        return tuple(registered)

    def register_turtle(self, name: str) -> bool:
        """Subscribe to ``/{name}/pose`` if not already registered."""
        turtle_name = normalize_turtle_name(name)
        if not turtle_name:
            raise ValueError("turtle name must be non-empty")

        with self._lock:
            if self._stopped:
                raise RuntimeError("PoseHub is stopped")
            if turtle_name in self._subscribers:
                return False

            topic = f"/{turtle_name}/pose"
            self._subscribers[turtle_name] = self._make_subscriber(
                topic, self._make_pose_callback(turtle_name)
            )
            return True

    def unregister_turtle(self, name: str) -> bool:
        """Unsubscribe and forget latest pose for ``name``."""
        turtle_name = normalize_turtle_name(name)
        with self._lock:
            sub = self._subscribers.pop(turtle_name, None)
            self._latest_poses.pop(turtle_name, None)
        if sub is None:
            return False
        unregister = getattr(sub, "unregister", None)
        if callable(unregister):
            unregister()
        return True

    def snapshot(self) -> Dict[str, Tuple[Any, Any]]:
        """Return ``{name: (pose, stamp)}`` for the latest known poses."""
        with self._lock:
            return dict(self._latest_poses)

    def stop(self) -> None:
        with self._lock:
            self._stopped = True
            subscribers = tuple(self._subscribers.values())
            self._subscribers.clear()
            self._latest_poses.clear()
            self._consumers.clear()
        for sub in subscribers:
            unregister = getattr(sub, "unregister", None)
            if callable(unregister):
                unregister()

    def on_turtle_spawned(self, name: str) -> None:
        self.register_turtle(name)

    def on_turtle_killed(self, name: str) -> None:
        self.unregister_turtle(name)

    def _make_pose_callback(self, turtle_name: str) -> Callable[[Any], None]:
        def _on_pose(msg: Any) -> None:
            stamp = self._now()
            with self._lock:
                if self._stopped or turtle_name not in self._subscribers:
                    return
                self._latest_poses[turtle_name] = (msg, stamp)
                consumers = tuple(self._consumers)
            for consumer in consumers:
                consumer(turtle_name, msg, stamp)

        return _on_pose

    def _make_subscriber(self, topic: str, callback: Callable[[Any], None]) -> Any:
        if self._subscriber_factory is not None:
            return self._subscriber_factory(topic, callback)
        import rospy
        from turtlesim.msg import Pose

        return rospy.Subscriber(topic, Pose, callback, queue_size=1)

    def _get_published_topics(self) -> Iterable[Tuple[str, str]]:
        if self._topic_provider is not None:
            return self._topic_provider()
        import rospy

        return rospy.get_published_topics()

    def _now(self) -> Any:
        if self._time_source is not None:
            return self._time_source()
        import rospy

        return rospy.Time.now()
