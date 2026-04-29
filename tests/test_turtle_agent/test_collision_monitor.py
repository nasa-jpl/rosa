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
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from collision_monitor import (  # noqa: E402
    CollisionEvent,
    CollisionMonitor,
    collision_event_to_record,
)
from obstacle_store import (  # noqa: E402
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleStore,
    SegmentsGeometry,
)


def pose(x: float, y: float):
    return SimpleNamespace(x=x, y=y, theta=0.0)


class TestCollisionEventToRecord(unittest.TestCase):
    def test_includes_event_fields_and_pose(self) -> None:
        p = SimpleNamespace(
            x=1.0,
            y=2.0,
            theta=0.5,
            linear_velocity=0.0,
            angular_velocity=0.0,
        )
        ev = CollisionEvent(
            event_type="enter",
            collision_type="turtle_obstacle",
            turtles=("turtle1",),
            stamp=SimpleNamespace(secs=3, nsecs=4),
            pose=p,
            obstacle_id="wet",
            details={"geometry": "AabbGeometry"},
        )
        rec = collision_event_to_record(ev)
        self.assertEqual(rec["event_type"], "enter")
        self.assertEqual(rec["collision_type"], "turtle_obstacle")
        self.assertEqual(rec["turtles"], ["turtle1"])
        self.assertEqual(rec["obstacle_id"], "wet")
        self.assertEqual(
            rec["t_ros"],
            {"secs": 3, "nsecs": 4},
        )
        self.assertEqual(rec["x"], 1.0)
        self.assertEqual(rec["y"], 2.0)


class TestCollisionMonitor(unittest.TestCase):
    def test_circle_obstacle_enter_stay_exit(self):
        store = ObstacleStore()
        store.upsert(Obstacle("circle", "static", CircleGeometry(0.0, 0.0, 1.0), None))
        monitor = CollisionMonitor(store, turtle_radius=0.5)

        monitor.on_pose("turtle1", pose(1.0, 0.0), "t0")
        self.assertEqual(monitor.events[-1].event_type, "enter")
        self.assertEqual(monitor.events[-1].collision_type, "turtle_obstacle")
        self.assertEqual(monitor.events[-1].obstacle_id, "circle")

        monitor.clear_events()
        monitor.on_pose("turtle1", pose(1.0, 0.0), "t1")
        self.assertEqual([e.event_type for e in monitor.events], ["stay"])

        monitor.clear_events()
        monitor.on_pose("turtle1", pose(5.0, 0.0), "t2")
        self.assertEqual([e.event_type for e in monitor.events], ["exit"])

    def test_aabb_obstacle_collision(self):
        store = ObstacleStore()
        store.upsert(Obstacle("box", "static", AabbGeometry(2.0, 2.0, 4.0, 4.0), None))
        monitor = CollisionMonitor(store, turtle_radius=0.5)

        monitor.on_pose("turtle1", pose(1.6, 3.0), "t0")

        self.assertEqual(len(monitor.events), 1)
        self.assertEqual(monitor.events[0].obstacle_id, "box")
        self.assertEqual(monitor.events[0].event_type, "enter")

    def test_segments_obstacle_collision(self):
        store = ObstacleStore()
        store.upsert(
            Obstacle(
                "wall",
                "static",
                SegmentsGeometry((((0.0, 0.0), (2.0, 0.0)),)),
                None,
            )
        )
        monitor = CollisionMonitor(store, turtle_radius=0.25)

        monitor.on_pose("turtle1", pose(1.0, 0.2), "t0")

        self.assertEqual(len(monitor.events), 1)
        self.assertEqual(monitor.events[0].obstacle_id, "wall")

    def test_turtle_turtle_enter_and_exit(self):
        store = ObstacleStore()
        monitor = CollisionMonitor(store, turtle_radius=0.5)

        monitor.on_pose("turtle1", pose(0.0, 0.0), "t0")
        self.assertEqual(monitor.events, ())
        monitor.on_pose("turtle2", pose(0.75, 0.0), "t1")
        self.assertEqual(monitor.events[-1].event_type, "enter")
        self.assertEqual(monitor.events[-1].collision_type, "turtle_turtle")
        self.assertEqual(monitor.events[-1].turtles, ("turtle1", "turtle2"))

        monitor.clear_events()
        monitor.on_pose("turtle2", pose(2.0, 0.0), "t2")
        self.assertEqual([e.event_type for e in monitor.events], ["exit"])

    def test_emit_stay_false_suppresses_repeat_events(self):
        store = ObstacleStore()
        store.upsert(Obstacle("c", "static", CircleGeometry(0.0, 0.0, 1.0), None))
        monitor = CollisionMonitor(store, turtle_radius=0.5, emit_stay=False)

        monitor.on_pose("turtle1", pose(0.0, 0.0), "t0")
        monitor.clear_events()
        monitor.on_pose("turtle1", pose(0.0, 0.0), "t1")

        self.assertEqual(monitor.events, ())

    def test_expired_obstacle_snapshot_generates_exit(self):
        store = ObstacleStore()
        store.upsert(
            Obstacle(
                "temp",
                "temporary",
                CircleGeometry(0.0, 0.0, 1.0),
                time.monotonic() + 0.05,
            )
        )
        monitor = CollisionMonitor(store, turtle_radius=0.5)
        monitor.on_pose("turtle1", pose(0.0, 0.0), "t0")
        monitor.clear_events()

        time.sleep(0.1)
        monitor.on_pose("turtle1", pose(0.0, 0.0), "t1")

        self.assertEqual([e.event_type for e in monitor.events], ["exit"])

    def test_turtle_kind_obstacle_is_skipped_for_obstacle_layer(self):
        store = ObstacleStore()
        store.upsert(Obstacle("t2", "turtle", CircleGeometry(0.0, 0.0, 10.0), None))
        monitor = CollisionMonitor(store, turtle_radius=0.5)

        monitor.on_pose("turtle1", pose(0.0, 0.0), "t0")

        self.assertEqual(monitor.events, ())


if __name__ == "__main__":
    unittest.main()
