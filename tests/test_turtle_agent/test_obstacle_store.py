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

import random
import sys
import threading
import time
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from collision_geometry import (  # noqa: E402
    any_segment_intersects_disc,
    circle_intersects_aabb,
    circles_overlap,
)
from obstacle_store import (  # noqa: E402
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleStore,
    SegmentsGeometry,
)


class TestObstacleStoreSingleThread(unittest.TestCase):
    def test_upsert_snapshot_remove(self):
        store = ObstacleStore()
        o = Obstacle(
            id="a",
            kind="static",
            geometry=CircleGeometry(1.0, 2.0, 0.5),
            expires_at=None,
        )
        store.upsert(o)
        snap = store.snapshot()
        self.assertEqual(len(snap), 1)
        self.assertEqual(snap[0].id, "a")
        self.assertIsInstance(snap[0].geometry, CircleGeometry)
        self.assertTrue(store.remove("a"))
        self.assertFalse(store.remove("a"))
        self.assertEqual(len(store.snapshot()), 0)

    def test_clear(self):
        store = ObstacleStore()
        store.upsert(Obstacle("x", "static", CircleGeometry(0.0, 0.0, 1.0), None))
        store.clear()
        self.assertEqual(len(store), 0)

    def test_temporary_expires_on_snapshot(self):
        store = ObstacleStore()
        deadline = time.monotonic() + 0.05
        store.upsert(
            Obstacle(
                "e",
                "temporary",
                CircleGeometry(0.0, 0.0, 1.0),
                deadline,
            )
        )
        time.sleep(0.1)
        self.assertEqual(len(store.snapshot()), 0)

    def test_temporary_expires_on_get(self):
        store = ObstacleStore()
        deadline = time.monotonic() + 0.05
        store.upsert(
            Obstacle("e", "temporary", CircleGeometry(0.0, 0.0, 1.0), deadline)
        )
        time.sleep(0.1)
        self.assertIsNone(store.get("e"))

    def test_purge_expired(self):
        store = ObstacleStore()
        store.upsert(
            Obstacle(
                "e",
                "temporary",
                CircleGeometry(0.0, 0.0, 1.0),
                time.monotonic() - 1.0,
            )
        )
        store.purge_expired()
        self.assertEqual(len(store), 0)

    def test_validation_temporary_requires_deadline(self):
        with self.assertRaises(ValueError):
            ObstacleStore().upsert(
                Obstacle("bad", "temporary", CircleGeometry(0.0, 0.0, 1.0), None)
            )

    def test_validation_static_may_not_have_deadline(self):
        with self.assertRaises(ValueError):
            ObstacleStore().upsert(
                Obstacle(
                    "bad",
                    "static",
                    CircleGeometry(0.0, 0.0, 1.0),
                    time.monotonic() + 10.0,
                )
            )

    def test_snapshot_segments_detached_from_store_updates(self):
        store = ObstacleStore()
        store.upsert(
            Obstacle(
                "s",
                "static",
                SegmentsGeometry((((0.0, 0.0), (1.0, 0.0)),)),
                None,
            )
        )
        snap0 = store.snapshot()
        segs0 = snap0[0].geometry.segments
        store.upsert(
            Obstacle(
                "s",
                "static",
                SegmentsGeometry((((0.0, 0.0), (2.0, 0.0)),)),
                None,
            )
        )
        self.assertEqual(len(segs0), 1)
        self.assertEqual(segs0[0], ((0.0, 0.0), (1.0, 0.0)))

    def test_collision_geometry_circle(self):
        store = ObstacleStore()
        store.upsert(Obstacle("c", "static", CircleGeometry(0.0, 0.0, 1.0), None))
        ob = store.snapshot()[0]
        g = ob.geometry
        self.assertTrue(circles_overlap(0.5, 0.0, 0.5, g.cx, g.cy, g.r, eps=1e-9))

    def test_collision_geometry_segments(self):
        store = ObstacleStore()
        store.upsert(
            Obstacle(
                "w",
                "static",
                SegmentsGeometry((((-1.0, 0.0), (1.0, 0.0)),)),
                None,
            )
        )
        ob = store.snapshot()[0]
        self.assertTrue(
            any_segment_intersects_disc(ob.geometry.segments, 0.0, 0.25, 0.3)
        )

    def test_collision_geometry_aabb(self):
        store = ObstacleStore()
        store.upsert(
            Obstacle(
                "b",
                "static",
                AabbGeometry(2.0, 2.0, 4.0, 4.0),
                None,
            )
        )
        ob = store.snapshot()[0]
        b = ob.geometry
        self.assertTrue(
            circle_intersects_aabb(3.0, 3.0, 0.5, b.min_x, b.min_y, b.max_x, b.max_y)
        )


class TestObstacleStoreConcurrent(unittest.TestCase):
    def test_concurrent_upsert_remove_snapshot(self):
        store = ObstacleStore()
        rng = random.Random(0)
        errors: list[BaseException] = []
        barrier = threading.Barrier(4)

        def worker(tid: int) -> None:
            try:
                barrier.wait()
                for _ in range(200):
                    op = rng.randint(0, 2)
                    oid = f"{tid}_{rng.randint(0, 9)}"
                    if op == 0:
                        store.upsert(
                            Obstacle(
                                oid,
                                "static",
                                CircleGeometry(rng.random(), rng.random(), 0.1),
                                None,
                            )
                        )
                    elif op == 1:
                        store.remove(oid)
                    else:
                        snap = store.snapshot()
                        self.assertIsInstance(snap, tuple)
                        for ob in snap:
                            _ = ob.id
            except BaseException as e:  # pragma: no cover - surfaced in test
                errors.append(e)

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(errors, [])
        self.assertEqual(len(store.snapshot()), len(store))


if __name__ == "__main__":
    unittest.main()
