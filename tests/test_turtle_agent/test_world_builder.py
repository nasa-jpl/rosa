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
import types
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from obstacle_store import (  # noqa: E402
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleStore,
    SegmentsGeometry,
)
from world_builder import (  # noqa: E402
    RenderSegment,
    draw_static_world,
    render_segments_from_obstacles,
)


class TestWorldBuilderRenderSegments(unittest.TestCase):
    def test_segments_geometry_passes_through(self):
        obstacles = [
            Obstacle(
                "wall",
                "static",
                SegmentsGeometry((((0.0, 0.0), (1.0, 0.0)),)),
                None,
            )
        ]
        self.assertEqual(
            render_segments_from_obstacles(obstacles),
            (RenderSegment(0.0, 0.0, 1.0, 0.0),),
        )

    def test_aabb_becomes_four_edges(self):
        obstacles = [Obstacle("box", "static", AabbGeometry(1.0, 2.0, 3.0, 4.0), None)]
        self.assertEqual(
            render_segments_from_obstacles(obstacles),
            (
                RenderSegment(1.0, 2.0, 3.0, 2.0),
                RenderSegment(3.0, 2.0, 3.0, 4.0),
                RenderSegment(3.0, 4.0, 1.0, 4.0),
                RenderSegment(1.0, 4.0, 1.0, 2.0),
            ),
        )

    def test_circle_becomes_closed_polygon(self):
        obstacles = [Obstacle("circle", "static", CircleGeometry(0.0, 0.0, 1.0), None)]
        segments = render_segments_from_obstacles(obstacles, circle_segments=4)
        self.assertEqual(len(segments), 4)
        self.assertAlmostEqual(segments[0].x1, 1.0)
        self.assertAlmostEqual(segments[-1].x2, 1.0)
        self.assertAlmostEqual(segments[-1].y2, 0.0)


class TestWorldBuilderRosCalls(unittest.TestCase):
    def test_draw_static_world_uses_builder_turtle_and_cleanup(self):
        calls = []

        class FakeRospy(types.ModuleType):
            def wait_for_service(self, name, timeout=5.0):
                calls.append(("wait", name, timeout))

            def ServiceProxy(self, name, service_type):
                def proxy(*args, **kwargs):
                    calls.append(("call", name, args, kwargs))

                return proxy

        turtlesim = types.ModuleType("turtlesim")
        turtlesim_srv = types.ModuleType("turtlesim.srv")
        turtlesim_srv.Kill = object
        turtlesim_srv.SetPen = object
        turtlesim_srv.Spawn = object
        turtlesim_srv.TeleportAbsolute = object
        turtlesim.srv = turtlesim_srv

        old_rospy = sys.modules.get("rospy")
        old_turtlesim = sys.modules.get("turtlesim")
        old_turtlesim_srv = sys.modules.get("turtlesim.srv")
        sys.modules["rospy"] = FakeRospy("rospy")
        sys.modules["turtlesim"] = turtlesim
        sys.modules["turtlesim.srv"] = turtlesim_srv
        try:
            store = ObstacleStore()
            store.upsert(
                Obstacle(
                    "wall",
                    "static",
                    SegmentsGeometry((((0.0, 0.0), (1.0, 0.0)),)),
                    None,
                )
            )
            count = draw_static_world(store, turtle_name="builder", service_timeout=1.0)
        finally:
            if old_rospy is None:
                sys.modules.pop("rospy", None)
            else:
                sys.modules["rospy"] = old_rospy
            if old_turtlesim is None:
                sys.modules.pop("turtlesim", None)
            else:
                sys.modules["turtlesim"] = old_turtlesim
            if old_turtlesim_srv is None:
                sys.modules.pop("turtlesim.srv", None)
            else:
                sys.modules["turtlesim.srv"] = old_turtlesim_srv

        self.assertEqual(count, 1)
        self.assertIn(
            (
                "call",
                "/spawn",
                (),
                {"x": 0.0, "y": 0.0, "theta": 0.0, "name": "builder"},
            ),
            calls,
        )
        self.assertIn(
            (
                "call",
                "/builder/set_pen",
                (),
                {"r": 0, "g": 0, "b": 0, "width": 2, "off": 1},
            ),
            calls,
        )
        self.assertIn(
            (
                "call",
                "/builder/teleport_absolute",
                (),
                {"x": 1.0, "y": 0.0, "theta": 0.0},
            ),
            calls,
        )
        kill_calls = [c for c in calls if c[0] == "call" and c[1] == "/kill"]
        self.assertEqual(len(kill_calls), 2)


if __name__ == "__main__":
    unittest.main()
