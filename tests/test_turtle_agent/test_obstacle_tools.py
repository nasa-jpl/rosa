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

import json
import sys
import time
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from obstacle_store import AabbGeometry, ObstacleStore  # noqa: E402
from tools import obstacle as obstacle_tools  # noqa: E402


class TestObstacleTools(unittest.TestCase):
    def setUp(self):
        self.store = ObstacleStore()
        obstacle_tools.configure_obstacle_store(self.store)

    def test_add_list_remove_static_obstacle(self):
        geometry_json = json.dumps(
            {"type": "aabb", "min_x": 1, "min_y": 2, "max_x": 3, "max_y": 4}
        )
        out = obstacle_tools.add_obstacle.invoke(
            {
                "obstacle_id": "pond",
                "geometry_json": geometry_json,
                "kind": "static",
            }
        )
        self.assertIn("Added static obstacle", out)

        obstacle = self.store.get("pond")
        self.assertIsNotNone(obstacle)
        assert obstacle is not None
        self.assertEqual(obstacle.kind, "static")
        self.assertIsInstance(obstacle.geometry, AabbGeometry)

        listed = json.loads(obstacle_tools.list_obstacles.invoke({}))
        self.assertEqual(listed["obstacles"][0]["id"], "pond")

        removed = obstacle_tools.remove_obstacle.invoke({"obstacle_id": "pond"})
        self.assertIn("Removed obstacle", removed)
        self.assertIsNone(self.store.get("pond"))

    def test_add_rejects_invalid_geometry(self):
        out = obstacle_tools.add_obstacle.invoke(
            {
                "obstacle_id": "bad",
                "geometry_json": json.dumps({"type": "unknown"}),
                "kind": "static",
            }
        )
        self.assertIn("Failed to add obstacle", out)
        self.assertEqual(len(self.store), 0)

    def test_add_temporary_rejects_non_positive_ttl(self):
        out = obstacle_tools.add_obstacle.invoke(
            {
                "obstacle_id": "bad",
                "geometry_json": json.dumps(
                    {"type": "circle", "cx": 0, "cy": 0, "r": 1}
                ),
                "kind": "temporary",
                "ttl_seconds": 0,
            }
        )
        self.assertIn("ttl_seconds must be positive for temporary obstacles", out)
        self.assertEqual(len(self.store), 0)

    def test_temporary_expires_from_tool_listing(self):
        obstacle_tools.add_obstacle.invoke(
            {
                "obstacle_id": "short",
                "geometry_json": json.dumps(
                    {"type": "circle", "cx": 0, "cy": 0, "r": 1}
                ),
                "kind": "temporary",
                "ttl_seconds": 0.01,
            }
        )
        time.sleep(0.03)
        listed = json.loads(obstacle_tools.list_obstacles.invoke({}))
        self.assertEqual(listed["obstacles"], [])


if __name__ == "__main__":
    unittest.main()
