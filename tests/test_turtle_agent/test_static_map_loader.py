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
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
_CONFIG = _REPO_ROOT / "src" / "turtle_agent" / "config"
sys.path.insert(0, str(_SCRIPTS))

from obstacle_store import (  # noqa: E402
    CircleGeometry,
    ObstacleStore,
    SegmentsGeometry,
)
from static_map_loader import (  # noqa: E402
    StaticMapLoadError,
    load_file,
    load_into_store,
)


class TestStaticMapLoader(unittest.TestCase):
    def test_load_into_store_circle_and_segments(self):
        store = ObstacleStore()
        data = {
            "obstacles": [
                {
                    "id": "p",
                    "geometry": {"type": "circle", "cx": 1.0, "cy": 2.0, "r": 0.5},
                },
                {
                    "id": "w",
                    "geometry": {
                        "type": "segments",
                        "segments": [[[0.0, 0.0], [1.0, 0.0]]],
                    },
                },
            ]
        }
        n = load_into_store(store, data, source="inline")
        self.assertEqual(n, 2)
        self.assertEqual(len(store), 2)
        snap = {o.id: o for o in store.snapshot()}
        self.assertIsInstance(snap["p"].geometry, CircleGeometry)
        self.assertEqual(
            (snap["p"].geometry.cx, snap["p"].geometry.cy, snap["p"].geometry.r),
            (1.0, 2.0, 0.5),
        )
        self.assertIsInstance(snap["w"].geometry, SegmentsGeometry)
        self.assertEqual(snap["w"].geometry.segments, (((0.0, 0.0), (1.0, 0.0)),))

    def test_on_duplicate_id_error_in_store(self):
        store = ObstacleStore()
        load_into_store(
            store,
            {
                "obstacles": [
                    {
                        "id": "a",
                        "geometry": {"type": "circle", "cx": 0, "cy": 0, "r": 1},
                    }
                ]
            },
        )
        with self.assertRaises(StaticMapLoadError) as ctx:
            load_into_store(
                store,
                {
                    "obstacles": [
                        {
                            "id": "a",
                            "geometry": {"type": "circle", "cx": 1, "cy": 1, "r": 1},
                        }
                    ]
                },
                on_duplicate_id="error",
            )
        self.assertIn("already exists", str(ctx.exception))

    def test_on_duplicate_id_error_same_document(self):
        store = ObstacleStore()
        with self.assertRaises(StaticMapLoadError) as ctx:
            load_into_store(
                store,
                {
                    "obstacles": [
                        {
                            "id": "a",
                            "geometry": {"type": "circle", "cx": 0, "cy": 0, "r": 1},
                        },
                        {
                            "id": "a",
                            "geometry": {"type": "circle", "cx": 1, "cy": 1, "r": 1},
                        },
                    ]
                },
                on_duplicate_id="error",
            )
        self.assertIn("duplicate id", str(ctx.exception))

    def test_replace_overwrites(self):
        store = ObstacleStore()
        load_into_store(
            store,
            {
                "obstacles": [
                    {
                        "id": "a",
                        "geometry": {"type": "circle", "cx": 0, "cy": 0, "r": 1},
                    }
                ]
            },
        )
        load_into_store(
            store,
            {
                "obstacles": [
                    {
                        "id": "a",
                        "geometry": {"type": "circle", "cx": 5, "cy": 5, "r": 2},
                    }
                ]
            },
            on_duplicate_id="replace",
        )
        o = store.get("a")
        assert o is not None
        self.assertEqual(o.geometry.cx, 5.0)

    def test_unknown_geometry_type_message(self):
        with self.assertRaises(StaticMapLoadError) as ctx:
            load_into_store(
                ObstacleStore(),
                {
                    "obstacles": [
                        {
                            "id": "x",
                            "geometry": {"type": "nope", "cx": 0, "cy": 0, "r": 1},
                        }
                    ]
                },
                source="testsrc",
            )
        msg = str(ctx.exception)
        self.assertIn("unknown geometry.type", msg)
        self.assertIn("testsrc", msg)
        self.assertIn("index=0", msg)

    def test_load_file_yaml_tmp(self):
        store = ObstacleStore()
        doc = {
            "obstacles": [
                {"id": "z", "geometry": {"type": "circle", "cx": 0, "cy": 0, "r": 0.25}}
            ]
        }
        with tempfile.NamedTemporaryFile(
            suffix=".yaml", mode="w", delete=False, encoding="utf-8"
        ) as f:
            import yaml as _yaml

            _yaml.safe_dump(doc, f)
            path = f.name
        try:
            n = load_file(store, path)
            self.assertEqual(n, 1)
            self.assertIsNotNone(store.get("z"))
        finally:
            Path(path).unlink(missing_ok=True)

    def test_load_file_json_tmp(self):
        store = ObstacleStore()
        doc = {
            "obstacles": [
                {"id": "j", "geometry": {"type": "circle", "cx": 3, "cy": 4, "r": 5}}
            ]
        }
        with tempfile.NamedTemporaryFile(
            suffix=".json", mode="w", delete=False, encoding="utf-8"
        ) as f:
            json.dump(doc, f)
            path = f.name
        try:
            load_file(store, path)
            o = store.get("j")
            assert o is not None
            self.assertEqual(o.geometry.r, 5.0)
        finally:
            Path(path).unlink(missing_ok=True)

    def test_temporary_kind_without_ttl_uses_default(self):
        store = ObstacleStore()
        load_into_store(
            store,
            {
                "obstacles": [
                    {
                        "id": "e",
                        "kind": "temporary",
                        "geometry": {"type": "circle", "cx": 0.0, "cy": 0.0, "r": 0.1},
                    }
                ]
            },
        )
        o = store.get("e")
        assert o is not None
        self.assertEqual(o.kind, "temporary")
        self.assertIsNotNone(o.expires_at)

    def test_temporary_explicit_ttl_seconds(self):
        import time as _time

        store = ObstacleStore()
        before = _time.monotonic()
        load_into_store(
            store,
            {
                "obstacles": [
                    {
                        "id": "e",
                        "kind": "temporary",
                        "ttl_seconds": 30.0,
                        "geometry": {"type": "circle", "cx": 0, "cy": 0, "r": 0.1},
                    }
                ]
            },
        )
        o = store.get("e")
        assert o is not None
        self.assertGreater(o.expires_at, before + 29.0)
        self.assertLess(o.expires_at, before + 31.0)

    def test_sample_config_repo_path(self):
        sample = _CONFIG / "static_obstacles_turtlesim.yaml"
        self.assertTrue(sample.is_file(), msg=f"missing {sample}")
        store = ObstacleStore()
        n = load_file(store, sample)
        self.assertEqual(n, 11)
        ids = {o.id for o in store.snapshot()}
        self.assertEqual(
            ids,
            {
                "wall-south",
                "wall-east",
                "wall-north",
                "wall-west",
                "wet-top",
                "wet-right",
                "wet-bottom",
                "a-point",
                "b-point",
                "c-point",
                "d-point",
            },
        )
        wet = store.get("wet-top")
        assert wet is not None
        self.assertEqual(wet.kind, "temporary")
        self.assertIsNotNone(wet.expires_at)


if __name__ == "__main__":
    unittest.main()
