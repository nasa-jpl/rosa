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

import os
import sys
import unittest
import unittest.mock
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from pose_logger import (  # noqa: E402
    POSE_LOG_INTERVAL_SEC,
    PoseLogConsumer,
    build_log_dir,
    build_location_log_path,
    pose_to_record,
    resolve_log_root,
)


class TestBuildLogDir(unittest.TestCase):
    def test_layout(self):
        root = Path("/tmp/logs-root")
        p = build_log_dir(root, "2026-04-23", "sess-1", "turtle1")
        self.assertEqual(
            p,
            Path("/tmp/logs-root/2026-04-23/sess-1/turtle1"),
        )

    def test_location_log_path(self):
        root = Path("/tmp/logs-root")
        p = build_location_log_path(root, "2026-04-23", "sess-1", "turtle1")
        self.assertEqual(
            p,
            Path("/tmp/logs-root/2026-04-23/sess-1/turtle1/location.jsonl"),
        )


class TestPoseToRecord(unittest.TestCase):
    def test_with_dict_stamp(self):
        pose = SimpleNamespace(
            x=1.0,
            y=2.0,
            theta=0.5,
            linear_velocity=0.1,
            angular_velocity=0.2,
        )
        rec = pose_to_record(pose, {"secs": 10, "nsecs": 20})
        self.assertEqual(
            rec,
            {
                "t_ros": {"secs": 10, "nsecs": 20},
                "x": 1.0,
                "y": 2.0,
                "theta": 0.5,
                "linear_velocity": 0.1,
                "angular_velocity": 0.2,
            },
        )

    def test_with_object_stamp(self):
        pose = SimpleNamespace(
            x=0.0,
            y=0.0,
            theta=0.0,
            linear_velocity=0.0,
            angular_velocity=0.0,
        )
        stamp = SimpleNamespace(secs=3, nsecs=400000000)
        rec = pose_to_record(pose, stamp)
        self.assertEqual(rec["t_ros"], {"secs": 3, "nsecs": 400000000})


class TestResolveLogRoot(unittest.TestCase):
    def tearDown(self):
        os.environ.pop("TURTLE_AGENT_LOG_ROOT", None)

    def test_env_override(self):
        with unittest.mock.patch.dict(
            os.environ, {"TURTLE_AGENT_LOG_ROOT": "/tmp/custom-logs"}
        ):
            self.assertEqual(resolve_log_root(), Path("/tmp/custom-logs").resolve())


class TestPoseLogIntervalConstant(unittest.TestCase):
    def test_default_one_second(self):
        self.assertEqual(POSE_LOG_INTERVAL_SEC, 1.0)


class TestPoseLogConsumer(unittest.TestCase):
    def test_writes_one_file_per_turtle_with_sampling(self):
        import json
        import tempfile

        pose = SimpleNamespace(
            x=1.0,
            y=2.0,
            theta=0.5,
            linear_velocity=0.1,
            angular_velocity=0.2,
        )
        with tempfile.TemporaryDirectory() as tmp:
            consumer = PoseLogConsumer(
                period=1.0,
                log_root=Path(tmp),
                date_str="2026-04-23",
                session_id="sess-1",
            )
            try:
                consumer.on_pose("turtle1", pose, {"secs": 10, "nsecs": 0})
                consumer.on_pose("turtle1", pose, {"secs": 10, "nsecs": 500000000})
                consumer.on_pose("turtle1", pose, {"secs": 11, "nsecs": 0})
            finally:
                consumer.close()

            path = Path(tmp) / "2026-04-23" / "sess-1" / "turtle1" / "location.jsonl"
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["x"], 1.0)


if __name__ == "__main__":
    unittest.main()
