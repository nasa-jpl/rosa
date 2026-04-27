import json
import sys
import unittest
import uuid
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from memory_converter import (  # noqa: E402
    MemoryConverter,
    build_long_term_path,
    build_short_term_path,
    parse_context_from_log_path,
)
from pose_logger import build_location_log_path  # noqa: E402
from command_logger import build_command_log_path  # noqa: E402


class TestMemoryConverter(unittest.TestCase):
    def test_parse_context_from_log_path(self):
        path = Path("/tmp/logs/2026-04-27/sess-1/turtle1/location.jsonl")
        session_id, turtle_id = parse_context_from_log_path(path)
        self.assertEqual(session_id, "sess-1")
        self.assertEqual(turtle_id, "turtle1")

    def test_convert_session_writes_short_and_long_term(self):
        log_root = _REPO_ROOT / "logs"
        memory_root = _SCRIPTS / "memory"
        date_str = "2026-04-27"
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        test_case_id = f"tc-{uuid.uuid4().hex[:8]}"

        location_path = build_location_log_path(log_root, date_str, session_id, turtle_id)
        command_path = build_command_log_path(log_root, date_str, session_id, turtle_id)
        location_path.parent.mkdir(parents=True, exist_ok=True)

        location_rows = [
            {
                "t_ros": {"secs": 100, "nsecs": 0},
                "x": 1.0,
                "y": 1.0,
                "theta": 0.0,
                "linear_velocity": 0.0,
                "angular_velocity": 0.0,
            },
            {
                "t_ros": {"secs": 101, "nsecs": 0},
                "x": 2.0,
                "y": 2.0,
                "theta": 0.5,
                "linear_velocity": 0.0,
                "angular_velocity": 0.0,
            },
        ]
        command_rows = [
            {
                "type": "intent",
                "t_ms": 100000,
                "task_family": "trace_shape",
                "slots": {"shape": "rectangle", "size": 2.0},
            },
            {
                "type": "skill",
                "t_ms": 100000,
                "skill": "move_forward",
                "args": {"distance": 2.0},
                "status": "success",
                "result": "ok",
            },
            {
                "type": "skill",
                "t_ms": 101000,
                "skill": "rotate",
                "args": {"degrees": 90},
                "status": "success",
                "result": "ok",
            },
        ]
        location_path.write_text(
            "\n".join(json.dumps(r, ensure_ascii=False) for r in location_rows) + "\n",
            encoding="utf-8",
        )
        command_path.write_text(
            "\n".join(json.dumps(r, ensure_ascii=False) for r in command_rows) + "\n",
            encoding="utf-8",
        )

        converter = MemoryConverter(memory_root=memory_root)
        long_path = build_long_term_path(memory_root, session_id)
        before_long_count = (
            len(long_path.read_text(encoding="utf-8").splitlines()) if long_path.exists() else 0
        )
        result = converter.convert_session(
            date_str=date_str,
            session_id=session_id,
            turtle_id=turtle_id,
            test_case_id=test_case_id,
            log_root=log_root,
        )

        self.assertEqual(result["short_term_written"], 2)
        self.assertEqual(result["long_term_written"], 1)

        short_path = build_short_term_path(memory_root, session_id, test_case_id)
        short_rows = [json.loads(line) for line in short_path.read_text(encoding="utf-8").splitlines()]
        long_rows = [json.loads(line) for line in long_path.read_text(encoding="utf-8").splitlines()]

        self.assertEqual(len(short_rows), 2)
        self.assertEqual(short_rows[0]["session_id"], session_id)
        self.assertEqual(short_rows[1]["plan"]["current_step_idx"], 2)
        self.assertEqual(len(long_rows), before_long_count + 1)
        self.assertEqual(long_rows[-1]["record_type"], "raw_episode")
        self.assertEqual(long_rows[-1]["session_id"], session_id)
        self.assertEqual(long_rows[-1]["turtle_id"], turtle_id)


if __name__ == "__main__":
    unittest.main()
