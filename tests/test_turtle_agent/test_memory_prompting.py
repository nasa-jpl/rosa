import sys
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from memory_prompting import (  # noqa: E402
    build_memory_context,
    infer_query_context,
    load_long_term_records,
)


class TestMemoryPrompting(unittest.TestCase):
    def test_infer_query_context_korean_from_to(self):
        ctx = infer_query_context("A에서 B로 가")
        self.assertEqual(ctx["task_family"], "navigate")
        self.assertEqual(ctx["slots"]["from"], "A")
        self.assertEqual(ctx["slots"]["to"], "B")
        self.assertEqual(ctx["experience_key"], "navigate|from:A|to:B")

    def test_build_memory_context_prefers_exact_key(self):
        records = [
            {
                "turtle_id": "turtle1",
                "payload": {
                    "operation": {
                        "nl_goal": {"text": "A to B with detour"},
                        "intent_norm": {
                            "task_family": "navigate",
                            "slots": {"from": "A", "to": "B"},
                        },
                    },
                    "evidence": {"collision_enter_count": 2, "success_rate": 1.0},
                },
            },
            {
                "turtle_id": "turtle1",
                "payload": {
                    "operation": {
                        "nl_goal": {"text": "B to A direct"},
                        "intent_norm": {
                            "task_family": "navigate",
                            "slots": {"from": "B", "to": "A"},
                        },
                    },
                    "evidence": {"collision_enter_count": 0, "success_rate": 1.0},
                },
            },
        ]
        context, hits = build_memory_context("A에서 B로 가", records, top_k=1)
        self.assertEqual(hits, 1)
        self.assertIn("A to B with detour", context)
        self.assertNotIn("B to A direct", context)

    def test_load_long_term_records_filters_by_turtle(self):
        with tempfile.TemporaryDirectory() as td:
            memory_root = Path(td)
            long_dir = memory_root / "long_term"
            long_dir.mkdir(parents=True, exist_ok=True)
            path = long_dir / "long_sessionid_s1.jsonl"
            path.write_text(
                '{"turtle_id":"turtle1","payload":{"operation":{"intent_norm":{"task_family":"navigate","slots":{"from":"A","to":"B"}}}}}\n'
                '{"turtle_id":"turtle2","payload":{"operation":{"intent_norm":{"task_family":"navigate","slots":{"from":"A","to":"B"}}}}}\n',
                encoding="utf-8",
            )
            rows = load_long_term_records(memory_root, "turtle1")
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["turtle_id"], "turtle1")


if __name__ == "__main__":
    unittest.main()
