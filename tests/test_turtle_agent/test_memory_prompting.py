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

    def test_infer_query_context_draw_line_to_coords_is_navigate(self):
        ctx = infer_query_context("draw a line to 10, 5 while avoiding obstacles")
        self.assertEqual(ctx["task_family"], "navigate")
        self.assertEqual(ctx["experience_key"], "navigate")

    def test_infer_query_context_leading_hangul_jamo_stripped_for_english_command(self):
        ctx = infer_query_context("ㅇ draw a line to 10,5 while avoiding obstacles")
        self.assertEqual(ctx["task_family"], "navigate")
        self.assertEqual(ctx["experience_key"], "navigate")

    def test_infer_query_context_move_back_to_coords_is_navigate(self):
        ctx = infer_query_context("move back to 1, 5 while avoiding obstacles")
        self.assertEqual(ctx["task_family"], "navigate")
        self.assertEqual(ctx["experience_key"], "navigate")

    def test_build_memory_context_draw_line_matches_navigate_long_record(self):
        records = [
            {
                "turtle_id": "turtle1",
                "payload": {
                    "operation": {
                        "nl_goal": {"text": "go to 1, 5"},
                        "intent_norm": {"task_family": "navigate", "slots": {}},
                    },
                    "evidence": {"collision_enter_count": 1, "success_rate": 1.0},
                },
            },
        ]
        _, hits = build_memory_context(
            "draw a line to 10, 5 while avoiding obstacles", records, top_k=1
        )
        self.assertGreaterEqual(hits, 1)

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
        self.assertIn("Memory policy (strict):", context)
        self.assertIn("MUST: Do not choose a single straight-line path", context)

    def test_build_memory_context_includes_lessons_when_present(self):
        records = [
            {
                "turtle_id": "turtle1",
                "payload": {
                    "operation": {
                        "nl_goal": {"text": "go to 1, 5"},
                        "intent_norm": {"task_family": "navigate", "slots": {}},
                    },
                    "evidence": {"collision_enter_count": 0, "success_rate": 1.0},
                    "lessons": [
                        "첫 번째 교훈입니다.",
                        "두 번째 교훈입니다.",
                        "세 번째 교훈입니다.",
                    ],
                },
            },
        ]
        context, hits = build_memory_context("go to 2, 3", records, top_k=1)
        self.assertEqual(hits, 1)
        self.assertIn("Memory lessons:", context)
        self.assertIn("[memory 1]", context)
        self.assertIn("첫 번째 교훈입니다.", context)

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
