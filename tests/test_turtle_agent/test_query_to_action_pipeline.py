import json
import sys
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from action_mapper import map_intent_to_plan  # noqa: E402
from command_logger import build_command_log_path, CommandLogger  # noqa: E402
from llm import parse_intent_with_llm  # noqa: E402


class _FakeMessage:
    def __init__(self, content: str):
        self.content = content


class _FakeLLM:
    def __init__(self, content: str):
        self._content = content

    def invoke(self, _prompt: str):
        return _FakeMessage(self._content)


class TestIntentParsing(unittest.TestCase):
    def test_parse_intent_with_llm_json(self):
        llm = _FakeLLM(
            '{"task_family":"trace_shape","slots":{"shape":"rectangle","size":2.5}}'
        )
        intent = parse_intent_with_llm(llm, "사각형 그려줘")
        self.assertEqual(intent["task_family"], "trace_shape")
        self.assertEqual(intent["slots"]["shape"], "rectangle")

    def test_parse_intent_fallback(self):
        class _BrokenLLM:
            def invoke(self, _prompt: str):
                raise RuntimeError("boom")

        intent = parse_intent_with_llm(_BrokenLLM(), "회전해줘")
        self.assertEqual(intent["task_family"], "rotate")

    def test_parse_move_box_fallback(self):
        class _BrokenLLM:
            def invoke(self, _prompt: str):
                raise RuntimeError("boom")

        intent = parse_intent_with_llm(_BrokenLLM(), "박스를 옮겨줘")
        self.assertEqual(intent["task_family"], "move_object")
        self.assertEqual(intent["slots"]["object"], "box")


class TestActionMapper(unittest.TestCase):
    def test_rectangle_plan_has_steps(self):
        intent = {"task_family": "trace_shape", "slots": {"shape": "rectangle", "size": 2.0}}
        plan = map_intent_to_plan(intent)
        self.assertEqual(len(plan), 8)
        self.assertEqual(plan[0].name, "forward")
        self.assertEqual(plan[1].name, "rotate")

    def test_move_object_plan_has_box_steps(self):
        intent = {
            "task_family": "move_object",
            "slots": {"size": 2.0, "from_x": 2.0, "from_y": 2.0, "to_x": 7.0, "to_y": 7.0},
        }
        plan = map_intent_to_plan(intent)
        self.assertEqual([step.name for step in plan], ["draw_box", "clear_canvas", "draw_box"])


class TestCommandLogger(unittest.TestCase):
    def test_build_command_log_path(self):
        p = build_command_log_path(Path("/tmp/logs"), "2026-04-27", "sess-1", "turtle1")
        self.assertEqual(p, Path("/tmp/logs/2026-04-27/sess-1/turtle1/command.jsonl"))

    def test_intent_is_replaced_per_query_and_skill_logged(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = CommandLogger(
                log_root=Path(tmp),
                date_str="2026-04-27",
                session_id="sess-1",
            )
            try:
                logger.log_intent("turtle1", {"task_family": "rotate", "slots": {"degrees": 90}})
                logger.log_intent("turtle1", {"task_family": "goto", "slots": {"x": 1, "y": 2}})
                logger.log_skill(
                    "turtle1",
                    skill="rotate",
                    args={"degrees": 90},
                    status="success",
                    result="ok",
                )
            finally:
                logger.close()

            path = Path(tmp) / "2026-04-27" / "sess-1" / "turtle1" / "command.jsonl"
            doc = json.loads(path.read_text(encoding="utf-8").strip())
            self.assertEqual(doc["session_id"], "sess-1")
            self.assertEqual(doc["turtle_id"], "turtle1")
            self.assertEqual(doc["intent"]["type"], "intent")
            self.assertEqual(doc["intent"]["task_family"], "goto")
            self.assertEqual(len(doc["skills"]), 1)
            self.assertEqual(doc["skills"][0]["type"], "skill")


if __name__ == "__main__":
    unittest.main()
