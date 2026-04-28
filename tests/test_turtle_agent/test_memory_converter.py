import json
import os
import sys
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from memory_converter import (  # noqa: E402
    MemoryConverter,
    build_long_term_path,
    build_short_term_path,
    parse_context_from_log_path,
)
from pose_logger import build_collision_log_path, build_location_log_path  # noqa: E402
from command_logger import build_command_log_path  # noqa: E402


class TestMemoryConverter(unittest.TestCase):
    def test_parse_context_from_log_path(self):
        path = Path("/tmp/logs/2026-04-27/sess-1/turtle1/location.jsonl")
        session_id, turtle_id = parse_context_from_log_path(path)
        self.assertEqual(session_id, "sess-1")
        self.assertEqual(turtle_id, "turtle1")

    def test_three_phase_short_term_lifecycle(self):
        memory_root = _SCRIPTS / "memory"
        converter = MemoryConverter(memory_root=memory_root)
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        query_id = "q-1"

        rec = converter.begin_query_short_term(
            session_id=session_id,
            query_id=query_id,
            turtle_id=turtle_id,
            raw_text="relay race",
            intent="navigate",
            constraints={},
        )
        self.assertEqual(rec["decision_state"]["status"], "in_progress")
        converter.update_query_short_term(
            session_id=session_id,
            query_id=query_id,
            turtle_id=turtle_id,
            plan_step={"skill": "move_forward", "status": "success"},
            execution_step={
                "t_ms": 1000,
                "skill": "move_forward",
                "args": {"velocity": 1.0},
                "status": "success",
                "result": "ok",
            },
            collision_event={
                "type": "turtle_obstacle",
                "phase": "enter",
                "event_type": "enter",
                "collision_type": "turtle_obstacle",
                "obstacle_kind": "temporary",
                "t_ros": {"secs": 1, "nsecs": 0},
                "turtles": [turtle_id],
            },
        )
        out = converter.finalize_query_short_term(
            session_id=session_id,
            query_id=query_id,
            turtle_id=turtle_id,
            test_case_id="tc-lifecycle",
            finalized_at_unix_ms=1200,
            start_pose={"x": 1.0, "y": 1.0, "theta": 0.0},
            final_pose={"x": 2.0, "y": 2.0, "theta": 0.3},
            success=True,
            terminal_reason="goal_reached",
        )
        self.assertEqual(out["short_term_written"], 1)
        short_path = build_short_term_path(memory_root, session_id, "tc-lifecycle")
        rows = [json.loads(line) for line in short_path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(rows[-1]["decision_state"]["status"], "completed")
        self.assertEqual(rows[-1]["decision_state"]["final_pose"]["x"], 2.0)
        self.assertEqual(rows[-1]["evidence"]["execution_steps"][0]["source"], "worker")
        self.assertEqual(rows[-1]["evidence"]["collision_events"][0]["source"], "sensor")

    def test_convert_session_writes_short_and_long_term(self):
        log_root = _REPO_ROOT / "logs"
        memory_root = _SCRIPTS / "memory"
        date_str = "2026-04-27"
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        test_case_id = f"tc-{uuid.uuid4().hex[:8]}"
        obstacle_run_cmd = (
            "rostopic pub -r 22 /turtle1/cmd_vel geometry_msgs/Twist "
            "'{linear: {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        )

        location_path = build_location_log_path(log_root, date_str, session_id, turtle_id)
        command_path = build_command_log_path(log_root, date_str, session_id, turtle_id)
        location_path.parent.mkdir(parents=True, exist_ok=True)

        # 실제 장애물 월드 실행에서 수집한 location 로그 패턴을 반영한 샘플
        location_rows = [
            {
                "t_ros": {"secs": 1777265115, "nsecs": 450412273},
                "x": 5.579644680023193,
                "y": 5.544444561004639,
                "theta": 0.0,
                "linear_velocity": 0.550000011920929,
                "angular_velocity": 0.0,
            },
            {
                "t_ros": {"secs": 1777265118, "nsecs": 474242448},
                "x": 7.242844581604004,
                "y": 5.544444561004639,
                "theta": 0.0,
                "linear_velocity": 0.550000011920929,
                "angular_velocity": 0.0,
            },
            {
                "t_ros": {"secs": 1777265125, "nsecs": 530264377},
                "x": 11.088889122009277,
                "y": 5.544444561004639,
                "theta": 0.0,
                "linear_velocity": 0.550000011920929,
                "angular_velocity": 0.0,
            },
        ]
        command_doc = {
            "session_id": session_id,
            "turtle_id": turtle_id,
            "intent": {
                "type": "intent",
                "t_ms": 1777265115000,
                "task_family": "manual_cmd_vel",
                "slots": {"topic": "/turtle1/cmd_vel", "rate_hz": 22, "velocity": 0.55},
                "natural_language": obstacle_run_cmd,
            },
            "skills": [
                {
                    "type": "skill",
                    "t_ms": 1777265115000,
                    "skill": "move_forward",
                    "args": {"velocity": 0.55, "steps": 1},
                    "status": "success",
                    "result": "ok",
                },
                {
                    "type": "skill",
                    "t_ms": 1777265125000,
                    "skill": "move_forward",
                    "args": {"velocity": 0.55, "steps": 1},
                    "status": "success",
                    "result": "ok",
                },
            ],
        }
        location_path.write_text(
            "\n".join(json.dumps(r, ensure_ascii=False) for r in location_rows) + "\n",
            encoding="utf-8",
        )
        command_path.write_text(json.dumps(command_doc, ensure_ascii=False) + "\n", encoding="utf-8")

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
        self.assertEqual(result["long_term_written"], 0)

        short_path = build_short_term_path(memory_root, session_id, test_case_id)
        short_rows = [json.loads(line) for line in short_path.read_text(encoding="utf-8").splitlines()]
        long_rows = (
            [json.loads(line) for line in long_path.read_text(encoding="utf-8").splitlines()]
            if long_path.exists()
            else []
        )

        self.assertEqual(len(short_rows), 2)
        self.assertEqual(short_rows[0]["identity"]["session_id"], session_id)
        self.assertEqual(short_rows[0]["goal"]["raw_text"], obstacle_run_cmd)
        self.assertEqual(short_rows[0]["decision_state"]["status"], "in_progress")
        self.assertEqual(short_rows[1]["decision_state"]["current_step_index"], 2)
        self.assertEqual(short_rows[1]["decision_state"]["status"], "completed")
        self.assertIn("start_pose", short_rows[0]["decision_state"])
        self.assertIn("final_pose", short_rows[0]["decision_state"])
        self.assertIn("source", short_rows[0]["decision_state"])
        self.assertEqual(len(long_rows), before_long_count)

    def test_convert_session_merges_collision_jsonl_into_short_steps(self):
        """collision.jsonl 시각과 스킬 t_ms가 같은 기준이면 스텝별 events에 붙습니다."""
        log_root = _REPO_ROOT / "logs"
        memory_root = _SCRIPTS / "memory"
        date_str = "2026-04-27"
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        test_case_id = f"tc-{uuid.uuid4().hex[:8]}"

        # memory_converter._pose_to_unix_ms: ms = secs*1000 + nsecs/1e6
        def _ms_to_t_ros(ms: int) -> dict:
            secs = ms // 1000
            rem_ms = ms - secs * 1000
            return {"secs": int(secs), "nsecs": int(rem_ms * 1_000_000)}

        t0 = 1777265114000
        t1 = 1777265116000
        t_coll_b = 1777265115000  # strictly between t0 and t1 for second skill bucket

        location_path = build_location_log_path(log_root, date_str, session_id, turtle_id)
        command_path = build_command_log_path(log_root, date_str, session_id, turtle_id)
        collision_path = build_collision_log_path(log_root, date_str, session_id)
        location_path.parent.mkdir(parents=True, exist_ok=True)

        location_rows = [
            {
                "t_ros": _ms_to_t_ros(t0),
                "x": 5.5,
                "y": 5.5,
                "theta": 0.0,
                "linear_velocity": 0.0,
                "angular_velocity": 0.0,
            }
        ]
        command_doc = {
            "session_id": session_id,
            "turtle_id": turtle_id,
            "intent": {
                "type": "intent",
                "t_ms": t0,
                "task_family": "navigate",
                "natural_language": "go",
            },
            "skills": [
                {
                    "type": "skill",
                    "t_ms": t0,
                    "skill": "skill_a",
                    "args": {},
                    "status": "success",
                    "result": "",
                },
                {
                    "type": "skill",
                    "t_ms": t1,
                    "skill": "skill_b",
                    "args": {},
                    "status": "success",
                    "result": "",
                },
            ],
        }

        def _coll_row(ms: int, et: str) -> dict:
            return {
                "event_type": et,
                "collision_type": "turtle_obstacle",
                "turtles": [turtle_id],
                "obstacle_id": "obs1",
                "details": {"obstacle_kind": "temporary"},
                "t_ros": _ms_to_t_ros(ms),
                "x": 1.0,
                "y": 2.0,
                "theta": 0.0,
                "linear_velocity": 0.0,
                "angular_velocity": 0.0,
            }

        location_path.write_text(
            "\n".join(json.dumps(r, ensure_ascii=False) for r in location_rows) + "\n",
            encoding="utf-8",
        )
        command_path.write_text(json.dumps(command_doc, ensure_ascii=False) + "\n", encoding="utf-8")
        collision_path.write_text(
            json.dumps(_coll_row(t0, "enter"), ensure_ascii=False)
            + "\n"
            + json.dumps(_coll_row(t_coll_b, "exit"), ensure_ascii=False)
            + "\n",
            encoding="utf-8",
        )

        converter = MemoryConverter(memory_root=memory_root)
        converter.convert_session(
            date_str=date_str,
            session_id=session_id,
            turtle_id=turtle_id,
            test_case_id=test_case_id,
            log_root=log_root,
        )

        short_path = build_short_term_path(memory_root, session_id, test_case_id)
        rows = [
            json.loads(line) for line in short_path.read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(rows[-1]["evidence"]["execution_steps"]), 2)
        collisions = rows[-1]["evidence"]["collision_events"]
        self.assertEqual(len(collisions), 2)
        self.assertEqual(collisions[0]["type"], "turtle_obstacle")
        self.assertEqual(collisions[0]["event_type"], "enter")
        self.assertEqual(collisions[0]["obstacle_kind"], "temporary")
        self.assertEqual(collisions[1]["event_type"], "exit")

        ev = MemoryConverter._collect_collision_evidence(rows)
        self.assertEqual(ev["collision_events"], 2)
        self.assertEqual(ev["collision_enter_count"], 1)

    def test_convert_session_control_mode_writes_nothing(self):
        log_root = _REPO_ROOT / "logs"
        memory_root = _SCRIPTS / "memory"
        date_str = "2026-04-27"
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        test_case_id = f"tc-{uuid.uuid4().hex[:8]}"
        location_path = build_location_log_path(log_root, date_str, session_id, turtle_id)
        command_path = build_command_log_path(log_root, date_str, session_id, turtle_id)
        location_path.parent.mkdir(parents=True, exist_ok=True)
        location_path.write_text("", encoding="utf-8")
        command_path.write_text("", encoding="utf-8")

        converter = MemoryConverter(memory_root=memory_root)
        result = converter.convert_session(
            date_str=date_str,
            session_id=session_id,
            turtle_id=turtle_id,
            test_case_id=test_case_id,
            log_root=log_root,
            mode="control",
        )
        self.assertEqual(result["short_term_written"], 0)

    @patch.dict(os.environ, {"MEMORY_LESSONS_LLM": "0"})
    def test_finalize_session_writes_compressed_long_term(self):
        memory_root = _SCRIPTS / "memory"
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        session_dir = memory_root / "short_term" / session_id
        session_dir.mkdir(parents=True, exist_ok=True)

        base = {
            "identity": {
                "session_id": session_id,
                "query_id": f"{session_id}:1",
                "turtle_id": turtle_id,
            },
            "goal": {"raw_text": "draw me a pentagram", "intent": "trace_shape", "constraints": {}},
            "decision_state": {
                "status": "completed",
                "current_step_index": 1,
                "plan_steps": [{"skill": "rosa_response", "status": "success"}],
                "start_pose": {"x": 5.5, "y": 5.5, "theta": 1.2},
                "final_pose": {"x": 5.5, "y": 5.5, "theta": 1.2},
                "finalized_at_unix_ms": 1777275300520,
                "source": "control",
            },
            "evidence": {
                "execution_steps": [
                    {
                        "t_ms": 1777275300520,
                        "skill": "rosa_response",
                        "args": {"query": "draw me a pentagram"},
                        "status": "success",
                        "result": "radius 3.0",
                        "source": "worker",
                    }
                ],
                "collision_events": [
                    {
                        "type": "turtle_obstacle",
                        "event_type": "enter",
                        "phase": "enter",
                        "collision_type": "turtle_obstacle",
                        "obstacle_id": "wet",
                        "t_ros": {"secs": 1777275300, "nsecs": 520000000},
                        "turtles": [turtle_id],
                        "source": "sensor",
                    }
                ],
            },
            "outcome": {"success": True, "terminal_reason": "goal_reached"},
        }
        for idx in range(5):
            rec = dict(base)
            rec["decision_state"] = dict(base["decision_state"])
            rec["decision_state"]["finalized_at_unix_ms"] = 1777275300520 + idx
            path = session_dir / f"short_testid_tc-{idx}.jsonl"
            path.write_text(json.dumps(rec, ensure_ascii=False) + "\n", encoding="utf-8")

        converter = MemoryConverter(memory_root=memory_root)
        long_path = build_long_term_path(memory_root, session_id)
        before = len(long_path.read_text(encoding="utf-8").splitlines()) if long_path.exists() else 0
        written = converter.finalize_session(session_id=session_id, turtle_id=turtle_id)

        self.assertEqual(written, 1)
        rows = [json.loads(line) for line in long_path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(len(rows), before + 1)
        self.assertEqual(rows[-1]["record_type"], "compressed_routine")
        self.assertEqual(rows[-1]["turtle_id"], turtle_id)
        self.assertEqual(rows[-1]["payload"]["evidence"]["n_episodes"], 5)
        self.assertIn("collision_enter_count", rows[-1]["payload"]["evidence"])
        self.assertEqual(len(rows[-1]["payload"]["lessons"]), 3)

    @patch.dict(os.environ, {"MEMORY_LESSONS_LLM": "0"})
    def test_finalize_session_skips_bootstrap_first_query_when_enough_rows(self):
        memory_root = _SCRIPTS / "memory"
        session_id = f"sess-test-{uuid.uuid4().hex[:8]}"
        turtle_id = "turtle1"
        session_dir = memory_root / "short_term" / session_id
        session_dir.mkdir(parents=True, exist_ok=True)

        def _write_short(idx: int, query: str) -> None:
            rec = {
                "identity": {
                    "session_id": session_id,
                    "query_id": f"{session_id}:{idx+1}",
                    "turtle_id": turtle_id,
                },
                "goal": {"raw_text": query, "intent": "navigate", "constraints": {}},
                "decision_state": {
                    "status": "completed",
                    "current_step_index": 1,
                    "plan_steps": [{"skill": "rosa_response", "status": "success"}],
                    "start_pose": {"x": 5.5, "y": 5.5, "theta": 1.2},
                    "final_pose": {"x": 5.5, "y": 5.5, "theta": 1.2},
                    "finalized_at_unix_ms": 1777275300520 + idx,
                    "source": "control",
                },
                "evidence": {
                    "execution_steps": [
                        {
                            "t_ms": 1777275300520 + idx,
                            "skill": "rosa_response",
                            "args": {"query": query, "experience_key": "navigate"},
                            "status": "success",
                            "result": "ok",
                            "source": "worker",
                        }
                    ],
                    "collision_events": [],
                },
                "outcome": {"success": True, "terminal_reason": "goal_reached"},
            }
            (session_dir / f"short_testid_tc-{idx}.jsonl").write_text(
                json.dumps(rec, ensure_ascii=False) + "\n", encoding="utf-8"
            )

        _write_short(0, "go to 1, 5")
        for idx in range(1, 6):
            _write_short(idx, "draw a line to 10, 5")

        converter = MemoryConverter(memory_root=memory_root)
        long_path = build_long_term_path(memory_root, session_id)
        written = converter.finalize_session(session_id=session_id, turtle_id=turtle_id)

        self.assertEqual(written, 1)
        rows = [json.loads(line) for line in long_path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(rows[-1]["payload"]["operation"]["intent_norm"]["task_family"], "navigate")
        self.assertNotIn("go to 1, 5", rows[-1]["payload"]["operation"]["nl_goal"]["text"])

    @patch.dict(os.environ, {"MEMORY_LESSONS_LLM": "0"})
    def test_compressed_long_action_trace_lists_all_skills_in_multi_step_short(self):
        """한 short 행에 여러 스텝이 있으면 장기 action_trace·skill_sequence에 모두 포함합니다."""
        memory_root = _SCRIPTS / "memory"
        converter = MemoryConverter(memory_root=memory_root)
        one_short = {
            "identity": {"session_id": "s", "query_id": "s:1", "turtle_id": "t"},
            "goal": {"raw_text": "navigate and draw", "intent": "navigate", "constraints": {}},
            "decision_state": {
                "status": "completed",
                "current_step_index": 3,
                "plan_steps": [
                    {"skill": "teleport_absolute", "status": "success"},
                    {"skill": "draw_line_segment", "status": "success"},
                    {"skill": "rosa_response", "status": "success"},
                ],
                "start_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "final_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "finalized_at_unix_ms": 100,
                "source": "control",
            },
            "evidence": {
                "execution_steps": [
                    {
                        "t_ms": 10,
                        "skill": "teleport_absolute",
                        "args": {"x": 1.0, "y": 2.0},
                        "status": "success",
                        "result": "",
                        "source": "worker",
                    },
                    {
                        "t_ms": 20,
                        "skill": "draw_line_segment",
                        "args": {"x": 10.0, "y": 5.0},
                        "status": "success",
                        "result": "",
                        "source": "worker",
                    },
                    {
                        "t_ms": 30,
                        "skill": "rosa_response",
                        "args": {"query": "done"},
                        "status": "success",
                        "result": "ok",
                        "source": "worker",
                    },
                ],
                "collision_events": [],
            },
            "outcome": {"success": True, "terminal_reason": "goal_reached"},
        }
        rec = converter._build_compressed_long_record([one_short], "session-x", "turtle1")
        skills = [item["skill"] for item in rec["payload"]["action_trace"]]
        self.assertEqual(
            skills,
            ["teleport_absolute", "draw_line_segment", "rosa_response"],
        )
        self.assertEqual(rec["payload"]["routine"]["skill_sequence"], skills)
        self.assertEqual(len(rec["payload"]["lessons"]), 3)
        self.assertTrue(all(isinstance(s, str) and s.strip() for s in rec["payload"]["lessons"]))

    @patch.dict(os.environ, {"MEMORY_LESSONS_LLM": "0"})
    def test_compressed_long_cumulative_short_rows_no_duplicate_skills(self):
        """누적 short(스텝 수 증가)이면 새 스텝만 추가합니다."""
        memory_root = _SCRIPTS / "memory"
        converter = MemoryConverter(memory_root=memory_root)
        s1 = {
            "identity": {"session_id": "sid", "query_id": "sid:1", "turtle_id": "tid"},
            "goal": {"raw_text": "q", "intent": "natural_language_query", "constraints": {}},
            "decision_state": {
                "status": "completed",
                "current_step_index": 1,
                "plan_steps": [{"skill": "a", "status": "success"}],
                "start_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "final_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "finalized_at_unix_ms": 1,
                "source": "control",
            },
            "evidence": {
                "execution_steps": [
                    {
                        "t_ms": 1,
                        "skill": "a",
                        "args": {},
                        "status": "success",
                        "result": "",
                        "source": "worker",
                    }
                ],
                "collision_events": [],
            },
            "outcome": {"success": True, "terminal_reason": "goal_reached"},
        }
        s2 = {
            "identity": {"session_id": "sid", "query_id": "sid:2", "turtle_id": "tid"},
            "goal": {"raw_text": "q", "intent": "natural_language_query", "constraints": {}},
            "decision_state": {
                "status": "completed",
                "current_step_index": 2,
                "plan_steps": [
                    {"skill": "a", "status": "success"},
                    {"skill": "b", "status": "success"},
                ],
                "start_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "final_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "finalized_at_unix_ms": 2,
                "source": "control",
            },
            "evidence": {
                "execution_steps": [
                    {
                        "t_ms": 1,
                        "skill": "a",
                        "args": {},
                        "status": "success",
                        "result": "",
                        "source": "worker",
                    },
                    {
                        "t_ms": 2,
                        "skill": "b",
                        "args": {},
                        "status": "success",
                        "result": "",
                        "source": "worker",
                    },
                ],
                "collision_events": [],
            },
            "outcome": {"success": True, "terminal_reason": "goal_reached"},
        }
        rec = converter._build_compressed_long_record([s1, s2], "sid", "tid")
        skills = [item["skill"] for item in rec["payload"]["action_trace"]]
        self.assertEqual(skills, ["a", "b"])
        self.assertEqual(len(rec["payload"]["lessons"]), 3)


if __name__ == "__main__":
    unittest.main()
