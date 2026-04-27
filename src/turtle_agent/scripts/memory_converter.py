from __future__ import annotations

import json
import re
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from command_logger import build_command_log_path
from pose_logger import build_location_log_path, build_log_dir, resolve_log_root


def resolve_memory_root() -> Path:
    """기억 저장 루트 경로(`scripts/memory`)를 반환합니다."""
    return (Path(__file__).resolve().parent / "memory").resolve()


def build_short_term_path(memory_root: Path, session_id: str, test_case_id: str) -> Path:
    """단기 기억 파일 경로(`memory/short_term/<session_id>/short_testid_<id>.jsonl`)를 생성합니다."""
    return memory_root / "short_term" / session_id / f"short_testid_{test_case_id}.jsonl"


def build_long_term_path(memory_root: Path, session_id: str) -> Path:
    """장기 기억 파일 경로(`memory/long_term/long_sessionid_<session_id>.jsonl`)를 생성합니다."""
    return memory_root / "long_term" / f"long_sessionid_{session_id}.jsonl"


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    """JSONL 파일을 읽어 dict 리스트로 반환합니다."""
    if not path.exists():
        return []
    rows: List[Dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        rows.append(json.loads(stripped))
    return rows


def normalize_command_rows(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """command 로그를 이벤트 행 목록(intent 1개 + skill N개)으로 정규화합니다."""
    if not rows:
        return []

    # 신형 포맷: 세션 단일 객체 {"intent": {...}, "skills": [...]}
    if len(rows) == 1 and isinstance(rows[0], dict) and "skills" in rows[0]:
        doc = rows[0]
        normalized: List[Dict[str, Any]] = []
        intent = doc.get("intent")
        if isinstance(intent, dict) and intent:
            normalized.append(intent)
        skills = doc.get("skills", [])
        if isinstance(skills, list):
            normalized.extend([s for s in skills if isinstance(s, dict)])
        return normalized

    # 구형 포맷: JSONL 이벤트 누적
    return rows


def append_jsonl(path: Path, record: Dict[str, Any]) -> None:
    """JSONL 파일에 레코드를 한 줄 append 합니다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as fp:
        fp.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def parse_context_from_log_path(path: Path) -> Tuple[Optional[str], Optional[str]]:
    """로그 경로에서 세션/터틀 식별자를 추출합니다."""
    parts = path.parts
    try:
        idx = parts.index("logs")
    except ValueError:
        return None, None
    if len(parts) <= idx + 3:
        return None, None
    return parts[idx + 2], parts[idx + 3]


def _pose_to_unix_ms(pose_row: Dict[str, Any]) -> int:
    t_ros = pose_row.get("t_ros", {})
    secs = int(t_ros.get("secs", 0))
    nsecs = int(t_ros.get("nsecs", 0))
    return int(secs * 1000 + nsecs / 1_000_000)


def _pick_pose_for_time(location_rows: List[Dict[str, Any]], t_ms: int) -> Dict[str, float]:
    if not location_rows:
        return {"x": 0.0, "y": 0.0, "theta": 0.0}
    best = min(location_rows, key=lambda row: abs(_pose_to_unix_ms(row) - t_ms))
    return {
        "x": float(best.get("x", 0.0)),
        "y": float(best.get("y", 0.0)),
        "theta": float(best.get("theta", 0.0)),
    }


class MemoryConverter:
    """거북이 로그를 단기/장기 기억 스키마로 변환해 저장합니다."""

    def __init__(self, memory_root: Optional[Path] = None) -> None:
        """변환기 인스턴스를 초기화합니다."""
        self._memory_root = memory_root or resolve_memory_root()
    def convert_session(
        self,
        *,
        date_str: str,
        session_id: Optional[str] = None,
        turtle_id: Optional[str] = None,
        test_case_id: Optional[str] = None,
        log_root: Optional[Path] = None,
        write_long_term: bool = False,
    ) -> Dict[str, int]:
        """세션 로그를 읽어 단기 기억 파일로 변환 저장합니다."""
        log_root = log_root or resolve_log_root()

        location_path = build_location_log_path(
            log_root=log_root,
            date_str=date_str,
            session_id=session_id or "",
            turtle_id=turtle_id or "",
        )
        command_path = build_command_log_path(
            log_root=log_root,
            date_str=date_str,
            session_id=session_id or "",
            turtle_id=turtle_id or "",
        )

        # 세션/터틀은 명시 전달을 우선 사용하고, 미지정 시 경로에서 보조 추출한다.
        parsed_session, parsed_turtle = parse_context_from_log_path(location_path)
        resolved_session = session_id or parsed_session
        resolved_turtle = turtle_id or parsed_turtle
        if not resolved_session or not resolved_turtle:
            raise ValueError("session_id와 turtle_id를 확인할 수 없습니다.")

        location_rows = read_jsonl(location_path)
        command_rows = read_jsonl(command_path)
        command_rows = normalize_command_rows(command_rows)
        short_term_records = self._build_short_term_records(
            location_rows=location_rows,
            command_rows=command_rows,
            session_id=resolved_session,
            turtle_id=resolved_turtle,
        )

        resolved_test_case_id = test_case_id or resolved_session
        short_path = build_short_term_path(
            self._memory_root,
            resolved_session,
            resolved_test_case_id,
        )
        short_count = 0
        for rec in short_term_records:
            append_jsonl(short_path, rec)
            short_count += 1

        long_count = 0
        if write_long_term:
            long_count = self.finalize_session(
                session_id=resolved_session,
                turtle_id=resolved_turtle,
            )

        return {"short_term_written": short_count, "long_term_written": long_count}

    def finalize_session(self, *, session_id: str, turtle_id: str) -> int:
        """세션 단기 기억을 압축해 장기 기억 1건을 저장합니다."""
        short_rows = self._load_session_short_rows(session_id)
        if len(short_rows) < 5:
            return 0
        long_rec = self._build_compressed_long_record(
            short_term_batch=short_rows[:5],
            session_id=session_id,
            turtle_id=turtle_id,
        )
        long_path = build_long_term_path(self._memory_root, session_id)
        append_jsonl(long_path, long_rec)
        return 1

    def _build_short_term_records(
        self,
        *,
        location_rows: List[Dict[str, Any]],
        command_rows: List[Dict[str, Any]],
        session_id: str,
        turtle_id: str,
    ) -> List[Dict[str, Any]]:
        intents = [row for row in command_rows if row.get("type") == "intent"]
        skills = [row for row in command_rows if row.get("type") == "skill"]
        if not skills:
            return []

        intent = intents[0] if intents else {}
        nl_text = str(
            intent.get("natural_language")
            or intent.get("query")
            or intent.get("task_family", "unknown")
        )
        records: List[Dict[str, Any]] = []
        for idx, skill in enumerate(skills):
            t_ms = int(skill.get("t_ms", 0))
            plan_steps = [
                {"name": s.get("skill", ""), "status": s.get("status", "pending")}
                for s in skills[: idx + 1]
            ]
            trace_steps = []
            for s in skills[: idx + 1]:
                step_t_ms = int(s.get("t_ms", 0))
                trace_steps.append(
                    {
                        "t_ms": step_t_ms,
                        "pose": _pick_pose_for_time(location_rows, step_t_ms),
                        "skill_invocations": [
                            {
                                "skill": s.get("skill", ""),
                                "args": s.get("args", {}),
                                "status": s.get("status", "unknown"),
                                "result": s.get("result", ""),
                            }
                        ],
                        "events": [],
                    }
                )

            records.append(
                {
                    "session_id": session_id,
                    "turtle_id": turtle_id,
                    # NOTE: turtlesim에서는 좌표계가 사실상 단일(world)이라 frame_id 활용도가 낮습니다.
                    # 추후 world/map 식별이 필요해지면 frame_id를 map_id로 대체하는 방향을 검토합니다.
                    "clock": {"unix_ms": t_ms, "frame_id": "world"},
                    "active_goal": {
                        "natural_language": nl_text,
                        "constraints": {},
                    },
                    "plan": {
                        "steps": plan_steps,
                        "current_step_idx": idx + 1,
                    },
                    "execution_trace": {"steps": trace_steps},
                }
            )
        return records

    def _load_session_short_rows(self, session_id: str) -> List[Dict[str, Any]]:
        session_dir = self._memory_root / "short_term" / session_id
        if not session_dir.exists():
            return []
        rows: List[Dict[str, Any]] = []
        for path in sorted(session_dir.glob("short_testid_*.jsonl")):
            rows.extend(read_jsonl(path))
        return rows

    @staticmethod
    def _infer_task_family(query: str) -> str:
        q = (query or "").lower()
        if "pentagram" in q or "star" in q:
            return "trace_shape"
        if "rotate" in q or "turn" in q:
            return "rotate"
        if "goto" in q or "teleport" in q:
            return "goto"
        return "natural_language_query"

    @staticmethod
    def _extract_radius_series(action_trace: List[Dict[str, Any]]) -> List[float]:
        radii: List[float] = []
        for item in action_trace:
            result = str(item.get("result", ""))
            m = re.search(r"radius\s*\*{0,2}\s*([0-9]+(?:\.[0-9]+)?)", result, flags=re.I)
            if not m:
                continue
            try:
                radii.append(float(m.group(1)))
            except ValueError:
                continue
        return radii

    def _build_compressed_long_record(
        self,
        short_term_batch: List[Dict[str, Any]],
        session_id: str,
        turtle_id: str,
    ) -> Dict[str, Any]:
        first_goal = short_term_batch[0].get("active_goal", {}).get("natural_language", "")
        queries = [
            str(short.get("active_goal", {}).get("natural_language", "")).strip()
            for short in short_term_batch
            if str(short.get("active_goal", {}).get("natural_language", "")).strip()
        ]
        action_trace: List[Dict[str, Any]] = []
        for short in short_term_batch:
            steps = short.get("execution_trace", {}).get("steps", [])
            if not steps:
                continue
            step = steps[-1]
            invocations = step.get("skill_invocations", [])
            if not invocations:
                continue
            inv = invocations[0]
            action_trace.append(
                {
                    "t_ms": step.get("t_ms", 0),
                    "skill": inv.get("skill", ""),
                    "args": inv.get("args", {}),
                    "status": inv.get("status", "unknown"),
                    "result": inv.get("result", ""),
                }
            )

        all_success = all(item.get("status") == "success" for item in action_trace) if action_trace else False
        radii = self._extract_radius_series(action_trace)
        decay_ratio = 1.0
        if len(radii) >= 2 and radii[-2] > 0:
            decay_ratio = round(radii[-1] / radii[-2], 3)
        primary_skill = action_trace[0].get("skill", "unknown") if action_trace else "unknown"
        task_family = self._infer_task_family(first_goal)
        return {
            "record_id": str(uuid.uuid4()),
            "record_type": "compressed_routine",
            "turtle_id": turtle_id,
            "payload": {
                "operation": {
                    "nl_goal": {
                        "text": f"{first_goal} (follow-ups: {' / '.join(queries[1:])})"
                        if len(queries) > 1
                        else first_goal
                    },
                    "intent_norm": {
                        "task_family": task_family,
                        "slots": {"shape": "pentagram"} if task_family == "trace_shape" else {},
                    },
                },
                "action_trace": action_trace,
                "outcome": {
                    "success": all_success,
                    "terminal_reason": "goal_reached" if all_success else "execution_failed",
                },
                "routine": {
                    "name": "pentagram_repeat_with_scaling" if task_family == "trace_shape" else "query_routine",
                    "skill_sequence": [primary_skill],
                    "default_args": {"center_x": 5.544, "center_y": 5.544},
                    "param_policy": {
                        "radius_decay_ratio": decay_ratio,
                        "followup_rules": [
                            {"trigger": "another one", "action": "reuse previous shape"},
                            {"trigger": "smaller", "action": "apply radius decay"},
                        ],
                    },
                },
                "evidence": {
                    "n_episodes": len(action_trace),
                    "success_rate": round(
                        sum(1 for item in action_trace if item.get("status") == "success") / max(1, len(action_trace)),
                        3,
                    ),
                },
                "lessons": [
                    "Follow-up references should inherit prior shape context.",
                    "When user asks for smaller output, reduce radius progressively.",
                ],
            },
            "meta": {
                "session_id": session_id,
                "compression": {"method": "deterministic"},
                "created_at_unix_ms": int(short_term_batch[-1].get("clock", {}).get("unix_ms", 0)),
            },
        }
