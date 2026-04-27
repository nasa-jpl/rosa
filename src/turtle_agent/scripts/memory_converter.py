from __future__ import annotations

import json
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
        self._pending_short_term: List[Dict[str, Any]] = []

    def convert_session(
        self,
        *,
        date_str: str,
        session_id: Optional[str] = None,
        turtle_id: Optional[str] = None,
        test_case_id: Optional[str] = None,
        log_root: Optional[Path] = None,
    ) -> Dict[str, int]:
        """세션 로그를 읽어 단기/장기 기억 파일로 변환 저장합니다."""
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
        long_path = build_long_term_path(self._memory_root, resolved_session)

        short_count = 0
        long_count = 0
        for rec in short_term_records:
            append_jsonl(short_path, rec)
            self._pending_short_term.append(rec)
            short_count += 1
            if len(self._pending_short_term) >= 5:
                long_rec = self._build_long_term_record(
                    self._pending_short_term[:5],
                    resolved_session,
                    resolved_turtle,
                )
                append_jsonl(long_path, long_rec)
                self._pending_short_term = self._pending_short_term[5:]
                long_count += 1

        # 세션 종료 flush: 남은 단기 기억이 있으면 장기 기억으로 저장.
        if self._pending_short_term:
            long_rec = self._build_long_term_record(
                self._pending_short_term,
                resolved_session,
                resolved_turtle,
            )
            append_jsonl(long_path, long_rec)
            self._pending_short_term = []
            long_count += 1

        return {"short_term_written": short_count, "long_term_written": long_count}

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
        intent_norm = {
            "task_family": intent.get("task_family", "unknown"),
            "slots": intent.get("slots", {}),
        }

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
                    "clock": {"unix_ms": t_ms, "frame_id": "world"},
                    "active_goal": {
                        "natural_language": nl_text,
                        "constraints": {},
                    },
                    "intent_norm": intent_norm,
                    "plan": {
                        "steps": plan_steps,
                        "current_step_idx": idx + 1,
                    },
                    "execution_trace": {"steps": trace_steps},
                }
            )
        return records

    def _build_long_term_record(
        self,
        short_term_batch: List[Dict[str, Any]],
        session_id: str,
        turtle_id: str,
    ) -> Dict[str, Any]:
        latest = short_term_batch[-1]
        first_goal = short_term_batch[0].get("active_goal", {}).get("natural_language", "")
        intent_norm = short_term_batch[0].get(
            "intent_norm", {"task_family": "unknown", "slots": {}}
        )
        action_trace = []
        for short in short_term_batch:
            for step in short.get("execution_trace", {}).get("steps", []):
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
        return {
            "record_id": str(uuid.uuid4()),
            "record_type": "raw_episode",
            "session_id": session_id,
            "turtle_id": turtle_id,
            "payload": {
                "operation": {
                    "nl_goal": {"text": first_goal},
                    "intent_norm": intent_norm,
                },
                "action_trace": action_trace,
                "outcome": {
                    "success": all_success,
                    "terminal_reason": "goal_reached" if all_success else "incomplete",
                },
            },
        }
