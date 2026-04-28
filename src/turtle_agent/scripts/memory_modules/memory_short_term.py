from __future__ import annotations

"""Short-term memory schema and pipeline.

Schema overview:
- identity: session/query/turtle 식별자
- goal: 원문 질의, 의도, 제약조건
- decision_state: 진행상태, 계획스텝, 시작/종료 pose, finalize 시각
- evidence: 실행 스텝, 충돌 이벤트
- outcome: 성공 여부, 종료 사유

Legacy compatibility:
- collision event는 type, phase, event_type을 함께 유지합니다.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional


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


def _collision_row_to_trace_event(row: Dict[str, Any]) -> Dict[str, Any]:
    """collision.jsonl 한 줄을 short-term evidence.collision_events 항목으로 변환합니다."""
    et = str(row.get("event_type", "") or "")
    details = row.get("details") if isinstance(row.get("details"), dict) else {}
    obstacle_kind = row.get("obstacle_kind")
    if obstacle_kind is None:
        obstacle_kind = details.get("obstacle_kind")
    if obstacle_kind is not None:
        obstacle_kind = str(obstacle_kind).strip().lower() or None
    out: Dict[str, Any] = {
        "type": row.get("collision_type"),
        "phase": et,
        "event_type": et,
        "collision_type": row.get("collision_type"),
        "obstacle_kind": obstacle_kind,
        "obstacle_id": row.get("obstacle_id"),
        "turtles": row.get("turtles"),
    }
    if row.get("t_ros") is not None:
        out["t_ros"] = row.get("t_ros")
    if row.get("details") is not None:
        out["details"] = row.get("details")
    return out


def _filter_collision_rows_for_turtle(
    rows: List[Dict[str, Any]], turtle_id: str
) -> List[Dict[str, Any]]:
    tid = str(turtle_id)
    out: List[Dict[str, Any]] = []
    for row in rows:
        turtles = row.get("turtles")
        if not turtles:
            out.append(row)
            continue
        if tid in [str(x) for x in turtles]:
            out.append(row)
    return out


def _skill_index_for_collision_time(c_ms: int, skill_times: List[int]) -> int:
    if not skill_times:
        return 0
    if len(skill_times) == 1:
        return 0
    if c_ms <= skill_times[0]:
        return 0
    if c_ms > skill_times[-1]:
        return len(skill_times) - 1
    for i in range(1, len(skill_times)):
        if skill_times[i - 1] < c_ms <= skill_times[i]:
            return i
    return len(skill_times) - 1


def _bucket_collision_events_by_skill_index(
    collision_rows: List[Dict[str, Any]],
    skills: List[Dict[str, Any]],
) -> List[List[Dict[str, Any]]]:
    n = len(skills)
    buckets: List[List[Dict[str, Any]]] = [[] for _ in range(n)]
    if n == 0:
        return buckets
    skill_times = [int(s.get("t_ms", 0)) for s in skills]
    for row in collision_rows:
        try:
            c_ms = _pose_to_unix_ms(row)
        except (TypeError, ValueError, KeyError):
            continue
        idx = _skill_index_for_collision_time(c_ms, skill_times)
        buckets[idx].append(_collision_row_to_trace_event(row))
    return buckets


@dataclass
class ShortTermCreateInput:
    session_id: str
    query_id: str
    turtle_id: str
    raw_text: str
    intent: str
    constraints: Dict[str, Any]


def create_short_term_record(
    *,
    session_id: str,
    query_id: str,
    turtle_id: str,
    raw_text: str,
    intent: str,
    constraints: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "identity": {
            "session_id": session_id,
            "query_id": query_id,
            "turtle_id": turtle_id,
        },
        "goal": {
            "raw_text": str(raw_text or ""),
            "intent": str(intent or "natural_language_query"),
            "constraints": constraints or {},
        },
        "decision_state": {
            "status": "in_progress",
            "current_step_index": 0,
            "plan_steps": [],
            "start_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "final_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "finalized_at_unix_ms": 0,
            "source": "control",
        },
        "evidence": {
            "execution_steps": [],
            "collision_events": [],
        },
        "outcome": {
            "success": False,
            "terminal_reason": "in_progress",
        },
    }


def update_short_term_record(
    record: Dict[str, Any],
    *,
    plan_step: Optional[Dict[str, Any]] = None,
    execution_step: Optional[Dict[str, Any]] = None,
    collision_event: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    decision_state = record.setdefault("decision_state", {})
    evidence = record.setdefault("evidence", {})
    if plan_step is not None:
        plan_steps = decision_state.setdefault("plan_steps", [])
        plan_steps.append(dict(plan_step))
        decision_state["current_step_index"] = len(plan_steps)
    if execution_step is not None:
        steps = evidence.setdefault("execution_steps", [])
        step = dict(execution_step)
        step.setdefault("source", "worker")
        steps.append(step)
    if collision_event is not None:
        events = evidence.setdefault("collision_events", [])
        event = dict(collision_event)
        event.setdefault("source", "sensor")
        events.append(event)
    return record


def finalize_short_term_record(
    record: Dict[str, Any],
    *,
    finalized_at_unix_ms: int,
    start_pose: Dict[str, float],
    final_pose: Dict[str, float],
    success: bool,
    terminal_reason: str,
) -> Dict[str, Any]:
    decision_state = record.setdefault("decision_state", {})
    if str(terminal_reason) == "in_progress":
        decision_state["status"] = "in_progress"
    else:
        decision_state["status"] = "completed" if success else "failed"
    decision_state["start_pose"] = dict(start_pose)
    decision_state["final_pose"] = dict(final_pose)
    decision_state["finalized_at_unix_ms"] = int(finalized_at_unix_ms)
    outcome = record.setdefault("outcome", {})
    outcome["success"] = bool(success)
    outcome["terminal_reason"] = str(terminal_reason)
    return record


def build_short_term_records(
    *,
    location_rows: List[Dict[str, Any]],
    command_rows: List[Dict[str, Any]],
    session_id: str,
    turtle_id: str,
    collision_rows: Optional[List[Dict[str, Any]]] = None,
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
    collision_by_skill = _bucket_collision_events_by_skill_index(collision_rows or [], skills)

    records: List[Dict[str, Any]] = []
    for idx, skill in enumerate(skills):
        t_ms = int(skill.get("t_ms", 0))
        query_id = f"{session_id}:{idx + 1}"
        record = create_short_term_record(
            session_id=session_id,
            query_id=query_id,
            turtle_id=turtle_id,
            raw_text=nl_text,
            intent=str(intent.get("task_family", "natural_language_query")),
            constraints={},
        )
        for j, s in enumerate(skills[: idx + 1]):
            step_t_ms = int(s.get("t_ms", 0))
            update_short_term_record(
                record,
                plan_step={
                    "skill": s.get("skill", ""),
                    "status": s.get("status", "pending"),
                },
                execution_step={
                    "t_ms": step_t_ms,
                    "skill": s.get("skill", ""),
                    "args": s.get("args", {}),
                    "status": s.get("status", "unknown"),
                    "result": s.get("result", ""),
                },
            )
            for event in list(collision_by_skill[j]):
                update_short_term_record(record, collision_event=event)
        start_pose = (
            _pick_pose_for_time(location_rows, int(skills[0].get("t_ms", 0)))
            if skills
            else {"x": 0.0, "y": 0.0, "theta": 0.0}
        )
        final_pose = _pick_pose_for_time(location_rows, t_ms)
        is_last = idx == (len(skills) - 1)
        terminal_status = str(skill.get("status", "unknown")).lower()
        records.append(
            finalize_short_term_record(
                record,
                finalized_at_unix_ms=t_ms,
                start_pose=start_pose,
                final_pose=final_pose,
                success=bool(is_last and terminal_status == "success"),
                terminal_reason="goal_reached"
                if is_last and terminal_status == "success"
                else ("execution_failed" if is_last else "in_progress"),
            )
        )
    return records


class BaseShortTermModeAdapter:
    """single/control 모드가 같은 short-term 스키마를 채우기 위한 공통 어댑터 인터페이스."""

    def build_records(
        self,
        *,
        session_id: str,
        turtle_id: str,
        location_rows: List[Dict[str, Any]],
        command_rows: List[Dict[str, Any]],
        collision_rows: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        raise NotImplementedError


class SingleModeMemoryAdapter(BaseShortTermModeAdapter):
    """single 모드(command/location/collision 로그 기반) short-term 생성."""

    def build_records(
        self,
        *,
        session_id: str,
        turtle_id: str,
        location_rows: List[Dict[str, Any]],
        command_rows: List[Dict[str, Any]],
        collision_rows: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        return build_short_term_records(
            location_rows=location_rows,
            command_rows=command_rows,
            session_id=session_id,
            turtle_id=turtle_id,
            collision_rows=collision_rows,
        )


class ControlModeMemoryAdapter(BaseShortTermModeAdapter):
    """
    control 모드는 런타임 이벤트(begin/update/finalize)가 직접 short-term을 채웁니다.
    convert_session 경로에서는 생성하지 않습니다.
    """

    def build_records(
        self,
        *,
        session_id: str,
        turtle_id: str,
        location_rows: List[Dict[str, Any]],
        command_rows: List[Dict[str, Any]],
        collision_rows: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        return []


__all__ = [
    "ShortTermCreateInput",
    "BaseShortTermModeAdapter",
    "SingleModeMemoryAdapter",
    "ControlModeMemoryAdapter",
    "create_short_term_record",
    "update_short_term_record",
    "finalize_short_term_record",
    "build_short_term_records",
    "_filter_collision_rows_for_turtle",
]

