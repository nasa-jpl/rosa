from __future__ import annotations

"""Long-term compressed memory schema and pipeline.

Schema overview:
- record_id, record_type, turtle_id: 장기 레코드 식별 정보
- payload.operation: 자연어 목표/의도 정규화
- payload.action_trace: 실행 스텝의 직렬화된 시퀀스
- payload.outcome: 세션 성공 여부와 종료 사유
- payload.routine: 반복 가능한 스킬 시퀀스/파라미터 정책
- payload.evidence: 에피소드 수, 성공률, 충돌 집계
- payload.lessons: 세션 교훈 3문장
- meta: session_id, 압축 방식, 생성 시각
"""

import json
import logging
import os
import re
import uuid
from typing import Any, Dict, List, Optional, Tuple

_LESSONS_TARGET_LINES = 3
_LOGGER = logging.getLogger(__name__)


def _parse_lesson_lines(text: str, *, max_lines: int = _LESSONS_TARGET_LINES) -> List[str]:
    lines: List[str] = []
    for raw in (text or "").strip().splitlines():
        line = raw.strip()
        if not line:
            continue
        line = re.sub(r"^[\s>*-]*\d+[\).\s]+", "", line)
        line = line.lstrip("-•* ").strip()
        if line.startswith('"') and line.endswith('"') and len(line) > 2:
            line = line[1:-1].strip()
        if line:
            lines.append(line)
        if len(lines) >= max_lines:
            break
    return lines


def is_bootstrap_query_record(short_row: Dict[str, Any]) -> bool:
    goal = str(short_goal_text(short_row)).strip().lower()
    if not goal:
        return False
    if not any(token in goal for token in ("go to", "move to", "goto", "teleport")):
        return False
    return bool(re.search(r"\d+(?:\.\d+)?\s*,\s*\d+(?:\.\d+)?", goal))


def select_compression_batch(short_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if len(short_rows) >= 6 and is_bootstrap_query_record(short_rows[0]):
        return short_rows[1:]
    return short_rows


def infer_task_family(query: str) -> str:
    q = (query or "").lower()
    if "pentagram" in q or "star" in q:
        return "trace_shape"
    if "rotate" in q or "turn" in q:
        return "rotate"
    if any(token in q for token in ("go to", "move to", "goto", "teleport", "이동", "가줘", "로 가")):
        return "goto"
    return "natural_language_query"


def steps_equivalent(a: Dict[str, Any], b: Dict[str, Any]) -> bool:
    if int(a.get("t_ms", 0)) != int(b.get("t_ms", 0)):
        return False
    ai = (a.get("skill_invocations") or [{}])[0]
    bi = (b.get("skill_invocations") or [{}])[0]
    return str(ai.get("skill", "")) == str(bi.get("skill", ""))


def short_goal_text(short_row: Dict[str, Any]) -> str:
    goal = short_row.get("goal", {})
    return str(goal.get("raw_text", "")) if isinstance(goal, dict) else ""


def short_trace_steps(short_row: Dict[str, Any]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    ev = short_row.get("evidence", {})
    execution_steps = ev.get("execution_steps", []) if isinstance(ev, dict) else []
    if not isinstance(execution_steps, list):
        return out
    for item in execution_steps:
        if not isinstance(item, dict):
            continue
        out.append(
            {
                "t_ms": int(item.get("t_ms", 0)),
                "skill_invocations": [
                    {
                        "skill": item.get("skill", ""),
                        "args": item.get("args", {}),
                        "status": item.get("status", "unknown"),
                        "result": item.get("result", ""),
                    }
                ],
            }
        )
    return out


def short_collision_events(short_row: Dict[str, Any]) -> List[Dict[str, Any]]:
    ev = short_row.get("evidence", {})
    events = ev.get("collision_events", []) if isinstance(ev, dict) else []
    return [e for e in events if isinstance(e, dict)] if isinstance(events, list) else []


def short_unix_ms(short_row: Dict[str, Any]) -> int:
    ds = short_row.get("decision_state", {})
    return int(ds.get("finalized_at_unix_ms", 0)) if isinstance(ds, dict) else 0


def is_strict_prefix_steps(prev_steps: List[Dict[str, Any]], curr_steps: List[Dict[str, Any]]) -> bool:
    if len(curr_steps) < len(prev_steps):
        return False
    for i in range(len(prev_steps)):
        if not steps_equivalent(prev_steps[i], curr_steps[i]):
            return False
    return True


def append_action_trace_from_short_row(
    action_trace: List[Dict[str, Any]],
    prev_steps: Optional[List[Dict[str, Any]]],
    curr_steps: List[Dict[str, Any]],
) -> None:
    def emit_step(step: Dict[str, Any]) -> None:
        for inv in step.get("skill_invocations") or []:
            action_trace.append(
                {
                    "t_ms": int(step.get("t_ms", 0)),
                    "skill": str(inv.get("skill", "")),
                    "args": inv.get("args", {}),
                    "status": str(inv.get("status", "unknown")),
                    "result": str(inv.get("result", "")),
                }
            )

    if not curr_steps:
        return
    if prev_steps is None:
        for step in curr_steps:
            emit_step(step)
        return
    if is_strict_prefix_steps(prev_steps, curr_steps):
        delta = curr_steps[len(prev_steps) :]
        if delta:
            for step in delta:
                emit_step(step)
        else:
            emit_step(curr_steps[-1])
        return
    for step in curr_steps:
        emit_step(step)


def extract_radius_series(action_trace: List[Dict[str, Any]]) -> List[float]:
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


def extract_context_from_action_trace(
    action_trace: List[Dict[str, Any]], fallback_goal: str
) -> Tuple[str, Dict[str, str]]:
    for item in action_trace:
        args = item.get("args", {})
        if not isinstance(args, dict):
            continue
        exp_key = str(args.get("experience_key", "")).strip()
        if exp_key.startswith("navigate|from:") and "|to:" in exp_key:
            m = re.match(r"^navigate\|from:([^|]+)\|to:(.+)$", exp_key)
            if m:
                return "navigate", {"from": m.group(1), "to": m.group(2)}
        if exp_key in ("navigate", "rotate", "trace_shape", "natural_language_query"):
            return exp_key, {}
        query = str(args.get("query", "")).strip()
        if query:
            inferred = infer_task_family(query)
            if inferred == "goto":
                return "navigate", {}
            return inferred, {}
    inferred_from_goal = infer_task_family(fallback_goal)
    if inferred_from_goal == "goto":
        return "navigate", {}
    return inferred_from_goal, {}


def collision_event_fingerprint(event: Dict[str, Any]) -> Tuple[Any, ...]:
    tr = event.get("t_ros")
    if isinstance(tr, dict):
        tkey = (int(tr.get("secs", 0)), int(tr.get("nsecs", 0)))
    else:
        tkey = (None, None)
    return (
        tkey,
        str(event.get("obstacle_id") or event.get("obstacle") or ""),
        str(event.get("event_type") or event.get("phase") or ""),
        str(event.get("collision_type") or ""),
    )


def collect_collision_evidence(short_term_batch: List[Dict[str, Any]]) -> Dict[str, Any]:
    collision_events = 0
    collision_enter_count = 0
    collision_obstacles: set[str] = set()
    seen_fp: set[Tuple[Any, ...]] = set()
    for short in short_term_batch:
        for event in short_collision_events(short):
            row_kind = str(event.get("type", "")).lower()
            phase = str(event.get("phase") or event.get("event") or "").lower()
            subtype = str(event.get("event_type") or "").lower()
            collision_type = str(event.get("collision_type") or "").lower()
            if (
                "collision" not in row_kind
                and "collision" not in phase
                and "collision" not in collision_type
                and collision_type not in ("turtle_obstacle", "turtle_turtle")
            ):
                continue
            fp = collision_event_fingerprint(event)
            if fp in seen_fp:
                continue
            seen_fp.add(fp)
            collision_events += 1
            if phase in ("enter", "collision_enter") or subtype == "enter":
                collision_enter_count += 1
            obstacle = (
                event.get("obstacle")
                or event.get("obstacle_id")
                or event.get("name")
                or event.get("obstacle_name")
            )
            if obstacle:
                collision_obstacles.add(str(obstacle))
    return {
        "collision_events": collision_events,
        "collision_enter_count": collision_enter_count,
        "collision_obstacles": sorted(collision_obstacles),
    }


def lessons_context_payload(
    short_term_batch: List[Dict[str, Any]],
    *,
    collision_ev: Dict[str, Any],
    task_family: str,
    action_trace: List[Dict[str, Any]],
    all_success: bool,
    first_goal: str,
    queries: List[str],
) -> Dict[str, Any]:
    goals = []
    seen: set[str] = set()
    for q in [first_goal] + queries:
        g = str(q).strip()
        if g and g not in seen:
            seen.add(g)
            goals.append(g)
    skills_used: List[str] = []
    sk_seen: set[str] = set()
    for item in action_trace:
        sk = str(item.get("skill", "")).strip()
        if sk and sk not in sk_seen:
            sk_seen.add(sk)
            skills_used.append(sk)
    return {
        "task_family": task_family,
        "goals": goals,
        "collision": dict(collision_ev),
        "skills_used": skills_used[:40],
        "all_tool_steps_succeeded": all_success,
    }


def fallback_lessons_lines(
    *,
    collision_ev: Dict[str, Any],
    task_family: str,
    first_goal: str,
    all_success: bool,
    action_trace: List[Dict[str, Any]],
) -> List[str]:
    enters = int(collision_ev.get("collision_enter_count", 0))
    obstacles = collision_ev.get("collision_obstacles") or []
    obs_txt = ", ".join(str(x) for x in obstacles[:5]) if obstacles else "없음"
    line1 = (
        f"이번 세션의 주요 목표는 「{first_goal[:120]}」이며 "
        f"작업 유형은 {task_family}로 분류되었습니다."
    )
    if enters > 0:
        line2 = (
            f"장애물 구간에서 충돌 진입이 {enters}회 기록되었고 "
            f"관련 장애물 식별자는 {obs_txt}입니다."
        )
    else:
        line2 = "기록된 충돌 진입은 없었고, 주행 중 장애물 관통 이벤트도 집계되지 않았습니다."
    n_tools = len(action_trace)
    ok = "모든 도구 단계가 성공으로 끝났습니다." if all_success else "일부 단계에서 실패가 있었습니다."
    line3 = f"총 {n_tools}개의 도구 호출이 있었으며, {ok}"
    return [line1, line2, line3]


def summarize_lessons_with_llm(
    short_term_batch: List[Dict[str, Any]],
    *,
    collision_ev: Dict[str, Any],
    task_family: str,
    action_trace: List[Dict[str, Any]],
    all_success: bool,
    first_goal: str,
    queries: List[str],
) -> List[str]:
    fb = fallback_lessons_lines(
        collision_ev=collision_ev,
        task_family=task_family,
        first_goal=first_goal or "(미상)",
        all_success=all_success,
        action_trace=action_trace,
    )
    if os.getenv("MEMORY_LESSONS_LLM", "1").strip().lower() in ("0", "false", "no", "off"):
        return fb

    payload = lessons_context_payload(
        short_term_batch,
        collision_ev=collision_ev,
        task_family=task_family,
        action_trace=action_trace,
        all_success=all_success,
        first_goal=first_goal,
        queries=queries,
    )
    try:
        from langchain_core.messages import HumanMessage

        from llm import get_llm

        ctx = json.dumps(payload, ensure_ascii=False, indent=2)
        if len(ctx) > 12000:
            ctx = ctx[:12000] + "\n…(truncated)"

        prompt = (
            "당신은 turtle_agent의 단기 메모리(short-term) 요약을 읽고 "
            "같은 세션에서 다음에 활용할 교훈만 추립니다.\n\n"
            "규칙:\n"
            "- 단기 기록에서 실제로 나타난 목표·행동·충돌·성공 여부만 근거로 씁니다. 추측은 최소화합니다.\n"
            "- 정확히 세 문장만 출력합니다. 각 문장은 한 줄에 하나씩입니다.\n"
            "- 번호, 글머리표, 따옴표 장식 없이 평문만 사용합니다.\n"
            "- 한국어로 작성합니다.\n\n"
            "입력 요약(JSON):\n"
            f"{ctx}"
        )
        llm = get_llm(streaming=False)
        msg = llm.invoke([HumanMessage(content=prompt)])
        raw = getattr(msg, "content", None)
        text = raw if isinstance(raw, str) else str(raw or "")
        parsed = _parse_lesson_lines(text, max_lines=_LESSONS_TARGET_LINES)
        out: List[str] = []
        for i in range(_LESSONS_TARGET_LINES):
            out.append(parsed[i] if i < len(parsed) else fb[i])
        return out
    except Exception as exc:
        _LOGGER.warning("long-term lessons LLM unavailable, using fallback: %s", exc)
        return fb


def create_long_term_record(
    short_term_batch: List[Dict[str, Any]],
    session_id: str,
    turtle_id: str,
) -> Dict[str, Any]:
    first_goal = short_goal_text(short_term_batch[0])
    queries = [str(short_goal_text(short)).strip() for short in short_term_batch if str(short_goal_text(short)).strip()]
    action_trace: List[Dict[str, Any]] = []
    prev_trace_steps: Optional[List[Dict[str, Any]]] = None
    for short in short_term_batch:
        steps = short_trace_steps(short)
        if not steps:
            continue
        append_action_trace_from_short_row(action_trace, prev_trace_steps, steps)
        prev_trace_steps = steps

    all_success = all(item.get("status") == "success" for item in action_trace) if action_trace else False
    radii = extract_radius_series(action_trace)
    decay_ratio = 1.0
    if len(radii) >= 2 and radii[-2] > 0:
        decay_ratio = round(radii[-1] / radii[-2], 3)
    primary_skill = action_trace[0].get("skill", "unknown") if action_trace else "unknown"
    skill_sequence = [str(item.get("skill", "")) for item in action_trace if item.get("skill")]
    task_family, slots = extract_context_from_action_trace(action_trace, first_goal)
    collision_ev = collect_collision_evidence(short_term_batch)
    lessons_lines = summarize_lessons_with_llm(
        short_term_batch,
        collision_ev=collision_ev,
        task_family=task_family,
        action_trace=action_trace,
        all_success=all_success,
        first_goal=str(first_goal or ""),
        queries=queries,
    )
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
                    "slots": {"shape": "pentagram"} if task_family == "trace_shape" else slots,
                },
            },
            "action_trace": action_trace,
            "outcome": {
                "success": all_success,
                "terminal_reason": "goal_reached" if all_success else "execution_failed",
            },
            "routine": {
                "name": "pentagram_repeat_with_scaling" if task_family == "trace_shape" else "query_routine",
                "skill_sequence": skill_sequence or ([primary_skill] if primary_skill != "unknown" else []),
                "default_args": {"center_x": 5.544, "center_y": 5.544} if task_family == "trace_shape" else {},
                "param_policy": {
                    "radius_decay_ratio": decay_ratio,
                    "followup_rules": [
                        {"trigger": "another one", "action": "reuse previous shape"},
                        {"trigger": "smaller", "action": "apply radius decay"},
                    ],
                }
                if task_family == "trace_shape"
                else {},
            },
            "evidence": {
                "n_episodes": len(action_trace),
                "success_rate": round(
                    sum(1 for item in action_trace if item.get("status") == "success") / max(1, len(action_trace)),
                    3,
                ),
                "collision_events": collision_ev["collision_events"],
                "collision_enter_count": collision_ev["collision_enter_count"],
                "collision_obstacles": collision_ev["collision_obstacles"],
            },
            "lessons": lessons_lines,
        },
        "meta": {
            "session_id": session_id,
            "compression": {"method": "deterministic"},
            "created_at_unix_ms": short_unix_ms(short_term_batch[-1]),
        },
    }

