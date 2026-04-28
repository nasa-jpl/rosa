from __future__ import annotations

import json
import logging
import os
import re
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from command_logger import build_command_log_path
from pose_logger import (
    build_collision_log_path,
    build_location_log_path,
    build_log_dir,
    resolve_log_root,
)


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


def _collision_row_to_trace_event(row: Dict[str, Any]) -> Dict[str, Any]:
    """collision.jsonl 한 줄을 execution_trace.steps[].events 항목으로 변환합니다."""
    et = str(row.get("event_type", "") or "")
    out: Dict[str, Any] = {
        "type": "collision",
        "phase": et,
        "event_type": et,
        "collision_type": row.get("collision_type"),
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
    """
    충돌 시각을 스킬 로그 시간 경계로 버킷합니다.
    - 첫 스킬 이전~(intent, t0]: 0번 스킬
    - (t_{i-1}, t_i]: i번 스킬
    - 마지막 스킬 시각 이후: 마지막 스킬
    """
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


_LESSONS_TARGET_LINES = 3
_LOGGER = logging.getLogger(__name__)


def _parse_lesson_lines(text: str, *, max_lines: int = _LESSONS_TARGET_LINES) -> List[str]:
    """LLM 응답에서 비어 있지 않은 문장 줄만 최대 max_lines개 추출합니다."""
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


def _bucket_collision_events_by_skill_index(
    collision_rows: List[Dict[str, Any]],
    skills: List[Dict[str, Any]],
) -> List[List[Dict[str, Any]]]:
    """스킬 인덱스별로 충돌 trace 이벤트 목록을 만듭니다."""
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

        collision_path = build_collision_log_path(
            log_root, date_str, resolved_session
        )

        location_rows = read_jsonl(location_path)
        command_rows = read_jsonl(command_path)
        command_rows = normalize_command_rows(command_rows)
        collision_rows = _filter_collision_rows_for_turtle(
            read_jsonl(collision_path), resolved_turtle
        )
        short_term_records = self._build_short_term_records(
            location_rows=location_rows,
            command_rows=command_rows,
            session_id=resolved_session,
            turtle_id=resolved_turtle,
            collision_rows=collision_rows,
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
        batch = self._select_compression_batch(short_rows)
        if len(batch) < 5:
            return 0
        long_rec = self._build_compressed_long_record(
            short_term_batch=batch[:5],
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
        collision_by_skill = _bucket_collision_events_by_skill_index(
            collision_rows or [], skills
        )

        records: List[Dict[str, Any]] = []
        for idx, skill in enumerate(skills):
            t_ms = int(skill.get("t_ms", 0))
            plan_steps = [
                {"name": s.get("skill", ""), "status": s.get("status", "pending")}
                for s in skills[: idx + 1]
            ]
            trace_steps = []
            for j, s in enumerate(skills[: idx + 1]):
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
                        "events": list(collision_by_skill[j]),
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
    def _is_bootstrap_query_record(short_row: Dict[str, Any]) -> bool:
        goal = str(short_row.get("active_goal", {}).get("natural_language", "")).strip().lower()
        if not goal:
            return False
        if not any(token in goal for token in ("go to", "move to", "goto", "teleport")):
            return False
        # 좌표 이동 초기화 쿼리(예: "go to 1, 5")를 bootstrap으로 간주
        return bool(re.search(r"\d+(?:\.\d+)?\s*,\s*\d+(?:\.\d+)?", goal))

    def _select_compression_batch(self, short_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        long 압축에 사용할 short 배치를 선택합니다.
        - 기본: 앞에서 5개 사용
        - 예외: 6개 이상이고 첫 레코드가 bootstrap 쿼리면 첫 1개를 제외
        """
        if len(short_rows) >= 6 and self._is_bootstrap_query_record(short_rows[0]):
            return short_rows[1:]
        return short_rows

    @staticmethod
    def _infer_task_family(query: str) -> str:
        q = (query or "").lower()
        if "pentagram" in q or "star" in q:
            return "trace_shape"
        if "rotate" in q or "turn" in q:
            return "rotate"
        if any(token in q for token in ("go to", "move to", "goto", "teleport", "이동", "가줘", "로 가")):
            return "goto"
        return "natural_language_query"

    @staticmethod
    def _steps_equivalent(a: Dict[str, Any], b: Dict[str, Any]) -> bool:
        """같은 누적 trace 안에서 동일 스텝인지 판별합니다(t_ms + 첫 스킬명)."""
        if int(a.get("t_ms", 0)) != int(b.get("t_ms", 0)):
            return False
        ai = (a.get("skill_invocations") or [{}])[0]
        bi = (b.get("skill_invocations") or [{}])[0]
        return str(ai.get("skill", "")) == str(bi.get("skill", ""))

    @staticmethod
    def _is_strict_prefix_steps(
        prev_steps: List[Dict[str, Any]], curr_steps: List[Dict[str, Any]]
    ) -> bool:
        """curr가 prev 스텝 목록을 앞에서부터 그대로 이어 받은 누적인지 여부."""
        if len(curr_steps) < len(prev_steps):
            return False
        for i in range(len(prev_steps)):
            if not MemoryConverter._steps_equivalent(prev_steps[i], curr_steps[i]):
                return False
        return True

    @staticmethod
    def _append_action_trace_from_short_row(
        action_trace: List[Dict[str, Any]],
        prev_steps: Optional[List[Dict[str, Any]]],
        curr_steps: List[Dict[str, Any]],
    ) -> None:
        """
        한 short 행의 execution_trace.steps를 action_trace 항목으로 펼칩니다.
        - 이전 행과 누적 관계(접두 일치 + 길이 증가 또는 동일)면 새로 붙은 스텝만 추가합니다.
        - 그렇지 않으면 이번 행의 전체 스텝을 한 에피소드로 추가합니다.
        - 누적으로 델타가 비었는데 스텝 길이가 같으면(동일 스냅샷 반복) 마지막 스텝을 한 번 더 넣어
          서로 다른 short 레코드(파일)가 에피소드로 유지되도록 합니다.
        """

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

        if MemoryConverter._is_strict_prefix_steps(prev_steps, curr_steps):
            delta = curr_steps[len(prev_steps) :]
            if delta:
                for step in delta:
                    emit_step(step)
            else:
                emit_step(curr_steps[-1])
            return

        for step in curr_steps:
            emit_step(step)

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

    @staticmethod
    def _extract_context_from_action_trace(
        action_trace: List[Dict[str, Any]], fallback_goal: str
    ) -> Tuple[str, Dict[str, str]]:
        """action args(experience_key/query) 우선으로 task_family/slots를 추론합니다."""
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
                inferred = MemoryConverter._infer_task_family(query)
                if inferred == "goto":
                    return "navigate", {}
                return inferred, {}
        inferred_from_goal = MemoryConverter._infer_task_family(fallback_goal)
        if inferred_from_goal == "goto":
            return "navigate", {}
        return inferred_from_goal, {}

    @staticmethod
    def _collision_event_fingerprint(event: Dict[str, Any]) -> Tuple[Any, ...]:
        """누적 short 레코드에서 동일 충돌이 여러 번 나타날 때 집계 중복을 막습니다."""
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

    @staticmethod
    def _collect_collision_evidence(short_term_batch: List[Dict[str, Any]]) -> Dict[str, Any]:
        collision_events = 0
        collision_enter_count = 0
        collision_obstacles: set[str] = set()
        seen_fp: set[Tuple[Any, ...]] = set()
        for short in short_term_batch:
            steps = short.get("execution_trace", {}).get("steps", [])
            for step in steps:
                for event in step.get("events", []) or []:
                    if not isinstance(event, dict):
                        continue
                    row_kind = str(event.get("type", "")).lower()
                    phase = str(event.get("phase") or event.get("event") or "").lower()
                    subtype = str(event.get("event_type") or "").lower()
                    # short에 붙는 이벤트는 보통 type=collision, enter/stay/exit 는 event_type(#16)
                    if "collision" not in row_kind and "collision" not in phase:
                        continue
                    fp = MemoryConverter._collision_event_fingerprint(event)
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

    @staticmethod
    def _lessons_context_payload(
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

    @staticmethod
    def _fallback_lessons_lines(
        *,
        collision_ev: Dict[str, Any],
        task_family: str,
        first_goal: str,
        all_success: bool,
        action_trace: List[Dict[str, Any]],
    ) -> List[str]:
        """API 없이 규칙 기반으로 3문장 교훈을 만듭니다."""
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

    def _summarize_lessons_with_llm(
        self,
        short_term_batch: List[Dict[str, Any]],
        *,
        collision_ev: Dict[str, Any],
        task_family: str,
        action_trace: List[Dict[str, Any]],
        all_success: bool,
        first_goal: str,
        queries: List[str],
    ) -> List[str]:
        """
        short 압축 배치를 바탕으로 교훈 3문장을 생성합니다.
        ``MEMORY_LESSONS_LLM=0`` 이면 LLM을 쓰지 않고 규칙 기반만 사용합니다.
        """
        fb = self._fallback_lessons_lines(
            collision_ev=collision_ev,
            task_family=task_family,
            first_goal=first_goal or "(미상)",
            all_success=all_success,
            action_trace=action_trace,
        )
        if os.getenv("MEMORY_LESSONS_LLM", "1").strip().lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return fb

        payload = self._lessons_context_payload(
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
                "당신은 거북이 로봇의 단기 메모리(short-term) 요약을 읽고 "
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
                if i < len(parsed):
                    out.append(parsed[i])
                else:
                    out.append(fb[i])
            return out
        except Exception as exc:
            _LOGGER.warning("long-term lessons LLM unavailable, using fallback: %s", exc)
            return fb

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
        prev_trace_steps: Optional[List[Dict[str, Any]]] = None
        for short in short_term_batch:
            steps = short.get("execution_trace", {}).get("steps", [])
            if not steps:
                continue
            MemoryConverter._append_action_trace_from_short_row(
                action_trace, prev_trace_steps, steps
            )
            prev_trace_steps = steps

        all_success = all(item.get("status") == "success" for item in action_trace) if action_trace else False
        radii = self._extract_radius_series(action_trace)
        decay_ratio = 1.0
        if len(radii) >= 2 and radii[-2] > 0:
            decay_ratio = round(radii[-1] / radii[-2], 3)
        primary_skill = action_trace[0].get("skill", "unknown") if action_trace else "unknown"
        skill_sequence = [str(item.get("skill", "")) for item in action_trace if item.get("skill")]
        task_family, slots = self._extract_context_from_action_trace(action_trace, first_goal)
        collision_ev = self._collect_collision_evidence(short_term_batch)
        lessons_lines = self._summarize_lessons_with_llm(
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
                "created_at_unix_ms": int(short_term_batch[-1].get("clock", {}).get("unix_ms", 0)),
            },
        }
