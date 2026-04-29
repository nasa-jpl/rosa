from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def infer_query_context(query: str) -> Dict[str, Any]:
    text = str(query or "").strip()
    # IME/오타: 선두 한글 자모 짧은 접두 + 공백 후 영문 명령인 경우 접두 제거 (예: "ㅇ draw ...")
    m_prefix = re.match(r"^([\u3131-\u318e]{1,3})\s+(.*)$", text)
    if m_prefix and re.match(r"^[A-Za-z]", m_prefix.group(2)):
        text = m_prefix.group(2).strip()
    lowered = text.lower()
    slots: Dict[str, str] = {}

    # English style: "from A to B"
    m_en = re.search(r"\bfrom\s+([A-Za-z0-9_-]+)\s+to\s+([A-Za-z0-9_-]+)\b", lowered)
    if m_en:
        slots["from"] = m_en.group(1).upper()
        slots["to"] = m_en.group(2).upper()

    # Korean style: "A에서 B로 가"
    m_ko = re.search(r"([A-Za-z0-9_-]+)\s*에서\s*([A-Za-z0-9_-]+)\s*로", text)
    if m_ko:
        slots["from"] = m_ko.group(1).upper()
        slots["to"] = m_ko.group(2).upper()

    task_family = "natural_language_query"
    # NOTE: 현재 retrieval 품질 검증 범위가 navigation 중심이라 navigate 분기를 명시적으로 유지한다.
    # intent_norm(task_family=navigate, from/to slots)과 키를 맞춰 recall 일관성을 확보하는 목적이며,
    # 추후 trace_shape/rotate 확장 시에는 task별 parser/score 전략을 분리할 계획이다.
    # 좌표 기반 이동/선분 (memory 매칭을 navigate와 맞춤 — long intent_norm 과 일치시키기 위함)
    if re.search(
        r"\b(?:draw\s+(?:a\s+)?line|line)\s+to\s+(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)",
        lowered,
    ):
        task_family = "navigate"
    elif re.search(
        r"\b(?:move\s+back\s+to|return\s+to)\s+(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)",
        lowered,
    ):
        task_family = "navigate"
    elif slots.get("from") and slots.get("to"):
        task_family = "navigate"
    elif any(
        token in lowered
        for token in (
            "go to",
            "move to",
            "move back to",
            "return to",
            "goto",
            "이동",
            "가줘",
            "가 ",
            "로 가",
        )
    ):
        task_family = "navigate"
    elif any(token in lowered for token in ("star", "pentagram", "별", "오각별")):
        task_family = "trace_shape"
    elif any(token in lowered for token in ("rotate", "turn", "회전")):
        task_family = "rotate"

    if task_family == "navigate" and slots.get("from") and slots.get("to"):
        experience_key = f"navigate|from:{slots['from']}|to:{slots['to']}"
    else:
        experience_key = task_family

    return {
        "task_family": task_family,
        "slots": slots,
        "experience_key": experience_key,
    }


def load_long_term_records(memory_root: Path, turtle_id: str) -> List[Dict[str, Any]]:
    long_dir = memory_root / "long_term"
    if not long_dir.exists():
        return []
    out: List[Dict[str, Any]] = []
    for path in sorted(long_dir.glob("long_sessionid_*.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            row = json.loads(stripped)
            if str(row.get("turtle_id", "")) != str(turtle_id):
                continue
            out.append(row)
    return out


def _record_context(row: Dict[str, Any]) -> Dict[str, Any]:
    payload = row.get("payload", {})
    operation = payload.get("operation", {})
    intent_norm = operation.get("intent_norm", {})
    task_family = str(intent_norm.get("task_family", ""))
    slots = intent_norm.get("slots", {})
    if not isinstance(slots, dict):
        slots = {}
    from_slot = str(slots.get("from", "")).upper() if slots.get("from") else ""
    to_slot = str(slots.get("to", "")).upper() if slots.get("to") else ""
    key = task_family
    if task_family == "navigate" and from_slot and to_slot:
        key = f"navigate|from:{from_slot}|to:{to_slot}"
    return {
        "task_family": task_family,
        "slots": slots,
        "experience_key": key,
    }


def _safe_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number):
        return None
    return number


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _quality_band(row: Dict[str, Any]) -> str:
    payload = row.get("payload", {})
    evidence = payload.get("evidence", {})
    outcome = payload.get("outcome", {})
    success = bool(outcome.get("success", False))
    success_rate = _safe_float(evidence.get("success_rate"))
    collisions = _safe_int(evidence.get("collision_enter_count"), 0)
    if success and (success_rate is None or success_rate >= 0.8) and collisions <= 1:
        return "high"
    if (not success) or collisions >= 3 or (success_rate is not None and success_rate < 0.4):
        return "low"
    return "mid"


def _slot_specificity(record_ctx: Dict[str, Any]) -> int:
    slots = record_ctx.get("slots", {})
    has_from = bool(str(slots.get("from", "")).strip())
    has_to = bool(str(slots.get("to", "")).strip())
    return int(has_from) + int(has_to)


def _is_intent_match(query_ctx: Dict[str, Any], record_ctx: Dict[str, Any]) -> bool:
    task_family = str(query_ctx.get("task_family", "")).strip()
    if not task_family:
        return False
    return task_family == str(record_ctx.get("task_family", "")).strip()


def _score_record(query_ctx: Dict[str, Any], record_ctx: Dict[str, Any], quality: str) -> int:
    score = 0
    if query_ctx["experience_key"] and query_ctx["experience_key"] == record_ctx["experience_key"]:
        score += 35
    if query_ctx["task_family"] and query_ctx["task_family"] == record_ctx["task_family"]:
        score += 50
    q_slots = query_ctx.get("slots", {})
    r_slots = record_ctx.get("slots", {})
    if q_slots.get("from") and str(r_slots.get("from", "")).upper() == q_slots.get("from"):
        score += 20
    if q_slots.get("to") and str(r_slots.get("to", "")).upper() == q_slots.get("to"):
        score += 20
    # NOTE: 피드백에 따라 "성공 사례 우선" 대신 "실패 사례 우선" retrieval 정책을 적용한다.
    # 당장은 task-agnostic 규칙으로 일반화하지 않고, navigation 실효성 검증을 우선한다.
    if quality == "low":
        score += 8
    elif quality == "high":
        score -= 8
    return score


def _dedupe_key(record_ctx: Dict[str, Any]) -> str:
    slots = record_ctx.get("slots", {})
    from_slot = str(slots.get("from", "")).upper().strip()
    to_slot = str(slots.get("to", "")).upper().strip()
    return f"{record_ctx.get('task_family', '')}|from:{from_slot}|to:{to_slot}"


def _record_sort_tuple(score: int, record_ctx: Dict[str, Any], row: Dict[str, Any]) -> Tuple[int, int, int, float, int]:
    payload = row.get("payload", {})
    evidence = payload.get("evidence", {})
    success_rate = _safe_float(evidence.get("success_rate")) or 0.0
    collisions = _safe_int(evidence.get("collision_enter_count"), 0)
    created_at = _safe_int(row.get("meta", {}).get("created_at_unix_ms"), 0)
    # 정렬 우선순위(내림차순): score > slot_specificity > 충돌 많음 > 성공률 낮음 > 최신성
    return (score, _slot_specificity(record_ctx), collisions, -success_rate, created_at)


def build_memory_context(query: str, records: List[Dict[str, Any]], top_k: int = 3) -> Tuple[str, int]:
    query_ctx = infer_query_context(query)
    max_k = max(0, int(top_k))
    if max_k == 0:
        return "", 0
    min_score = 45
    deduped: Dict[str, Tuple[Tuple[int, int, int, float, int], Dict[str, Any], str, Dict[str, Any], int]] = {}
    for row in records:
        record_ctx = _record_context(row)
        if not _is_intent_match(query_ctx, record_ctx):
            continue
        quality = _quality_band(row)
        score = _score_record(query_ctx, record_ctx, quality)
        if score < min_score:
            continue
        sort_tuple = _record_sort_tuple(score, record_ctx, row)
        key = _dedupe_key(record_ctx)
        prev = deduped.get(key)
        if prev is None or sort_tuple > prev[0]:
            deduped[key] = (sort_tuple, row, quality, record_ctx, score)
    if not deduped:
        return "", 0
    ranked = sorted(deduped.values(), key=lambda item: item[0], reverse=True)
    selected = ranked[: min(max_k, len(ranked))]
    lines: List[str] = []
    do_lines: List[str] = []
    dont_lines: List[str] = []
    for idx, (_, row, quality, _, score) in enumerate(selected, start=1):
        payload = row.get("payload", {})
        op = payload.get("operation", {})
        goal_text = str(op.get("nl_goal", {}).get("text", ""))
        evidence = payload.get("evidence", {})
        collisions = _safe_int(evidence.get("collision_enter_count"), 0)
        success_rate = _safe_float(evidence.get("success_rate"))
        lines.append(
            f"{idx}. score={score} quality={quality} goal={goal_text[:120]} / collision_enter_count={collisions} / success_rate={success_rate}"
        )
        raw_lessons = payload.get("lessons")
        if isinstance(raw_lessons, list):
            for lesson in raw_lessons:
                if isinstance(lesson, str) and lesson.strip():
                    if quality == "low":
                        dont_lines.append(f"[memory {idx}] {lesson.strip()}")
                    else:
                        do_lines.append(f"[memory {idx}] {lesson.strip()}")
    policy_lines = ["MUST: Use selected memory as execution policy, not commentary."]
    if query_ctx.get("task_family") == "navigate":
        policy_lines.append("MUST: Avoid single long straight moves when uncertainty exists.")
    if dont_lines:
        policy_lines.append("MUST: Treat DON'T items as forbidden patterns for this query.")
    policy_lines = [
        line for line in policy_lines if line.strip()
    ]
    context = "Memory policy (strict):\n"
    context += "\n".join(f"- {line}" for line in policy_lines)
    context += "\n\nMemory evidence:\n"
    context += "\n".join(lines)
    if do_lines:
        context += "\n\nDO rules:\n" + "\n".join(f"- {line}" for line in do_lines[:6])
    if dont_lines:
        context += "\n\nDON'T rules:\n" + "\n".join(f"- {line}" for line in dont_lines[:6])
    return context, len(selected)
