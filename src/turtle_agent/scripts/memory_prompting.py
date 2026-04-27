from __future__ import annotations

import json
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


def _score_record(query_ctx: Dict[str, Any], record_ctx: Dict[str, Any]) -> int:
    score = 0
    if query_ctx["experience_key"] and query_ctx["experience_key"] == record_ctx["experience_key"]:
        score += 100
    if query_ctx["task_family"] and query_ctx["task_family"] == record_ctx["task_family"]:
        score += 30
    q_slots = query_ctx.get("slots", {})
    r_slots = record_ctx.get("slots", {})
    if q_slots.get("from") and str(r_slots.get("from", "")).upper() == q_slots.get("from"):
        score += 10
    if q_slots.get("to") and str(r_slots.get("to", "")).upper() == q_slots.get("to"):
        score += 10
    return score


def build_memory_context(query: str, records: List[Dict[str, Any]], top_k: int = 2) -> Tuple[str, int]:
    query_ctx = infer_query_context(query)
    ranked: List[Tuple[int, Dict[str, Any]]] = []
    for row in records:
        record_ctx = _record_context(row)
        score = _score_record(query_ctx, record_ctx)
        if score <= 0:
            continue
        ranked.append((score, row))
    if not ranked:
        return "", 0
    ranked.sort(key=lambda item: item[0], reverse=True)
    selected = [row for _, row in ranked[: max(1, int(top_k))]]
    lines: List[str] = []
    lesson_lines: List[str] = []
    max_collision_enter = 0
    for idx, row in enumerate(selected, start=1):
        payload = row.get("payload", {})
        op = payload.get("operation", {})
        goal_text = str(op.get("nl_goal", {}).get("text", ""))
        evidence = payload.get("evidence", {})
        collisions = int(evidence.get("collision_enter_count", 0))
        max_collision_enter = max(max_collision_enter, collisions)
        success_rate = evidence.get("success_rate")
        lines.append(
            f"{idx}. goal={goal_text[:160]} / collision_enter_count={collisions} / success_rate={success_rate}"
        )
        raw_lessons = payload.get("lessons")
        if isinstance(raw_lessons, list):
            for _, lesson in enumerate(raw_lessons):
                if isinstance(lesson, str) and lesson.strip():
                    lesson_lines.append(f"[memory {idx}] {lesson.strip()}")
    policy_lines = [
        "MUST: Use memory as executable policy for this query, not as optional commentary.",
        "MUST: Prefer multi-step segmented movement over a single direct movement command.",
    ]
    if query_ctx.get("task_family") == "navigate" and max_collision_enter > 0:
        policy_lines.extend(
            [
                "MUST: Do not choose a single straight-line path for this navigation.",
                "MUST: If a move attempt causes collision, immediately replan with smaller segmented moves.",
            ]
        )
    context = (
        "Memory policy (strict):\n"
        + "\n".join(f"- {line}" for line in policy_lines)
        + "\n\nMemory evidence:\n"
        + "\n".join(lines)
    )
    if lesson_lines:
        context += "\n\nMemory lessons:\n" + "\n".join(f"- {line}" for line in lesson_lines)
    return context, len(selected)
