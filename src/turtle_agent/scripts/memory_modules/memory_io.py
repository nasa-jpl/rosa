from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def resolve_memory_root() -> Path:
    """기억 저장 루트 경로(`scripts/memory`)를 반환합니다."""
    return (Path(__file__).resolve().parent.parent / "memory").resolve()


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

