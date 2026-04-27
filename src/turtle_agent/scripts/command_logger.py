from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

from pose_logger import build_log_dir, resolve_log_root


def build_command_log_path(
    log_root: Path, date_str: str, session_id: str, turtle_id: str
) -> Path:
    """명령 로그 파일 경로(`logs/<date>/<session>/<turtle>/command.jsonl`)를 반환합니다."""
    return build_log_dir(log_root, date_str, session_id, turtle_id) / "command.jsonl"


class CommandLogger:
    """의도/스킬 실행 레코드를 세션 단일 JSON 객체로 기록합니다."""

    def __init__(
        self,
        *,
        log_root: Optional[Path] = None,
        session_id: Optional[str] = None,
        date_str: Optional[str] = None,
    ) -> None:
        """세션 단위 command logger를 초기화합니다."""
        self._log_root = log_root or resolve_log_root()
        self._session_id = session_id or str(uuid.uuid4())
        self._date_str = date_str or datetime.now().strftime("%Y-%m-%d")
        self._lock = threading.Lock()
        self._records_by_turtle: Dict[str, Dict[str, Any]] = {}
        self._closed = False

    def path_for(self, turtle_id: str) -> Path:
        """주어진 터틀 ID의 command 로그 파일 경로를 생성합니다."""
        turtle_id = str(turtle_id).replace("/", "")
        return build_command_log_path(
            self._log_root,
            self._date_str,
            self._session_id,
            turtle_id,
        )

    @property
    def session_id(self) -> str:
        """현재 command 로그 세션 ID를 반환합니다."""
        return self._session_id

    @property
    def date_str(self) -> str:
        """현재 command 로그 날짜 키(YYYY-MM-DD)를 반환합니다."""
        return self._date_str

    def log_intent(self, turtle_id: str, intent: Dict[str, Any]) -> None:
        """현재 질의의 의도(intent)를 기록하고 기존 스킬 목록을 초기화합니다."""
        turtle_id = str(turtle_id).replace("/", "")
        if not turtle_id:
            return
        with self._lock:
            if self._closed:
                return
            session_record = self._get_or_create_record_locked(turtle_id)
            rec = {
                "type": "intent",
                "t_ms": int(time.time() * 1000),
                "task_family": str(intent.get("task_family", "unknown")),
                "slots": intent.get("slots", {}),
            }
            if "natural_language" in intent:
                rec["natural_language"] = intent.get("natural_language")
            if "query" in intent:
                rec["query"] = intent.get("query")
            session_record["intent"] = rec
            # One query = one command snapshot.
            session_record["skills"] = []
            self._flush_record_locked(turtle_id)

    def log_skill(
        self,
        turtle_id: str,
        *,
        skill: str,
        args: Dict[str, Any],
        status: str,
        result: Any,
    ) -> None:
        """스킬 호출 결과를 터틀별 command 로그에 기록합니다."""
        turtle_id = str(turtle_id).replace("/", "")
        if not turtle_id:
            return
        with self._lock:
            if self._closed:
                return
            session_record = self._get_or_create_record_locked(turtle_id)
            rec = {
                "type": "skill",
                "t_ms": int(time.time() * 1000),
                "skill": str(skill),
                "args": args,
                "status": status,
                "result": result,
            }
            session_record["skills"].append(rec)
            self._flush_record_locked(turtle_id)

    def _get_or_create_record_locked(self, turtle_id: str) -> Dict[str, Any]:
        record = self._records_by_turtle.get(turtle_id)
        if record is not None:
            return record

        record = {
            "session_id": self._session_id,
            "turtle_id": turtle_id,
            "intent": None,
            "skills": [],
        }
        self._records_by_turtle[turtle_id] = record
        return record

    def _flush_record_locked(self, turtle_id: str) -> None:
        path = self.path_for(turtle_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = self._records_by_turtle[turtle_id]
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(json.dumps(payload, separators=(",", ":"), ensure_ascii=False) + "\n")

    def close(self) -> None:
        """close 호출 이후 로그 기록을 중단합니다."""
        with self._lock:
            self._closed = True
            self._records_by_turtle.clear()
