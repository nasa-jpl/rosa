from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Set, TextIO

from pose_logger import build_log_dir, resolve_log_root


def build_command_log_path(
    log_root: Path, date_str: str, session_id: str, turtle_id: str
) -> Path:
    """명령 로그 파일 경로(`logs/<date>/<session>/<turtle>/command.jsonl`)를 반환합니다."""
    return build_log_dir(log_root, date_str, session_id, turtle_id) / "command.jsonl"


class CommandLogger:
    """의도/스킬 실행 레코드를 `command.jsonl`에 누적 기록합니다."""

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
        self._files: Dict[str, TextIO] = {}
        self._logged_intent_turtles: Set[str] = set()
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

    def log_intent(self, turtle_id: str, intent: Dict[str, Any]) -> None:
        """세션 시작 의도(intent)를 터틀별로 1회만 기록합니다."""
        turtle_id = str(turtle_id).replace("/", "")
        if not turtle_id:
            return
        with self._lock:
            if self._closed or turtle_id in self._logged_intent_turtles:
                return
            rec = {
                "type": "intent",
                "t_ms": int(time.time() * 1000),
                "task_family": str(intent.get("task_family", "unknown")),
                "slots": intent.get("slots", {}),
            }
            self._write_record_locked(turtle_id, rec)
            self._logged_intent_turtles.add(turtle_id)

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
            rec = {
                "type": "skill",
                "t_ms": int(time.time() * 1000),
                "skill": str(skill),
                "args": args,
                "status": status,
                "result": result,
            }
            self._write_record_locked(turtle_id, rec)

    def _write_record_locked(self, turtle_id: str, record: Dict[str, Any]) -> None:
        fp = self._files.get(turtle_id)
        if fp is None:
            path = self.path_for(turtle_id)
            path.parent.mkdir(parents=True, exist_ok=True)
            fp = open(path, "a", encoding="utf-8")
            self._files[turtle_id] = fp

        fp.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        fp.flush()

    def close(self) -> None:
        """열린 로그 파일 핸들을 모두 flush/close 합니다."""
        with self._lock:
            self._closed = True
            files = tuple(self._files.values())
            self._files.clear()

        for fp in files:
            try:
                fp.flush()
            except OSError:
                pass
            try:
                fp.close()
            except OSError:
                pass
