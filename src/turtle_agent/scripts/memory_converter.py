from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from command_logger import build_command_log_path
from memory_modules.memory_io import (
    append_jsonl,
    build_long_term_path,
    build_short_term_path,
    normalize_command_rows,
    parse_context_from_log_path,
    read_jsonl,
    resolve_memory_root,
)
from memory_modules.memory_long_term import (
    collect_collision_evidence,
    create_long_term_record,
    select_compression_batch,
)
from memory_modules.memory_short_term import (
    BaseShortTermModeAdapter,
    ControlModeMemoryAdapter,
    ShortTermCreateInput,
    SingleModeMemoryAdapter,
    _filter_collision_rows_for_turtle,
    create_short_term_record,
    finalize_short_term_record,
    update_short_term_record,
)
from pose_logger import build_collision_log_path, build_location_log_path, resolve_log_root


class MemoryConverter:
    """이동 로그를 단기/장기 기억 스키마로 변환해 저장합니다."""

    def __init__(self, memory_root: Optional[Path] = None) -> None:
        self._memory_root = memory_root or resolve_memory_root()
        self._pending_short_terms: Dict[Tuple[str, str, str], Dict[str, Any]] = {}

    @staticmethod
    def create_short_term_record(
        *,
        session_id: str,
        query_id: str,
        turtle_id: str,
        raw_text: str,
        intent: str,
        constraints: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        return create_short_term_record(
            session_id=session_id,
            query_id=query_id,
            turtle_id=turtle_id,
            raw_text=raw_text,
            intent=intent,
            constraints=constraints,
        )

    @staticmethod
    def update_short_term_record(
        record: Dict[str, Any],
        *,
        plan_step: Optional[Dict[str, Any]] = None,
        execution_step: Optional[Dict[str, Any]] = None,
        collision_event: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        return update_short_term_record(
            record,
            plan_step=plan_step,
            execution_step=execution_step,
            collision_event=collision_event,
        )

    @staticmethod
    def finalize_short_term_record(
        record: Dict[str, Any],
        *,
        finalized_at_unix_ms: int,
        start_pose: Dict[str, float],
        final_pose: Dict[str, float],
        success: bool,
        terminal_reason: str,
    ) -> Dict[str, Any]:
        return finalize_short_term_record(
            record,
            finalized_at_unix_ms=finalized_at_unix_ms,
            start_pose=start_pose,
            final_pose=final_pose,
            success=success,
            terminal_reason=terminal_reason,
        )

    def convert_session(
        self,
        *,
        date_str: str,
        session_id: Optional[str] = None,
        turtle_id: Optional[str] = None,
        test_case_id: Optional[str] = None,
        log_root: Optional[Path] = None,
        write_long_term: bool = False,
        mode: str = "single",
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

        parsed_session, parsed_turtle = parse_context_from_log_path(location_path)
        resolved_session = session_id or parsed_session
        resolved_turtle = turtle_id or parsed_turtle
        if not resolved_session or not resolved_turtle:
            raise ValueError("session_id와 turtle_id를 확인할 수 없습니다.")

        collision_path = build_collision_log_path(log_root, date_str, resolved_session)
        location_rows = read_jsonl(location_path)
        command_rows = normalize_command_rows(read_jsonl(command_path))
        collision_rows = _filter_collision_rows_for_turtle(
            read_jsonl(collision_path), resolved_turtle
        )

        adapter: BaseShortTermModeAdapter
        if str(mode).strip().lower() == "control":
            adapter = ControlModeMemoryAdapter()
        else:
            adapter = SingleModeMemoryAdapter()
        short_term_records = adapter.build_records(
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

    def begin_query_short_term(
        self,
        *,
        session_id: str,
        query_id: str,
        turtle_id: str,
        raw_text: str,
        intent: str,
        constraints: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        key = (session_id, query_id, turtle_id)
        rec = self.create_short_term_record(
            session_id=session_id,
            query_id=query_id,
            turtle_id=turtle_id,
            raw_text=raw_text,
            intent=intent,
            constraints=constraints,
        )
        self._pending_short_terms[key] = rec
        return rec

    def update_query_short_term(
        self,
        *,
        session_id: str,
        query_id: str,
        turtle_id: str,
        plan_step: Optional[Dict[str, Any]] = None,
        execution_step: Optional[Dict[str, Any]] = None,
        collision_event: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        key = (session_id, query_id, turtle_id)
        if key not in self._pending_short_terms:
            raise KeyError("pending short-term record not found")
        rec = self._pending_short_terms[key]
        return self.update_short_term_record(
            rec,
            plan_step=plan_step,
            execution_step=execution_step,
            collision_event=collision_event,
        )

    def finalize_query_short_term(
        self,
        *,
        session_id: str,
        query_id: str,
        turtle_id: str,
        test_case_id: str,
        finalized_at_unix_ms: int,
        start_pose: Dict[str, float],
        final_pose: Dict[str, float],
        success: bool,
        terminal_reason: str,
    ) -> Dict[str, int]:
        key = (session_id, query_id, turtle_id)
        if key not in self._pending_short_terms:
            raise KeyError("pending short-term record not found")
        rec = self.finalize_short_term_record(
            self._pending_short_terms.pop(key),
            finalized_at_unix_ms=finalized_at_unix_ms,
            start_pose=start_pose,
            final_pose=final_pose,
            success=success,
            terminal_reason=terminal_reason,
        )
        short_path = build_short_term_path(self._memory_root, session_id, test_case_id)
        append_jsonl(short_path, rec)
        return {"short_term_written": 1, "long_term_written": 0}

    def _load_session_short_rows(self, session_id: str) -> List[Dict[str, Any]]:
        session_dir = self._memory_root / "short_term" / session_id
        if not session_dir.exists():
            return []
        rows: List[Dict[str, Any]] = []
        for path in sorted(session_dir.glob("short_testid_*.jsonl")):
            rows.extend(read_jsonl(path))
        return rows

    def finalize_session(self, *, session_id: str, turtle_id: str) -> int:
        short_rows = self._load_session_short_rows(session_id)
        batch = select_compression_batch(short_rows)
        if len(batch) < 5:
            return 0
        long_rec = self.create_long_term_record(batch[:5], session_id, turtle_id)
        long_path = build_long_term_path(self._memory_root, session_id)
        append_jsonl(long_path, long_rec)
        return 1

    def create_long_term_record(
        self,
        short_term_batch: List[Dict[str, Any]],
        session_id: str,
        turtle_id: str,
    ) -> Dict[str, Any]:
        return create_long_term_record(short_term_batch, session_id, turtle_id)

    def _build_compressed_long_record(
        self,
        short_term_batch: List[Dict[str, Any]],
        session_id: str,
        turtle_id: str,
    ) -> Dict[str, Any]:
        return self.create_long_term_record(short_term_batch, session_id, turtle_id)

    @staticmethod
    def _collect_collision_evidence(
        short_term_batch: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        return collect_collision_evidence(short_term_batch)


__all__ = [
    "MemoryConverter",
    "ShortTermCreateInput",
    "build_long_term_path",
    "build_short_term_path",
    "parse_context_from_log_path",
    "resolve_memory_root",
]
