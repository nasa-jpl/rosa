from __future__ import annotations

"""Memory schema contracts aligned with team decisions.

- #48: memory module split ownership
- #52: control-worker contracts
"""

from typing import Any, Dict, Literal, TypedDict


class WorkerTask(TypedDict):
    task_id: str
    prompt: str
    timeout: float
    priority: int


class WorkerResult(TypedDict):
    task_id: str
    status: Literal["success", "error", "timeout", "cancelled", "unknown"]
    result: Any
    error: str


MEMORY_SCHEMA_OWNERSHIP: Dict[str, str] = {
    "io": "memory_io.py",
    "short_term": "memory_short_term.py",
    "long_term": "memory_long_term.py",
}

