from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List


@dataclass
class PlanStep:
    """실행 가능한 단일 동작 스텝을 표현합니다."""

    name: str
    args: Dict[str, Any]
    status: str = "pending"


def map_intent_to_plan(intent: Dict[str, Any]) -> List[PlanStep]:
    """의도 정보(task_family/slots)를 실행 스텝 목록으로 매핑합니다."""
    task_family = str(intent.get("task_family", "")).strip().lower()
    slots = intent.get("slots", {}) or {}

    if task_family == "trace_shape":
        shape = str(slots.get("shape", "")).strip().lower()
        size = float(slots.get("size", 2.0))
        if shape == "rectangle":
            return _rectangle_plan(size=size)
        return []

    if task_family == "goto":
        if "x" not in slots or "y" not in slots:
            return []
        x = float(slots["x"])
        y = float(slots["y"])
        theta = float(slots.get("theta", 0.0))
        return [PlanStep(name="goto", args={"x": x, "y": y, "theta": theta})]

    if task_family == "rotate":
        degrees = float(slots.get("degrees", 90.0))
        direction = str(slots.get("direction", "left")).lower()
        return [PlanStep(name="rotate", args={"degrees": degrees, "direction": direction})]

    if task_family == "move_object":
        size = float(slots.get("size", 2.0))
        from_x = float(slots.get("from_x", 2.0))
        from_y = float(slots.get("from_y", 2.0))
        to_x = float(slots.get("to_x", 7.0))
        to_y = float(slots.get("to_y", 7.0))
        return [
            PlanStep(name="draw_box", args={"x": from_x, "y": from_y, "size": size}),
            PlanStep(name="clear_canvas", args={}),
            PlanStep(name="draw_box", args={"x": to_x, "y": to_y, "size": size}),
        ]

    return []


def _rectangle_plan(size: float) -> List[PlanStep]:
    steps: List[PlanStep] = []
    for _ in range(4):
        steps.append(PlanStep(name="forward", args={"distance": size}))
        steps.append(PlanStep(name="rotate", args={"degrees": 90.0, "direction": "left"}))
    return steps
