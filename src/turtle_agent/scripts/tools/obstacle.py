#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""LangChain tools for obstacle CRUD backed by ``ObstacleStore``.

These tools intentionally do not expose HTTP. ``TurtleAgent`` injects the same
in-process store that static map loading and future collision checks use.
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

from langchain.agents import tool

from obstacle_store import (
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleGeometry,
    ObstacleStore,
    SegmentsGeometry,
)

_STORE: Optional[ObstacleStore] = None


class ObstacleToolError(ValueError):
    """Invalid tool input or missing store configuration."""


def configure_obstacle_store(store: ObstacleStore) -> None:
    """Inject the process-local store used by obstacle tools."""
    global _STORE
    _STORE = store


def _require_store() -> ObstacleStore:
    if _STORE is None:
        raise ObstacleToolError("ObstacleStore is not configured.")
    return _STORE


def _coerce_float(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ObstacleToolError(f"{key} must be a number.")
    return float(value)


def _parse_geometry_json(geometry_json: str) -> ObstacleGeometry:
    try:
        raw = json.loads(geometry_json)
    except json.JSONDecodeError as e:
        raise ObstacleToolError(f"Invalid geometry_json: {e}") from e
    if not isinstance(raw, dict):
        raise ObstacleToolError("geometry_json must decode to a JSON object.")
    return _parse_geometry(raw)


def _parse_geometry(raw: Dict[str, Any]) -> ObstacleGeometry:
    gtype = raw.get("type")
    if not isinstance(gtype, str):
        raise ObstacleToolError("geometry.type must be a string.")
    gtype_l = gtype.strip().lower()
    if gtype_l == "circle":
        for key in ("cx", "cy", "r"):
            if key not in raw:
                raise ObstacleToolError(f"geometry.{key} is required for circle.")
        return CircleGeometry(
            _coerce_float(raw["cx"], "cx"),
            _coerce_float(raw["cy"], "cy"),
            _coerce_float(raw["r"], "r"),
        )
    if gtype_l == "aabb":
        for key in ("min_x", "min_y", "max_x", "max_y"):
            if key not in raw:
                raise ObstacleToolError(f"geometry.{key} is required for aabb.")
        return AabbGeometry(
            _coerce_float(raw["min_x"], "min_x"),
            _coerce_float(raw["min_y"], "min_y"),
            _coerce_float(raw["max_x"], "max_x"),
            _coerce_float(raw["max_y"], "max_y"),
        )
    if gtype_l == "segments":
        segments = raw.get("segments")
        if not isinstance(segments, list):
            raise ObstacleToolError("geometry.segments must be a list.")
        parsed = []
        for i, seg in enumerate(segments):
            if not isinstance(seg, list) or len(seg) != 2:
                raise ObstacleToolError(f"segments[{i}] must be [p1, p2].")
            p1, p2 = seg
            if not isinstance(p1, list) or len(p1) != 2:
                raise ObstacleToolError(f"segments[{i}][0] must be [x, y].")
            if not isinstance(p2, list) or len(p2) != 2:
                raise ObstacleToolError(f"segments[{i}][1] must be [x, y].")
            parsed.append(
                (
                    (_coerce_float(p1[0], "x1"), _coerce_float(p1[1], "y1")),
                    (_coerce_float(p2[0], "x2"), _coerce_float(p2[1], "y2")),
                )
            )
        return SegmentsGeometry(tuple(parsed))
    raise ObstacleToolError(
        f"unknown geometry.type {gtype!r}; expected circle, aabb, or segments."
    )


def _geometry_to_dict(geometry: ObstacleGeometry) -> Dict[str, Any]:
    if isinstance(geometry, CircleGeometry):
        return {"type": "circle", "cx": geometry.cx, "cy": geometry.cy, "r": geometry.r}
    if isinstance(geometry, AabbGeometry):
        return {
            "type": "aabb",
            "min_x": geometry.min_x,
            "min_y": geometry.min_y,
            "max_x": geometry.max_x,
            "max_y": geometry.max_y,
        }
    return {
        "type": "segments",
        "segments": [[[x1, y1], [x2, y2]] for (x1, y1), (x2, y2) in geometry.segments],
    }


@tool
def add_obstacle(
    obstacle_id: str,
    geometry_json: str,
    kind: str = "static",
    ttl_seconds: float = 0.0,
) -> str:
    """
    Add or replace an obstacle in the shared ObstacleStore.

    ``geometry_json`` uses the same shape as static map ``geometry`` entries:
    ``{"type":"circle","cx":1,"cy":2,"r":0.5}``,
    ``{"type":"aabb","min_x":1,"min_y":1,"max_x":2,"max_y":2}``, or
    ``{"type":"segments","segments":[[[0,0],[1,0]]]}``.

    ``kind`` is ``static`` for an obstacle that remains until removed, or
    ``temporary`` for an obstacle that expires after ``ttl_seconds``.
    """
    try:
        oid = obstacle_id.strip()
        if not oid:
            raise ObstacleToolError("obstacle_id must be a non-empty string.")
        kind_l = kind.strip().lower()
        if kind_l not in ("static", "temporary"):
            raise ObstacleToolError("kind must be 'static' or 'temporary'.")
        expires_at = None
        if kind_l == "temporary":
            ttl = _coerce_float(ttl_seconds, "ttl_seconds")
            if ttl <= 0:
                raise ObstacleToolError(
                    "ttl_seconds must be positive for temporary obstacles."
                )
            expires_at = time.monotonic() + ttl
        geometry = _parse_geometry_json(geometry_json)
        _require_store().upsert(
            Obstacle(
                id=oid,
                kind=kind_l,
                geometry=geometry,
                expires_at=expires_at,
            )
        )
        if kind_l == "temporary":
            return f"Added temporary obstacle '{oid}' with ttl_seconds={ttl}."
        return f"Added static obstacle '{oid}'."
    except (ObstacleToolError, ValueError) as e:
        return f"Failed to add obstacle: {e}"


@tool
def remove_obstacle(obstacle_id: str) -> str:
    """Remove an obstacle by id from the shared ObstacleStore."""
    try:
        oid = obstacle_id.strip()
        if not oid:
            raise ObstacleToolError("obstacle_id must be a non-empty string.")
        removed = _require_store().remove(oid)
        if removed:
            return f"Removed obstacle '{oid}'."
        return f"Obstacle '{oid}' was not found."
    except ObstacleToolError as e:
        return f"Failed to remove obstacle: {e}"


@tool
def list_obstacles(include_expired: bool = False) -> str:
    """List current obstacles from the shared ObstacleStore as JSON."""
    try:
        if include_expired:
            return (
                "Expired obstacles cannot be listed because ObstacleStore purges "
                "them during reads."
            )
        rows = []
        now = time.monotonic()
        for obstacle in _require_store().snapshot():
            expires_in = None
            if obstacle.expires_at is not None:
                expires_in = max(0.0, obstacle.expires_at - now)
            rows.append(
                {
                    "id": obstacle.id,
                    "kind": obstacle.kind,
                    "expires_in_seconds": expires_in,
                    "geometry": _geometry_to_dict(obstacle.geometry),
                }
            )
        return json.dumps({"obstacles": rows}, ensure_ascii=False, sort_keys=True)
    except ObstacleToolError as e:
        return f"Failed to list obstacles: {e}"
