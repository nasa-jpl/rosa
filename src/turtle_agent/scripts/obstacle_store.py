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

"""Thread-safe in-memory obstacle registry for collision checking (no ROS imports).

Each :class:`Obstacle` is immutable. Geometry variants align with
``collision_geometry`` so consumers can call overlap helpers without conversion:

- :class:`CircleGeometry` — use ``cx, cy, r`` with ``circles_overlap``,
  ``point_in_circle``, ``point_circle_distance``, ``segment_intersects_disc``.
- :class:`SegmentsGeometry` — ``segments`` is ``tuple[Segment, ...]`` (same
  ``Segment`` as in ``collision_geometry``); pass to ``any_segment_intersects_disc``.
- :class:`AabbGeometry` — ``min_x, min_y, max_x, max_y`` with
  ``circle_intersects_aabb`` (disc vs obstacle box).

``ObstacleStore`` uses :class:`threading.RLock`. :meth:`snapshot` returns a tuple
of obstacles consistent with a single lock acquisition (after purging expired
``temporary`` entries), suitable for one collision-evaluation pass.

Temporary-obstacle TTL uses :func:`time.monotonic` deadlines in
:attr:`Obstacle.expires_at`.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Dict, Literal, Optional, Tuple, Union

from collision_geometry import Segment

ObstacleKind = Literal["static", "temporary", "turtle"]


@dataclass(frozen=True)
class CircleGeometry:
    cx: float
    cy: float
    r: float


@dataclass(frozen=True)
class SegmentsGeometry:
    segments: Tuple[Segment, ...]


@dataclass(frozen=True)
class AabbGeometry:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


ObstacleGeometry = Union[CircleGeometry, SegmentsGeometry, AabbGeometry]


@dataclass(frozen=True)
class Obstacle:
    id: str
    kind: ObstacleKind
    geometry: ObstacleGeometry
    expires_at: Optional[float]


def _normalize_geometry(geometry: ObstacleGeometry) -> ObstacleGeometry:
    if isinstance(geometry, SegmentsGeometry):
        segs = tuple(
            ((float(x1), float(y1)), (float(x2), float(y2)))
            for (x1, y1), (x2, y2) in geometry.segments
        )
        return SegmentsGeometry(segs)
    if isinstance(geometry, CircleGeometry):
        return CircleGeometry(float(geometry.cx), float(geometry.cy), float(geometry.r))
    return AabbGeometry(
        float(geometry.min_x),
        float(geometry.min_y),
        float(geometry.max_x),
        float(geometry.max_y),
    )


def _validate_obstacle(obstacle: Obstacle) -> None:
    if obstacle.kind == "temporary":
        if obstacle.expires_at is None:
            raise ValueError(
                "temporary obstacles require expires_at (monotonic deadline)"
            )
    elif obstacle.expires_at is not None:
        raise ValueError("only temporary obstacles may set expires_at")


def _copy_obstacle_for_snapshot(obstacle: Obstacle) -> Obstacle:
    """Detach segment tuples so snapshots never share mutable views."""
    g = obstacle.geometry
    if isinstance(g, SegmentsGeometry):
        segs = tuple(
            ((float(x1), float(y1)), (float(x2), float(y2)))
            for (x1, y1), (x2, y2) in g.segments
        )
        g = SegmentsGeometry(segs)
        return replace(obstacle, geometry=g)
    return obstacle


class ObstacleStore:
    """Thread-safe store: upsert/remove/get/snapshot with temporary-obstacle expiry."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._items: Dict[str, Obstacle] = {}

    def upsert(self, obstacle: Obstacle) -> None:
        normalized = replace(obstacle, geometry=_normalize_geometry(obstacle.geometry))
        _validate_obstacle(normalized)
        with self._lock:
            self._purge_expired_unlocked()
            self._items[normalized.id] = normalized

    def remove(self, obstacle_id: str) -> bool:
        with self._lock:
            self._purge_expired_unlocked()
            if obstacle_id in self._items:
                del self._items[obstacle_id]
                return True
            return False

    def clear(self) -> None:
        with self._lock:
            self._items.clear()

    def get(self, obstacle_id: str) -> Optional[Obstacle]:
        with self._lock:
            self._purge_expired_unlocked()
            ob = self._items.get(obstacle_id)
            if ob is None:
                return None
            return _copy_obstacle_for_snapshot(ob)

    def snapshot(self) -> Tuple[Obstacle, ...]:
        with self._lock:
            self._purge_expired_unlocked()
            return tuple(_copy_obstacle_for_snapshot(o) for o in self._items.values())

    def __len__(self) -> int:
        with self._lock:
            self._purge_expired_unlocked()
            return len(self._items)

    def purge_expired(self) -> None:
        """Remove all obstacles past ``expires_at`` (monotonic)."""
        with self._lock:
            self._purge_expired_unlocked()

    def _purge_expired_unlocked(self) -> None:
        now = time.monotonic()
        expired = [
            oid
            for oid, o in self._items.items()
            if o.expires_at is not None and o.expires_at <= now
        ]
        for oid in expired:
            del self._items[oid]
