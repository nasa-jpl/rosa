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

"""2D collision geometry for turtlesim-style overlap checks (no ROS imports).

`CollisionMonitor` and similar code can import:

- ``circle_circle_signed_gap`` — center distance minus sum of radii; &lt; 0 overlap, &gt; 0 separated.
- ``circles_overlap`` — bool overlap/touch with optional ``eps`` tolerance.
- ``point_in_circle`` — point inside or on the circle boundary.
- ``point_circle_distance`` — signed distance (negative inside, zero on boundary, positive outside).
- ``segment_intersects_disc`` — closed disc vs line segment (walls / polyline edges).
- ``circle_intersects_aabb`` — circle vs axis-aligned box (first-order obstacle approximation).
- ``any_segment_intersects_disc`` — any segment in an iterable hits the disc.

All coordinates are plane Cartesian (e.g. turtlesim x, y). Uses only the standard library.
"""

from __future__ import annotations

import math
from typing import Iterable, Tuple

Point = Tuple[float, float]
Segment = Tuple[Point, Point]


def circle_circle_signed_gap(
    cx1: float,
    cy1: float,
    r1: float,
    cx2: float,
    cy2: float,
    r2: float,
) -> float:
    """Return (center distance) − (r1 + r2)."""
    d = math.hypot(cx2 - cx1, cy2 - cy1)
    return d - (r1 + r2)


def circles_overlap(
    cx1: float,
    cy1: float,
    r1: float,
    cx2: float,
    cy2: float,
    r2: float,
    *,
    eps: float = 0.0,
) -> bool:
    """True if closed discs overlap or touch, within ``eps`` on the signed gap."""
    return circle_circle_signed_gap(cx1, cy1, r1, cx2, cy2, r2) <= eps


def point_in_circle(
    px: float,
    py: float,
    cx: float,
    cy: float,
    r: float,
    *,
    eps: float = 0.0,
) -> bool:
    """True if the point lies inside the disc or on its boundary (within ``eps``)."""
    return math.hypot(px - cx, py - cy) <= r + eps


def point_circle_distance(
    px: float, py: float, cx: float, cy: float, r: float
) -> float:
    """Signed distance from point to circle boundary (inside &lt; 0, outside &gt; 0)."""
    return math.hypot(px - cx, py - cy) - r


def _closest_point_on_segment(
    x1: float, y1: float, x2: float, y2: float, px: float, py: float
) -> Tuple[float, float]:
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0.0:
        return x1, y1
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    return x1 + t * dx, y1 + t * dy


def segment_intersects_disc(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    cx: float,
    cy: float,
    r: float,
    *,
    eps: float = 0.0,
) -> bool:
    """True if the closed disc intersects the line segment (endpoints included)."""
    qx, qy = _closest_point_on_segment(x1, y1, x2, y2, cx, cy)
    dist = math.hypot(qx - cx, qy - cy)
    return dist <= r + eps


def circle_intersects_aabb(
    cx: float,
    cy: float,
    r: float,
    min_x: float,
    min_y: float,
    max_x: float,
    max_y: float,
    *,
    eps: float = 0.0,
) -> bool:
    """True if the closed disc intersects the closed axis-aligned rectangle."""
    qx = min(max(cx, min_x), max_x)
    qy = min(max(cy, min_y), max_y)
    dist = math.hypot(cx - qx, cy - qy)
    return dist <= r + eps


def any_segment_intersects_disc(
    segments: Iterable[Segment],
    cx: float,
    cy: float,
    r: float,
    *,
    eps: float = 0.0,
) -> bool:
    """True if any ``((x1,y1),(x2,y2))`` segment intersects the closed disc."""
    for (x1, y1), (x2, y2) in segments:
        if segment_intersects_disc(x1, y1, x2, y2, cx, cy, r, eps=eps):
            return True
    return False
