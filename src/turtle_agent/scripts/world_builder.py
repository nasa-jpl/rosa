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

"""Draw static obstacles in turtlesim from an ``ObstacleStore`` snapshot.

This module is deliberately not a LangChain tool. It is called once during node
startup after static map loading and before the agent conversation begins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

from obstacle_store import (
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleStore,
    SegmentsGeometry,
)


@dataclass(frozen=True)
class RenderSegment:
    x1: float
    y1: float
    x2: float
    y2: float


def render_segments_from_store(
    store: ObstacleStore, *, circle_segments: int = 24
) -> Tuple[RenderSegment, ...]:
    """Return line segments for all non-expired obstacles in ``store``."""
    return render_segments_from_obstacles(
        store.snapshot(), circle_segments=circle_segments
    )


def render_segments_from_obstacles(
    obstacles: Iterable[Obstacle], *, circle_segments: int = 24
) -> Tuple[RenderSegment, ...]:
    """Convert obstacle geometry to line segments that a builder turtle can draw."""
    segments = []
    for obstacle in obstacles:
        segments.extend(
            _segments_for_obstacle(obstacle, circle_segments=circle_segments)
        )
    return tuple(segments)


def _segments_for_obstacle(
    obstacle: Obstacle, *, circle_segments: int
) -> Tuple[RenderSegment, ...]:
    geometry = obstacle.geometry
    if isinstance(geometry, SegmentsGeometry):
        return tuple(
            RenderSegment(x1, y1, x2, y2) for (x1, y1), (x2, y2) in geometry.segments
        )
    if isinstance(geometry, AabbGeometry):
        min_x, min_y = geometry.min_x, geometry.min_y
        max_x, max_y = geometry.max_x, geometry.max_y
        return (
            RenderSegment(min_x, min_y, max_x, min_y),
            RenderSegment(max_x, min_y, max_x, max_y),
            RenderSegment(max_x, max_y, min_x, max_y),
            RenderSegment(min_x, max_y, min_x, min_y),
        )
    if circle_segments < 3:
        raise ValueError("circle_segments must be >= 3")
    points = []
    for i in range(circle_segments):
        theta = 2.0 * math.pi * i / circle_segments
        points.append(
            (
                geometry.cx + geometry.r * math.cos(theta),
                geometry.cy + geometry.r * math.sin(theta),
            )
        )
    return tuple(
        RenderSegment(x1, y1, x2, y2)
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )


def draw_static_world(
    store: ObstacleStore,
    *,
    turtle_name: str = "world_builder",
    circle_segments: int = 24,
    pen_rgb: Tuple[int, int, int] = (40, 40, 40),
    pen_width: int = 2,
    service_timeout: float = 5.0,
) -> int:
    """Draw the current store snapshot with a temporary builder turtle.

    Returns the number of line segments drawn. The builder turtle is removed on
    exit when possible.
    """
    import rospy
    from turtlesim.srv import Kill, SetPen, Spawn, TeleportAbsolute

    segments = render_segments_from_store(store, circle_segments=circle_segments)
    if not segments:
        return 0

    first = segments[0]
    turtle_name = turtle_name.replace("/", "")

    rospy.wait_for_service("/spawn", timeout=service_timeout)
    spawn = rospy.ServiceProxy("/spawn", Spawn)
    kill = rospy.ServiceProxy("/kill", Kill)

    def kill_builder() -> None:
        try:
            rospy.wait_for_service("/kill", timeout=service_timeout)
            kill(turtle_name)
        except Exception:
            pass

    # Avoid a stale builder from a previous run blocking spawn.
    kill_builder()
    spawn(x=first.x1, y=first.y1, theta=0.0, name=turtle_name)

    set_pen = rospy.ServiceProxy(f"/{turtle_name}/set_pen", SetPen)
    teleport = rospy.ServiceProxy(f"/{turtle_name}/teleport_absolute", TeleportAbsolute)
    try:
        rospy.wait_for_service(f"/{turtle_name}/set_pen", timeout=service_timeout)
        rospy.wait_for_service(
            f"/{turtle_name}/teleport_absolute", timeout=service_timeout
        )
        for segment in segments:
            set_pen(r=0, g=0, b=0, width=pen_width, off=1)
            teleport(x=segment.x1, y=segment.y1, theta=0.0)
            set_pen(
                r=int(pen_rgb[0]),
                g=int(pen_rgb[1]),
                b=int(pen_rgb[2]),
                width=pen_width,
                off=0,
            )
            teleport(x=segment.x2, y=segment.y2, theta=0.0)
        return len(segments)
    finally:
        kill_builder()
