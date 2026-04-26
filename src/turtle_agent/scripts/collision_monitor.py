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

"""PoseHub consumer that turns obstacle overlaps into collision events.

The monitor is deliberately ROS-independent: poses only need ``x`` and ``y``
attributes, and obstacles come from :class:`obstacle_store.ObstacleStore`.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Literal, Optional, Tuple

from collision_geometry import (
    any_segment_intersects_disc,
    circle_intersects_aabb,
    circles_overlap,
)
from obstacle_store import (
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleStore,
    SegmentsGeometry,
)

CollisionEventType = Literal["enter", "stay", "exit"]
CollisionType = Literal["turtle_obstacle", "turtle_turtle"]
CollisionKey = Tuple[str, str, str]
EventSink = Callable[["CollisionEvent"], None]


@dataclass(frozen=True)
class CollisionEvent:
    event_type: CollisionEventType
    collision_type: CollisionType
    turtles: Tuple[str, ...]
    stamp: Any
    pose: Any
    obstacle_id: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


class CollisionMonitor:
    """Detect turtle-obstacle and turtle-turtle collisions from pose updates."""

    def __init__(
        self,
        obstacle_store: ObstacleStore,
        *,
        turtle_radius: float = 0.5,
        emit_stay: bool = True,
        event_sink: Optional[EventSink] = None,
    ) -> None:
        if turtle_radius < 0.0:
            raise ValueError("turtle_radius must be non-negative")
        self._store = obstacle_store
        self._turtle_radius = float(turtle_radius)
        self._emit_stay = emit_stay
        self._event_sink = event_sink
        self._lock = threading.RLock()
        self._latest_poses: Dict[str, Tuple[Any, Any]] = {}
        self._active: Dict[CollisionKey, CollisionEvent] = {}
        self._events: list[CollisionEvent] = []

    @property
    def events(self) -> Tuple[CollisionEvent, ...]:
        with self._lock:
            return tuple(self._events)

    def clear_events(self) -> None:
        with self._lock:
            self._events.clear()

    def on_pose(self, turtle_name: str, pose: Any, stamp: Any) -> None:
        """PoseHub consumer callback receiving ``(turtle_name, pose, stamp)``."""
        with self._lock:
            self._latest_poses[str(turtle_name)] = (pose, stamp)
            current = self._collect_hits(stamp)
            events = self._transition_events(current, stamp)
            self._active = current
            self._events.extend(events)

        for event in events:
            if self._event_sink is not None:
                self._event_sink(event)

    def _collect_hits(self, stamp: Any) -> Dict[CollisionKey, CollisionEvent]:
        current: Dict[CollisionKey, CollisionEvent] = {}
        obstacles = self._store.snapshot()
        poses = dict(self._latest_poses)

        for turtle_name in sorted(poses):
            pose, _ = poses[turtle_name]
            tx, ty = _pose_xy(pose)
            for obstacle in obstacles:
                if obstacle.kind == "turtle":
                    continue
                if _turtle_hits_obstacle(tx, ty, self._turtle_radius, obstacle):
                    key = ("turtle_obstacle", turtle_name, obstacle.id)
                    current[key] = CollisionEvent(
                        event_type="enter",
                        collision_type="turtle_obstacle",
                        turtles=(turtle_name,),
                        obstacle_id=obstacle.id,
                        stamp=stamp,
                        pose=pose,
                        details={
                            "obstacle_kind": obstacle.kind,
                            "geometry": type(obstacle.geometry).__name__,
                        },
                    )

        names = sorted(poses)
        for i, turtle_a in enumerate(names):
            pose_a, _ = poses[turtle_a]
            ax, ay = _pose_xy(pose_a)
            for turtle_b in names[i + 1 :]:
                pose_b, _ = poses[turtle_b]
                bx, by = _pose_xy(pose_b)
                if circles_overlap(
                    ax,
                    ay,
                    self._turtle_radius,
                    bx,
                    by,
                    self._turtle_radius,
                ):
                    key = ("turtle_turtle", turtle_a, turtle_b)
                    current[key] = CollisionEvent(
                        event_type="enter",
                        collision_type="turtle_turtle",
                        turtles=(turtle_a, turtle_b),
                        obstacle_id=None,
                        stamp=stamp,
                        pose=pose_a,
                        details={"turtle_radius": self._turtle_radius},
                    )
        return current

    def _transition_events(
        self, current: Dict[CollisionKey, CollisionEvent], stamp: Any
    ) -> Tuple[CollisionEvent, ...]:
        events = []

        for key in sorted(current):
            hit = current[key]
            if key not in self._active:
                events.append(hit)
            elif self._emit_stay:
                events.append(replace(hit, event_type="stay"))

        for key in sorted(set(self._active) - set(current)):
            previous = self._active[key]
            exit_pose = self._pose_for_event(previous)
            events.append(
                replace(previous, event_type="exit", stamp=stamp, pose=exit_pose)
            )

        return tuple(events)

    def _pose_for_event(self, event: CollisionEvent) -> Any:
        for turtle_name in event.turtles:
            latest = self._latest_poses.get(turtle_name)
            if latest is not None:
                return latest[0]
        return event.pose


def collision_event_to_record(event: CollisionEvent) -> Dict[str, Any]:
    """Build one JSON-serializable line for session ``collision.jsonl``."""
    from pose_logger import pose_to_record

    out: Dict[str, Any] = {
        "event_type": event.event_type,
        "collision_type": event.collision_type,
        "turtles": list(event.turtles),
    }
    if event.obstacle_id is not None:
        out["obstacle_id"] = event.obstacle_id
    if event.details is not None:
        out["details"] = event.details
    out.update(pose_to_record(event.pose, event.stamp))
    return out


def _pose_xy(pose: Any) -> Tuple[float, float]:
    return float(getattr(pose, "x")), float(getattr(pose, "y"))


def _turtle_hits_obstacle(
    turtle_x: float, turtle_y: float, turtle_radius: float, obstacle: Obstacle
) -> bool:
    geometry = obstacle.geometry
    if isinstance(geometry, CircleGeometry):
        return circles_overlap(
            turtle_x,
            turtle_y,
            turtle_radius,
            geometry.cx,
            geometry.cy,
            geometry.r,
        )
    if isinstance(geometry, AabbGeometry):
        return circle_intersects_aabb(
            turtle_x,
            turtle_y,
            turtle_radius,
            geometry.min_x,
            geometry.min_y,
            geometry.max_x,
            geometry.max_y,
        )
    if isinstance(geometry, SegmentsGeometry):
        return any_segment_intersects_disc(
            geometry.segments, turtle_x, turtle_y, turtle_radius
        )
    raise TypeError(f"unsupported obstacle geometry: {type(geometry).__name__}")
