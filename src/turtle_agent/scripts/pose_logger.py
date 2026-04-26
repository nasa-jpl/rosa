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

"""Periodic turtlesim pose logging to JSONL.

The pure helpers and :class:`PoseLogConsumer` are ROS-free at import time. ROS 1
imports are delayed to runtime wrappers so unit tests can run without turtlesim.
"""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, TextIO

POSE_LOG_INTERVAL_SEC = 1.0


def resolve_log_root() -> Path:
    """Log root: TURTLE_AGENT_LOG_ROOT if set, else ./logs from current working directory."""
    env = os.environ.get("TURTLE_AGENT_LOG_ROOT", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    return (Path.cwd() / "logs").resolve()


def build_log_dir(
    log_root: Path, date_str: str, session_id: str, turtle_id: str
) -> Path:
    """logs/<date>/<session>/<turtle>/"""
    return log_root / date_str / session_id / turtle_id


def build_location_log_path(
    log_root: Path, date_str: str, session_id: str, turtle_id: str
) -> Path:
    """Return ``logs/<date>/<session>/<turtle>/location.jsonl``."""
    return build_log_dir(log_root, date_str, session_id, turtle_id) / "location.jsonl"


def pose_to_record(pose: Any, stamp: Any) -> Dict[str, Any]:
    """
    Build one JSON-serializable dict from a turtlesim Pose-like object and a time stamp.

    stamp: rospy.Time or any object with .secs and .nsecs, or dict with those keys.
    """
    if hasattr(stamp, "secs"):
        t_ros = {"secs": int(stamp.secs), "nsecs": int(stamp.nsecs)}
    else:
        t_ros = {"secs": int(stamp["secs"]), "nsecs": int(stamp["nsecs"])}
    return {
        "t_ros": t_ros,
        "x": float(pose.x),
        "y": float(pose.y),
        "theta": float(pose.theta),
        "linear_velocity": float(pose.linear_velocity),
        "angular_velocity": float(pose.angular_velocity),
    }


def _stamp_to_seconds(stamp: Any) -> float:
    if hasattr(stamp, "secs"):
        return float(stamp.secs) + float(stamp.nsecs) / 1_000_000_000.0
    if isinstance(stamp, dict) and "secs" in stamp and "nsecs" in stamp:
        return float(stamp["secs"]) + float(stamp["nsecs"]) / 1_000_000_000.0
    return time.monotonic()


class PoseLogConsumer:
    """PoseHub consumer that writes one ``location.jsonl`` file per turtle."""

    def __init__(
        self,
        *,
        period: float = POSE_LOG_INTERVAL_SEC,
        log_root: Optional[Path] = None,
        session_id: Optional[str] = None,
        date_str: Optional[str] = None,
    ) -> None:
        self._period = float(period)
        self._log_root = log_root or resolve_log_root()
        self._session_id = session_id or str(uuid.uuid4())
        self._date_str = date_str or datetime.now().strftime("%Y-%m-%d")
        self._lock = threading.Lock()
        self._files: Dict[str, TextIO] = {}
        self._paths: Dict[str, Path] = {}
        self._last_written_at: Dict[str, float] = {}
        self._closed = False

    @property
    def session_id(self) -> str:
        return self._session_id

    def path_for(self, turtle_id: str) -> Path:
        turtle_id = str(turtle_id).replace("/", "")
        return build_location_log_path(
            self._log_root, self._date_str, self._session_id, turtle_id
        )

    def on_pose(self, turtle_id: str, pose: Any, stamp: Any) -> None:
        stamp_seconds = _stamp_to_seconds(stamp)
        turtle_id = str(turtle_id).replace("/", "")
        if not turtle_id:
            return

        with self._lock:
            if self._closed:
                return
            last = self._last_written_at.get(turtle_id)
            if last is not None and stamp_seconds - last < self._period:
                return

            fp = self._files.get(turtle_id)
            if fp is None:
                path = self.path_for(turtle_id)
                path.parent.mkdir(parents=True, exist_ok=True)
                fp = open(path, "a", encoding="utf-8")
                self._files[turtle_id] = fp
                self._paths[turtle_id] = path

            line = json.dumps(pose_to_record(pose, stamp), separators=(",", ":")) + "\n"
            fp.write(line)
            fp.flush()
            self._last_written_at[turtle_id] = stamp_seconds

    def __call__(self, turtle_id: str, pose: Any, stamp: Any) -> None:
        self.on_pose(turtle_id, pose, stamp)

    def close(self) -> None:
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

    def close_all(self) -> None:
        self.close()


class PoseLogger:
    """
    Subscribe to /{turtle_id}/pose and append one JSON line per period.

    This single-turtle wrapper is kept for compatibility. New multi-turtle code
    should use :class:`PoseLogConsumer` with ``PoseHub``.

    Requires rospy.AsyncSpinner (or similar) so callbacks run while the main thread is busy.
    """

    def __init__(
        self,
        turtle_id: Optional[str] = None,
        period: Optional[float] = None,
        log_root: Optional[Path] = None,
    ):
        self._turtle_id_param = turtle_id
        self._period_param = period
        self._log_root_override = log_root

        self._lock = threading.Lock()
        self._last_pose: Any = None
        self._stopped = False

        self._sub: Any = None
        self._timer: Any = None
        self._fp: Optional[TextIO] = None
        self._path: Optional[Path] = None
        self._shutdown_hooked = False

    def start(self) -> None:
        import rospy
        from turtlesim.msg import Pose

        if self._fp is not None:
            return

        self._stopped = False

        turtle_id = self._turtle_id_param
        if turtle_id is None:
            turtle_id = rospy.get_param(
                "~turtle_id",
                os.environ.get("TURTLE_TURTLE_ID", "turtle1"),
            )
        turtle_id = str(turtle_id).replace("/", "")

        period = self._period_param
        if period is None:
            period = float(rospy.get_param("~pose_log_interval", POSE_LOG_INTERVAL_SEC))

        log_root = self._log_root_override or resolve_log_root()
        session_id = str(uuid.uuid4())
        date_str = datetime.now().strftime("%Y-%m-%d")
        log_dir = build_log_dir(log_root, date_str, session_id, turtle_id)
        log_dir.mkdir(parents=True, exist_ok=True)
        self._path = log_dir / "location.jsonl"
        self._fp = open(self._path, "a", encoding="utf-8")

        self._turtle_id = turtle_id
        self._period = period

        topic = f"/{turtle_id}/pose"
        self._sub = rospy.Subscriber(topic, Pose, self._on_pose, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(self._period), self._on_timer)

        if not self._shutdown_hooked:
            rospy.on_shutdown(self._on_ros_shutdown)
            self._shutdown_hooked = True

        rospy.loginfo(
            "PoseLogger writing to %s (every %.3f s)", self._path, self._period
        )

    def _on_ros_shutdown(self) -> None:
        self.stop()

    def _on_pose(self, msg: Any) -> None:
        with self._lock:
            if not self._stopped:
                self._last_pose = msg

    def _on_timer(self, _event: Any) -> None:
        import rospy

        stamp = rospy.Time.now()
        with self._lock:
            if self._stopped or self._last_pose is None or self._fp is None:
                return
            rec = pose_to_record(self._last_pose, stamp)
            line = json.dumps(rec, separators=(",", ":")) + "\n"
            try:
                self._fp.write(line)
                self._fp.flush()
            except OSError as e:
                rospy.logwarn("PoseLogger write failed: %s", e)

    def stop(self) -> None:
        import rospy

        with self._lock:
            if self._stopped:
                return
            self._stopped = True

        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None
        if self._fp is not None:
            try:
                self._fp.flush()
            except OSError:
                pass
            try:
                self._fp.close()
            except OSError:
                pass
            self._fp = None

        if self._path is not None:
            rospy.loginfo("PoseLogger stopped (%s)", self._path)
            self._path = None

        self._last_pose = None
