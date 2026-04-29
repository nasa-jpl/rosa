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

"""ROS wiring: append :class:`CollisionEvent` to JSONL and optionally log."""

from __future__ import annotations

from typing import Callable

import rospy
from collision_monitor import CollisionEvent, collision_event_to_record
from pose_logger import CollisionJsonlWriter
from ros_params import get_bool_param


def make_collision_event_sink(
    jsonl: CollisionJsonlWriter,
) -> Callable[[CollisionEvent], None]:
    """Append each emitted event to ``collision.jsonl`` (``stay`` optional via param).

    Terminal logging is off unless ``~collision_log_to_console`` is true; JSONL is always
    written for non-filtered events.
    """
    log_stay = get_bool_param("~collision_log_stay", False)
    log_to_console = get_bool_param("~collision_log_to_console", False)

    def _sink(event: CollisionEvent) -> None:
        if event.event_type == "stay" and not log_stay:
            return
        jsonl.write_record(collision_event_to_record(event))
        if log_to_console:
            rospy.loginfo("collision event: %s", event)

    return _sink
