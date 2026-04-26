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

"""Load and optionally draw static obstacles into a shared ObstacleStore."""

from __future__ import annotations

from obstacle_store import ObstacleStore
from static_map_loader import StaticMapLoadError, load_file
from world_builder import draw_static_world


def load_static_world(obstacle_store: ObstacleStore) -> None:
    """Load configured static obstacles and optionally draw them in turtlesim."""
    import rospy

    path = str(rospy.get_param("~static_obstacles_file", "")).strip()
    if path:
        try:
            load_file(obstacle_store, path)
        except StaticMapLoadError as e:
            rospy.logerr("static obstacles: %s", e)
            raise
        if rospy.get_param("~draw_static_world", True):
            try:
                count = draw_static_world(obstacle_store)
                rospy.loginfo("static world builder drew %s segments", count)
            except Exception as e:
                rospy.logerr("static world builder failed: %s", e)
                if rospy.get_param("~world_builder_required", True):
                    raise
