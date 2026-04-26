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

import rospy

_TURTLE_LIFECYCLE_LISTENER = None


def configure_turtle_lifecycle_listener(listener):
    """Inject a listener notified after successful turtle spawn/kill calls."""
    global _TURTLE_LIFECYCLE_LISTENER
    _TURTLE_LIFECYCLE_LISTENER = listener


def notify_turtle_spawned(name: str) -> None:
    listener = _TURTLE_LIFECYCLE_LISTENER
    if listener is None:
        return
    try:
        listener.on_turtle_spawned(name)
    except Exception as e:
        rospy.logwarn("turtle lifecycle listener spawn notification failed: %s", e)


def notify_turtle_killed(name: str) -> None:
    listener = _TURTLE_LIFECYCLE_LISTENER
    if listener is None:
        return
    try:
        listener.on_turtle_killed(name)
    except Exception as e:
        rospy.logwarn("turtle lifecycle listener kill notification failed: %s", e)
