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

"""Small helpers for reading ROS parameters with explicit type coercion."""

from __future__ import annotations

from typing import Any


def coerce_bool_param_value(value: Any) -> bool:
    """Interpret common string false values before falling back to ``bool``."""
    if isinstance(value, str):
        return value.strip().lower() not in ("0", "false", "no", "off", "")
    return bool(value)


def get_bool_param(name: str, default: bool) -> bool:
    """Read a ROS parameter and coerce it to bool in a string-safe way."""
    import rospy

    return coerce_bool_param_value(rospy.get_param(name, default))
