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

"""Load static obstacles from YAML/JSON or ROS param into :class:`ObstacleStore`.

YAML/JSON document shape::

    obstacles:
      - id: wall-south
        kind: static   # optional; omitted defaults to static
        geometry:
          type: segments
          segments:
            - [[0.0, 0.0], [11.0, 0.0]]

Field mapping (file keys → :mod:`obstacle_store` types):

- ``id`` (str, required) → :attr:`Obstacle.id`
- ``kind``: ``\"static\"`` (default) or ``\"temporary\"``. Temporary deadlines use
  ``ttl_seconds`` if given, else :data:`DEFAULT_TEMPORARY_TTL_SECONDS` at load time.
- ``geometry.type``:

  - ``circle`` — keys ``cx``, ``cy``, ``r`` (numbers) → :class:`CircleGeometry`
  - ``aabb`` — ``min_x``, ``min_y``, ``max_x``, ``max_y`` → :class:`AabbGeometry`
  - ``segments`` — ``segments`` is a list of ``[[x1,y1],[x2,y2]]`` pairs →
    :class:`SegmentsGeometry` (same tuple layout as :class:`collision_geometry.Segment`)

**Duplicate ids:** default ``on_duplicate_id=\"replace\"`` matches :meth:`ObstacleStore.upsert`
(last write wins). With ``on_duplicate_id=\"error\"``, a duplicate id in the store
(or duplicated earlier in the same document) raises :exc:`StaticMapLoadError`.

**Files:** :func:`load_file` uses the path suffix (``.yaml``/``.yml`` → PyYAML,
``.json`` → :mod:`json`). Other paths try JSON first, then YAML.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import (
    Any,
    Dict,
    List,
    Literal,
    Mapping,
    MutableMapping,
    Optional,
    Tuple,
    Union,
)

import yaml

from obstacle_store import (
    AabbGeometry,
    CircleGeometry,
    Obstacle,
    ObstacleGeometry,
    ObstacleStore,
    SegmentsGeometry,
)

OnDuplicateId = Literal["replace", "error"]

# Used when ``kind: temporary`` and ``ttl_seconds`` is omitted (seconds).
DEFAULT_TEMPORARY_TTL_SECONDS = 31_536_000.0


class StaticMapLoadError(ValueError):
    """Invalid static map data or policy violation (duplicate id, wrong kind, etc.)."""


def _err(
    msg: str, *, source: Optional[str] = None, index: Optional[int] = None
) -> StaticMapLoadError:
    parts = [msg]
    if source is not None:
        parts.append(f"source={source!r}")
    if index is not None:
        parts.append(f"index={index}")
    return StaticMapLoadError("; ".join(parts))


def _coerce_float(
    x: Any, *, key: str, source: Optional[str], index: Optional[int]
) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise _err(
            f"{key} must be a number, got {type(x).__name__}",
            source=source,
            index=index,
        )
    return float(x)


def _parse_segment(
    raw: Any, *, source: Optional[str], index: Optional[int], seg_index: int
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise _err(
            f"segments[{seg_index}] must be [p1, p2]",
            source=source,
            index=index,
        )
    p1, p2 = raw
    if not isinstance(p1, (list, tuple)) or len(p1) != 2:
        raise _err(
            f"segments[{seg_index}][0] must be [x,y]", source=source, index=index
        )
    if not isinstance(p2, (list, tuple)) or len(p2) != 2:
        raise _err(
            f"segments[{seg_index}][1] must be [x,y]", source=source, index=index
        )
    x1 = _coerce_float(p1[0], key="x1", source=source, index=index)
    y1 = _coerce_float(p1[1], key="y1", source=source, index=index)
    x2 = _coerce_float(p2[0], key="x2", source=source, index=index)
    y2 = _coerce_float(p2[1], key="y2", source=source, index=index)
    return ((x1, y1), (x2, y2))


def _parse_geometry(
    geom: Any, *, source: Optional[str], index: Optional[int]
) -> ObstacleGeometry:
    if not isinstance(geom, MutableMapping):
        raise _err("geometry must be a mapping", source=source, index=index)
    gtype = geom.get("type")
    if gtype is None:
        raise _err("geometry.type is required", source=source, index=index)
    if not isinstance(gtype, str):
        raise _err("geometry.type must be a string", source=source, index=index)
    gtype_l = gtype.strip().lower()
    if gtype_l == "circle":
        for k in ("cx", "cy", "r"):
            if k not in geom:
                raise _err(
                    f"geometry.{k} is required for circle", source=source, index=index
                )
        return CircleGeometry(
            _coerce_float(geom["cx"], key="cx", source=source, index=index),
            _coerce_float(geom["cy"], key="cy", source=source, index=index),
            _coerce_float(geom["r"], key="r", source=source, index=index),
        )
    if gtype_l == "aabb":
        for k in ("min_x", "min_y", "max_x", "max_y"):
            if k not in geom:
                raise _err(
                    f"geometry.{k} is required for aabb", source=source, index=index
                )
        return AabbGeometry(
            _coerce_float(geom["min_x"], key="min_x", source=source, index=index),
            _coerce_float(geom["min_y"], key="min_y", source=source, index=index),
            _coerce_float(geom["max_x"], key="max_x", source=source, index=index),
            _coerce_float(geom["max_y"], key="max_y", source=source, index=index),
        )
    if gtype_l == "segments":
        if "segments" not in geom:
            raise _err(
                "geometry.segments is required for segments", source=source, index=index
            )
        segs_raw = geom["segments"]
        if not isinstance(segs_raw, (list, tuple)):
            raise _err("geometry.segments must be a list", source=source, index=index)
        segs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = [
            _parse_segment(s, source=source, index=index, seg_index=i)
            for i, s in enumerate(segs_raw)
        ]
        return SegmentsGeometry(tuple(segs))
    raise _err(
        f"unknown geometry.type {gtype!r} (expected circle, aabb, segments)",
        source=source,
        index=index,
    )


def _parse_obstacle_entry(raw: Any, *, source: Optional[str], index: int) -> Obstacle:
    if not isinstance(raw, MutableMapping):
        raise _err("obstacle entry must be a mapping", source=source, index=index)
    oid = raw.get("id")
    if oid is None or not isinstance(oid, str) or not oid.strip():
        raise _err("id must be a non-empty string", source=source, index=index)
    kind = raw.get("kind", "static")
    if not isinstance(kind, str):
        raise _err("kind must be a string", source=source, index=index)
    kind_l = kind.strip().lower()
    if kind_l not in ("static", "temporary"):
        raise _err(
            f"map kind must be 'static' or 'temporary', got {kind!r}",
            source=source,
            index=index,
        )
    if "geometry" not in raw:
        raise _err("geometry is required", source=source, index=index)
    geometry = _parse_geometry(raw["geometry"], source=source, index=index)
    if kind_l == "static":
        return Obstacle(
            id=oid.strip(),
            kind="static",
            geometry=geometry,
            expires_at=None,
        )
    raw_ttl = raw.get("ttl_seconds", DEFAULT_TEMPORARY_TTL_SECONDS)
    ttl = _coerce_float(raw_ttl, key="ttl_seconds", source=source, index=index)
    if ttl <= 0:
        raise _err(
            "ttl_seconds must be positive for temporary obstacles",
            source=source,
            index=index,
        )
    return Obstacle(
        id=oid.strip(),
        kind="temporary",
        geometry=geometry,
        expires_at=time.monotonic() + ttl,
    )


def load_into_store(
    store: ObstacleStore,
    data: Mapping[str, Any],
    *,
    on_duplicate_id: OnDuplicateId = "replace",
    source: Optional[str] = None,
) -> int:
    """Parse ``data`` (must contain key ``obstacles``: list) and upsert into ``store``.

    Returns the number of obstacles loaded.
    """
    if "obstacles" not in data:
        raise _err("root key 'obstacles' is required", source=source, index=None)
    items = data["obstacles"]
    if not isinstance(items, list):
        raise _err("'obstacles' must be a list", source=source, index=None)

    seen_in_doc: set[str] = set()
    count = 0
    for i, raw in enumerate(items):
        ob = _parse_obstacle_entry(raw, source=source, index=i)
        if on_duplicate_id == "error":
            if ob.id in seen_in_doc:
                raise _err(
                    f"duplicate id {ob.id!r} in same document",
                    source=source,
                    index=i,
                )
            if store.get(ob.id) is not None:
                raise _err(
                    f"id {ob.id!r} already exists in store",
                    source=source,
                    index=i,
                )
        seen_in_doc.add(ob.id)
        store.upsert(ob)
        count += 1
    return count


def _parse_file_bytes(raw: bytes, path: Path) -> Dict[str, Any]:
    text = raw.decode("utf-8")
    suf = path.suffix.lower()
    if suf == ".json":
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError as e:
            raise StaticMapLoadError(f"invalid JSON in {path}: {e}") from e
    elif suf in (".yaml", ".yml"):
        try:
            parsed = yaml.safe_load(text)
        except yaml.YAMLError as e:
            raise StaticMapLoadError(f"invalid YAML in {path}: {e}") from e
    else:
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            try:
                parsed = yaml.safe_load(text)
            except yaml.YAMLError as e:
                raise StaticMapLoadError(
                    f"file {path} is not valid JSON or YAML: {e}"
                ) from e
    if not isinstance(parsed, dict):
        raise StaticMapLoadError(
            f"document root in {path} must be a mapping, got {type(parsed)}"
        )
    return parsed


def load_file(
    store: ObstacleStore,
    path: Union[str, Path],
    *,
    on_duplicate_id: OnDuplicateId = "replace",
) -> int:
    """Load obstacles from a YAML or JSON file into ``store``."""
    p = Path(path).expanduser()
    if not p.is_file():
        raise StaticMapLoadError(f"not a file: {p}")
    data = _parse_file_bytes(p.read_bytes(), p)
    return load_into_store(store, data, on_duplicate_id=on_duplicate_id, source=str(p))


def load_from_rosparam(
    store: ObstacleStore,
    param_name: str = "~static_obstacles",
    *,
    on_duplicate_id: OnDuplicateId = "replace",
) -> int:
    """Load from ``rospy.get_param(param_name)`` (dict with ``obstacles`` or a list of entries)."""
    import rospy

    raw: Any = rospy.get_param(param_name, None)
    if raw is None:
        return 0
    if isinstance(raw, list):
        data: Dict[str, Any] = {"obstacles": raw}
    elif isinstance(raw, dict):
        data = raw
    else:
        raise StaticMapLoadError(
            f"param {param_name!r} must be a dict or list, got {type(raw).__name__}"
        )
    return load_into_store(
        store,
        data,
        on_duplicate_id=on_duplicate_id,
        source=f"rosparam:{param_name}",
    )
