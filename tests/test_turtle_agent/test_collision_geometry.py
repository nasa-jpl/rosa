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

import math
import sys
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))

from collision_geometry import (  # noqa: E402
    any_segment_intersects_disc,
    circle_circle_signed_gap,
    circle_intersects_aabb,
    circles_overlap,
    point_circle_distance,
    point_in_circle,
    segment_intersects_disc,
)


class TestCirclesOverlap(unittest.TestCase):
    def test_separated(self):
        self.assertFalse(circles_overlap(0.0, 0.0, 1.0, 5.0, 0.0, 1.0))
        self.assertGreater(circle_circle_signed_gap(0.0, 0.0, 1.0, 5.0, 0.0, 1.0), 0.0)

    def test_overlapping(self):
        self.assertTrue(circles_overlap(0.0, 0.0, 2.0, 1.0, 0.0, 2.0))
        self.assertLess(circle_circle_signed_gap(0.0, 0.0, 2.0, 1.0, 0.0, 2.0), 0.0)

    def test_touching_exactly(self):
        # centers 3 apart, radii 1 and 2 -> touching
        self.assertTrue(circles_overlap(0.0, 0.0, 1.0, 3.0, 0.0, 2.0))
        self.assertAlmostEqual(
            circle_circle_signed_gap(0.0, 0.0, 1.0, 3.0, 0.0, 2.0),
            0.0,
        )

    def test_eps_treats_near_touch_as_overlap(self):
        gap = circle_circle_signed_gap(0.0, 0.0, 1.0, 3.0 + 1e-12, 0.0, 2.0)
        self.assertGreater(gap, 0.0)
        self.assertFalse(circles_overlap(0.0, 0.0, 1.0, 3.0 + 1e-12, 0.0, 2.0, eps=0.0))
        self.assertTrue(circles_overlap(0.0, 0.0, 1.0, 3.0 + 1e-12, 0.0, 2.0, eps=1e-9))


class TestPointCircle(unittest.TestCase):
    def test_inside_and_outside(self):
        self.assertTrue(point_in_circle(0.0, 0.0, 1.0, 1.0, 2.0))
        self.assertFalse(point_in_circle(5.0, 0.0, 0.0, 0.0, 1.0))

    def test_point_circle_distance(self):
        self.assertAlmostEqual(point_circle_distance(2.0, 0.0, 0.0, 0.0, 1.0), 1.0)
        self.assertAlmostEqual(point_circle_distance(0.5, 0.0, 0.0, 0.0, 1.0), -0.5)


class TestSegmentDisc(unittest.TestCase):
    def test_segment_crosses_circle(self):
        self.assertTrue(segment_intersects_disc(-2.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0))

    def test_segment_passes_outside(self):
        self.assertFalse(segment_intersects_disc(3.0, 3.0, 5.0, 3.0, 0.0, 0.0, 1.0))

    def test_degenerate_segment_is_point_inside(self):
        self.assertTrue(segment_intersects_disc(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0))


class TestCircleAabb(unittest.TestCase):
    def test_circle_inside_box(self):
        self.assertTrue(circle_intersects_aabb(0.5, 0.5, 0.1, 0.0, 0.0, 1.0, 1.0))

    def test_circle_far_from_box(self):
        self.assertFalse(circle_intersects_aabb(10.0, 10.0, 0.5, 0.0, 0.0, 1.0, 1.0))

    def test_circle_touches_box_corner_exterior(self):
        # Center (2,2): closest point on [0,1]^2 is (1,1); distance sqrt(2). Radius just encloses it.
        d = math.hypot(1.0, 1.0)
        self.assertTrue(circle_intersects_aabb(2.0, 2.0, d, 0.0, 0.0, 1.0, 1.0))
        self.assertFalse(
            circle_intersects_aabb(2.0, 2.0, d - 1e-9, 0.0, 0.0, 1.0, 1.0, eps=0.0)
        )


class TestAnySegmentIntersectsDisc(unittest.TestCase):
    def test_one_hit(self):
        segs = [((5.0, 5.0), (6.0, 5.0)), ((-1.0, 0.0), (1.0, 0.0))]
        self.assertTrue(any_segment_intersects_disc(segs, 0.0, 0.0, 0.5))

    def test_none_hit(self):
        segs = [((3.0, 3.0), (4.0, 3.0)), ((3.0, 4.0), (4.0, 4.0))]
        self.assertFalse(any_segment_intersects_disc(segs, 0.0, 0.0, 1.0))


if __name__ == "__main__":
    unittest.main()
