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
import statistics
import unittest

from src.rosa.tools.calculation import (
    add_all,
    multiply_all,
    mean,
    median,
    mode,
    variance,
    add,
    subtract,
    multiply,
    divide,
    exponentiate,
    modulo,
    sine,
    cosine,
    tangent,
    asin,
    acos,
    atan,
    sinh,
    cosh,
    tanh,
    count_list,
    count_words,
    count_lines,
    degrees_to_radians,
    radians_to_degrees,
)


class TestCalculationTools(unittest.TestCase):

    def test_add_all_returns_sum_of_numbers(self):
        self.assertEqual(add_all.invoke({"numbers": [1, 2, 3]}), 6)
        self.assertEqual(add_all.invoke({"numbers": []}), 0)

    def test_multiply_all_returns_product_of_numbers(self):
        self.assertEqual(multiply_all.invoke({"numbers": [1, 2, 3]}), 6)
        self.assertEqual(multiply_all.invoke({"numbers": [1, 0, 3]}), 0)

    def test_mean_returns_mean_and_stdev_of_numbers(self):
        self.assertEqual(mean.invoke({"numbers": [1, 2, 3]}), {"mean": 2, "stdev": 1})
        with self.assertRaises(statistics.StatisticsError):
            mean.invoke({"numbers": []})

    def test_median_returns_median_of_numbers(self):
        self.assertEqual(median.invoke({"numbers": [1, 2, 3]}), 2)
        self.assertEqual(median.invoke({"numbers": [1, 2, 3, 4]}), 2.5)

    def test_mode_returns_mode_of_numbers(self):
        self.assertEqual(mode.invoke({"numbers": [1, 1, 2, 3]}), 1)
        self.assertEqual(mode.invoke({"numbers": [1, 2, 3]}), 1)

    def test_variance_returns_variance_of_numbers(self):
        self.assertEqual(variance.invoke({"numbers": [1, 2, 3]}), 1)
        with self.assertRaises(statistics.StatisticsError):
            variance.invoke({"numbers": [1]})

    def test_add_returns_sum_of_xy_pairs(self):
        self.assertEqual(
            add.invoke({"xy_pairs": [(1, 2), (3, 4)]}), [{"1+2": 3}, {"3+4": 7}]
        )

    def test_subtract_returns_difference_of_xy_pairs(self):
        self.assertEqual(
            subtract.invoke({"xy_pairs": [(1, 2), (3, 4)]}), [{"1-2": -1}, {"3-4": -1}]
        )

    def test_multiply_returns_product_of_xy_pairs(self):
        self.assertEqual(
            multiply.invoke({"xy_pairs": [(1, 2), (3, 4)]}), [{"1*2": 2}, {"3*4": 12}]
        )

    def test_divide_returns_quotient_of_xy_pairs(self):
        self.assertEqual(
            divide.invoke({"xy_pairs": [(1, 2), (3, 4)]}), [{"1/2": 0.5}, {"3/4": 0.75}]
        )
        self.assertEqual(divide.invoke({"xy_pairs": [(1, 0)]}), [{"1/0": "undefined"}])

    def test_exponentiate_returns_exponentiation_of_xy_pairs(self):
        self.assertEqual(
            exponentiate.invoke({"xy_pairs": [(2, 3), (3, 2)]}),
            [{"2^3": 8}, {"3^2": 9}],
        )

    def test_modulo_returns_modulo_of_xy_pairs(self):
        self.assertEqual(
            modulo.invoke({"xy_pairs": [(5, 3), (10, 2)]}), [{"5%3": 2}, {"10%2": 0}]
        )
        self.assertEqual(modulo.invoke({"xy_pairs": [(1, 0)]}), [{"1%0": "undefined"}])

    def test_sine_returns_sine_of_x_values(self):
        self.assertAlmostEqual(
            sine.invoke({"x_values": [0, math.pi / 2]}),
            [{"sin(0.0)": 0.0}, {"sin(1.5707963267948966)": 1.0}],
        )

    def test_cosine_returns_cosine_of_x_values(self):
        cosines = cosine.invoke({"x_values": [0, math.pi / 2]})
        self.assertAlmostEqual(cosines[0]["cos(0.0)"], 1.0, delta=0.0000000000000001)
        self.assertAlmostEqual(
            cosines[1]["cos(1.5707963267948966)"], 0.0, delta=0.0000000000000001
        )

    def test_tangent_returns_tangent_of_x_values(self):
        # Convert the above to use assertAlmostEqual
        tangents = tangent.invoke({"x_values": [0, math.pi / 4]})
        self.assertAlmostEqual(tangents[0]["tan(0.0)"], 0.0, delta=0.0000000000000001)
        self.assertAlmostEqual(
            tangents[1]["tan(0.7853981633974483)"], 1.0, delta=0.000000000000001
        )

    def test_asin_returns_arcsine_of_x_values(self):
        self.assertEqual(
            asin.invoke({"x_values": [0, 1]}),
            [{"asin(0.0)": 0.0}, {"asin(1.0)": 1.5707963267948966}],
        )
        self.assertEqual(asin.invoke({"x_values": [2]}), [{"asin(2.0)": "undefined"}])

    def test_acos_returns_arccosine_of_x_values(self):
        self.assertEqual(
            acos.invoke({"x_values": [0, 1]}),
            [{"acos(0.0)": 1.5707963267948966}, {"acos(1.0)": 0.0}],
        )
        self.assertEqual(acos.invoke({"x_values": [2]}), [{"acos(2.0)": "undefined"}])

    def test_atan_returns_arctangent_of_x_values(self):
        self.assertEqual(
            atan.invoke({"x_values": [0, 1]}),
            [{"atan(0.0)": 0.0}, {"atan(1.0)": 0.7853981633974483}],
        )

    def test_sinh_returns_hyperbolic_sine_of_x_values(self):
        self.assertEqual(
            sinh.invoke({"x_values": [0, 1]}),
            [{"sinh(0.0)": 0.0}, {"sinh(1.0)": 1.1752011936438014}],
        )

    def test_cosh_returns_hyperbolic_cosine_of_x_values(self):
        self.assertAlmostEqual(
            cosh.invoke({"x_values": [0, 1]}),
            [{"cosh(0.0)": 1.0}, {"cosh(1.0)": 1.5430806348152437}],
        )

    def test_tanh_returns_hyperbolic_tangent_of_x_values(self):
        self.assertEqual(
            tanh.invoke({"x_values": [0, 1]}),
            [{"tanh(0.0)": 0.0}, {"tanh(1.0)": 0.7615941559557649}],
        )

    def test_count_list_returns_number_of_items_in_list(self):
        self.assertEqual(count_list.invoke({"items": [1, 2, 3]}), 3)
        self.assertEqual(count_list.invoke({"items": []}), 0)

    def test_count_words_returns_number_of_words_in_string(self):
        self.assertEqual(count_words.invoke({"text": "Hello world"}), 2)
        self.assertEqual(count_words.invoke({"text": ""}), 0)

    def test_count_lines_returns_number_of_lines_in_string(self):
        self.assertEqual(count_lines.invoke({"text": "Hello\nworld"}), 2)
        self.assertEqual(count_lines.invoke({"text": ""}), 1)

    def test_degrees_to_radians_converts_degrees_to_radians(self):
        self.assertEqual(
            degrees_to_radians.invoke({"degrees": [0, 180]}),
            {0: "0.0 radians.", 180: "3.14159 radians."},
        )

    def test_radians_to_degrees_converts_radians_to_degrees(self):
        self.assertEqual(
            radians_to_degrees.invoke({"radians": [0, 3.14159]}),
            {0: "0.0 degrees.", 3.14159: "180.0 degrees."},
        )


if __name__ == "__main__":
    unittest.main()
