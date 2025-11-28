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
from typing import List

from langchain.agents import tool


@tool
def add_all(numbers: List[float]) -> float:
    """Returns the sum of a list of numbers."""
    return sum(numbers)


@tool
def multiply_all(numbers: List[float]) -> float:
    """Returns the product of a list of numbers."""
    result = 1
    for number in numbers:
        result *= number
    return result


@tool
def mean(numbers: List[float]) -> dict:
    """Returns the mean of a list of numbers."""
    return {
        "mean": statistics.mean(numbers),
        "stdev": statistics.stdev(numbers),
    }


@tool
def median(numbers: List[float]) -> float:
    """Returns the median of a list of numbers."""
    return statistics.median(numbers)


@tool
def mode(numbers: List[float]) -> List[float]:
    """Returns the mode of a list of numbers."""
    return statistics.mode(numbers)


@tool
def variance(numbers: List[float]) -> float:
    """Returns the variance of a list of numbers."""
    return statistics.variance(numbers)


@tool
def add(xy_pairs: List[tuple]) -> List[dict]:
    """Performs addition on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}+{y}": x + y,
        }
        results.append(result)
    return results


@tool
def subtract(xy_pairs: List[tuple]) -> List[dict]:
    """Performs subtraction on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}-{y}": x - y,
        }
        results.append(result)
    return results


@tool
def multiply(xy_pairs: List[tuple]) -> List[dict]:
    """Performs multiplication on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}*{y}": x * y,
        }
        results.append(result)
    return results


@tool
def divide(xy_pairs: List[tuple]) -> List[dict]:
    """Performs division on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}/{y}": x / y if y != 0 else "undefined",
        }
        results.append(result)
    return results


@tool
def exponentiate(xy_pairs: List[tuple]) -> List[dict]:
    """Performs exponentiation on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}^{y}": x**y,
        }
        results.append(result)
    return results


@tool
def modulo(xy_pairs: List[tuple]) -> List[dict]:
    """Performs modulo on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}%{y}": x % y if y != 0 else "undefined",
        }
        results.append(result)
    return results


@tool
def sine(x_values: List[float]) -> List[dict]:
    """Performs sine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"sin({x})": math.sin(x),
        }
        results.append(result)
    return results


@tool
def cosine(x_values: List[float]) -> List[dict]:
    """Performs cosine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"cos({x})": math.cos(x),
        }
        results.append(result)
    return results


@tool
def tangent(x_values: List[float]) -> List[dict]:
    """Performs tangent on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"tan({x})": math.tan(x),
        }
        results.append(result)
    return results


@tool
def asin(x_values: List[float]) -> List[dict]:
    """Performs arcsine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        try:
            result = {
                f"asin({x})": math.asin(x),
            }
        except ValueError:
            result = {
                f"asin({x})": "undefined",
            }
        results.append(result)
    return results


@tool
def acos(x_values: List[float]) -> List[dict]:
    """Performs arccosine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        try:
            result = {
                f"acos({x})": math.acos(x),
            }
        except ValueError:
            result = {
                f"acos({x})": "undefined",
            }
        results.append(result)
    return results


@tool
def atan(x_values: List[float]) -> List[dict]:
    """
    Calculate arctangent (inverse tangent) of input values. Returns angle in radians.
    Use this to find an angle from a slope (rise/run).
    
    For finding angle from point A to point B, use atan2 instead (it's better).
    
    Example: atan(1) = π/4 ≈ 0.785 radians = 45 degrees

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"atan({x})": math.atan(x),
        }
        results.append(result)
    return results


@tool
def sinh(x_values: List[float]) -> List[dict]:
    """Performs hyperbolic sine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"sinh({x})": math.sinh(x),
        }
        results.append(result)
    return results


@tool
def cosh(x_values: List[float]) -> List[dict]:
    """Performs hyperbolic cosine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"cosh({x})": math.cosh(x),
        }
        results.append(result)
    return results


@tool
def tanh(x_values: List[float]) -> List[dict]:
    """Performs hyperbolic tangent on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"tanh({x})": math.tanh(x),
        }
        results.append(result)
    return results


@tool
def count_list(items: List) -> int:
    """Returns the number of items in a list."""
    return len(items)


@tool
def count_words(text: str) -> int:
    """Returns the number of words in a string."""
    return len(text.split())


@tool
def count_lines(text: str) -> int:
    """Returns the number of lines in a string."""
    return len(text.split("\n"))


@tool
def degrees_to_radians(degrees: List[float]):
    """
    Convert degrees to radians. Use this for angle conversions.

    :param degrees: A list of one or more degrees to convert to radians.
    """
    rads = {}
    for degree in degrees:
        rads[degree] = degree * (math.pi / 180)
    return rads


@tool
def radians_to_degrees(radians: List[float]):
    """
    Convert radians to degrees. Use this for angle conversions.

    :param radians: A list of one or more radians to convert to degrees.
    """
    degs = {}
    for radian in radians:
        degs[radian] = radian * (180 / math.pi)
    return degs


@tool
def sqrt(x_values: List[float]) -> List[dict]:
    """
    Calculate the square root of input values. Essential for distance calculations.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        if x < 0:
            result = {f"sqrt({x})": "undefined (negative number)"}
        else:
            result = {f"sqrt({x})": math.sqrt(x)}
        results.append(result)
    return results


@tool
def atan2(y_x_pairs: List[tuple]) -> List[dict]:
    """
    Calculate the angle (in radians) from the positive x-axis to the point (x, y).
    This is the MOST IMPORTANT tool for calculating angles between two points.
    
    To find the angle from point (x1, y1) to point (x2, y2):
    Use atan2(y2-y1, x2-x1)
    
    Example: angle from (1, 1) to (3, 4) = atan2(4-1, 3-1) = atan2(3, 2) ≈ 0.98 radians

    :arg y_x_pairs: A list of tuples containing (y, x) values - NOTE: y comes FIRST, then x
    """
    results = []
    for y, x in y_x_pairs:
        result = {f"atan2({y}, {x})": math.atan2(y, x)}
        results.append(result)
    return results


@tool
def distance_between_points(point_pairs: List[tuple]) -> List[dict]:
    """
    Calculate the straight-line distance between two points using the Pythagorean theorem.
    Formula: sqrt((x2-x1)^2 + (y2-y1)^2)
    
    This is essential for determining how far the turtle needs to move.

    :arg point_pairs: A list of tuples, each containing ((x1, y1), (x2, y2))
    Example: [((0, 0), (3, 4))] calculates distance from (0,0) to (3,4) = 5.0
    """
    results = []
    for (x1, y1), (x2, y2) in point_pairs:
        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx**2 + dy**2)
        result = {f"distance from ({x1},{y1}) to ({x2},{y2})": dist}
        results.append(result)
    return results


@tool
def calculate_line_angle_and_distance(point_pairs: List[tuple]) -> List[dict]:
    """
    Calculate BOTH the angle (in radians) and distance needed to draw a line from point A to point B.
    This is a high-level helper that combines atan2 and distance calculations.
    
    Use this when planning to draw a line between two specific coordinates.
    The angle returned is relative to the positive x-axis (right = 0, up = π/2).

    :arg point_pairs: A list of tuples, each containing ((x1, y1), (x2, y2))
    Example: [((2, 3), (5, 7))] returns angle and distance from (2,3) to (5,7)
    """
    results = []
    for (x1, y1), (x2, y2) in point_pairs:
        dx = x2 - x1
        dy = y2 - y1
        angle = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)
        result = {
            f"line from ({x1},{y1}) to ({x2},{y2})": {
                "angle_radians": angle,
                "angle_degrees": angle * (180 / math.pi),
                "distance": distance,
            }
        }
        results.append(result)
    return results
