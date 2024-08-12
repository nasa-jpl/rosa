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
    """Performs arctangent on the input values x.

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
    Convert degrees to radians.

    :param degrees: A list of one or more degrees to convert to radians.
    """
    rads = {}
    for degree in degrees:
        rads[degree] = f"{degree * (3.14159 / 180)} radians."
    return rads


@tool
def radians_to_degrees(radians: List[float]):
    """
    Convert radians to degrees.

    :param radians: A list of one or more radians to convert to degrees.
    """
    degs = {}
    for radian in radians:
        degs[radian] = f"{radian * (180 / 3.14159)} degrees."
    return degs
