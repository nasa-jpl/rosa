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

from math import cos, sin, sqrt
from typing import List

import rospy
from geometry_msgs.msg import Twist
from langchain.agents import tool
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, TeleportRelative, Kill, SetPen

cmd_vel_pubs = {}


def add_cmd_vel_pub(name: str, publisher: rospy.Publisher):
    global cmd_vel_pubs
    cmd_vel_pubs[name] = publisher


def remove_cmd_vel_pub(name: str):
    global cmd_vel_pubs
    cmd_vel_pubs.pop(name, None)


# Add the default turtle1 publisher on startup
add_cmd_vel_pub("turtle1", rospy.Publisher(f"/turtle1/cmd_vel", Twist, queue_size=10))


def within_bounds(x: float, y: float) -> tuple:
    """
    Check if the given x, y coordinates are within the bounds of the turtlesim environment.

    :param x: The x-coordinate.
    :param y: The y-coordinate.
    """
    if 0 <= x <= 11 and 0 <= y <= 11:
        return True, "Coordinates are within bounds."
    else:
        return False, f"({x}, {y}) will be out of bounds. Range is [0, 11] for each."


def will_be_within_bounds(
    name: str, velocity: float, lateral: float, angle: float, duration: float = 1.0
) -> tuple:
    """Check if the turtle will be within bounds after publishing a twist command."""
    # Get the current pose of the turtle
    pose = get_turtle_pose.invoke({"names": [name]})
    current_x = pose[name].x
    current_y = pose[name].y
    current_theta = pose[name].theta

    # Calculate the new position and orientation
    if abs(angle) < 1e-6:  # Straight line motion
        new_x = (
            current_x
            + (velocity * cos(current_theta) - lateral * sin(current_theta)) * duration
        )
        new_y = (
            current_y
            + (velocity * sin(current_theta) + lateral * cos(current_theta)) * duration
        )
    else:  # Circular motion
        radius = sqrt(velocity**2 + lateral**2) / abs(angle)
        center_x = current_x - radius * sin(current_theta)
        center_y = current_y + radius * cos(current_theta)
        angle_traveled = angle * duration
        new_x = center_x + radius * sin(current_theta + angle_traveled)
        new_y = center_y - radius * cos(current_theta + angle_traveled)

        # Check if any point on the circle is out of bounds
        for t in range(int(duration) + 1):
            angle_t = current_theta + angle * t
            x_t = center_x + radius * sin(angle_t)
            y_t = center_y - radius * cos(angle_t)
            in_bounds, _ = within_bounds(x_t, y_t)
            if not in_bounds:
                return (
                    False,
                    f"The circular path will go out of bounds at ({x_t:.2f}, {y_t:.2f}).",
                )

    # Check if the final x, y coordinates are within bounds
    in_bounds, message = within_bounds(new_x, new_y)
    if not in_bounds:
        return (
            False,
            f"This command will move the turtle out of bounds to ({new_x:.2f}, {new_y:.2f}).",
        )

    return True, f"The turtle will remain within bounds at ({new_x:.2f}, {new_y:.2f})."


@tool
def spawn_turtle(name: str, x: float, y: float, theta: float) -> str:
    """
    Spawn a turtle at the given x, y, and theta coordinates.

    :param name: name of the turtle.
    :param x: x-coordinate.
    :param y: y-coordinate.
    :param theta: angle.
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    # Remove any forward slashes from the name
    name = name.replace("/", "")

    try:
        rospy.wait_for_service("/spawn", timeout=5)
    except rospy.ROSException:
        return f"Failed to spawn {name}: service not available."

    try:
        spawn = rospy.ServiceProxy("/spawn", Spawn)
        spawn(x=x, y=y, theta=theta, name=name)

        global cmd_vel_pubs
        cmd_vel_pubs[name] = rospy.Publisher(f"/{name}/cmd_vel", Twist, queue_size=10)

        return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
    except Exception as e:
        return f"Failed to spawn {name}: {e}"


@tool
def kill_turtle(names: List[str]):
    """
    Removes a turtle from the turtlesim environment.

    :param names: List of names of the turtles to remove (do not include the forward slash).
    """

    # Remove any forward slashes from the names
    names = [name.replace("/", "") for name in names]
    response = ""
    global cmd_vel_pubs

    for name in names:
        try:
            rospy.wait_for_service(f"/{name}/kill", timeout=5)
        except rospy.ROSException:
            response += f"Failed to kill {name}: /{name}/kill service not available.\n"
            continue
        try:
            kill = rospy.ServiceProxy(f"/{name}/kill", Kill)
            kill()

            cmd_vel_pubs.pop(name, None)

            response += f"Successfully killed {name}.\n"
        except rospy.ServiceException as e:
            response += f"Failed to kill {name}: {e}\n"

    return response


@tool
def clear_turtlesim():
    """Clears the turtlesim background and sets the color to the value of the background parameters."""
    try:
        rospy.wait_for_service("/clear", timeout=5)
    except rospy.ROSException:
        return "Failed to clear the turtlesim background: /clear service not available."
    try:
        clear = rospy.ServiceProxy("/clear", Empty)
        clear()
        return "Successfully cleared the turtlesim background."
    except rospy.ServiceException as e:
        return f"Failed to clear the turtlesim background: {e}"


@tool
def get_turtle_pose(names: List[str]) -> dict:
    """
    Get the pose of one or more turtles.

    :param names: List of names of the turtles to get the pose of.
    """

    # Remove any forward slashes from the names
    names = [name.replace("/", "") for name in names]
    poses = {}

    # Get the pose of each turtle
    for name in names:
        try:
            msg = rospy.wait_for_message(f"/{name}/pose", Pose, timeout=5)
            poses[name] = msg
        except rospy.ROSException:
            return {
                "Error": f"Failed to get pose for {name}: /{name}/pose not available."
            }
    return poses


@tool
def teleport_absolute(
    name: str, x: float, y: float, theta: float, hide_pen: bool = True
):
    """
    Teleport a turtle to exact coordinates with a specific heading angle.
    Use this to position the turtle precisely before drawing.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param x: The x-coordinate, range: [0, 11]. 0 is left edge, 11 is right edge.
    :param y: The y-coordinate, range: [0, 11]. 0 is bottom edge, 11 is top edge.
    :param theta: Heading angle in radians. 0=right, π/2≈1.57=up, π≈3.14=left, 3π/2≈4.71=down
    :param hide_pen: If True (default), pen is turned off during teleport so no line is drawn
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    try:
        rospy.wait_for_service(f"/{name}/teleport_absolute", timeout=5)
    except rospy.ROSException:
        return f"Failed to teleport the {name}: /{name}/teleport_absolute service not available."

    try:
        teleport = rospy.ServiceProxy(f"/{name}/teleport_absolute", TeleportAbsolute)
        if hide_pen:
            set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 1, "off": 1})
        teleport(x=x, y=y, theta=theta)
        if hide_pen:
            set_pen.invoke(
                {"name": name, "r": 30, "g": 30, "b": 255, "width": 1, "off": 0}
            )
        current_pose = get_turtle_pose.invoke({"names": [name]})

        return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
    except rospy.ServiceException as e:
        return f"Failed to teleport the turtle: {e}"


@tool
def teleport_relative(name: str, linear: float, angular: float):
    """
    Teleport a turtle relative to its current position and orientation.
    Use this to adjust heading without drawing, or to move without precise positioning.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param linear: distance to move forward (positive) or backward (negative)
    :param angular: angle to rotate in RADIANS. Positive = counterclockwise, negative = clockwise
    """
    in_bounds, message = will_be_within_bounds(name, linear, 0.0, angular)
    if not in_bounds:
        return message

    try:
        rospy.wait_for_service(f"/{name}/teleport_relative", timeout=5)
    except rospy.ROSException:
        return f"Failed to teleport the {name}: /{name}/teleport_relative service not available."
    try:
        teleport = rospy.ServiceProxy(f"/{name}/teleport_relative", TeleportRelative)
        teleport(linear=linear, angular=angular)
        current_pose = get_turtle_pose.invoke({"names": [name]})
        return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
    except rospy.ServiceException as e:
        return f"Failed to teleport the turtle: {e}"


@tool
def publish_twist_to_cmd_vel(
    name: str,
    velocity: float,
    lateral: float,
    angle: float,
    steps: int = 1,
):
    """
    Publish a Twist message to move the turtle. This DRAWS a line as the turtle moves.
    Each step represents 1 second of movement. Distance traveled = velocity × steps.
    
    For STRAIGHT lines: set angle=0 and use velocity for distance.
    For CURVED lines: combine velocity and angle (creates an arc).
    For ROTATION only: set velocity=0 and use angle.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param velocity: linear velocity in units/second. Positive=forward, negative=backward. Distance = velocity × steps.
    :param lateral: lateral velocity (strafe). Positive=left, negative=right. Usually 0 for standard movement.
    :param angle: angular velocity in radians/second. Positive=counterclockwise, negative=clockwise. Usually 0 for straight lines.
    :param steps: Number of seconds to publish this command. Total distance = velocity × steps.
    """
    # Remove any forward slashes from the name
    name = name.replace("/", "")

    # Check if the movement will keep the turtle within bounds
    in_bounds, message = will_be_within_bounds(
        name, velocity, lateral, angle, duration=steps
    )
    if not in_bounds:
        return message

    vel = Twist()
    vel.linear.x, vel.linear.y, vel.linear.z = velocity, lateral, 0.0
    vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, angle

    try:
        global cmd_vel_pubs
        pub = cmd_vel_pubs[name]

        for _ in range(steps):
            pub.publish(vel)
            rospy.sleep(1)
    except Exception as e:
        return f"Failed to publish {vel} to /{name}/cmd_vel: {e}"
    finally:
        current_pose = get_turtle_pose.invoke({"names": [name]})
        return (
            f"New Pose ({name}): x={current_pose[name].x}, y={current_pose[name].y}, "
            f"theta={current_pose[name].theta} rads, "
            f"linear_velocity={current_pose[name].linear_velocity}, "
            f"angular_velocity={current_pose[name].angular_velocity}."
        )


@tool
def stop_turtle(name: str):
    """
    Stop a turtle by publishing a Twist message with zero linear and angular velocities.

    :param name: name of the turtle
    """
    return publish_twist_to_cmd_vel.invoke(
        {
            "name": name,
            "velocity": 0.0,
            "lateral": 0.0,
            "angle": 0.0,
        }
    )


@tool
def reset_turtlesim():
    """
    Resets the turtlesim, removes all turtles, clears any markings, and creates a new default turtle at the center.
    """
    try:
        rospy.wait_for_service("/reset", timeout=5)
    except rospy.ROSException:
        return (
            "Failed to reset the turtlesim environment: /reset service not available."
        )
    try:
        reset = rospy.ServiceProxy("/reset", Empty)
        reset()

        # Clear the cmd_vel publishers
        global cmd_vel_pubs
        cmd_vel_pubs.clear()
        cmd_vel_pubs["turtle1"] = rospy.Publisher(
            f"/turtle1/cmd_vel", Twist, queue_size=10
        )

        return "Successfully reset the turtlesim environment. Ignore all previous commands, failures, and goals."
    except rospy.ServiceException as e:
        return f"Failed to reset the turtlesim environment: {e}"


@tool
def set_pen(name: str, r: int, g: int, b: int, width: int, off: int):
    """
    Control the turtle's pen for drawing lines.
    Turn pen OFF before teleporting to reposition without drawing.
    Turn pen ON before using publish_twist_to_cmd_vel to draw lines.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param r: red value (0-255)
    :param g: green value (0-255)
    :param b: blue value (0-255)
    :param width: width of the pen line (1-5 recommended)
    :param off: 0 = pen ON (will draw), 1 = pen OFF (will not draw)
    """
    # Remove any forward slashes from the name
    name = name.replace("/", "")

    try:
        rospy.wait_for_service(f"/{name}/set_pen", timeout=5)
    except rospy.ROSException:
        return f"Failed to set the pen color for the turtle: /{name}/set_pen service not available."
    try:
        set_pen = rospy.ServiceProxy(f"/{name}/set_pen", SetPen)
        set_pen(r=r, g=g, b=b, width=width, off=off)
        return f"Successfully set the pen color for the turtle: {name}."
    except rospy.ServiceException as e:
        return f"Failed to set the pen color for the turtle: {e}"


@tool
def has_moved_to_expected_coordinates(
    name: str, expected_x: float, expected_y: float, tolerance: float = 0.1
) -> str:
    """
    Check if the turtle has moved to the expected position.

    :param name: name of the turtle
    :param expected_x: expected x-coordinate
    :param expected_y: expected y-coordinate
    :param tolerance: tolerance level for the comparison
    """
    current_pose = get_turtle_pose.invoke({"names": [name]})
    current_x = current_pose[name].x
    current_y = current_pose[name].y

    distance = ((current_x - expected_x) ** 2 + (current_y - expected_y) ** 2) ** 0.5
    if distance <= tolerance:
        return (
            f"{name} has moved to the expected position ({expected_x}, {expected_y})."
        )
    else:
        return f"{name} has NOT moved to the expected position ({expected_x}, {expected_y})."


@tool
def draw_line_segment(name: str, x1: float, y1: float, x2: float, y2: float) -> str:
    """
    Draw a single straight line from point (x1,y1) to point (x2,y2).
    
    This is a high-level convenience tool that automatically:
    1. Calculates the angle and distance needed
    2. Turns off the pen
    3. Teleports to the starting point with correct heading
    4. Turns on the pen
    5. Draws the line
    
    Use this instead of manually doing the calculate/teleport/draw sequence.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param x1: starting x coordinate
    :param y1: starting y coordinate
    :param x2: ending x coordinate
    :param y2: ending y coordinate
    :return: status message with final position
    """
    from math import atan2, sqrt
    
    # Calculate angle and distance
    dx = x2 - x1
    dy = y2 - y1
    angle = atan2(dy, dx)
    distance = sqrt(dx**2 + dy**2)
    
    # Check bounds
    in_bounds_start, msg = within_bounds(x1, y1)
    if not in_bounds_start:
        return f"Start point {msg}"
    
    in_bounds_end, msg = within_bounds(x2, y2)
    if not in_bounds_end:
        return f"End point {msg}"
    
    # Turn off pen
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    
    # Teleport to start with correct angle
    teleport_absolute.invoke({"name": name, "x": x1, "y": y1, "theta": angle, "hide_pen": True})
    
    # Turn on pen
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    
    # Draw the line (angle=0 because heading is already set)
    result = publish_twist_to_cmd_vel.invoke({
        "name": name,
        "velocity": distance,
        "lateral": 0,
        "angle": 0,
        "steps": 1
    })
    
    return f"Line drawn from ({x1},{y1}) to ({x2},{y2}). {result}"


@tool
def draw_rectangle(
    name: str, x: float, y: float, width: float, height: float, filled: bool = False
) -> str:
    """
    Draw a perfect rectangle with exact corners and no angle drift.
    
    This tool automatically handles the complex workflow of teleporting to each edge
    with the exact angle to ensure perfectly straight lines and right angles.
    
    The rectangle is drawn counterclockwise starting from the bottom-left corner.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param x: x coordinate of bottom-left corner
    :param y: y coordinate of bottom-left corner
    :param width: width of the rectangle (extends to the right)
    :param height: height of the rectangle (extends upward)
    :param filled: if True, fills the rectangle with horizontal lines (not just outline)
    :return: status message with rectangle bounds
    """
    from math import pi
    
    # Check all corners are in bounds
    corners = [
        (x, y, "bottom-left"),
        (x + width, y, "bottom-right"),
        (x + width, y + height, "top-right"),
        (x, y + height, "top-left"),
    ]
    
    for cx, cy, corner_name in corners:
        in_bounds, msg = within_bounds(cx, cy)
        if not in_bounds:
            return f"Rectangle {corner_name} corner {msg}"
    
    # Draw the four edges using teleport_absolute for precision
    # Bottom edge (left to right)
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    teleport_absolute.invoke({"name": name, "x": x, "y": y, "theta": 0, "hide_pen": True})
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    publish_twist_to_cmd_vel.invoke({"name": name, "velocity": width, "lateral": 0, "angle": 0, "steps": 1})
    
    # Right edge (bottom to top)
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    teleport_absolute.invoke({"name": name, "x": x + width, "y": y, "theta": pi/2, "hide_pen": True})
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    publish_twist_to_cmd_vel.invoke({"name": name, "velocity": height, "lateral": 0, "angle": 0, "steps": 1})
    
    # Top edge (right to left)
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    teleport_absolute.invoke({"name": name, "x": x + width, "y": y + height, "theta": pi, "hide_pen": True})
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    publish_twist_to_cmd_vel.invoke({"name": name, "velocity": width, "lateral": 0, "angle": 0, "steps": 1})
    
    # Left edge (top to bottom)
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    teleport_absolute.invoke({"name": name, "x": x, "y": y + height, "theta": 3*pi/2, "hide_pen": True})
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    publish_twist_to_cmd_vel.invoke({"name": name, "velocity": height, "lateral": 0, "angle": 0, "steps": 1})
    
    # Fill if requested
    if filled:
        fill_step = 0.1  # Distance between fill lines
        y_current = y + fill_step
        while y_current < y + height:
            set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 1, "off": 1})
            teleport_absolute.invoke({"name": name, "x": x, "y": y_current, "theta": 0, "hide_pen": True})
            set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 1, "off": 0})
            publish_twist_to_cmd_vel.invoke({"name": name, "velocity": width, "lateral": 0, "angle": 0, "steps": 1})
            y_current += fill_step
    
    return f"Rectangle drawn: bottom-left=({x},{y}), width={width}, height={height}, filled={filled}"


@tool
def draw_polyline(name: str, points: List[tuple], closed: bool = False) -> str:
    """
    Draw a series of connected straight line segments through multiple points.
    
    This tool automatically calculates angles and distances for each segment
    and uses the proper teleport technique to ensure clean, precise lines.
    
    Example: draw_polyline('turtle1', [(2,2), (5,2), (5,5), (2,5)], closed=True)
    draws a square from (2,2) to (5,2) to (5,5) to (2,5) and back to (2,2).

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param points: list of (x, y) coordinate tuples to connect, e.g., [(1,1), (3,4), (5,2)]
    :param closed: if True, draws a final line segment back to the first point
    :return: status message with number of segments drawn
    """
    if len(points) < 2:
        return "Error: Need at least 2 points to draw a polyline"
    
    # Check all points are in bounds
    for i, (x, y) in enumerate(points):
        in_bounds, msg = within_bounds(x, y)
        if not in_bounds:
            return f"Point {i} at ({x},{y}) {msg}"
    
    # Draw each segment
    segments_drawn = 0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        draw_line_segment.invoke({"name": name, "x1": x1, "y1": y1, "x2": x2, "y2": y2})
        segments_drawn += 1
    
    # Close the shape if requested
    if closed and len(points) > 2:
        x1, y1 = points[-1]
        x2, y2 = points[0]
        draw_line_segment.invoke({"name": name, "x1": x1, "y1": y1, "x2": x2, "y2": y2})
        segments_drawn += 1
    
    return f"Polyline drawn with {segments_drawn} segments through {len(points)} points. Closed: {closed}"


@tool
def calculate_rectangle_bounds(x: float, y: float, width: float, height: float) -> dict:
    """
    Calculate all four corner coordinates of a rectangle.
    
    This is useful for planning layouts and checking for overlaps before drawing.
    Returns a dictionary with clearly labeled corner positions.

    :param x: x coordinate of bottom-left corner
    :param y: y coordinate of bottom-left corner
    :param width: width of the rectangle
    :param height: height of the rectangle
    :return: dict with 'bottom_left', 'bottom_right', 'top_left', 'top_right', 'center', and ranges
    """
    return {
        "bottom_left": (x, y),
        "bottom_right": (x + width, y),
        "top_right": (x + width, y + height),
        "top_left": (x, y + height),
        "center": (x + width/2, y + height/2),
        "x_range": (x, x + width),
        "y_range": (y, y + height),
        "width": width,
        "height": height,
    }


@tool
def check_rectangles_overlap(rect1: tuple, rect2: tuple) -> dict:
    """
    Check if two rectangles overlap or intersect.
    
    Essential for validating that doors, windows, and other components don't
    conflict with each other before drawing.
    
    Each rectangle is specified as (x, y, width, height) where (x,y) is the
    bottom-left corner.

    :param rect1: tuple of (x, y, width, height) for first rectangle
    :param rect2: tuple of (x, y, width, height) for second rectangle
    :return: dict with 'overlap' (bool), 'message', and 'details' about the overlap
    """
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    
    # Calculate bounds
    r1_left = x1
    r1_right = x1 + w1
    r1_bottom = y1
    r1_top = y1 + h1
    
    r2_left = x2
    r2_right = x2 + w2
    r2_bottom = y2
    r2_top = y2 + h2
    
    # Check for overlap
    # Rectangles overlap if they overlap in both x and y dimensions
    x_overlap = not (r1_right <= r2_left or r2_right <= r1_left)
    y_overlap = not (r1_top <= r2_bottom or r2_top <= r1_bottom)
    
    overlap = x_overlap and y_overlap
    
    if overlap:
        # Calculate overlap region
        overlap_left = max(r1_left, r2_left)
        overlap_right = min(r1_right, r2_right)
        overlap_bottom = max(r1_bottom, r2_bottom)
        overlap_top = min(r1_top, r2_top)
        
        overlap_width = overlap_right - overlap_left
        overlap_height = overlap_top - overlap_bottom
        
        return {
            "overlap": True,
            "message": f"Rectangles overlap! Overlap region: ({overlap_left:.2f},{overlap_bottom:.2f}) to ({overlap_right:.2f},{overlap_top:.2f})",
            "details": {
                "overlap_region": {
                    "x": overlap_left,
                    "y": overlap_bottom,
                    "width": overlap_width,
                    "height": overlap_height,
                },
                "rect1_bounds": f"x:[{r1_left:.2f},{r1_right:.2f}] y:[{r1_bottom:.2f},{r1_top:.2f}]",
                "rect2_bounds": f"x:[{r2_left:.2f},{r2_right:.2f}] y:[{r2_bottom:.2f},{r2_top:.2f}]",
            }
        }
    else:
        return {
            "overlap": False,
            "message": "Rectangles do not overlap. Safe to draw both.",
            "details": {
                "rect1_bounds": f"x:[{r1_left:.2f},{r1_right:.2f}] y:[{r1_bottom:.2f},{r1_top:.2f}]",
                "rect2_bounds": f"x:[{r2_left:.2f},{r2_right:.2f}] y:[{r2_bottom:.2f},{r2_top:.2f}]",
            }
        }


@tool
def draw_circle(name: str, center_x: float, center_y: float, radius: float, segments: int = 36) -> str:
    """
    Draw a circle by approximating it with multiple small arc segments.
    
    The circle is drawn by moving the turtle in a circular path while the pen is down.
    More segments = smoother circle, but slower to draw. 36 segments usually looks good.
    
    Technical details: This uses the turtle's curved motion capability (velocity + angular velocity)
    to draw smooth circular arcs. The circle is drawn counterclockwise starting from the rightmost point.

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param center_x: x coordinate of circle center
    :param center_y: y coordinate of circle center
    :param radius: radius of the circle
    :param segments: number of segments to approximate the circle (default 36, more = smoother)
    :return: status message
    """
    from math import pi, cos, sin
    
    # Validate parameters
    if radius <= 0:
        return f"Radius must be positive, got {radius}"
    
    if segments <= 0:
        return f"Segments must be positive, got {segments}"
    
    # Check corners of bounding box
    for point_name, x, y in [
        ("center", center_x, center_y),
        ("rightmost", center_x + radius, center_y),
        ("leftmost", center_x - radius, center_y),
        ("topmost", center_x, center_y + radius),
        ("bottommost", center_x, center_y - radius),
    ]:
        in_bounds, msg = within_bounds(x, y)
        if not in_bounds:
            return f"Circle {point_name} point {msg}"
    
    # Calculate arc parameters
    # We'll draw the circle as small arcs
    # Arc length per segment = 2*pi*radius / segments
    # Angular velocity = 2*pi / total_time
    # Linear velocity = arc_length / time_per_segment
    
    angle_per_segment = 2 * pi / segments  # radians per segment
    arc_length_per_segment = 2 * pi * radius / segments
    time_per_segment = 1.0  # 1 second per segment
    
    linear_velocity = arc_length_per_segment / time_per_segment
    angular_velocity = angle_per_segment / time_per_segment
    
    # Start at rightmost point of circle (center_x + radius, center_y)
    # Heading should be tangent to circle = pi/2 (pointing up)
    start_x = center_x + radius
    start_y = center_y
    start_theta = pi / 2  # pointing up (tangent to circle at right side)
    
    # Move to start position
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    teleport_absolute.invoke({"name": name, "x": start_x, "y": start_y, "theta": start_theta, "hide_pen": True})
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    
    # Draw each segment
    for i in range(segments):
        publish_twist_to_cmd_vel.invoke({
            "name": name,
            "velocity": linear_velocity,
            "lateral": 0,
            "angle": angular_velocity,
            "steps": 1
        })
    
    return f"Circle drawn: center=({center_x},{center_y}), radius={radius}, segments={segments}"


@tool
def draw_arc(
    name: str,
    center_x: float,
    center_y: float,
    radius: float,
    start_angle: float,
    arc_angle: float,
    segments: int = 18
) -> str:
    """
    Draw an arc (part of a circle) from start_angle for arc_angle radians.
    
    This is perfect for drawing curved shapes like clouds, rainbows, or semicircles.
    The arc is drawn counterclockwise if arc_angle is positive, clockwise if negative.
    
    Examples:
    - Semicircle (top half): start_angle=0, arc_angle=π (3.14159)
    - Quarter circle: start_angle=0, arc_angle=π/2 (1.5708)
    - Cloud bump: start_angle=0, arc_angle=π (half circle)

    :param name: name of the turtle (without forward slash, e.g., 'turtle1')
    :param center_x: x coordinate of arc center
    :param center_y: y coordinate of arc center
    :param radius: radius of the arc
    :param start_angle: starting angle in radians (0=right, π/2=up, π=left, 3π/2=down)
    :param arc_angle: how many radians to sweep (positive=counterclockwise, negative=clockwise)
    :param segments: number of segments to approximate the arc (default 18)
    :return: status message
    """
    from math import pi, cos, sin, fabs
    
    # Validate parameters
    if radius <= 0:
        return f"Radius must be positive, got {radius}"
    
    if segments <= 0:
        return f"Segments must be positive, got {segments}"
    
    if fabs(arc_angle) < 0.01:
        return f"Arc angle too small: {arc_angle} radians"
    
    # Calculate start position on the arc
    start_x = center_x + radius * cos(start_angle)
    start_y = center_y + radius * sin(start_angle)
    
    # Check if start point is in bounds
    in_bounds, msg = within_bounds(start_x, start_y)
    if not in_bounds:
        return f"Arc start point {msg}"
    
    # Calculate end position to check bounds
    end_angle = start_angle + arc_angle
    end_x = center_x + radius * cos(end_angle)
    end_y = center_y + radius * sin(end_angle)
    
    in_bounds, msg = within_bounds(end_x, end_y)
    if not in_bounds:
        return f"Arc end point {msg}"
    
    # Calculate motion parameters
    angle_per_segment = arc_angle / segments
    arc_length_per_segment = fabs(arc_angle) * radius / segments
    time_per_segment = 1.0
    
    linear_velocity = arc_length_per_segment / time_per_segment
    angular_velocity = angle_per_segment / time_per_segment
    
    # Start heading should be tangent to the circle
    # Tangent is perpendicular to radius, so add π/2 to start_angle
    start_theta = start_angle + (pi / 2 if arc_angle > 0 else -pi / 2)
    
    # Move to start position
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 1})
    teleport_absolute.invoke({"name": name, "x": start_x, "y": start_y, "theta": start_theta, "hide_pen": True})
    set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 2, "off": 0})
    
    # Draw each segment
    for i in range(segments):
        publish_twist_to_cmd_vel.invoke({
            "name": name,
            "velocity": linear_velocity,
            "lateral": 0,
            "angle": angular_velocity,
            "steps": 1
        })
    
    return f"Arc drawn: center=({center_x},{center_y}), radius={radius}, start={start_angle:.2f}rad, sweep={arc_angle:.2f}rad"
