#  Copyright (c) 2025. Jet Propulsion Laboratory. All rights reserved.
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

import time
from math import cos, sin, sqrt
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from langchain.agents import tool
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import Kill, SetPen, Spawn, TeleportAbsolute, TeleportRelative

# Global node instance and publishers
_node = None
cmd_vel_pubs = {}
_service_clients = {}
_subscriptions = {}
_latest_poses = {}


def init_ros2_node():
    """Initialize ROS2 node if not already initialized."""
    global _node
    if _node is None:
        if not rclpy.ok():
            rclpy.init()
        _node = Node("turtle_tools_node")
    return _node


def get_node():
    """Get the global ROS2 node instance."""
    global _node
    if _node is None:
        _node = init_ros2_node()
    return _node


def get_service_client(service_name: str, service_type):
    """Get or create a service client, reusing existing ones."""
    global _service_clients
    if service_name not in _service_clients:
        node = get_node()
        _service_clients[service_name] = node.create_client(service_type, service_name)
    return _service_clients[service_name]


def get_pose_subscription(turtle_name: str):
    """Get or create a pose subscription for a turtle, reusing existing ones."""
    global _subscriptions, _latest_poses
    if turtle_name not in _subscriptions:
        node = get_node()

        # Create callback that updates the latest pose
        def pose_callback(msg):
            _latest_poses[turtle_name] = msg

        _subscriptions[turtle_name] = node.create_subscription(
            Pose, f"/{turtle_name}/pose", pose_callback, 10
        )
    return _subscriptions[turtle_name]


def add_cmd_vel_pub(name: str):
    """Add a cmd_vel publisher for a turtle."""
    global cmd_vel_pubs
    node = get_node()
    cmd_vel_pubs[name] = node.create_publisher(Twist, f"/{name}/cmd_vel", 10)


def remove_cmd_vel_pub(name: str):
    """Remove a cmd_vel publisher for a turtle."""
    global cmd_vel_pubs
    if name in cmd_vel_pubs:
        # Don't explicitly destroy - let node cleanup handle it
        cmd_vel_pubs.pop(name, None)


# Add the default turtle1 publisher on startup
def initialize_default_turtle():
    """Initialize the default turtle1 publisher and subscription."""
    if "turtle1" not in cmd_vel_pubs:
        add_cmd_vel_pub("turtle1")
    # Also initialize subscription for turtle1
    get_pose_subscription("turtle1")


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

    # Check if the turtle exists
    if name not in pose:
        error_msg = f"Turtle '{name}' not found. Make sure turtlesim is running and the turtle exists."
        print(f"[TURTLE TOOL DEBUG] {error_msg}")
        print(
            f"[TURTLE TOOL DEBUG] Available turtles in pose dict: {list(pose.keys())}"
        )
        return False, error_msg

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
        node = get_node()
        client = get_service_client("/spawn", Spawn)

        if not client.wait_for_service(timeout_sec=5.0):
            return f"Failed to spawn {name}: service not available."

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is not None:
            add_cmd_vel_pub(name)
            return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
        else:
            return f"Failed to spawn {name}: service call failed."
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
    node = get_node()

    for name in names:
        client = get_service_client("/kill", Kill)

        if not client.wait_for_service(timeout_sec=5.0):
            response += f"Failed to kill {name}: /kill service not available.\n"
            continue

        try:
            request = Kill.Request()
            request.name = name

            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            if future.result() is not None:
                remove_cmd_vel_pub(name)
                response += f"Successfully killed {name}.\n"
            else:
                response += f"Failed to kill {name}: service call failed.\n"
        except Exception as e:
            response += f"Failed to kill {name}: {e}\n"

    return response


@tool
def clear_turtlesim():
    """Clears the turtlesim background and sets the color to the value of the background parameters."""
    node = get_node()
    client = get_service_client("/clear", Empty)

    if not client.wait_for_service(timeout_sec=5.0):
        return "Failed to clear the turtlesim background: /clear service not available."

    try:
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is not None:
            return "Successfully cleared the turtlesim background."
        else:
            return "Failed to clear the turtlesim background: service call failed."
    except Exception as e:
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
    node = get_node()

    # Get the pose of each turtle
    for name in names:
        try:
            # Ensure subscription exists for this turtle
            get_pose_subscription(name)

            # Try to get the latest pose from cache first
            global _latest_poses
            if name in _latest_poses:
                poses[name] = _latest_poses[name]
                continue

            # If no cached pose, wait for a message
            timeout = 5.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                if name in _latest_poses:
                    poses[name] = _latest_poses[name]
                    break

            if name not in poses:
                return {
                    "Error": f"Failed to get pose for {name}: /{name}/pose not available."
                }
        except Exception as e:
            return {"Error": f"Failed to get pose for {name}: {e}"}

    return poses


@tool
def teleport_absolute(
    name: str, x: float, y: float, theta: float, hide_pen: bool = True
):
    """
    Instantly teleport a turtle to exact coordinates. Use for positioning, not drawing.

    COORDINATES: TurtleSim uses (0,0) at bottom-left, (11,11) at top-right
    ANGLES: 0 = facing right, π/2 = up, π = left, 3π/2 = down

    :param name: turtle name (e.g. 'turtle1')
    :param x: x-coordinate (0-11, left to right)
    :param y: y-coordinate (0-11, bottom to top)
    :param theta: orientation angle in radians
    :param hide_pen: True = no trail during teleport, False = show trail
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    name = name.replace("/", "")
    node = get_node()
    client = get_service_client(f"/{name}/teleport_absolute", TeleportAbsolute)

    if not client.wait_for_service(timeout_sec=5.0):
        return f"Failed to teleport the {name}: /{name}/teleport_absolute service not available."

    try:
        if hide_pen:
            set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 1, "off": 1})

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if hide_pen:
            set_pen.invoke(
                {"name": name, "r": 30, "g": 30, "b": 255, "width": 1, "off": 0}
            )

        if future.result() is not None:
            current_pose = get_turtle_pose.invoke({"names": [name]})
            if name not in current_pose:
                return f"Teleport completed, but couldn't verify pose - turtle '{name}' not found."
            return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
        else:
            return "Failed to teleport the turtle: service call failed."
    except Exception as e:
        return f"Failed to teleport the turtle: {e}"


@tool
def teleport_relative(name: str, linear: float, angular: float):
    """
    Teleport a turtle relative to its current position.

    :param name: name of the turtle
    :param linear: linear distance
    :param angular: angular distance
    """
    # Only check bounds if there's linear movement
    # Pure rotations (linear=0) should always be allowed
    if abs(linear) > 1e-6:
        in_bounds, message = will_be_within_bounds(name, linear, 0.0, angular)
        if not in_bounds:
            return message

    name = name.replace("/", "")
    node = get_node()
    client = get_service_client(f"/{name}/teleport_relative", TeleportRelative)

    if not client.wait_for_service(timeout_sec=5.0):
        return f"Failed to teleport the {name}: /{name}/teleport_relative service not available."

    try:
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is not None:
            current_pose = get_turtle_pose.invoke({"names": [name]})
            if name not in current_pose:
                return f"Teleport completed, but couldn't verify pose - turtle '{name}' not found."
            return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
        else:
            return "Failed to teleport the turtle: service call failed."
    except Exception as e:
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
    Move a turtle using velocity commands. This is the PRIMARY movement tool for smooth, continuous motion.

    CRITICAL USAGE PATTERNS:
    - STRAIGHT LINES: velocity > 0, angle = 0 (for drawing star lines, square sides, etc.)
    - TURN IN PLACE: velocity = 0, angle != 0 (for sharp turns between lines)
    - CIRCLES: velocity > 0, angle > 0 (ONLY for circular motion - creates curved path)
    - STOP: velocity = 0, angle = 0

    FOR STARS/POLYGONS: Use separate calls - first straight line (angle=0), then turn (velocity=0)
    FOR CIRCLES: Use simultaneous velocity + angle (e.g., velocity=0.5, angle=0.1, steps=100)

    :param name: turtle name (without forward slash, e.g. 'turtle1')
    :param velocity: forward speed (positive=forward, negative=backward)
    :param lateral: sideways speed (positive=left, negative=right, usually 0)
    :param angle: rotation speed (positive=counterclockwise, negative=clockwise)
    :param steps: duration in seconds (each step = 1 second of movement)
    """
    # Remove any forward slashes from the name
    name = name.replace("/", "")

    # Check if the movement will keep the turtle within bounds
    in_bounds, message = will_be_within_bounds(
        name, velocity, lateral, angle, duration=steps
    )
    if not in_bounds:
        return message

    # Ensure publisher exists
    if name not in cmd_vel_pubs:
        add_cmd_vel_pub(name)

    vel = Twist()
    vel.linear.x, vel.linear.y, vel.linear.z = velocity, lateral, 0.0
    vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, angle

    try:
        node = get_node()
        pub = cmd_vel_pubs[name]

        # Get initial pose for error detection
        initial_pose = get_turtle_pose.invoke({"names": [name]})
        if name not in initial_pose:
            return f"Cannot move turtle '{name}' - turtle not found."

        initial_x = initial_pose[name].x
        initial_y = initial_pose[name].y

        for step in range(steps):
            pub.publish(vel)
            time.sleep(1.0)  # Sleep for 1 second between publishes

            # Allow more time for pose updates to propagate
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)

            # Check for position after each step to detect issues early
            intermediate_pose = get_turtle_pose.invoke({"names": [name]})
            if name in intermediate_pose:
                current_x = intermediate_pose[name].x
                current_y = intermediate_pose[name].y

                # Detect if turtle hit a wall (position clamped at boundary)
                if (
                    current_x <= 0.01
                    or current_x >= 10.99
                    or current_y <= 0.01
                    or current_y >= 10.99
                ):
                    return f"Movement stopped at step {step+1}/{steps} - turtle hit boundary at ({current_x:.3f}, {current_y:.3f}). Check bounds before moving."

        # Allow extra time for final pose to update
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Get final pose
        current_pose = get_turtle_pose.invoke({"names": [name]})
        if name not in current_pose:
            return f"Movement completed, but couldn't verify pose - turtle '{name}' not found."

        # Calculate actual movement distance for validation
        final_x = current_pose[name].x
        final_y = current_pose[name].y
        actual_distance = (
            (final_x - initial_x) ** 2 + (final_y - initial_y) ** 2
        ) ** 0.5

        return (
            f"New Pose ({name}): x={current_pose[name].x:.3f}, y={current_pose[name].y:.3f}, "
            f"theta={current_pose[name].theta:.3f} rads, "
            f"moved {actual_distance:.3f} units"
        )
    except Exception as e:
        return f"Failed to publish {vel} to /{name}/cmd_vel: {e}"


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
    node = get_node()
    client = get_service_client("/reset", Empty)

    if not client.wait_for_service(timeout_sec=5.0):
        return (
            "Failed to reset the turtlesim environment: /reset service not available."
        )

    try:
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is not None:
            # Clear all cached state
            global cmd_vel_pubs, _latest_poses
            # Don't explicitly destroy publishers - let node cleanup handle it
            cmd_vel_pubs.clear()
            _latest_poses.clear()

            # Wait for turtlesim to fully reset and turtle1 to be available
            time.sleep(2.0)

            # Re-add turtle1 publisher and subscription
            add_cmd_vel_pub("turtle1")
            get_pose_subscription("turtle1")

            # Verify turtle1 is actually available by attempting to get its pose
            max_retries = 5
            for attempt in range(max_retries):
                try:
                    # Spin a few times to allow pose messages to arrive
                    for _ in range(10):
                        rclpy.spin_once(node, timeout_sec=0.1)

                    if "turtle1" in _latest_poses:
                        break
                    time.sleep(1.0)
                except Exception:
                    if attempt < max_retries - 1:
                        time.sleep(1.0)
                    else:
                        return "Successfully reset turtlesim but turtle1 may not be fully ready yet. Try your command again in a moment."

            return "Successfully reset the turtlesim environment. Ignore all previous commands, failures, and goals."
        else:
            return "Failed to reset the turtlesim environment: service call failed."
    except Exception as e:
        return f"Failed to reset the turtlesim environment: {e}"


@tool
def set_pen(name: str, r: int, g: int, b: int, width: int, off: int):
    """
    Set the pen color and width for the turtle. The pen is used to draw lines on the turtlesim canvas.

    :param name: name of the turtle
    :param r: red value
    :param g: green value
    :param b: blue value
    :param width: width of the pen.
    :param off: 0=on, 1=off
    """
    # Remove any forward slashes from the name
    name = name.replace("/", "")

    node = get_node()
    client = get_service_client(f"/{name}/set_pen", SetPen)

    if not client.wait_for_service(timeout_sec=5.0):
        return f"Failed to set the pen color for the turtle: /{name}/set_pen service not available."

    try:
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is not None:
            return f"Successfully set the pen color for the turtle: {name}."
        else:
            return "Failed to set the pen color for the turtle: service call failed."
    except Exception as e:
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
    if name not in current_pose:
        return f"Cannot check position - turtle '{name}' not found. Make sure turtlesim is running."

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
def draw_circle(name: str, radius: float = 2.0, speed: float = 1.0) -> str:
    """
    Draw a perfect circle using physics-based circular motion.
    Use this tool specifically for circles, not for other shapes like stars or squares.

    :param name: turtle name
    :param radius: circle radius in grid units (default 2.0, try 1.0-3.0 for best results)
    :param speed: drawing speed (default 1.0, higher = faster but less smooth)
    """
    # Remove any forward slashes from the name
    name = name.replace("/", "")

    # Calculate angular velocity for circular motion: v = r * ω, so ω = v / r
    angular_velocity = speed / radius

    # Calculate time to complete a full circle: t = 2πr / v
    import math

    circle_time = (2 * math.pi * radius) / speed

    # Calculate number of steps (1 step = 1 second)
    steps = int(circle_time) + 1

    try:
        # Skip pose checking to avoid node lifecycle issues - just try to draw the circle
        # publish_twist_to_cmd_vel will handle bounds checking internally

        # Use publish_twist_to_cmd_vel for smooth circular motion
        result = publish_twist_to_cmd_vel.invoke(
            {
                "name": name,
                "velocity": speed,
                "lateral": 0.0,
                "angle": angular_velocity,
                "steps": steps,
            }
        )

        if "not found" in result or "Error" in result:
            return f"Cannot draw circle - turtle '{name}' not found. Make sure turtlesim is running and the turtle exists."

        return f"Drew circle with radius {radius:.2f} and speed {speed:.2f}. {result}"

    except Exception as e:
        return f"Failed to draw circle: {e}"


@tool
def draw_square(name: str, side_length: float = 2.0) -> str:
    """
    Draw a square by moving forward and turning 90 degrees four times.

    :param name: turtle name
    :param side_length: length of each side (default 2.0)
    """
    name = name.replace("/", "")

    try:
        import math

        results = []

        for i in range(4):
            # Move forward for one side
            result = publish_twist_to_cmd_vel.invoke(
                {
                    "name": name,
                    "velocity": side_length,
                    "lateral": 0.0,
                    "angle": 0.0,
                    "steps": 1,
                }
            )
            results.append(f"Side {i+1}: {result}")

            # Turn 90 degrees (π/2 radians)
            turn_result = publish_twist_to_cmd_vel.invoke(
                {
                    "name": name,
                    "velocity": 0.0,
                    "lateral": 0.0,
                    "angle": math.pi / 2,
                    "steps": 1,
                }
            )
            results.append(f"Turn {i+1}: {turn_result}")

        return f"Drew square with side length {side_length}. " + " ".join(results)

    except Exception as e:
        return f"Failed to draw square: {e}"


@tool
def move_to_center(name: str) -> str:
    """
    Move turtle to the center of the turtlesim window (5.5, 5.5) facing right.

    :param name: turtle name
    """
    return teleport_absolute.invoke(
        {"name": name, "x": 5.5, "y": 5.5, "theta": 0.0, "hide_pen": True}
    )


@tool
def set_pen_color(name: str, color: str) -> str:
    """
    Set the pen color using common color names.

    :param name: turtle name
    :param color: color name ('red', 'green', 'blue', 'yellow', 'purple', 'cyan', 'white', 'black')
    """
    color_map = {
        "red": (255, 0, 0),
        "green": (0, 255, 0),
        "blue": (0, 0, 255),
        "yellow": (255, 255, 0),
        "purple": (255, 0, 255),
        "cyan": (0, 255, 255),
        "white": (255, 255, 255),
        "black": (0, 0, 0),
    }

    if color.lower() not in color_map:
        return f"Unknown color '{color}'. Available: {', '.join(color_map.keys())}"

    r, g, b = color_map[color.lower()]
    return set_pen.invoke({"name": name, "r": r, "g": g, "b": b, "width": 1, "off": 0})


@tool
def pen_up(name: str) -> str:
    """Turn off the pen so turtle moves without drawing."""
    return set_pen.invoke({"name": name, "r": 0, "g": 0, "b": 0, "width": 1, "off": 1})


@tool
def pen_down(name: str) -> str:
    """Turn on the pen so turtle draws while moving."""
    return set_pen.invoke(
        {"name": name, "r": 30, "g": 30, "b": 255, "width": 1, "off": 0}
    )


@tool
def validate_shape_fits(
    name: str,
    shape_type: str,
    size: float,
    start_x: float = None,
    start_y: float = None,
) -> str:
    """
    Pre-validate if a shape will fit within turtlesim bounds before attempting to draw it.
    Use this BEFORE starting any multi-step shape drawing to prevent failures.

    :param name: turtle name
    :param shape_type: 'hexagon', 'square', 'circle', 'pentagon', etc.
    :param size: size parameter (side length for polygons, radius for circles)
    :param start_x: optional starting x position (uses current position if not provided)
    :param start_y: optional starting y position (uses current position if not provided)
    """
    import math

    # Get current position if start position not provided
    if start_x is None or start_y is None:
        pose = get_turtle_pose.invoke({"names": [name]})
        if name not in pose:
            return f"Cannot validate - turtle '{name}' not found."
        current_x = pose[name].x if start_x is None else start_x
        current_y = pose[name].y if start_y is None else start_y
    else:
        current_x, current_y = start_x, start_y

    # Calculate bounding box for different shapes
    if shape_type.lower() == "hexagon":
        # Regular hexagon: width = 2 * size, height = sqrt(3) * size
        height = math.sqrt(3) * size
        min_x = current_x - size
        max_x = current_x + size
        min_y = current_y - height / 2
        max_y = current_y + height / 2

    elif shape_type.lower() == "square":
        # Square: width = height = size
        min_x = current_x
        max_x = current_x + size
        min_y = current_y
        max_y = current_y + size

    elif shape_type.lower() == "circle":
        # Circle: diameter = 2 * size (size = radius)
        min_x = current_x - size
        max_x = current_x + size
        min_y = current_y - size
        max_y = current_y + size

    elif shape_type.lower() == "pentagon":
        # Regular pentagon: approximate bounding box
        min_x = current_x - size
        max_x = current_x + size
        min_y = current_y - size
        max_y = current_y + size

    else:
        return f"Unknown shape type '{shape_type}'. Supported: hexagon, square, circle, pentagon."

    # Check if bounding box fits within turtlesim bounds [0,11] x [0,11]
    fits_x = (min_x >= 0) and (max_x <= 11)
    fits_y = (min_y >= 0) and (max_y <= 11)

    if fits_x and fits_y:
        return f"✓ {shape_type.capitalize()} with size {size} WILL FIT starting at ({current_x:.1f}, {current_y:.1f}). Bounding box: ({min_x:.1f}, {min_y:.1f}) to ({max_x:.1f}, {max_y:.1f})"
    else:
        problems = []
        if not fits_x:
            problems.append(
                f"X range ({min_x:.1f} to {max_x:.1f}) exceeds bounds [0, 11]"
            )
        if not fits_y:
            problems.append(
                f"Y range ({min_y:.1f} to {max_y:.1f}) exceeds bounds [0, 11]"
            )

        # Calculate proper suggested position based on actual bounding box
        # We need: min_x >= 0, max_x <= 11, min_y >= 0, max_y <= 11
        # Given the bounding box calculations above, solve for valid starting position

        # Calculate offset from starting position to bounding box edges
        offset_left = current_x - min_x  # how far left the shape extends
        offset_right = max_x - current_x  # how far right the shape extends
        offset_down = current_y - min_y  # how far down the shape extends
        offset_up = max_y - current_y  # how far up the shape extends

        # Calculate valid range for starting position
        min_valid_x = 0 + offset_left  # leftmost valid starting x
        max_valid_x = 11 - offset_right  # rightmost valid starting x
        min_valid_y = 0 + offset_down  # lowest valid starting y
        max_valid_y = 11 - offset_up  # highest valid starting y

        # Suggest position closest to current position within valid range
        better_x = max(min_valid_x, min(max_valid_x, current_x))
        better_y = max(min_valid_y, min(max_valid_y, current_y))

        return f"✗ {shape_type.capitalize()} with size {size} will NOT FIT starting at ({current_x:.1f}, {current_y:.1f}). Problems: {'; '.join(problems)}. Try starting near ({better_x:.1f}, {better_y:.1f}) instead."


@tool
def verify_position_accuracy(
    name: str, expected_x: float, expected_y: float, tolerance: float = 0.1
) -> str:
    """
    Verify that the turtle is at the expected position within tolerance.
    Use this after movements to detect drift and precision issues.

    :param name: turtle name
    :param expected_x: expected x coordinate
    :param expected_y: expected y coordinate
    :param tolerance: acceptable distance from expected position (default 0.1)
    """
    import math

    pose = get_turtle_pose.invoke({"names": [name]})
    if name not in pose:
        return f"Cannot verify position - turtle '{name}' not found."

    actual_x = pose[name].x
    actual_y = pose[name].y

    distance = math.sqrt((actual_x - expected_x) ** 2 + (actual_y - expected_y) ** 2)

    if distance <= tolerance:
        return f"✓ Position accurate: expected ({expected_x:.3f}, {expected_y:.3f}), actual ({actual_x:.3f}, {actual_y:.3f}), error {distance:.3f}"
    else:
        return f"✗ Position drift detected: expected ({expected_x:.3f}, {expected_y:.3f}), actual ({actual_x:.3f}, {actual_y:.3f}), error {distance:.3f} > tolerance {tolerance}. Consider using teleport_absolute to correct."


# Initialize default turtle publisher
initialize_default_turtle()
