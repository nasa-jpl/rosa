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

from math import cos, sin
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
    elif x < 0 or x > 11 or y < 0 or y > 11:
        return False, f"({x}, {y}) will be out of bounds. Range is [0, 11] for each."


def will_be_within_bounds(name: str, linear_velocity: tuple, angular: float) -> tuple:
    """Check if the turtle will be within bounds after publishing a twist command."""
    # Get the current pose of the turtle
    rospy.loginfo(
        f"Checking if {name} will be within bounds after publishing a twist command."
    )

    pose = get_turtle_pose.invoke({"names": [name]})
    current_x = pose[name].x
    current_y = pose[name].y
    current_theta = pose[name].theta

    # Use trigonometry to calculate the new x, y coordinates
    x_displacement = linear_velocity[0] * cos(current_theta)
    y_displacement = linear_velocity[0] * sin(current_theta)

    # Calculate the new x, y coordinates. If the
    new_x = current_x + x_displacement
    new_y = current_y + y_displacement

    # Check if the new x, y coordinates are within bounds
    in_bounds, _ = within_bounds(new_x, new_y)
    if not in_bounds:
        return (
            False,
            f"This command will move the turtle out of bounds to ({new_x}, {new_y}).",
        )

    return within_bounds(new_x, new_y)


@tool
def spawn_turtle(name: str, x: float, y: float, theta: float) -> str:
    """
    Spawn a turtle at the given x, y, and theta coordinates.

    :param name: name of the turtle.
    :param x: x-coordinate.
    :param y: y-coordinate.
    :param theta: angle.
    """
    in_bound, message = within_bounds(x, y)
    if not in_bound:
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
        rospy.loginfo(f"Turtle ({name}) spawned at x: {x}, y: {y}, theta: {theta}.")

        global cmd_vel_pubs
        cmd_vel_pubs[name] = rospy.Publisher(f"/{name}/cmd_vel", Twist, queue_size=10)

        return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
    except Exception as e:
        rospy.logerr(f"Failed to spawn {name}: {e}")
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
            rospy.loginfo(f"Successfully killed turtle ({name}).")

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
        rospy.loginfo("Successfully cleared the turtlesim background.")
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


@tool
def teleport_absolute(
    name: str, x: float, y: float, theta: float, hide_pen: bool = True
):
    """
    Teleport a turtle to the given x, y, and theta coordinates.

    :param name: name of the turtle
    :param x: The x-coordinate, range: [0, 11]
    :param y: The y-coordinate, range: [0, 11]
    :param theta: angle
    :param hide_pen: True to hide the pen (do not show movement trace on screen), False to show the pen
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

        rospy.loginfo(f"Teleported {name} to ({x}, {y}) at {theta} radians.")
        return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
    except rospy.ServiceException as e:
        return f"Failed to teleport the turtle: {e}"


@tool
def teleport_relative(name: str, linear: float, angular: float):
    """
    Teleport a turtle relative to its current position.

    :param name: name of the turtle
    :param linear: linear distance
    :param angular: angular distance
    """
    in_bounds, message = will_be_within_bounds(name, (linear, 0.0, 0.0), angular)
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
        rospy.loginfo(f"Teleported {name} by (linear={linear}, angular={angular}).")
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
    Publish a Twist message to the /{name}/cmd_vel topic to move a turtle robot.
    Use a combination of linear and angular velocities to move the turtle in the desired direction.

    :param name: name of the turtle (do not include the forward slash)
    :param velocity: linear velocity, where positive is forward and negative is backward
    :param lateral: lateral velocity, where positive is left and negative is right
    :param angle: angular velocity, where positive is counterclockwise and negative is clockwise
    :param steps: Number of times to publish the twist message
    """

    # Test the effects of publishing a twist with linear=(1.0, -1.0) and angular_z=1.0

    # Remove any forward slashes from the name
    name = name.replace("/", "")
    vel = Twist()
    vel.linear.x, vel.linear.y, vel.linear.z = velocity, lateral, 0.0
    vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, angle

    try:
        global cmd_vel_pubs
        pub = cmd_vel_pubs[name]

        for _ in range(steps):
            pub.publish(vel)
            rospy.sleep(1)
        rospy.loginfo(
            f"Published Twist (/{name}/cmd_vel): linear=({velocity}, {lateral}), angular={angle}."
        )
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
            "linear_velocity": (0.0, 0.0, 0.0),
            "angular_velocity": (0.0, 0.0, 0.0),
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
        rospy.loginfo("Successfully reset the turtlesim environment.")

        # Clear the cmd_vel publishers
        global cmd_vel_pubs
        cmd_vel_pubs.clear()
        cmd_vel_pubs["turtle1"] = rospy.Publisher(
            f"/turtle1/cmd_vel", Twist, queue_size=10
        )

        return "Successfully reset the turtlesim environment. Ignore all previous commands, failures, and goals."
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset the turtlesim environment: {e}")
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
