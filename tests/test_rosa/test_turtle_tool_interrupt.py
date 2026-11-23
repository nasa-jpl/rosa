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

"""Ctrl+C during a TurtleSim move must reach the agent loop.

The turtle tools are loaded with small stand-ins for rospy and the ROS
message/service modules, so these tests run without ROS installed.
"""

import importlib.util
import os
import signal
import sys
import threading
import time
import types
import unittest
from unittest import mock

TOOLS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "src", "turtle_agent", "scripts", "tools", "turtle.py",
)


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class _Publisher:
    def __init__(self, *args, **kwargs):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.linear.y, msg.angular.z))


def _pose(*args, **kwargs):
    return types.SimpleNamespace(
        x=5.5, y=5.5, theta=0.0, linear_velocity=0.0, angular_velocity=0.0
    )


def _load_turtle_tools():
    rospy = types.ModuleType("rospy")
    rospy.Publisher = _Publisher
    rospy.sleep = time.sleep
    rospy.ROSException = type("ROSException", (Exception,), {})
    rospy.ServiceException = type("ServiceException", (Exception,), {})
    rospy.wait_for_service = lambda *args, **kwargs: None
    rospy.ServiceProxy = lambda *args, **kwargs: (lambda *a, **k: None)
    rospy.wait_for_message = _pose

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = _Twist
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Empty = object
    turtlesim = types.ModuleType("turtlesim")
    turtlesim_msg = types.ModuleType("turtlesim.msg")
    turtlesim_msg.Pose = object
    turtlesim_srv = types.ModuleType("turtlesim.srv")
    for name in ("Spawn", "TeleportAbsolute", "TeleportRelative", "Kill", "SetPen"):
        setattr(turtlesim_srv, name, object)

    stand_ins = {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
        "turtlesim": turtlesim,
        "turtlesim.msg": turtlesim_msg,
        "turtlesim.srv": turtlesim_srv,
    }
    # Import LangChain before patching: patch.dict restores sys.modules on exit,
    # and a second LangChain import in the same process fails.
    import langchain.agents  # noqa: F401

    with mock.patch.dict(sys.modules, stand_ins):
        spec = importlib.util.spec_from_file_location(
            "turtle_tools_under_test", TOOLS_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class TestTurtleToolInterrupt(unittest.TestCase):
    """Ctrl+C mid-move stops the turtle and is not swallowed by the tool."""

    @classmethod
    def setUpClass(cls):
        cls.tools = _load_turtle_tools()

    def setUp(self):
        if threading.current_thread() is not threading.main_thread():
            self.skipTest("signals are delivered to the main thread only")
        self.pub = _Publisher()
        self.tools.cmd_vel_pubs["turtle1"] = self.pub
        # Same effect as GracefulInterruptHandler: SIGINT raises KeyboardInterrupt.
        self.previous_handler = signal.signal(signal.SIGINT, signal.default_int_handler)
        self.timer = None

    def tearDown(self):
        if self.timer is not None:
            self.timer.cancel()
        signal.signal(signal.SIGINT, self.previous_handler)

    def _press_ctrl_c_after(self, seconds):
        self.timer = threading.Timer(seconds, os.kill, (os.getpid(), signal.SIGINT))
        self.timer.start()

    def test_interrupt_during_move_reaches_caller(self):
        self._press_ctrl_c_after(0.3)
        with self.assertRaises(KeyboardInterrupt):
            self.tools.publish_twist_to_cmd_vel.invoke(
                {"name": "turtle1", "velocity": 1.0, "lateral": 0.0, "angle": 0.0, "steps": 3}
            )

    def test_interrupt_during_move_sends_zero_velocity(self):
        self._press_ctrl_c_after(0.3)
        with self.assertRaises(KeyboardInterrupt):
            self.tools.publish_twist_to_cmd_vel.invoke(
                {"name": "turtle1", "velocity": 1.0, "lateral": 0.0, "angle": 0.0, "steps": 3}
            )
        self.assertEqual(self.pub.sent[-1], (0.0, 0.0, 0.0))

    def test_interrupt_during_rectangle_skips_remaining_edges(self):
        self._press_ctrl_c_after(0.3)
        with self.assertRaises(KeyboardInterrupt):
            self.tools.draw_rectangle.invoke(
                {"name": "turtle1", "x": 2.0, "y": 2.0, "width": 1.0, "height": 1.0}
            )
        moving = [msg for msg in self.pub.sent if msg != (0.0, 0.0, 0.0)]
        self.assertEqual(len(moving), 1, "only the first edge should have started")


if __name__ == "__main__":
    unittest.main()
