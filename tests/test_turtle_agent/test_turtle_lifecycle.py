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

import importlib
import sys
import types
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "src" / "turtle_agent" / "scripts"
sys.path.insert(0, str(_SCRIPTS))


class FakeTool:
    def __init__(self, func):
        self.func = func

    def invoke(self, args):
        return self.func(**args)


def fake_tool(func):
    return FakeTool(func)


class FakeRospy(types.ModuleType):
    class ROSException(Exception):
        pass

    class ServiceException(Exception):
        pass

    def __init__(self):
        super().__init__("rospy")
        self.calls = []
        self.warnings = []

    def Publisher(self, topic, msg_type, queue_size=10):
        return {"topic": topic, "msg_type": msg_type, "queue_size": queue_size}

    def wait_for_service(self, name, timeout=5):
        self.calls.append(("wait_for_service", name, timeout))

    def ServiceProxy(self, name, service_type):
        def proxy(*args, **kwargs):
            self.calls.append(("service", name, args, kwargs))

        return proxy

    def logwarn(self, message, *args):
        self.warnings.append(message % args if args else message)


class FakeListener:
    def __init__(self):
        self.spawned = []
        self.killed = []

    def on_turtle_spawned(self, name):
        self.spawned.append(name)

    def on_turtle_killed(self, name):
        self.killed.append(name)


class TestTurtleLifecycleHooks(unittest.TestCase):
    def setUp(self):
        self.old_modules = {}
        self.stub_names = [
            "rospy",
            "geometry_msgs",
            "geometry_msgs.msg",
            "langchain",
            "langchain.agents",
            "std_srvs",
            "std_srvs.srv",
            "turtlesim",
            "turtlesim.msg",
            "turtlesim.srv",
            "turtle_lifecycle",
            "tools.turtle",
        ]
        for name in self.stub_names:
            self.old_modules[name] = sys.modules.get(name)
            sys.modules.pop(name, None)

        self.rospy = FakeRospy()
        sys.modules["rospy"] = self.rospy

        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.Twist = object
        geometry_msgs.msg = geometry_msgs_msg
        sys.modules["geometry_msgs"] = geometry_msgs
        sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

        langchain = types.ModuleType("langchain")
        langchain_agents = types.ModuleType("langchain.agents")
        langchain_agents.tool = fake_tool
        langchain.agents = langchain_agents
        sys.modules["langchain"] = langchain
        sys.modules["langchain.agents"] = langchain_agents

        std_srvs = types.ModuleType("std_srvs")
        std_srvs_srv = types.ModuleType("std_srvs.srv")
        std_srvs_srv.Empty = object
        std_srvs.srv = std_srvs_srv
        sys.modules["std_srvs"] = std_srvs
        sys.modules["std_srvs.srv"] = std_srvs_srv

        turtlesim = types.ModuleType("turtlesim")
        turtlesim_msg = types.ModuleType("turtlesim.msg")
        turtlesim_msg.Pose = object
        turtlesim_srv = types.ModuleType("turtlesim.srv")
        turtlesim_srv.Spawn = object
        turtlesim_srv.TeleportAbsolute = object
        turtlesim_srv.TeleportRelative = object
        turtlesim_srv.Kill = object
        turtlesim_srv.SetPen = object
        turtlesim.msg = turtlesim_msg
        turtlesim.srv = turtlesim_srv
        sys.modules["turtlesim"] = turtlesim
        sys.modules["turtlesim.msg"] = turtlesim_msg
        sys.modules["turtlesim.srv"] = turtlesim_srv

        self.turtle = importlib.import_module("tools.turtle")

    def tearDown(self):
        self.turtle.configure_turtle_lifecycle_listener(None)
        sys.modules.pop("turtle_lifecycle", None)
        sys.modules.pop("tools.turtle", None)
        for name, module in self.old_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def test_spawn_success_notifies_listener(self):
        listener = FakeListener()
        self.turtle.configure_turtle_lifecycle_listener(listener)

        output = self.turtle.spawn_turtle.invoke(
            {"name": "turtle2", "x": 1.0, "y": 2.0, "theta": 0.0}
        )

        self.assertIn("turtle2 spawned", output)
        self.assertEqual(listener.spawned, ["turtle2"])

    def test_kill_success_notifies_listener(self):
        listener = FakeListener()
        self.turtle.configure_turtle_lifecycle_listener(listener)

        output = self.turtle.kill_turtle.invoke({"names": ["turtle2"]})

        self.assertIn("Successfully killed turtle2", output)
        self.assertEqual(listener.killed, ["turtle2"])


if __name__ == "__main__":
    unittest.main()
