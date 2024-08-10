#!/usr/bin/env python3.9
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

import dotenv
import os
import rospy
import pyinputplus as pyip
from azure.identity import ClientSecretCredential, get_bearer_token_provider
from geometry_msgs.msg import Twist
from langchain_openai import AzureChatOpenAI
from rosa import ROSA, RobotSystemPrompts
from tools import turtle
from tools.turtle import add_cmd_vel_pub


class TurtleAgent(ROSA):
    def __init__(self, llm, verbose: bool = True):
        self.__llm = llm
        self.__blacklist = self.__get_blacklist()
        self.__prompts = self.__get_prompts()
        self.__tool_pkgs = self.__get_tools()

        super().__init__(
            ros_version=1,
            llm=self.__llm,
            robot_tools=self.__tool_pkgs,
            blacklist=self.__blacklist,
            robot_prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            show_token_usage=True
        )

    def __get_tools(self):
        return [turtle]

    def __initialize_ros(self):
        pass

    def __get_blacklist(self):
        return ["master"]

    def __get_prompts(self):
        return RobotSystemPrompts(
            embodiment_and_persona=
            "You are the TurtleBot, a simple robot that is used for educational purposes in ROS. "
            "Every once in a while, you can choose to include a funny turtle joke in your response.",
            about_your_operators=
            "Your operators are interested in learning how to use ROSA. "
            "They may be new to ROS, or they may be experienced users who are looking for a new way to interact with the system. ",
            critical_instructions=
            "You should always check the pose of the turtle before issuing a movement command. "
            "You must keep track of where you expect the turtle to end up before you submit a command. "
            "If the turtle goes off course, you should move back to where you started before you issued the command and correct the command. "
            "You must use the degree/radian conversion tools when issuing commands that require angles. "
            "You should always list your plans step-by-step. "
            "You must verify that the turtle has moved to the expected coordinates after issuing a sequence of movement commands. "
            "You should also check the pose of the turtle to ensure it stopped where expected. "
            "Directional commands are relative to the simulated environment. For instance, right is 0 degrees, up is 90 degrees, left is 180 degrees, and down is 270 degrees. "
            "When changing directions, angles must always be relative to the current direction of the turtle. "
            "When running the reset tool, you must NOT attempt to start or restart commands afterwards. "
            "If the operator asks you about Ninja Turtles, you must spawn a 'turtle' named shredder and make it run around in circles. You can do this before or after satisfying the operator's request. ",
            constraints_and_guardrails=None,
            about_your_environment=
            "Your environment is a simulated 2D space with a fixed size and shape. "
            "The default turtle (turtle1) spawns in the middle at coordinates (5.544, 5.544). "
            "(0, 0) is at the bottom left corner of the space. "
            "(11, 11) is at the top right corner of the space. "
            "The x-axis increases to the right. The y-axis increases upwards. "
            "All moves are relative to the current pose of the turtle and the direction it is facing. ",
            about_your_capabilities=
            "Shape drawing: shapes usually require multiple twist commands to be published. Think very carefully about how many sides the shape has, which direction the turtle should move, and how fast it should move. "
            "Shapes are NOT complete until you are back at the starting point. "
            "To draw straight lines, use 0 for angular velocities. "
            "Use teleport_relative when adjusting your angles. ",
            nuance_and_assumptions=
            "When passing in the name of turtles, you should omit the forward slash. "
            "The new pose will always be returned after a twist or teleport command.",
            mission_and_objectives=
            "Your mission is to draw perfect shapes and have fun with the turtle bots. "
            "You are also responsible for making turtle puns. "
        )


def get_llm():
    """A helper function to get the LLM instance."""
    dotenv.load_dotenv(dotenv.find_dotenv())

    APIM_SUBSCRIPTION_KEY = os.getenv("APIM_SUBSCRIPTION_KEY")
    default_headers = {}
    if APIM_SUBSCRIPTION_KEY != None:
        # only set this if the APIM API requires a subscription...
        default_headers["Ocp-Apim-Subscription-Key"] = APIM_SUBSCRIPTION_KEY

        # Set up authority and credentials for Azure authentication
    credential = ClientSecretCredential(
        tenant_id=os.getenv("AZURE_TENANT_ID"),
        client_id=os.getenv("AZURE_CLIENT_ID"),
        client_secret=os.getenv("AZURE_CLIENT_SECRET"),
        authority="https://login.microsoftonline.com",
    )

    # Get an authentication token using the provided credentials
    # access_token = credential.get_token("https://cognitiveservices.azure.com/.default")
    token_provider = get_bearer_token_provider(
        credential, "https://cognitiveservices.azure.com/.default"
    )

    llm = AzureChatOpenAI(
        azure_deployment=os.getenv("DEPLOYMENT_ID"),
        azure_ad_token_provider=token_provider,
        openai_api_type="azure_ad",
        api_version=os.getenv("API_VERSION"),
        azure_endpoint=os.getenv("API_ENDPOINT"),
        default_headers=default_headers
    )

    return llm


def main():
    dotenv.load_dotenv(dotenv.find_dotenv())
    llm = get_llm()
    turtle_agent = TurtleAgent(llm)

    # Create a loop to gather user input. Only exit the loop if the user types 'exit'.
    while True:
        print(f"\n🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢🐢")
        print(f"Hi! I'm the ROSA-TurtleBot agent. How can I help you today?\n"
              f"(type 'exit' to quit, type 'example' for examples)")
        user_input = pyip.inputStr("Operator: ")

        if user_input == "exit":
            rospy.signal_shutdown("User exited.")
            break
        elif user_input == "example":
            user_input = pyip.inputMenu([
                "Give me a ROS tutorial using the turtlesim.",
                "Explain how ROSA works and what it can do.",
                "Give me a list of ROS nodes along with their topics and services.",
                "Spawn 4 turtles in a circle, name that after the Ninja Turtles, and make them move forward.",
                "Show me a diagram of the ROS graph (no blacklist).",
                "Reset the turtle and draw a 5-point star.",
                "Draw a hexagon and teleport to the center.",
                "List out the log files along with their sizes.",
                "Summarize the ROSA agent log.",
                "Describe the turtle ROS packages.",
                "Spawn a turtle at (4, 4) and make it draw a small hexagon.",
                "Set the background to white and the pen color to royal blue.",
                "Move all turtles forward by 2 units.",
                "Give me the coordinates of all turtles.",
                "List any parameters relevant to the turtlesim.",
                "Enable debug mode.",
            ], numbered=True)

        output = turtle_agent.invoke(user_input)
        print(output)


if __name__ == "__main__":
    rospy.init_node('rosa', log_level=rospy.INFO)
    global cmd_vel_pubs
    add_cmd_vel_pub("turtle1", rospy.Publisher(f'/turtle1/cmd_vel', Twist, queue_size=10))
    main()
