# ROS Agent (ROSA)

ROSA is an AI agent that can be used to interact with ROS (Robot Operating System) and perform various tasks. 
It is built using [Langchain](https://python.langchain.com/v0.2/docs/introduction/) and the 
[ROS](https://www.ros.org/) framework.

## Installation

Requirements:
- Python 3.9 or higher
- ROS Noetic (or higher)

**Note:** ROS Noetic uses Python3.8, but LangChain requires Python3.9 or higher. To use ROSA with ROS Noetic,
you will need to create a virtual environment with Python3.9 or higher and install ROSA in that environment.

Use pip to install ROSA:

```bash
pip3 install jpl-rosa
```

# TurtleSim Demo
We have included a demo that uses ROSA to control the TurtleSim robot in simulation. To run the demo, you will need
to have Docker installed on your machine.

## Setup

1. Clone this repository
2. Configure the LLM in `src/turtle_agent/scripts/llm.py`
3. Run the demo script: `./demo.sh`
4. Start ROSA in the new Docker session: `catkin build && source devel/setup.bash && roslaunch turtle_agent agent`
5. Run example queries: `examples`


# Adapting ROSA for Your Robot

ROSA is designed to be easily adaptable to different robots and environments. To adapt ROSA for your robot, you will 
can either (1) create a new class that inherits from the `ROSA` class, or (2) create a new instance of the `ROSA` class
and pass in the necessary parameters. The first option is recommended if you need to make significant changes to the
agent's behavior, while the second option is recommended if you want to use the agent with minimal changes.

In either case, ROSA is adapted by providing it with a new set of tools and/or prompts. The tools are used to interact
with the robot and the ROS environment, while the prompts are used to guide the agents behavior.

## Adding Tools
There are two methods for adding tools to ROSA:
1. Pass in a list of @tool functions using the `tools` parameter.
2. Pass in a list of Python packages containing @tool functions using the `tool_packages` parameter.

The first method is recommended if you have a small number of tools, while the second method is recommended if you have
a large number of tools or if you want to organize your tools into separate packages.

**Hint:** check `src/turtle_agent/scripts/turtle_agent.py` for examples on how to use both methods.

## Adding Prompts
To add prompts to ROSA, you need to create a new instance of the `RobotSystemPrompts` class and pass it to the `ROSA`
constructor using the `prompts` parameter. The `RobotSystemPrompts` class contains the following attributes:

- `embodiment_and_persona`: Gives the agent a sense of identity and helps it understand its role. 
- `about_your_operators`: Provides information about the operators who interact with the robot, which can help the agent
  understand the context of the interaction.
- `critical_instructions`: Provides critical instructions that the agent should follow to ensure the safety and
  well-being of the robot and its operators.
- `constraints_and_guardrails`: Gives the robot a sense of its limitations and informs its decision-making process.
- `about_your_environment`: Provides information about the physical and digital environment in which the robot operates.
- `about_your_capabilities`: Describes what the robot can and cannot do, which can help the agent understand its
  limitations. 
- `nuance_and_assumptions`: Provides information about the nuances and assumptions that the agent should consider when
  interacting with the robot.
- `mission_and_objectives`: Describes the mission and objectives of the robot, which can help the agent understand its
  purpose and goals.
- `environment_variables`: Provides information about the environment variables that the agent should consider when
  interacting with the robot. e.g. $ROS_MASTER_URI, or $ROS_IP.

## Example
Here is a quick and easy example showing how to add new tools and prompts to ROSA:
```python
from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts

@tool
def move_forward(distance: float) -> str:
    """
    Move the robot forward by the specified distance.
    
    :param distance: The distance to move the robot forward.
    """
    # Your code here ...
    return f"Moving forward by {distance} units."

prompts = RobotSystemPrompts(
    embodiment_and_persona="You are a cool robot that can move forward."
)

llm = get_your_llm_here()
rosa = ROSA(ros_version=1, llm=llm, tools=[move_forward])
rosa.invoke("Move forward by 2 units.")
```
