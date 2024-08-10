# ROS Agent (ROSA)

ROSA is an AI agent that can be used to interact with ROS (Robot Operating System) and perform various tasks. 
It is built using the Langchain framework and the [ROS](https://www.ros.org/) framework.

## Installation

Use pip to install ROSA:

```bash
pip install jpl-rosa
```

**Important:** ROS Noetic runs on Python 3.8, but LangChain is only available for Python >= 3.9. So you will
need to install Python3.9 separately, and run ROSA outside the ROS environment. This restriction is not true
for ROS2 variants.


# TurtleSim Demo
We have included a demo that uses ROSA to control the TurtleSim simulator. 

## Configure your LLM
You will need to configure your LLM by setting the environment variables found in `.env`. You will also need
to ensure the correct LLM is configured in the `src/turtle_agent/turtle_agent.py` file, specifically in the
`get_llm()` function.

After that is configured properly, you can run the demo using the following command:

```bash
./demo.sh
```

The above command will start Docker and launch the turtlesim node. To start ROSA, you can run the following command 
the new Docker session:

```bash
catkin build && source devel/setup.bash && roslaunch turtle_agent agent
```

## Example Queries
After launching the agent, you can get a list of example queries by typing `examples` in the terminal. 
You can then run any of the example queries by typing the query number (e.g. 2) and pressing enter.
