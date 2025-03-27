#!/bin/bash
source /opt/ros/noetic/setup.bash

# Start ROS core
roscore > /dev/null 2>&1 &
sleep 5

# Start turtlesim
if [ "$HEADLESS" = "false" ]; then
    rosrun turtlesim turtlesim_node &
else
    xvfb-run -a -s "-screen 0 1920x1080x24" rosrun turtlesim turtlesim_node &
fi
sleep 5

# Start web GUI if enabled
if [ "$WEB_GUI" = "true" ]; then
    echo "Starting web GUI on port 5000..."
    echo "Access the web interface at: http://localhost:5000"
    catkin build && source devel/setup.bash && roslaunch turtle_agent web_gui.launch &
else
    echo "Run 'start' to build and launch the ROSA-TurtleSim demo."
    echo "Run 'web_gui' to launch the web interface."
fi

# Keep the container running
exec /bin/bash 