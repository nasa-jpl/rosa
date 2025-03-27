#!/bin/bash
source /opt/ros/noetic/setup.bash

# Function to check if Python modules are installed
check_python_modules() {
    echo "Checking Python modules..."
    if ! python3.9 -c "import flask" 2>/dev/null; then
        echo "Error: Flask module not installed for Python 3.9"
        echo "Installing Flask..."
        pip3.9 install flask
    else
        echo "Flask is installed"
    fi
}

# Start ROS core
echo "Starting ROS core..."
roscore > /dev/null 2>&1 &
sleep 5

# Start turtlesim
echo "Starting turtlesim..."
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
    check_python_modules
    catkin build && source devel/setup.bash && roslaunch turtle_agent web_gui.launch &
    # Check if web GUI started successfully
    sleep 5
    if ! pgrep -f "web_gui.py" > /dev/null; then
        echo "Warning: Web GUI failed to start. Check the logs for details."
    fi
else
    echo "Run 'start' to build and launch the ROSA-TurtleSim demo."
    echo "Run 'web_gui' to launch the web interface."
fi

# Keep the container running
exec /bin/bash 