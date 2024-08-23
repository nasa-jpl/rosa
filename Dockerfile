FROM osrf/ros:noetic-desktop as rosa-ros1
LABEL authors="Rob Royce"

ENV DEBIAN_FRONTEND=noninteractive

# Install linux packages
RUN apt-get update && apt-get install -y \
    locales \
    lsb-release \
    git \
    subversion \
    nano \
    terminator \
    xterm \
    wget \
    curl \
    htop \
    gnome-terminal \
    libssl-dev \
    build-essential \
    dbus-x11 \
    software-properties-common \
    build-essential \
    ssh \
    ros-$(rosversion -d)-turtlesim

# RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3.9
RUN apt-get update && apt-get install -y python3-pip
RUN python3 -m pip install -U python-dotenv catkin_tools
RUN python3.9 -m pip install -U jpl-rosa>=1.0.5

# Configure ROS
RUN rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

COPY . /app/
WORKDIR /app/

# Uncomment this line to test with local ROSA package
# RUN python3.9 -m pip install --user -e .

# Run roscore in the background, then run `rosrun turtlesim turtlesim_node` in a new terminal, finally run main.py in a new terminal
CMD /bin/bash -c 'source /opt/ros/noetic/setup.bash &&  \
    roscore &  \
    sleep 2 &&  \
    rosrun turtlesim turtlesim_node > /dev/null &  \
    sleep 3 &&  \
    echo "" && \
    echo "Run \`catkin build && source devel/setup.bash && roslaunch turtle_agent agent\` to launch the ROSA-TurtleSim demo." &&  \
    /bin/bash'
