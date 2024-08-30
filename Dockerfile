FROM osrf/ros:noetic-desktop AS rosa-ros1
LABEL authors="Rob Royce"

ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false
ARG DEVELOPMENT=false

# Install linux packages
RUN apt-get update && apt-get install -y \
    ros-$(rosversion -d)-turtlesim \
    locales \
    xvfb \
    python3.9 \
    python3-pip

# RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install -U python-dotenv catkin_tools
RUN rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "alias start='catkin build && source devel/setup.bash && roslaunch turtle_agent agent.launch'" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

COPY . /app/
WORKDIR /app/

# Modify the RUN command to use ARG
RUN /bin/bash -c 'if [ "$DEVELOPMENT" = "true" ]; then \
    python3.9 -m pip install --user -e .; \
    else \
    python3.9 -m pip install -U jpl-rosa>=1.0.5; \
    fi'

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    roscore > /dev/null 2>&1 & \
    sleep 5 && \
    if [ \"$HEADLESS\" = \"false\" ]; then \
    rosrun turtlesim turtlesim_node & \
    else \
    xvfb-run -a -s \"-screen 0 1920x1080x24\" rosrun turtlesim turtlesim_node & \
    fi && \
    sleep 5 && \
    echo \"Run \\`start\\` to build and launch the ROSA-TurtleSim demo.\" && \
    /bin/bash"]
