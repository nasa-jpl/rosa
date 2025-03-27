FROM osrf/ros:noetic-desktop AS rosa-ros1
LABEL authors="Rob Royce"

ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false
ENV WEB_GUI=false
ARG DEVELOPMENT=false

# Install linux packages
RUN apt-get update && apt-get install -y \
    ros-$(rosversion -d)-turtlesim \
    locales \
    xvfb \
    python3.9 \
    python3-pip

# RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install -U python-dotenv catkin_tools flask
RUN rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "alias start='catkin build && source devel/setup.bash && roslaunch turtle_agent agent.launch'" >> /root/.bashrc && \
    echo "alias web_gui='catkin build && source devel/setup.bash && roslaunch turtle_agent web_gui.launch'" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

COPY . /app/
WORKDIR /app/

RUN /bin/bash -c 'if [ "$DEVELOPMENT" = "true" ]; then \
    python3.9 -m pip install --user -e .; \
    else \
    python3.9 -m pip install -U jpl-rosa>=1.0.7; \
    fi'

# Use the startup script
CMD ["/app/start.sh"]
