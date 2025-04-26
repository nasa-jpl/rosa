FROM osrf/ros:noetic-desktop AS rosa-ros1
LABEL authors="Rob Royce"

ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false
ARG DEVELOPMENT=false

# Enable BuildKit caching for apt packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install linux packages
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-$(rosversion -d)-turtlesim \
    locales \
    xvfb \
    python3.9 \
    python3-pip \
    libgl1-mesa-glx && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages with pip caching
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install -U python-dotenv catkin_tools

# Set up ROS environment
RUN rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "alias start='catkin build && source devel/setup.bash && roslaunch turtle_agent agent.launch'" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

# Copy application code
COPY . /app/
WORKDIR /app/

# Install application dependencies with pip caching
RUN --mount=type=cache,target=/root/.cache/pip \
    /bin/bash -c 'if [ "$DEVELOPMENT" = "true" ]; then \
    python3.9 -m pip install --user -e .; \
    else \
    python3.9 -m pip install -U jpl-rosa>=1.0.7; \
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
    