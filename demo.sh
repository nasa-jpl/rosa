#!/usr/bin/env bash
# Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This script is used to launch the ROSA demo in Docker

# Check if the user has docker installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install docker and try again."
    exit 1
fi

# Check if the user has docker-compose installed
if ! command -v docker-compose &> /dev/null; then
    echo "Docker-compose is not installed. Please install docker-compose and try again."
    exit 1
fi

# Get the platform
platform='unknown'
unamestr=$(uname)
if [ "$unamestr" == "Linux" ]; then
    platform='linux'
elif [ "$unamestr" == "Darwin" ]; then
    platform='mac'
elif [ "$unamestr" == "Windows" ]; then
    platform='win'
fi

# Enable X11 forwarding for mac and linux
if [ "$platform" == "mac" ] || [ "$platform" == "linux" ]; then
    echo "Enabling X11 forwarding..."
    export DISPLAY=host.docker.internal:0
    xhost +
elif [ "$platform" == "win" ]; then
    # Windows support is experimental
    echo "The ROSA-TurtleSim demo has not been tested on Windows. It may not work as expected."
    read -p "Do you want to continue? (y/n): " confirm
    if [ "$confirm" != "y" ]; then
        echo "Please check back later for Windows support."
        exit 0
    fi
    export DISPLAY=host.docker.internal:0
fi

# Build the docker image
echo "Building the docker image..."
docker build -t rosa -f Dockerfile .

# Run the docker container
echo "Running the docker container..."
docker run -it --rm --name rosa \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ./src:/app/src \
       -v ./data:/root/data \
       --network host \
       rosa

# Disable X11 forwarding
xhost -

exit 0
