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

# This script launches the ROSA demo in Docker

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Set default headless mode
HEADLESS=${HEADLESS:-false}
DEVELOPMENT=${DEVELOPMENT:-false}

# Enable X11 forwarding based on OS
echo "Enabling X11 forwarding..."
case "$(uname)" in
    Linux*)
        export DISPLAY=${DISPLAY:-:0}
        xhost + &>/dev/null || echo "Warning: xhost command failed"
        # Verify X11 is working
        if ! xset q &>/dev/null; then
            echo "Error: X11 forwarding is not working. Please check your X11 server."
            exit 1
        fi
        ;;
    Darwin*)
        # Keep XQuartz's DISPLAY or default to :0
        export DISPLAY=${DISPLAY:-:0}
        xhost + &>/dev/null || true
        
        # Check if XQuartz is running and properly configured
        if ! pgrep -xq "Xquartz" && ! pgrep -xq "X11"; then
            echo "Error: XQuartz is not running. Please start XQuartz and try again."
            exit 1
        fi
        
        # Warn if network connections are disabled
        if ! defaults read org.xquartz.X11 nolisten_tcp 2>/dev/null | grep -q 0; then
            echo "Warning: XQuartz may not allow network connections."
            echo "Enable in: XQuartz Preferences > Security > 'Allow connections from network clients'"
        fi
        ;;
    MINGW*|CYGWIN*|MSYS*)
        export DISPLAY=host.docker.internal:0
        ;;
    *)
        echo "Error: Unsupported operating system."
        exit 1
        ;;
esac

# Build and run the Docker container
CONTAINER_NAME="rosa-turtlesim-demo"

# Detect platform for Apple Silicon
PLATFORM_ARG=""
if [ "$(uname -m)" = "arm64" ]; then
    PLATFORM_ARG="--platform linux/amd64"
fi

echo "Building the $CONTAINER_NAME Docker image..."
docker build $PLATFORM_ARG --build-arg DEVELOPMENT=$DEVELOPMENT -t $CONTAINER_NAME -f Dockerfile . || {
    echo "Error: Docker build failed"
    exit 1
}

echo "Running the Docker container..."
if [ "$(uname)" = "Darwin" ]; then
    # macOS: Use host.docker.internal for X11
    docker run -it --rm --name $CONTAINER_NAME \
        $PLATFORM_ARG \
        -e DISPLAY=host.docker.internal:0 \
        -e HEADLESS=$HEADLESS \
        -e DEVELOPMENT=$DEVELOPMENT \
        -v "$PWD/src":/app/src \
        -v "$PWD/tests":/app/tests \
        $CONTAINER_NAME
else
    # Linux/WSL: Use unix socket
    docker run -it --rm --name $CONTAINER_NAME \
        -e DISPLAY=$DISPLAY \
        -e HEADLESS=$HEADLESS \
        -e DEVELOPMENT=$DEVELOPMENT \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$PWD/src":/app/src \
        -v "$PWD/tests":/app/tests \
        --network host \
        $CONTAINER_NAME
fi

# Disable X11 forwarding
xhost - &>/dev/null || true

exit 0
