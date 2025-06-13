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
case "$(uname)" in
    Linux*)
        echo "Enabling X11 forwarding for Linux..."
        # If running under WSL, use :0 for DISPLAY
        if [ -f /proc/version ] && grep -q "WSL" /proc/version; then
            export DISPLAY=:0
        else
            export DISPLAY=${DISPLAY:-:0}
        fi
        xhost +
        ;;
    Darwin*)
        echo "Enabling X11 forwarding for macOS..."
        # Check if XQuartz is running (multiple possible process names)
        if ! pgrep -i "xquartz\|x11" > /dev/null && ! ps aux | grep -i "[Xx]quartz" > /dev/null; then
            echo "Warning: XQuartz does not appear to be running. Please start XQuartz and try again."
            echo "You can install XQuartz from: https://www.xquartz.org/"
        fi
        # Use local display for xhost commands
        LOCAL_DISPLAY=${DISPLAY:-:0}
        echo "Allowing localhost connections with DISPLAY=$LOCAL_DISPLAY"
        DISPLAY=$LOCAL_DISPLAY xhost +localhost
        # Set DISPLAY for Docker container
        export DISPLAY=host.docker.internal:0
        ;;
    MINGW*|CYGWIN*|MSYS*)
        echo "Enabling X11 forwarding for Windows..."
        export DISPLAY=host.docker.internal:0
        ;;
    *)
        echo "Error: Unsupported operating system."
        exit 1
        ;;
esac

# Check if X11 forwarding is working
LOCAL_DISPLAY=${DISPLAY:-:0}
echo "Testing X11 connection with DISPLAY=$LOCAL_DISPLAY"
if ! DISPLAY=$LOCAL_DISPLAY xset q &>/dev/null; then
    echo "Error: X11 forwarding is not working with DISPLAY=$LOCAL_DISPLAY"
    echo "Current DISPLAY variable: ${DISPLAY:-<not set>}"
    echo "Trying alternative display settings..."
    
    # Try common display alternatives
    for test_display in ":0" ":1" "localhost:0" "127.0.0.1:0"; do
        echo "Testing DISPLAY=$test_display"
        if DISPLAY=$test_display xset q &>/dev/null 2>&1; then
            echo "Success! Using DISPLAY=$test_display"
            LOCAL_DISPLAY=$test_display
            break
        fi
    done
    
    # Final test
    if ! DISPLAY=$LOCAL_DISPLAY xset q &>/dev/null; then
        echo "Error: Could not establish X11 connection with any display setting."
        echo "Please ensure:"
        echo "1. XQuartz is running"
        echo "2. 'Allow connections from network clients' is enabled in XQuartz → Preferences → Security"
        echo "3. You have restarted XQuartz after changing the setting"
        exit 1
    fi
fi

# Re-run xhost with the correct display
echo "Ensuring localhost access with final DISPLAY=$LOCAL_DISPLAY"
DISPLAY=$LOCAL_DISPLAY xhost +localhost &>/dev/null

# Build and run the Docker container
CONTAINER_NAME="rosa-turtlesim-demo"
echo "Building the $CONTAINER_NAME Docker image..."
docker build --build-arg DEVELOPMENT=$DEVELOPMENT -t $CONTAINER_NAME -f Dockerfile . || { echo "Error: Docker build failed"; exit 1; }

echo "Running the Docker container..."
docker run -it --rm --name $CONTAINER_NAME \
    -e DISPLAY=$DISPLAY \
    -e HEADLESS=$HEADLESS \
    -e DEVELOPMENT=$DEVELOPMENT \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/src":/app/src \
    -v "$PWD/tests":/app/tests \
    --network host \
    $CONTAINER_NAME

# Disable X11 forwarding
xhost -

exit 0
