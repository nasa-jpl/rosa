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

# This script runs ROSA tests in Docker containers for different ROS distributions
# Usage: ./test.sh [--it] <ROS_DISTRO> <TEST_TYPE>
#   --it: Run Docker in interactive mode (for debugging)
#   ROS_DISTRO: noetic, humble, iron, jazzy
#   TEST_TYPE: unit, turtlesim, both
# Environment variables:
#   DEVELOPMENT=true - Mount local src/tests for live editing
#   HEADLESS=true - Run without GUI (for unit tests or headless turtlesim)

set -e

# Function to display usage
usage() {
    echo "Usage: $0 <ROS_DISTRO> <TEST_TYPE>"
    echo ""
    echo "ROS_DISTRO:"
    echo "  noetic   - ROS1 Noetic (Ubuntu 20.04, Python 3.9)"
    echo "  humble   - ROS2 Humble (Ubuntu 22.04, Python 3.10)"
    echo "  iron     - ROS2 Iron (Ubuntu 22.04, Python 3.10)"
    echo "  jazzy    - ROS2 Jazzy (Ubuntu 24.04, Python 3.12)"
    echo ""
    echo "TEST_TYPE:"
    echo "  unit     - Run unit tests only"
    echo "  turtlesim - Run turtlesim demo only"
    echo "  both     - Run unit tests then turtlesim demo"
    echo ""
    echo "Environment variables:"
    echo "  DEVELOPMENT=true - Mount local src/tests for development"
    echo "  HEADLESS=true    - Run without GUI"
    echo ""
    echo "Examples:"
    echo "  $0 noetic unit"
    echo "  $0 --it noetic unit  # Interactive mode for debugging"
    echo "  $0 humble turtlesim"
    echo "  DEVELOPMENT=true $0 jazzy both"
    exit 1
}

# Parse arguments
INTERACTIVE=false
if [ "$1" = "--it" ]; then
    INTERACTIVE=true
    shift
fi

# Check arguments
if [ $# -ne 2 ]; then
    echo "Error: Invalid number of arguments"
    usage
fi

ROS_DISTRO=$1
TEST_TYPE=$2

# Validate ROS_DISTRO
case "$ROS_DISTRO" in
    noetic|humble|iron|jazzy)
        ;;
    *)
        echo "Error: Invalid ROS_DISTRO '$ROS_DISTRO'"
        usage
        ;;
esac

# Validate TEST_TYPE
case "$TEST_TYPE" in
    unit|turtlesim|both)
        ;;
    *)
        echo "Error: Invalid TEST_TYPE '$TEST_TYPE'"
        usage
        ;;
esac

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Set environment variables
HEADLESS=${HEADLESS:-false}
DEVELOPMENT=${DEVELOPMENT:-false}

# Set ROS version for environment
case "$ROS_DISTRO" in
    noetic)
        ROS_VERSION=1
        PYTHON_CMD="python3.9"
        DOCKER_TARGET="ros1-noetic"
        ;;
    humble|iron|jazzy)
        ROS_VERSION=2
        case "$ROS_DISTRO" in
            humble|iron)
                PYTHON_CMD="python3.10"
                ;;
            jazzy)
                PYTHON_CMD="python3.12"
                ;;
        esac
        DOCKER_TARGET="ros2-$ROS_DISTRO"
        ;;
esac

# Setup X11 forwarding for turtlesim (only if needed)
if [ "$TEST_TYPE" = "turtlesim" ] || [ "$TEST_TYPE" = "both" ]; then
    if [ "$HEADLESS" = "false" ]; then
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
                xhost + >/dev/null 2>&1 || echo "Warning: Could not enable X11 forwarding"
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
                DISPLAY=$LOCAL_DISPLAY xhost +localhost >/dev/null 2>&1 || echo "Warning: Could not enable X11 forwarding"
                # Set DISPLAY for Docker container
                export DISPLAY=host.docker.internal:0
                ;;
            MINGW*|CYGWIN*|MSYS*)
                echo "Enabling X11 forwarding for Windows..."
                export DISPLAY=host.docker.internal:0
                ;;
            *)
                echo "Warning: Unsupported operating system for GUI. Setting HEADLESS=true"
                HEADLESS=true
                ;;
        esac
        
        # Test X11 connection (only for Linux/macOS)
        if [ "$(uname)" = "Linux" ] || [ "$(uname)" = "Darwin" ]; then
            LOCAL_DISPLAY_TEST=${LOCAL_DISPLAY:-${DISPLAY:-:0}}
            if [ "$(uname)" = "Darwin" ]; then
                LOCAL_DISPLAY_TEST=${LOCAL_DISPLAY:-:0}
            fi
            
            echo "Testing X11 connection with DISPLAY=$LOCAL_DISPLAY_TEST"
            if ! DISPLAY=$LOCAL_DISPLAY_TEST xset q &>/dev/null; then
                echo "Warning: X11 forwarding test failed with DISPLAY=$LOCAL_DISPLAY_TEST"
                echo "Trying alternative display settings..."
                
                # Try common display alternatives
                X11_WORKING=false
                for test_display in ":0" ":1" "localhost:0" "127.0.0.1:0"; do
                    echo "Testing DISPLAY=$test_display"
                    if DISPLAY=$test_display xset q &>/dev/null 2>&1; then
                        echo "Success! Using DISPLAY=$test_display for local testing"
                        if [ "$(uname)" = "Darwin" ]; then
                            # Re-run xhost with the correct display
                            echo "Ensuring localhost access with final DISPLAY=$test_display"
                            DISPLAY=$test_display xhost +localhost &>/dev/null
                        fi
                        X11_WORKING=true
                        break
                    fi
                done
                
                if [ "$X11_WORKING" = "false" ]; then
                    echo "Warning: Could not establish X11 connection with any display setting."
                    echo "GUI applications may not work. Consider setting HEADLESS=true"
                    if [ "$(uname)" = "Darwin" ]; then
                        echo "Please ensure:"
                        echo "1. XQuartz is running"
                        echo "2. 'Allow connections from network clients' is enabled in XQuartz → Preferences → Security"
                        echo "3. You have restarted XQuartz after changing the setting"
                    fi
                fi
            else
                echo "X11 connection test successful"
            fi
        fi
    fi
fi

# Build Docker container
CONTAINER_NAME="rosa-test-$ROS_DISTRO"
echo "Building $CONTAINER_NAME Docker image..."
docker build \
    --target "$DOCKER_TARGET" \
    --build-arg DEVELOPMENT=true \
    -t "$CONTAINER_NAME" \
    -f Dockerfile.test . || { echo "Error: Docker build failed"; exit 1; }

# Function to run unit tests
run_unit_tests() {
    echo "Running unit tests with $ROS_DISTRO..."
    
    local test_cmd
    if [ "$ROS_VERSION" = "1" ]; then
        test_cmd="source /root/.bashrc && cd /app && python3.9 -m unittest discover tests"
    else
        test_cmd="source /root/.bashrc && cd /app && $PYTHON_CMD -m unittest discover tests"
    fi
    
    if [ "$INTERACTIVE" = "true" ]; then
        echo "Starting interactive session. Run the following command manually:"
        echo "$test_cmd"
        docker run --rm -it \
            -e ROS_VERSION="$ROS_VERSION" \
            -e DEVELOPMENT="$DEVELOPMENT" \
            -v "$PWD/src":/app/src \
            -v "$PWD/tests":/app/tests \
            "$CONTAINER_NAME" \
            /bin/bash
    else
        docker run --rm \
            -e ROS_VERSION="$ROS_VERSION" \
            -e DEVELOPMENT="$DEVELOPMENT" \
            -v "$PWD/src":/app/src \
            -v "$PWD/tests":/app/tests \
            "$CONTAINER_NAME" \
            /bin/bash -c "$test_cmd"
    fi
}

# Function to run turtlesim demo
run_turtlesim_demo() {
    echo "Running turtlesim demo with $ROS_DISTRO..."
    
    local docker_args=()
    docker_args+=(--rm -it)
    docker_args+=(-e ROS_VERSION="$ROS_VERSION")
    docker_args+=(-e HEADLESS="$HEADLESS")
    docker_args+=(-e DEVELOPMENT="$DEVELOPMENT")
    docker_args+=(-v "$PWD/src":/app/src)
    docker_args+=(-v "$PWD/tests":/app/tests)
    docker_args+=(--network host)
    
    # Add X11 forwarding if not headless
    if [ "$HEADLESS" = "false" ]; then
        docker_args+=(-e DISPLAY="$DISPLAY")
        docker_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
    fi
    
    local demo_cmd
    if [ "$ROS_VERSION" = "1" ]; then
        if [ "$HEADLESS" = "false" ]; then
            demo_cmd="source /opt/ros/noetic/setup.bash && roscore > /dev/null 2>&1 & sleep 5 && rosrun turtlesim turtlesim_node & sleep 5 && echo 'TurtleSim is running. Run \`start\` to build and launch the ROSA-TurtleSim demo (turtle_agent).' && /bin/bash"
        else
            demo_cmd="source /opt/ros/noetic/setup.bash && roscore > /dev/null 2>&1 & sleep 5 && xvfb-run -a -s '-screen 0 1920x1080x24' rosrun turtlesim turtlesim_node & sleep 5 && echo 'TurtleSim is running in headless mode. Run \`start\` to build and launch the ROSA-TurtleSim demo (turtle_agent).' && /bin/bash"
        fi
    else
        if [ "$HEADLESS" = "false" ]; then
            demo_cmd="source /opt/ros/$ROS_DISTRO/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 run turtlesim turtlesim_node & sleep 5 && echo 'TurtleSim is running. Run \`start2\` to build and launch the ROSA-TurtleSim demo (turtle_agent2).' && /bin/bash"
        else
            demo_cmd="source /opt/ros/$ROS_DISTRO/setup.bash && source /app/ros2_ws/install/setup.bash && xvfb-run -a -s '-screen 0 1920x1080x24' ros2 run turtlesim turtlesim_node & sleep 5 && echo 'TurtleSim is running in headless mode. Run \`start2\` to build and launch the ROSA-TurtleSim demo (turtle_agent2).' && /bin/bash"
        fi
    fi
    
    docker run "${docker_args[@]}" "$CONTAINER_NAME" /bin/bash -c "$demo_cmd"
}

# Run tests based on TEST_TYPE
case "$TEST_TYPE" in
    unit)
        run_unit_tests
        ;;
    turtlesim)
        run_turtlesim_demo
        ;;
    both)
        run_unit_tests
        echo ""
        echo "Unit tests completed. Starting turtlesim demo..."
        echo ""
        run_turtlesim_demo
        ;;
esac

# Disable X11 forwarding if it was enabled
if [ "$TEST_TYPE" = "turtlesim" ] || [ "$TEST_TYPE" = "both" ]; then
    if [ "$HEADLESS" = "false" ]; then
        xhost - >/dev/null 2>&1 || true
    fi
fi

echo "Test run completed!"
exit 0