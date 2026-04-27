#!/usr/bin/env bash
# Capture location.jsonl (and optional command lines) for TASK #16 with:
#   turtlesim + static ObstacleStore (YAML) + world_builder draw + movement.
# Prereq: /app catkin devel, Xvfb (headless), ros-noetic, network not required.
# Usage: TASK16_LOG_ROOT=/path ./capture_task16_obstacle_logs.sh
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "${APP_ROOT}/devel/setup.bash"

export TURTLE_AGENT_LOG_ROOT="${TASK16_LOG_ROOT:-${APP_ROOT}/logs/task16-obstacle-run}"
export PYTHONPATH="${APP_ROOT}/src/turtle_agent/scripts${PYTHONPATH:+:$PYTHONPATH}"

# Clean old run output
rm -rf "${TURTLE_AGENT_LOG_ROOT}"
mkdir -p "${TURTLE_AGENT_LOG_ROOT}"

# --- 1) ROS core + turtlesim (turtle1 is default spawn) ---
pkill -f 'rosmaster' 2>/dev/null || true
pkill -f 'roscore' 2>/dev/null || true
pkill -f 'turtlesim_node' 2>/dev/null || true
sleep 1

roscore &
ROSCORE_PID=$!
sleep 3

xvfb-run -a rosrun turtlesim turtlesim_node &
TURTLE_PID=$!
sleep 4

# --- 2) & 3) Obstacle data (YAML) + world draw: rosparams for node /rosa ---
rosparam set /rosa/static_obstacles_file "${APP_ROOT}/src/turtle_agent/config/static_obstacles_turtlesim.yaml"
rosparam set /rosa/draw_static_world true
rosparam set /rosa/world_builder_required true

# --- 4) Pose logs + stdin-driven exit after cmd_vel burst ---
cd "${APP_ROOT}/src/turtle_agent/scripts"
{
  sleep 7
  timeout 12 rostopic pub -r 22 /turtle1/cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.55, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
  sleep 1
  printf 'exit\n'
} | timeout --signal=INT 55 python3.9 turtle_agent.py || true

kill "${TURTLE_PID}" 2>/dev/null || true
kill "${ROSCORE_PID}" 2>/dev/null || true
sleep 1
pkill -f 'rosmaster' 2>/dev/null || true

echo "--- Log root: ${TURTLE_AGENT_LOG_ROOT}"
find "${TURTLE_AGENT_LOG_ROOT}" -type f -name '*.jsonl' 2>/dev/null || true
