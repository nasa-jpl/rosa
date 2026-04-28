#!/usr/bin/env bash
set -euo pipefail

STREAMING="${1:-true}"
# Git Bash에서는 docker -v에 Windows 경로를 넘겨야 마운트가 안정적입니다.
if WORKSPACE_WIN="$(pwd -W 2>/dev/null)"; then
  WORKSPACE="${WORKSPACE_WIN}"
else
  WORKSPACE="$(pwd)"
fi

docker run -it --rm \
  --env-file .env \
  -e TERM=xterm-256color \
  -e ROSA_NO_CLEAR=1 \
  -e DISPLAY=host.docker.internal:0 \
  -v "${WORKSPACE}/src:/app/src" \
  rosa bash -lc "
set -e
cd /app
export PYTHONPATH=/app/src/turtle_agent/scripts:/app/src:\$PYTHONPATH
sed -i 's/\r$//' /app/src/turtle_agent/scripts/turtle_agent.py
echo '[VERIFY] tools.turtle=/app/src/turtle_agent/scripts/tools/turtle.py'
if grep -q 'Final pose=' /app/src/turtle_agent/scripts/tools/turtle.py; then
  echo '[VERIFY] patched_marker=True'
else
  echo '[VERIFY] patched_marker=False'
fi
catkin build
source devel/setup.bash
rosrun turtlesim turtlesim_node >/tmp/turtlesim.log 2>&1 &
sleep 2
if ! command -v rlwrap >/dev/null 2>&1; then
  apt-get update >/tmp/apt-update.log 2>&1 || true
  apt-get install -y rlwrap >/tmp/apt-install-rlwrap.log 2>&1 || true
fi

if command -v rlwrap >/dev/null 2>&1; then
  rlwrap -cAr roslaunch /app/src/turtle_agent/launch/agent.launch streaming:=${STREAMING}
else
  echo "[WARN] rlwrap unavailable; arrow-key editing may still be broken."
  roslaunch /app/src/turtle_agent/launch/agent.launch streaming:=${STREAMING}
fi
"
