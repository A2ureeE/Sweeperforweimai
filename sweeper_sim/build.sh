#!/usr/bin/env bash
# Build the sweeper_sim workspace in place.
# Run from inside this directory (sweeper_sim/).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# Force system python to avoid miniconda/venv picking a wrong interpreter.
export PATH=/usr/bin:$PATH

source /opt/ros/humble/setup.bash

# colcon treats the current directory as src by default when build/install live here.
colcon build --symlink-install \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 "$@"

echo
echo "[sweeper] Build finished."
echo "         source $HERE/install/setup.bash"
