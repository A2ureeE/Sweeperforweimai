#!/usr/bin/env bash
# One-click launcher: source and start the full simulation stack.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------
# Strip any conda/miniconda/anaconda paths from PATH so that
# #!/usr/bin/env python3 resolves to the system Python 3.10 that
# ROS 2 Humble's C extensions were compiled against.  Running ROS
# under conda's Python 3.11/3.12/3.13 will hit errors like:
#   ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# ---------------------------------------------------------------
if command -v conda >/dev/null 2>&1 || echo "$PATH" | grep -qiE "(mini|ana)conda"; then
  echo "[sweeper] Detected conda in PATH -- stripping it for this launch."
  export PATH=$(echo "$PATH" | tr ':' '\n' \
                | grep -viE '(mini|ana)conda' \
                | tr '\n' ':' | sed 's/:$//')
  unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE 2>/dev/null || true
  # Ensure /usr/bin (system python3.10) comes first
  export PATH=/usr/bin:$PATH
fi

# ---------------------------------------------------------------
# GUI sanity: RViz / gzclient need a display.  Headless falls back
# to gzserver only (no visual window).
# ---------------------------------------------------------------
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[sweeper] WARNING: No DISPLAY set - RViz and gzclient will fail."
  echo "          If you're in an IDE/SSH terminal, either:"
  echo "            * open a real GUI terminal, or"
  echo "            * run headless:  HEADLESS=1 ./run.sh"
  if [ "${HEADLESS:-0}" != "1" ]; then
    exit 1
  fi
  export GAZEBO_GUI=0
  export QT_QPA_PLATFORM=offscreen
fi

# ---------------------------------------------------------------
# Aggressive pre-flight cleanup.  ROS nodes launched by ros2 launch
# don't always shut down cleanly on SIGINT - especially when the
# parent terminal is killed abruptly - and stale coverage_node /
# controller_node / behavior_node instances will race with the
# fresh ones over the same topics, causing mysterious "path keeps
# rebuilding" or "cmd_vel jitter" bugs.  Kill everything first.
# ---------------------------------------------------------------
echo "[sweeper] Pre-flight cleanup: killing any stale sim processes..."
pkill -9 -x gzserver          2>/dev/null || true
pkill -9 -x gzclient          2>/dev/null || true
pkill -9 -x gazebo            2>/dev/null || true
pkill -9 -f 'ros2 launch sweeper' 2>/dev/null || true
pkill -9 -f '/sweeper_sim/install' 2>/dev/null || true
pkill -9 -f 'coverage_node'   2>/dev/null || true
pkill -9 -f 'controller_node' 2>/dev/null || true
pkill -9 -f 'planner_node'    2>/dev/null || true
pkill -9 -f 'behavior_node'   2>/dev/null || true
pkill -9 -f 'perception_node' 2>/dev/null || true
pkill -9 -f 'moving_obstacle' 2>/dev/null || true
pkill -9 -f 'mission_runner'  2>/dev/null || true
pkill -9 -f 'score_logger'    2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
sleep 2
# Second pass: anything that survived
if pgrep -f '/sweeper_sim/install' >/dev/null 2>&1; then
  echo "[sweeper] WARNING: survivors detected, second pass kill..."
  pkill -9 -f '/sweeper_sim/install' 2>/dev/null || true
  sleep 2
fi
# Final check
if pgrep -f '/sweeper_sim/install' >/dev/null 2>&1 \
   || pgrep -x gzserver >/dev/null 2>&1; then
  echo "[sweeper] ERROR: could not clean up stale processes:" >&2
  pgrep -af '/sweeper_sim/install|gzserver|gzclient' >&2 || true
  exit 1
fi
echo "[sweeper] Pre-flight cleanup OK."

source /opt/ros/humble/setup.bash
# Some systems (notably when gazebo was installed after ROS) don't get
# GAZEBO_* env vars via gazebo_ros's setup hook.  Source the gazebo one
# directly so gzserver can find its plugin libraries.
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi
source "$HERE/install/setup.bash"

LAUNCH_ARGS=("$@")
if [ "${HEADLESS:-0}" = "1" ]; then
  echo "[sweeper] HEADLESS mode: disabling gzclient and RViz."
  LAUNCH_ARGS+=("gui:=false" "rviz:=false")
fi

exec ros2 launch sweeper_bringup sweeper_sim.launch.py "${LAUNCH_ARGS[@]}"
