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

# Kill any zombie gazebo instances left from a previous ^Z / crash,
# otherwise the new gzserver fails with
#   "Unable to start server[bind: Address already in use]"
if pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null; then
  echo "[sweeper] Killing stale gzserver/gzclient..."
  pkill -9 -x gzserver  2>/dev/null || true
  pkill -9 -x gzclient  2>/dev/null || true
  pkill -9 -x gazebo    2>/dev/null || true
  sleep 1
fi

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
