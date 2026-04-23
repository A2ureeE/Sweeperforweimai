#!/usr/bin/env bash
# 最小转向方向验证：空白世界 + 只启 Gazebo + 车辆
# 发送 cmd_vel.angular.z = +0.5 rad (期望：车辆左转，yaw 增加)
# 发送 cmd_vel.angular.z = -0.5 rad (期望：车辆右转，yaw 减小)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v conda >/dev/null 2>&1; then
  export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE '(mini|ana)conda' | tr '\n' ':' | sed 's/:$//')
  unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE 2>/dev/null || true
  export PATH=/usr/bin:$PATH
fi

export QT_QPA_PLATFORM=offscreen
export GAZEBO_GUI=0

echo "[diag] Pre-flight cleanup..."
pkill -9 -x gzserver gzclient gazebo 2>/dev/null || true
pkill -9 -f '/sweeper_sim/install' 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
source "$HERE/install/setup.bash"

SIGN="${1:-+}"   # 运行脚本时传 +/- 或 auto
DURATION="${2:-5}"  # 前进时间（秒）

echo "[diag] Launching gazebo with diag_no_walls.world..."
ros2 launch sweeper_gazebo gazebo.launch.py \
  world:="$HERE/install/sweeper_gazebo/share/sweeper_gazebo/worlds/diag_no_walls.world" \
  gui:=false \
  x:=0.0 y:=0.0 yaw:=0.0 \
  > /tmp/diag_sign_launch.log 2>&1 &
LAUNCH_PID=$!

# 等 Gazebo 起来
sleep 8
echo "[diag] Gazebo up. Pub cmd_vel..."

if [ "$SIGN" = "auto" ]; then
  echo "=== Test 1: angular.z = +0.5 (expect yaw increase / LEFT turn) ==="
  timeout $DURATION ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.5}}" > /dev/null 2>&1 || true
  sleep 1
  ros2 topic echo --once /odom | grep -A3 'orientation' | head -6
  
  echo ""
  echo "=== Test 2: angular.z = -0.5 (expect yaw decrease / RIGHT turn) ==="
  # 重置位置
  ros2 service call /reset_simulation std_srvs/srv/Empty '{}' > /dev/null 2>&1 || true
  sleep 2
  timeout $DURATION ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: -0.5}}" > /dev/null 2>&1 || true
  sleep 1
  ros2 topic echo --once /odom | grep -A3 'orientation' | head -6
else
  VAL="${SIGN}0.5"
  echo "=== Publishing angular.z = $VAL ==="
  timeout $DURATION ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: $VAL}}" > /dev/null 2>&1 || true
  sleep 1
  echo "--- /odom final pose ---"
  ros2 topic echo --once /odom | head -20
fi

echo "[diag] Cleanup..."
kill $LAUNCH_PID 2>/dev/null || true
pkill -9 -f '/sweeper_sim/install' 2>/dev/null || true
pkill -9 -x gzserver 2>/dev/null || true
