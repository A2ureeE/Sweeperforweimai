#!/usr/bin/env bash
# 诊断脚本：测试车辆在无墙世界中能否完成 U-turn
# 用法：./diag_turn_test.sh
#
# 步骤：
#   1. 启动 gazebo (无墙世界 diag_no_walls.world)
#   2. spawn z200 车辆于 (0, 0, yaw=0)
#   3. 等待 12 秒（让 gazebo 物理稳定 + 初始下落稳定）
#   4. 发送 cmd_vel = (v=0.3, omega=对应 δ=50°)，持续 20 秒
#   5. 记录 /odom 轨迹到 /tmp/diag_odom.csv
#
# 预期：
#   若车辆能转：轨迹应为一个半径 ~0.88m 的圆弧
#   若车辆不能转：轨迹为直线或轻微偏转
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清 conda
if command -v conda >/dev/null 2>&1 || echo "$PATH" | grep -qiE "(mini|ana)conda"; then
  export PATH=$(echo "$PATH" | tr ':' '\n' \
                | grep -viE '(mini|ana)conda' \
                | tr '\n' ':' | sed 's/:$//')
  unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE 2>/dev/null || true
  export PATH=/usr/bin:$PATH
fi

# Headless
export GAZEBO_GUI=0
export QT_QPA_PLATFORM=offscreen

# 清空残留
pkill -9 -x gzserver gzclient gazebo 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi
source "$HERE/install/setup.bash"

WORLD_FILE="$HERE/install/sweeper_gazebo/share/sweeper_gazebo/worlds/diag_no_walls.world"
URDF_XACRO="$HERE/install/sweeper_description/share/sweeper_description/urdf/z200.urdf.xacro"

if [ ! -f "$WORLD_FILE" ]; then
  echo "[diag] ERROR: world file not found: $WORLD_FILE"
  echo "       请先运行: cd $HERE && colcon build --packages-select sweeper_gazebo"
  exit 1
fi

# 1. 启动 gzserver (无 client)
echo "[diag] 启动 gzserver (无墙世界)..."
gzserver "$WORLD_FILE" --verbose \
  -slibgazebo_ros_init.so \
  -slibgazebo_ros_factory.so \
  -slibgazebo_ros_force_system.so > /tmp/diag_gz.log 2>&1 &
GZ_PID=$!
sleep 4

# 2. robot_state_publisher: 提供 /robot_description topic
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro "$URDF_XACRO")" \
  -p use_sim_time:=true > /tmp/diag_rsp.log 2>&1 &
RSP_PID=$!
sleep 2

# 3. spawn z200
echo "[diag] Spawn z200 at (0, 0, yaw=0)..."
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity z200 \
  -x 0 -y 0 -z 0.1 -Y 0 > /tmp/diag_spawn.log 2>&1
sleep 3

# 4. 开始记录 /odom
echo "[diag] 记录 /odom 轨迹到 /tmp/diag_odom.csv..."
python3 - <<'PY' > /tmp/diag_odom.csv &
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

def yaw_from_q(q):
    # tf_transformations 简化：只算 yaw
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class Rec(Node):
    def __init__(self):
        super().__init__('diag_odom_rec')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.cb, qos)
        print("t,x,y,yaw_deg,vx,wz", flush=True)
        self.t0 = None

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = math.degrees(yaw_from_q(msg.pose.pose.orientation))
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        print(f"{t-self.t0:.2f},{x:.3f},{y:.3f},{yaw:.1f},{vx:.3f},{wz:.3f}", flush=True)

rclpy.init()
node = Rec()
rclpy.spin(node)
PY
REC_PID=$!
sleep 2

# 5. 等 12 秒让物理稳定 (不发 cmd_vel)
echo "[diag] 等 12 秒让物理稳定..."
sleep 12

# 6. 发送固定 cmd_vel
# 目标：v=0.3 m/s, δ=50°(=0.8727rad)
# tricycle_drive 插件解释 cmd_vel.angular.z 为转向角（rad）
# 见 motorController：alpha = atan2(wheel_separation * angular, 2*linear)
#   -- 实际上用 vel/steering: δ = angular.z（直接作为转向角）
# 但 run.sh 里的 controller 是: omega_z = v * tan(δ) / L
# 所以这里我们直接试两种方式：
# 方式 A: v=0.3, wz=0.3*tan(50°)/1.05 = 0.341 rad/s (controller 方式)

echo "[diag] 发送 cmd_vel: v=0.3 m/s, wz=0.341 rad/s (等效 δ=50°) 持续 20 秒..."
timeout 20 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.341}}' > /dev/null 2>&1 &
CMDVEL_PID=$!

sleep 22

# 7. 停车
echo "[diag] 发送 stop..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' > /dev/null 2>&1
sleep 2

# 8. 清理
echo "[diag] 清理..."
kill $CMDVEL_PID $REC_PID $RSP_PID 2>/dev/null || true
pkill -9 -x gzserver 2>/dev/null || true

echo "[diag] 完成。轨迹保存在 /tmp/diag_odom.csv"
echo "[diag] 前 10 行:"
head -11 /tmp/diag_odom.csv
echo "[diag] 后 10 行:"
tail -10 /tmp/diag_odom.csv
