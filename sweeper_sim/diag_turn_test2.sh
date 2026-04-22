#!/usr/bin/env bash
# 诊断 2：发送 angular.z 作为 steering angle (δ)，而不是角速度 (ω)
# 如果 tricycle_drive 把 angular.z 解释为 steering angle：
#   发 angular.z=0.8727 (50°) → 车以 R = L/tan(50°) = 0.88m 半径转弯
# 对比诊断 1 (angular.z=0.341) 观察到 R ≈ 2.93m
#   如果语义是 steering angle：应该 R = 1.05/tan(0.341) = 2.97m ✓ 吻合！
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v conda >/dev/null 2>&1 || echo "$PATH" | grep -qiE "(mini|ana)conda"; then
  export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE '(mini|ana)conda' | tr '\n' ':' | sed 's/:$//')
  unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE 2>/dev/null || true
  export PATH=/usr/bin:$PATH
fi
export GAZEBO_GUI=0
export QT_QPA_PLATFORM=offscreen

pkill -9 -x gzserver gzclient gazebo 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi
source "$HERE/install/setup.bash"

WORLD_FILE="$HERE/install/sweeper_gazebo/share/sweeper_gazebo/worlds/diag_no_walls.world"
URDF_XACRO="$HERE/install/sweeper_description/share/sweeper_description/urdf/z200.urdf.xacro"

echo "[diag2] 启动 gzserver..."
gzserver "$WORLD_FILE" --verbose \
  -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so \
  > /tmp/diag2_gz.log 2>&1 &
sleep 4

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro "$URDF_XACRO")" -p use_sim_time:=true \
  > /tmp/diag2_rsp.log 2>&1 &
sleep 2

echo "[diag2] Spawn z200..."
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description -entity z200 -x 0 -y 0 -z 0.1 -Y 0 > /tmp/diag2_spawn.log 2>&1
sleep 3

echo "[diag2] 记录 /odom..."
python3 - <<'PY' > /tmp/diag_odom2.csv &
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

def yaw_from_q(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class Rec(Node):
    def __init__(self):
        super().__init__('diag_odom_rec2')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.cb, qos)
        print("t,x,y,yaw_deg,vx,wz", flush=True)
        self.t0 = None
    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None: self.t0 = t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = math.degrees(yaw_from_q(msg.pose.pose.orientation))
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        print(f"{t-self.t0:.2f},{x:.3f},{y:.3f},{yaw:.1f},{vx:.3f},{wz:.3f}", flush=True)

rclpy.init()
rclpy.spin(Rec())
PY
REC_PID=$!
sleep 2

sleep 10

# 发送 angular.z = 0.8727 (50°)。车应该以 R = 1.05/tan(50°) = 0.88m 半径转弯
echo "[diag2] 发送 cmd_vel: v=0.3, angular.z=0.8727 (50° steering) 持续 15 秒..."
timeout 15 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.8727}}' > /dev/null 2>&1 &
sleep 17

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' > /dev/null 2>&1
sleep 2

kill $REC_PID 2>/dev/null || true
pkill -9 -x gzserver 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true

echo "[diag2] 完成。后 20 行:"
tail -20 /tmp/diag_odom2.csv
