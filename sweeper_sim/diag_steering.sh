#!/usr/bin/env bash
# 诊断 3：monitor /joint_states 看 front_steering_joint 的实际角度
# 发不同的 angular.z，看实际前轮转角是否跟得上命令

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
pkill -9 -f "ros2|sweeper" 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi
source "$HERE/install/setup.bash"

WORLD_FILE="$HERE/install/sweeper_gazebo/share/sweeper_gazebo/worlds/diag_no_walls.world"
URDF_XACRO="$HERE/install/sweeper_description/share/sweeper_description/urdf/z200.urdf.xacro"

echo "[diag3] 启动 gzserver..."
gzserver "$WORLD_FILE" --verbose \
  -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so \
  > /tmp/diag3_gz.log 2>&1 &
sleep 4

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro "$URDF_XACRO")" -p use_sim_time:=true \
  > /tmp/diag3_rsp.log 2>&1 &
sleep 2

ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description -entity z200 -x 0 -y 0 -z 0.1 -Y 0 > /tmp/diag3_spawn.log 2>&1
sleep 3

echo "[diag3] 同时记录 /odom（位置+速度）和 /joint_states（实际转向角）..."
python3 - <<'PY' > /tmp/diag_steering.csv &
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

def yaw_from_q(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class Rec(Node):
    def __init__(self):
        super().__init__('diag3_rec')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.create_subscription(JointState, '/joint_states', self.js_cb, qos)
        print("t,x,y,yaw_deg,vx,steer_deg", flush=True)
        self.t0 = None
        self.steer = 0.0
        self.odom = None

    def js_cb(self, msg):
        try:
            i = msg.name.index('front_steering_joint')
            self.steer = msg.position[i]
        except (ValueError, IndexError):
            return

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None: self.t0 = t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = math.degrees(yaw_from_q(msg.pose.pose.orientation))
        vx = msg.twist.twist.linear.x
        print(f"{t-self.t0:.2f},{x:.3f},{y:.3f},{yaw:.1f},{vx:.3f},{math.degrees(self.steer):.2f}", flush=True)

rclpy.init()
rclpy.spin(Rec())
PY
REC_PID=$!
sleep 2

echo "[diag3] 等 8 秒..."
sleep 8

echo "[diag3] 命令 v=0.3, angular.z=0.8727 (δ=50°) 持续 15 秒..."
timeout 15 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.8727}}' > /dev/null 2>&1 &
sleep 17

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' > /dev/null 2>&1
sleep 1

kill $REC_PID 2>/dev/null || true
pkill -9 -x gzserver 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true

echo "[diag3] 完成。"
