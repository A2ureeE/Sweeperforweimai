# sweeper_sim — 无人清扫车仿真 + 完整自主驾驶栈

面向无人清扫车比赛任务（固定锥桶避障、动态避障、锥桶限宽门、清扫覆盖、贴边清扫）
的 **Gazebo 仿真 + ROS 2 Humble 自主栈**。即装即跑。

```
┌───────────────────────────────────────────────────────────────────┐
│ perception  →  behavior (FSM)  →  planning (coverage + detour)   │
│       ↘                           ↓                               │
│        dynamic tracks     reference_path → controller → /cmd_vel │
└───────────────────────────────────────────────────────────────────┘
```

---

## 目录结构

| 包名 | 角色 |
|---|---|
| `sweeper_description` | URDF/xacro + robot_state_publisher |
| `sweeper_gazebo`      | Gazebo world（锥桶/限宽门/行人/墙）+ spawn |
| `sweeper_perception`  | 从 `/scan` 聚类检测锥桶、行人、限宽门、边缘；目标跟踪/动态判定 |
| `sweeper_planning`    | 覆盖规划（BCD 牛耕式 + 贴边首尾圈）+ 局部重规划（横向走廊采样，时空膨胀） |
| `sweeper_behavior`    | FSM 决策：COVERAGE / STATIC_DETOUR / DYNAMIC_AVOID / NARROW_GATE / EDGE_FOLLOW / STOP |
| `sweeper_control`     | Mode-aware Pure Pursuit（Ackermann 约束）+ TTC-yield |
| `sweeper_bringup`     | 总启动 launch + RViz 配置 |

---

## 运行前的依赖

Ubuntu 22.04 + ROS 2 Humble。安装其它必需包：

```bash
cd sweeper_sim
./install_deps.sh
```

它会 `apt` 安装：
`ros-humble-xacro` · `ros-humble-gazebo-ros-pkgs` · `ros-humble-gazebo-plugins` ·
`ros-humble-tf-transformations` · `ros-humble-joint-state-publisher` ·
`ros-humble-rviz2` · `ros-humble-rviz-default-plugins` ·
`python3-colcon-common-extensions` · `gazebo`。

---

## 编译

```bash
cd sweeper_sim
./build.sh           # 等价 colcon build --symlink-install
source install/setup.bash
```

编译产物会在 `sweeper_sim/{build,install,log}/` 下。

---

## 一键启动

```bash
cd sweeper_sim
./run.sh
```

这等价于：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sweeper_bringup sweeper_sim.launch.py
```

会依次：
1. 启动 Gazebo，加载 `sweep_course.world`（30×20 m 场地，含 6 个散落锥桶、2 对限宽门、循环移动的行人，以及 L 形墙作贴边目标）。
2. `robot_state_publisher` 发布 URDF。
3. `spawn_entity.py` 在 (−10, −8) 处放置 Z200 车体。
4. 4 秒后依次启动 perception / planning / control / behavior。
5. 打开 RViz2 并加载 `sweeper.rviz`（显示车体、激光、覆盖路径、参考路径、锥桶/行人/门/边缘 Marker）。

---

## 主要话题

| Topic | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/scan`                 | sensor_msgs/LaserScan | gazebo → perception | 主雷达 |
| `/odom`                 | nav_msgs/Odometry     | gazebo → * | 里程计（tricycle_drive_fixed 插件发布） |
| `/cmd_vel`              | geometry_msgs/Twist   | control → gazebo | `linear.x` + `angular.z` |
| `/coverage/path`        | nav_msgs/Path         | coverage → planner (latched) | 全场清扫参考路径 |
| `/coverage/markers`     | visualization_msgs/MarkerArray | → RViz | 绿色折线 |
| `/reference_path`       | nav_msgs/Path         | planner → control | 未来 ~10 m 已绕障参考段 |
| `/planner/blocked`      | std_msgs/Bool         | planner → behavior/control | 彻底无通行走廊 |
| `/planner/progress`     | std_msgs/Float32      | planner → * | 覆盖进度 0–1 |
| `/perception/cones`     | MarkerArray           | perception → behavior/RViz | 锥桶中心 |
| `/perception/pedestrians` | MarkerArray         | perception → behavior/RViz | 行人簇 |
| `/perception/gates`     | MarkerArray           | perception → behavior | 限宽门中线（箭头指向门法向） |
| `/perception/edges`     | MarkerArray           | perception → behavior | 边缘折线（贴边目标） |
| `/perception/dynamic_obstacles` | PoseArray    | perception → planner/control | 位姿.z 编码速率；orientation 编码方向 |
| `/perception/obstacle_points` | PolygonStamped | perception → planner | 所有障碍物聚类中心（供走廊采样） |
| `/behavior/mode`        | std_msgs/String       | behavior → control | 当前模式 |
| `/behavior/speed_limit` | std_msgs/Float32      | behavior → control | 速度限幅 |

---

## 调参速查

| 文件 | 说明 |
|---|---|
| `sweeper_planning/config/planning.yaml`   | 场地范围、扫描间距、走廊半宽 |
| `sweeper_perception/config/perception.yaml` | 聚类阈值、限宽门间距、动态判定速率 |
| `sweeper_control/config/control.yaml`      | 速度上限、Pure Pursuit 前视距离 |
| `sweeper_behavior/config/behavior.yaml`    | 各模式触发距离 |
| `sweeper_description/urdf/z200.urdf.xacro` | 车身/轮距/雷达位置/Gazebo 插件 |
| `sweeper_gazebo/worlds/sweep_course.world` | 锥桶/限宽门/行人位置 |

---

## 规划算法概览（对应赛题）

| 赛题 | 模块 & 算法 |
|---|---|
| 固定锥桶避障       | 感知 Euclidean 聚类（≤ 0.6 m 视为锥桶） + 横向走廊采样局部绕障（`planner_node`） |
| 移动障碍物避障     | 最近邻跟踪 + CV 运动模型 → 时空膨胀椭圆加入代价；控制侧 TTC-yield |
| 锥桶限宽门         | 锥桶对配对（1.2–2.5 m）产生门中线 → FSM 切到 `NARROW_GATE` → 控制器降速、收紧转向率 |
| 清扫覆盖率         | BCD 牛耕式覆盖 + 最小 `|shift|` 绕障 → 覆盖损失最小 |
| 贴边清扫           | 覆盖路径首尾两圈即为边界 offset 条带；FSM 在边界附近切到 `EDGE_FOLLOW` |

---

## 故障排除

* **Gazebo 起不来 / 黑屏**：首次加载 actor 需要下载 `walk.dae`，请保持网络畅通或将 actor 那段从 world 里注释掉。
* **车不走**：看 `/behavior/mode`；若一直 `STOP` 说明 `/planner/blocked=true`。看 `/perception/obstacle_points` 是否有异常近点。可临时放大 `corridor_half_width`。
* **雷达扫到车身**：本车 URDF 的雷达 z=0.6 m 低于车顶；如需更远可改 xacro 中 `laser_joint.origin.z`。
* **`tf_transformations` 找不到**：`sudo apt install ros-humble-tf-transformations`。
* **和真车 SDK 冲突**：真车节点（`mc`, `rtk`, `rslidar_sdk`）监听 CAN/串口，与仿真独立。仿真 `/cmd_vel` 默认由 tricycle_drive_fixed 插件消费。真车上把 `controller_node` 的 `/cmd_vel` remap 到底盘桥即可。

---

## 与真车（SDK/ 目录）的对接点

1. `mc` 包提供 CAN 底盘控制 → 新写一个 `mc_bridge` 节点订阅 `/cmd_vel` 并发布 `sweeper_interfaces/McCtrl`。
2. `rtk` 定位 → 写一个 `rtk_to_odom` 节点，把 RTK 经纬度映射为 UTM 并发布 `/odom` + `map→odom` TF，替代仿真里 tricycle_drive_fixed 的 odom 源。
3. `rslidar_sdk` → 发布 PointCloud2。需要额外加一个 `pointcloud_to_laserscan` 节点将其降维到 `/scan`，或把 `perception_node` 改成直接消费 PC2。

这些适配不影响仿真栈，保留接口整齐。

### 新增桥接包（真车部署必需）

| 包名 | 角色 |
|---|---|
| `mc_bridge` | 将 `/cmd_vel` (Twist) 转换为 `McCtrl` (CAN 控制指令)，含清扫部件独立控制 |
| `rtk_to_odom` | 将 RTK 经纬度转换为局部里程计坐标 (`/odom`) + `map→odom` TF |
| `lidar_adapter` | 将 RoboSense rslidar 的 `PointCloud2` 投影为 2D `LaserScan` → `/scan` |

#### 真车启动顺序

```bash
# 1. 启动激光雷达
ros2 launch rslidar_sdk start.py

# 2. 启动 RTK
ros2 launch rtk rtk.launch.py

# 3. 启动底盘 CAN
ros2 run mc mc_node

# 4. 启动桥接层
ros2 launch mc_bridge mc_bridge.launch.py
ros2 launch rtk_to_odom rtk_to_odom.launch.py
ros2 launch lidar_adapter lidar_adapter.launch.py

# 5. 启动自主驾驶栈（同仿真）
ros2 launch sweeper_bringup sweeper_sim.launch.py
```

**注意**：`mc_bridge` 会自动订阅 `/behavior/mode` 和 `/behavior/speed_limit`，与仿真栈的 `behavior_node` 无缝对接。清扫部件在 `STOP` 模式下自动关闭，在正常运行期间自动启动。
