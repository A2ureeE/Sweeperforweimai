# 速腾Airy激光雷达、RTK和底盘接口使用指南

## 项目简介

本项目包含以下组件：
- 速腾（RoboSense）Airy激光雷达SDK
- RTK定位模块
- 底盘控制接口（mc）
- 桥接节点（mc_bridge / rtk_to_odom / lidar_adapter）
- 自定义消息接口（sweeper_interfaces）
- Gazebo仿真 + 完整自主驾驶栈（sweeper_sim/）

支持在ROS2 Humble环境下使用这些组件。

## 环境要求

- Ubuntu 22.04
- ROS2 Humble desktop
- 速腾Airy激光雷达
- RTK定位模块
- 底盘系统（CAN总线控制）

## 编译与运行

1. 创建ROS2工作空间
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. 复制项目代码
   ```bash
   # 复制整个sweeper目录到src目录
   cp -r /path/to/sweeper/* ~/ros2_ws/src/
   ```

3. 编译项目
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. 运行各个组件
   
   - 运行激光雷达
   ```bash
   ros2 launch rslidar_sdk start.py
   ```
   
   - 运行RTK模块
   ```bash
   ros2 launch rtk rtk.launch.py
   ```
   
   - 运行底盘接口
   ```bash
   ros2 run mc mc_node
   ```

## 配置说明

### 激光雷达配置
- 配置文件位于 `rslidar_sdk/config/config.yaml`
- 可以根据实际情况修改配置文件中的参数，如激光雷达的IP地址、端口等

### RTK配置
- 配置文件位于 `rtk/config/rtk_params.yaml`
- 主要参数：
  - `serial_port`: RTK设备的串口名称（默认为 `/dev/ttyTHS1`）

### 底盘接口配置
- 配置文件位于 `mc/config/config.json`
- 主要参数：
  - `can_dev`: CAN总线设备名称（默认为 `can0`）

## 自定义消息

项目包含以下自定义消息：

### 底盘控制消息 (`sweeper_interfaces/msg/McCtrl.msg`)
```
#mcu部分
uint8 brake        #电磁刹指令 0开;1关
uint8 gear         #挡位 0空挡;1后退;2前进;3保留
uint16 rpm         #转速 量程：0-6000，对应实际电机转速0-6000rpm

#eps部分
float32 angle      #轮端转向角度 分辨率0.2° [-66.0,66.0] 适当缩减
uint16 angle_speed #转向角速度 120-1500rpm

#vcu部分
bool sweep          #一键清扫 true:清扫 false:不清扫 (优先级低于各独立开关)
int32 sweep_mode    #清扫模式: 0=标准模式(刷子), 1=混合模式(刷子+洒水)

# 各清扫部件独立控制 (当sweep为true时，以下开关可以单独控制对应部件)
bool enable_main_brush      # 主刷电机
bool enable_vacuum          # 吸尘电机
bool enable_dust_shake      # 振尘电机
bool enable_main_brush_pole # 主刷推杆 (下沉/抬升)
bool enable_flap_pole       # 前挡皮推杆电机
bool enable_side_brush      # 边刷电机
bool enable_water_pump      # 水泵电机
```

### RTK定位消息 (`sweeper_interfaces/msg/Rtk.msg`)
```
float64 lat        # 纬度
float64 lon        # 经度
float32 head       # 航向角
float32 speed      # 速度
int32 p_quality    # 位置质量
int32 h_quality    # 航向质量
```

### CAN帧消息 (`sweeper_interfaces/msg/CanFrame.msg`)
```
uint32 id
uint8 dlc
uint8[8] data
```

### 车辆身份消息 (`sweeper_interfaces/msg/VehicleIdentity.msg`)
```
bool ready
string vid
```

## URDF模型

项目包含机器人模型文件 `z200.urdf`，可用于仿真。

### 模型结构

URDF文件定义了机器人的连杆、关节和传感器位置，包括激光雷达、RTK和底盘的相对位置关系。

## 桥接节点（真车部署）

| 包 | 描述 |
|----|------|
| `mc_bridge` | 将 `/cmd_vel` 转换为 `McCtrl` 通过 CAN 总线控制底盘 |
| `rtk_to_odom` | 将 RTK 经纬度转换为 `/odom` 里程计坐标 |
| `lidar_adapter` | 将 `PointCloud2` 投影为 `LaserScan` 供感知节点使用 |