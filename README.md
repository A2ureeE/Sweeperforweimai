# Sweeperforweimai — 扫地机器人仿真系统

**为江苏大学生交通科技大赛企业命题赛道设计的扫地机器人方案**

A full-featured sweeping robot simulation designed for the **Jiangsu University Student Transportation Science and Technology Competition (江苏大学生交通科技大赛)** — Enterprise Proposition Track.

---

## 目录 / Contents

- [方案概述 / Overview](#方案概述--overview)
- [系统架构 / Architecture](#系统架构--architecture)
- [算法说明 / Algorithms](#算法说明--algorithms)
- [快速开始 / Quick Start](#快速开始--quick-start)
- [项目结构 / Project Structure](#项目结构--project-structure)
- [测试 / Tests](#测试--tests)
- [仿真结果 / Results](#仿真结果--results)

---

## 方案概述 / Overview

本方案实现了一个完整的自主扫地机器人软件仿真系统，包含：

| 模块 | 功能 |
|------|------|
| **环境地图** (`environment.py`) | 基于网格的二维地图，支持障碍物、充电站、已清洁区域 |
| **传感器模型** (`sensor.py`) | 碰撞传感器、悬崖传感器、激光雷达（Lidar）、灰尘传感器 |
| **机器人模型** (`robot.py`) | 状态机（清扫 / 低电量 / 返回 / 充电 / 满仓）、电池与尘仓管理 |
| **路径规划** (`planner.py`) | 弓字形（Boustrophedon）、螺旋形（Spiral）、随机弹跳（Random Bounce）、A\* 回基站 |
| **仿真引擎** (`simulation.py`) | 主循环、ASCII 渲染、Matplotlib 动画可视化 |

---

## 系统架构 / Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Simulation Loop                   │
│  ┌──────────┐    ┌──────────┐    ┌───────────────┐  │
│  │ Planner  │───▶│  Robot   │───▶│  Environment  │  │
│  │(waypoints│    │(move/    │    │ (grid, SLAM-  │  │
│  │ A* path) │    │ charge)  │    │  lite marks)  │  │
│  └──────────┘    └────┬─────┘    └───────────────┘  │
│                       │                              │
│               ┌───────▼──────┐                       │
│               │   Sensors    │                       │
│               │Bump|Cliff|   │                       │
│               │Lidar|Dirt    │                       │
│               └──────────────┘                       │
└─────────────────────────────────────────────────────┘
```

### 机器人状态机 / Robot State Machine

```
         ┌──────────────┐
    ┌────▶│   CLEANING   │◀───────────────────┐
    │     └──────┬───────┘                    │
    │            │ battery low / bin full      │
    │     ┌──────▼───────┐                    │
    │     │ LOW_BATTERY / │                    │
    │     │   FULL_BIN    │                    │
    │     └──────┬────────┘                    │
    │            │ navigate via A*             │
    │     ┌──────▼───────┐                    │
    │     │  RETURNING   │                    │
    │     └──────┬───────┘                    │
    │            │ arrived at charger          │
    │     ┌──────▼───────┐   fully charged    │
    └─────│   CHARGING   │────────────────────┘
          └──────────────┘
```

---

## 算法说明 / Algorithms

### 1. 弓字形覆盖 (Boustrophedon / Lawnmower)

逐行扫描，偶数行从左到右，奇数行从右到左，遇到障碍物跳过。  
适合结构规整的室内环境，覆盖率高，路径效率最优。

### 2. 螺旋形覆盖 (Spiral)

从外圈向内螺旋扫描，依次完成上、右、下、左四个边。  
适合开阔空间，减少重复清扫，可与其他算法组合使用。

### 3. 随机弹跳 (Random Bounce / iRobot-style)

沿直线行进直到碰到障碍物，随机选择新方向继续。  
实现简单，无需完整地图，适合资源受限的嵌入式系统。

### 4. A\* 回基站导航 (A\* Return-to-Base)

电量不足或尘仓满时，使用 A\* 算法找到从当前位置到充电站的最短路径，  
保证机器人在电量耗尽前安全返回。

---

## 快速开始 / Quick Start

### 环境准备

```bash
pip install -r requirements.txt
```

### 运行仿真

```bash
# 默认设置：弓字形算法 + 客厅地图
python main.py

# 螺旋算法 + 随机地图
python main.py --planner spiral --map random

# 随机弹跳算法 + 自定义地图尺寸
python main.py --planner random --map random --rows 25 --cols 25

# 开启 Matplotlib 动画（需要图形界面）
python main.py --vis

# 详细输出（每100步打印状态）
python main.py --verbose
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--planner` | 覆盖算法：`boustrophedon` / `spiral` / `random` | `boustrophedon` |
| `--map` | 地图类型：`room`（客厅预设）/ `random`（随机生成） | `room` |
| `--rows` / `--cols` | 随机地图尺寸 | 30 × 30 |
| `--obstacle-density` | 随机地图障碍物密度 (0.0–0.5) | 0.15 |
| `--max-steps` | 最大仿真步数 | 20000 |
| `--seed` | 随机种子（保证复现性） | 42 |
| `--vis` | 启用 Matplotlib 动画 | 关闭 |
| `--verbose` | 逐步输出遥测数据 | 关闭 |

### 编程接口

```python
from src.environment import Environment
from src.simulation import Simulation

# 创建客厅地图
env = Environment.create_room_with_furniture()

# 使用弓字形算法仿真
sim = Simulation(env, planner="boustrophedon", max_steps=10000)
result = sim.run()
print(result)
# {'coverage_pct': 93.79, 'steps': 438, ...}

# ASCII 可视化当前状态
print(sim.render_ascii())
```

---

## 项目结构 / Project Structure

```
Sweeperforweimai/
├── main.py                  # 命令行入口
├── requirements.txt         # Python 依赖
├── src/
│   ├── __init__.py
│   ├── environment.py       # 网格地图 (Cell, Environment)
│   ├── sensor.py            # 传感器模型 (Bump, Cliff, Lidar, Dirt)
│   ├── robot.py             # 机器人状态机与运动控制
│   ├── planner.py           # 路径规划算法 (Boustrophedon, Spiral, Random, A*)
│   └── simulation.py        # 仿真引擎与可视化
└── tests/
    ├── test_environment.py  # 地图单元测试 (19 tests)
    ├── test_planner.py      # 规划算法单元测试 (22 tests)
    └── test_robot.py        # 机器人单元测试 (13 tests)
```

---

## 测试 / Tests

```bash
# 运行全部单元测试
python -m pytest tests/ -v
# 54 passed ✓
```

---

## 仿真结果 / Results

以下结果在 **客厅预设地图（20×20 网格）** 上测试获得：

| 算法 | 覆盖率 | 步数 | 效率（格/步） |
|------|--------|------|--------------|
| Boustrophedon（弓字形）| **93.8%** | 438 | **0.69** |
| Spiral（螺旋形）| 85%+ | ~500 | 0.60+ |
| Random Bounce（随机弹跳）| 88.7% | 554 | 0.55 |

> 弓字形算法在结构化室内环境中综合性能最优，兼顾覆盖率与路径效率。

### 地图图例

| 符号 | 含义 |
|------|------|
| `·` | 未清洁区域 (FREE) |
| `█` | 障碍物 (OBSTACLE) |
| ` ` (空格) | 已清洁 (CLEANED) |
| `⚡` | 充电站 (CHARGING) |
| `R` | 机器人当前位置 |

---

## 核心特性总结 / Key Features

- ✅ 三种覆盖路径规划算法
- ✅ A\* 最短路径回充电站
- ✅ 电池电量管理（低电量自动返回充电）
- ✅ 尘仓容量管理（满仓自动返回清空）
- ✅ 多传感器仿真（碰撞 / 悬崖 / 激光雷达 / 灰尘）
- ✅ 支持任意尺寸地图与随机障碍物生成
- ✅ Matplotlib 动画可视化
- ✅ 54 个单元测试全部通过
- ✅ 完全类型注解，Python 3.9+
