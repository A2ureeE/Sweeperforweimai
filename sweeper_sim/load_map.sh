#!/usr/bin/env bash
# =============================================================
# 一键换图脚本 — load_map.sh
#
# 用法：
#   ./load_map.sh <world文件路径> [map_config参数...]
#
# 示例：
#   # 使用官方发布的地图（将world文件放到worlds/目录）
#   ./load_map.sh official_map.world
#
#   # 指定spawn位置（覆盖map_config.yaml中的默认值）
#   ./load_map.sh official_map.world --spawn-x -5.0 --spawn-y -4.0
#
#   # 完整参数（区域边界、限宽门位置）
#   ./load_map.sh official_map.world \
#       --area "-15 18 -10 10" \
#       --gate1 "3.0 -8.0" \
#       --gate2 "3.0  2.0"
#
# 说明：
#   - 脚本会自动更新 sweeper_bringup/config/map_config.yaml
#   - 然后重新 build 并启动仿真
#   - 原 map_config.yaml 的备份保存为 map_config.yaml.bak
# =============================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 解析参数 ──────────────────────────────────────────────────
WORLD_FILE=""
SPAWN_X=-10.0; SPAWN_Y=-8.0; SPAWN_YAW=0.0
AREA="-12.0 14.5 -9.0 9.5"
GATE1="3.0 -8.0"; GATE2="3.0 2.0"
HEADLESS="${HEADLESS:-0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    *.world|*.sdf)  WORLD_FILE="$1"; shift;;
    --spawn-x)      SPAWN_X="$2";   shift 2;;
    --spawn-y)      SPAWN_Y="$2";   shift 2;;
    --spawn-yaw)    SPAWN_YAW="$2"; shift 2;;
    --area)         AREA="$2";      shift 2;;
    --gate1)        GATE1="$2";     shift 2;;
    --gate2)        GATE2="$2";     shift 2;;
    --headless)     HEADLESS=1;     shift;;
    *) echo "[load_map] 未知参数: $1"; shift;;
  esac
done

if [[ -z "$WORLD_FILE" ]]; then
  echo "[load_map] 用法: $0 <world文件> [选项...]"
  echo "           将world文件放入 sweeper_gazebo/worlds/ 目录后运行"
  exit 1
fi

# ── 检查world文件 ─────────────────────────────────────────────
WORLDS_DIR="$HERE/sweeper_gazebo/worlds"
if [[ -f "$WORLD_FILE" ]]; then
  # 绝对路径或相对路径 — 复制到worlds目录
  WORLD_BASENAME=$(basename "$WORLD_FILE")
  if [[ "$WORLD_FILE" != "$WORLDS_DIR/$WORLD_BASENAME" ]]; then
    echo "[load_map] 复制 $WORLD_FILE → $WORLDS_DIR/"
    cp "$WORLD_FILE" "$WORLDS_DIR/$WORLD_BASENAME"
  fi
elif [[ -f "$WORLDS_DIR/$WORLD_FILE" ]]; then
  WORLD_BASENAME="$WORLD_FILE"
else
  echo "[load_map] 错误: 找不到世界文件: $WORLD_FILE"
  echo "           请将文件放入: $WORLDS_DIR/"
  exit 1
fi

# ── 解析area和gate参数 ────────────────────────────────────────
read -r XMIN XMAX YMIN YMAX <<< "$AREA"
read -r G1X G1Y                <<< "$GATE1"
read -r G2X G2Y                <<< "$GATE2"

# ── 备份并更新 map_config.yaml ────────────────────────────────
MAP_CFG="$HERE/sweeper_bringup/config/map_config.yaml"
cp "$MAP_CFG" "${MAP_CFG}.bak" 2>/dev/null || true

echo "[load_map] 更新 map_config.yaml..."
cat > "$MAP_CFG" <<YAML
# 自动生成 — $(date)
# 原配置备份: map_config.yaml.bak
map:
  world_file: "${WORLD_BASENAME}"

  spawn:
    x:   ${SPAWN_X}
    y:   ${SPAWN_Y}
    z:   0.1
    yaw: ${SPAWN_YAW}

  area:
    x_min: ${XMIN}
    x_max: ${XMAX}
    y_min: ${YMIN}
    y_max: ${YMAX}

  edge:
    follow_offset:  0.20
    sweep_offset:   0.80

  sweep:
    row_spacing:    1.00
    min_turn_radius: 0.95

  gates:
    - id: 1
      center: {x: ${G1X}, y: ${G1Y}}
      heading: 0.0
      width:   2.0
    - id: 2
      center: {x: ${G2X}, y: ${G2Y}}
      heading: 0.0
      width:   2.0

  known_obstacles: []

  dynamic_zone:
    center: {x: 0.0, y: 3.0}
    radius: 5.0

  scoring:
    coverage_goal_pct:   80.0
    edge_follow_goal_s:  30.0
    gate_pass_timeout_s: 120.0
YAML

echo "[load_map] map_config.yaml 已更新:"
echo "  世界文件: $WORLD_BASENAME"
echo "  起始位置: ($SPAWN_X, $SPAWN_Y), yaw=$SPAWN_YAW"
echo "  清扫区域: x=[$XMIN, $XMAX], y=[$YMIN, $YMAX]"
echo "  限宽门1: ($G1X, $G1Y)"
echo "  限宽门2: ($G2X, $G2Y)"

# ── 重新 build ────────────────────────────────────────────────
echo ""
echo "[load_map] 重新编译..."
source /opt/ros/humble/setup.bash
cd "$HERE"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  --packages-select sweeper_bringup sweeper_gazebo sweeper_planning \
  2>&1 | grep -E "Finished|Failed|Error|error" | head -20

echo ""
echo "[load_map] 启动仿真..."
HEADLESS=$HEADLESS ./run.sh
