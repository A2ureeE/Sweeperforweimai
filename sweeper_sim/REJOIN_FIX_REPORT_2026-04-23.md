# 清扫车避障回切修复记录（2026-04-23）

## 1. 背景与问题

在仿真中，车辆在穿过 Gate 2 后，容易在 coverage path 的 `519~525` 附近进入局部循环：

- 频繁在 `COVERAGE / STATIC_DETOUR / DYNAMIC_AVOID` 间切换；
- 控制器多次触发 `Stuck 检测触发`；
- 尽管避障偏移峰值已被压制到约 `1.05m`，仍会在回切后陷入局部不可行姿态。

核心原因不是“推开力过大”，而是“回切目标点虽然最近、也勉强安全，但几何上仍容易陷入局部困难区”。

---

## 2. 本轮代码修改

修改文件：

- `sweeper_sim/sweeper_planning/sweeper_planning/planner_node.py`

### 2.1 新增安全回切前推函数

新增/扩展函数：

- `_advance_to_safe_rejoin_idx(start_idx, clear_threshold, search_ahead, min_advance)`

作用：

1. 先对回切目标做最小前推（`min_advance`）；
2. 再在前向窗口内按段清障裕度（`_segment_min_clearance`）选择更安全的 rejoin 点；
3. 若找不到达到阈值的点，则回退到窗口内“最清晰”候选点。

### 2.2 穿门回切逻辑修改

在 Gate 回归路径生成处（`_in_gate_path` 结束分支）：

- 先求最近 coverage idx；
- 再调用 `_advance_to_safe_rejoin_idx(...)` 做安全前推；
- 新增日志 `穿门回归前推: idx a→b`。

当前参数：

- `search_ahead=80`
- `min_advance=12`

### 2.3 避障回切逻辑修改

在避障尾部清障放行分支（`_detour_clear_pending`）：

- 原“最近点回切”改为“前推 + 安全筛选”；
- 新增日志 `避障回切前推: idx a→b`。

当前参数：

- `search_ahead=60`
- `min_advance=6`

---

## 3. 运行日志观察（本轮）

### 3.1 生效证据

日志已出现如下信息，说明回切前推逻辑已加载并执行：

- `穿门回归前推: idx 22→28`
- `穿门回归前推: idx 520→525`

### 3.2 仍存在的现象（触发二次加固原因）

尽管已有前推，车辆仍在 `525` 附近反复触发：

- `Stuck 检测触发 — mode=COVERAGE/STATIC_DETOUR`

说明“仅安全筛选但前推幅度偏小”时，仍可能落回局部陷阱。因此本轮后半段引入了 `min_advance` 并提高了穿门回切最小前推。

---

## 4. 编译与静态检查

本轮修改后检查结果：

- `ReadLints`：无新增错误；
- `python3 -m py_compile planner_node.py`：通过；
- `colcon build --packages-select sweeper_planning`：通过。

---

## 5. 下一轮验证重点

重启仿真后重点观察：

1. Gate 2 后日志是否出现更大的回切前推（例如 `520→53x/54x`）；
2. `525` 附近 `Stuck` 触发频次是否显著下降；
3. 是否仍出现长时间 `COVERAGE ↔ STATIC_DETOUR ↔ DYNAMIC_AVOID` 抖动；
4. `DETOUR_DIAG` 最大偏移是否继续保持在约 `<=1.05m`。

---

## 6. 结论

本轮把问题从“是否过度偏移”进一步收敛到“回切目标点选择不够前瞻”。  
已完成从“最近点回切”到“最小前推 + 安全筛选”的策略升级，具备继续压制 `519~525` 局部循环的基础。
