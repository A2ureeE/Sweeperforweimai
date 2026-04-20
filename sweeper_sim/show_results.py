#!/usr/bin/env python3
"""
竞赛运行结果查看工具
用法：
  python3 show_results.py              # 查看最近一次运行
  python3 show_results.py <run_dir>    # 查看指定运行目录
  python3 show_results.py --list       # 列出所有历史运行
  python3 show_results.py --plot       # 最近运行 + 绘图 (需要 matplotlib)
"""
import json
import os
import sys
import csv
from datetime import datetime

LOG_BASE = os.path.expanduser('~/.ros/sweeper_logs')

# ANSI 颜色
C  = '\033[96m'   # cyan
G  = '\033[92m'   # green
Y  = '\033[93m'   # yellow
R  = '\033[91m'   # red
B  = '\033[1m'    # bold
E  = '\033[0m'    # reset


def color_score(s, mx):
    pct = s / mx if mx else 0
    if pct >= 0.8:  return f'{G}{s:.1f}/{mx}{E}'
    if pct >= 0.5:  return f'{Y}{s:.1f}/{mx}{E}'
    return f'{R}{s:.1f}/{mx}{E}'


def list_runs():
    if not os.path.exists(LOG_BASE):
        print(f'{R}日志目录不存在: {LOG_BASE}{E}')
        return
    runs = sorted([
        d for d in os.listdir(LOG_BASE)
        if d.startswith('run_') and os.path.isdir(os.path.join(LOG_BASE, d))
    ])
    if not runs:
        print(f'{Y}无历史运行记录{E}')
        return
    print(f'\n{B}历史运行记录 ({len(runs)} 次){E}')
    print(f'{"序号":>4}  {"目录名":25}  {"时间":20}  {"估算总分":>8}  {"覆盖率":>8}')
    print('─' * 70)
    for i, run in enumerate(runs):
        ts_str = run.replace('run_', '')
        try:
            dt = datetime.fromtimestamp(int(ts_str)).strftime('%Y-%m-%d %H:%M:%S')
        except Exception:
            dt = ts_str
        summary_f = os.path.join(LOG_BASE, run, 'summary.json')
        score_str = '—'
        cov_str   = '—'
        if os.path.exists(summary_f):
            with open(summary_f) as f:
                s = json.load(f)
            score_str = f"{s.get('estimated_total_score', '?')}"
            cov_str   = f"{s.get('diagnostics', {}).get('final_coverage_pct', '?')}%"
        print(f'{i+1:>4}  {run:25}  {dt:20}  {score_str:>8}  {cov_str:>8}')


def show_run(run_dir: str, plot: bool = False):
    summary_f = os.path.join(run_dir, 'summary.json')
    events_f  = os.path.join(run_dir, 'events.jsonl')
    traj_f    = os.path.join(run_dir, 'trajectory.csv')

    if not os.path.exists(summary_f):
        print(f'{R}找不到摘要文件: {summary_f}{E}')
        return

    with open(summary_f, encoding='utf-8') as f:
        s = json.load(f)

    total = s.get('estimated_total_score', 0)
    dur   = s.get('duration_s', 0)
    rows  = s.get('trajectory_rows', 0)
    scoring = s.get('scoring', {})
    diag    = s.get('diagnostics', {})

    print(f'\n{B}{"═"*55}{E}')
    print(f'{B}  竞赛运行摘要  {E}')
    print(f'{B}{"═"*55}{E}')
    print(f'  目录:    {C}{run_dir}{E}')
    print(f'  运行时长: {dur:.0f} 秒   轨迹行数: {rows}')
    print()

    # 评分总览
    print(f'{B}【评分总览】{E}')
    items = [
        ('避障规划能力', '避障规划能力_30分', 30),
        ('清扫覆盖率  ', '清扫覆盖率_15分',  15),
        ('贴边清扫精度', '贴边清扫精度_15分', 15),
        ('动态避障能力', '动态避障能力_25分', 25),
        ('限宽门通过  ', '限宽门通过_15分',  15),
    ]
    bar_w = 20
    for label, key, mx in items:
        data = scoring.get(key, {})
        sc   = data.get('estimated_score', 0)
        bar  = int(bar_w * sc / mx)
        bar_str = f'{"█"*bar}{"░"*(bar_w-bar)}'
        color_label = color_score(sc, mx)
        print(f'  {label}  {bar_str}  {color_label}')

    total_color = G if total >= 70 else (Y if total >= 50 else R)
    print(f'\n{B}  估算总分: {total_color}{total:.1f}{E}{B} / 100{E}\n')
    print('─' * 55)

    # 详细指标
    print(f'{B}【详细指标】{E}')

    # 覆盖率
    cov_data = scoring.get('清扫覆盖率_15分', {})
    ms = cov_data.get('milestones_hit', [])
    print(f'  覆盖率:  {C}{cov_data.get("final_pct", 0):.1f}%{E}   '
          f'里程碑: {ms}')

    # 贴边
    edge_data = scoring.get('贴边清扫精度_15分', {})
    print(f'  贴边时长: {C}{edge_data.get("edge_follow_s", 0):.1f}s{E}')

    # 限宽门
    gate_data = scoring.get('限宽门通过_15分', {})
    gates_p = gate_data.get('gates_passed', 0)
    gates_e = gate_data.get('gates_entered', 0)
    status  = G if gates_p > 0 else R
    print(f'  限宽门: {status}通过 {gates_p}/{gates_e} 次{E}')
    for gev in gate_data.get('gate_details', []):
        pos    = gev.get('pos', '')
        result = gev.get('result', '')
        t      = gev.get('t', 0)
        icon   = '✓' if result == 'passed' else '→'
        print(f'    {icon} t={t:.1f}s  位置={pos}  结果={result}')

    # 避障
    obs_data = scoring.get('避障规划能力_30分', {})
    stuck_n  = obs_data.get('stuck_count', 0)
    stk_c    = R if stuck_n > 0 else G
    print(f'  卡死次数: {stk_c}{stuck_n}{E}')
    for sk in obs_data.get('stuck_details', []):
        print(f'    ✗ t={sk.get("t",0):.1f}s  pos={sk.get("pos","")}')

    # 动态避障
    dyn_data = scoring.get('动态避障能力_25分', {})
    dyn_n    = dyn_data.get('dyn_avoid_count', 0)
    print(f'  动态避障: {C}{dyn_n} 次{E}')

    print('─' * 55)

    # 模式分布
    print(f'{B}【模式时长分布】{E}')
    md = diag.get('mode_durations_s', {})
    total_dur = sum(md.values()) or 1
    modes_sorted = sorted(md.items(), key=lambda x: -x[1])
    for mode, dur_s in modes_sorted:
        pct_m = 100 * dur_s / total_dur
        bar   = int(20 * pct_m / 100)
        print(f'  {mode:15}  {"█"*bar}{"░"*(20-bar)}  {dur_s:.1f}s ({pct_m:.0f}%)')
    print(f'  模式切换次数: {diag.get("mode_changes", 0)}')

    # 事件流
    if os.path.exists(events_f):
        print(f'\n{B}【事件流 (最近20条)】{E}')
        events = []
        with open(events_f) as f:
            for line in f:
                try: events.append(json.loads(line))
                except Exception: pass
        for ev in events[-20:]:
            t  = ev.pop('t_wall', 0)
            ev.pop('t_sim', None)
            etype = ev.pop('event', '?')
            rest  = ' '.join(f'{k}={v}' for k, v in ev.items())
            icon = {'mode_change': '⇄', 'gate_passed': '✓', 'gate_entering': '→',
                    'stuck_detected': '✗', 'recovery_end': '↺',
                    'coverage_milestone': '★', 'static_obstacle_detected': '!',
                    'dynamic_avoid_start': '⚡'}.get(etype, '·')
            print(f'  {icon} {t:6.1f}s  {Y}{etype}{E}  {rest}')

    # 绘图
    if plot and os.path.exists(traj_f):
        _plot_trajectory(traj_f)

    print(f'\n{B}文件:{E}')
    print(f'  轨迹: {traj_f}')
    print(f'  事件: {events_f}')
    print(f'  摘要: {summary_f}\n')


def _plot_trajectory(traj_f: str):
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print(f'{Y}(跳过绘图: pip install matplotlib){E}')
        return

    ts, xs, ys, covs, modes_raw = [], [], [], [], []
    with open(traj_f) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ts.append(float(row['t_wall']))
                xs.append(float(row['x']))
                ys.append(float(row['y']))
                covs.append(float(row['coverage_pct']))
                modes_raw.append(row['mode'])
            except Exception:
                pass

    if not ts:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('竞赛运行分析', fontsize=14)

    # 轨迹图
    ax = axes[0]
    ax.set_title('机器人轨迹')
    mode_colors = {
        'COVERAGE': '#4CAF50', 'EDGE_FOLLOW': '#2196F3',
        'STATIC_DETOUR': '#FF9800', 'DYNAMIC_AVOID': '#F44336',
        'NARROW_GATE': '#9C27B0', 'STOP': '#9E9E9E', 'UNKNOWN': '#BDBDBD',
    }
    prev_mode = modes_raw[0] if modes_raw else 'UNKNOWN'
    seg_x, seg_y = [xs[0]], [ys[0]]
    for i in range(1, len(xs)):
        if modes_raw[i] == prev_mode:
            seg_x.append(xs[i]); seg_y.append(ys[i])
        else:
            c = mode_colors.get(prev_mode, '#888')
            ax.plot(seg_x, seg_y, color=c, linewidth=1.5)
            seg_x, seg_y = [xs[i]], [ys[i]]
            prev_mode = modes_raw[i]
    c = mode_colors.get(prev_mode, '#888')
    ax.plot(seg_x, seg_y, color=c, linewidth=1.5)
    ax.plot(xs[0], ys[0], 'go', ms=8, label='起点')
    ax.plot(xs[-1], ys[-1], 'rs', ms=8, label='终点')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    # 图例
    handles = [plt.Line2D([0],[0], color=v, label=k, linewidth=2)
               for k, v in mode_colors.items()] + [
        plt.Line2D([0],[0], color='g', marker='o', label='起点', linestyle=''),
        plt.Line2D([0],[0], color='r', marker='s', label='终点', linestyle=''),
    ]
    ax.legend(handles=handles, fontsize=7, loc='upper right')

    # 覆盖率曲线
    ax2 = axes[1]
    ax2.set_title('覆盖率随时间变化')
    ax2.plot(ts, covs, color='#4CAF50', linewidth=2)
    ax2.axhline(y=80, color='orange', linestyle='--', label='80% 目标')
    ax2.fill_between(ts, covs, alpha=0.2, color='#4CAF50')
    ax2.set_xlabel('时间 (s)'); ax2.set_ylabel('覆盖率 (%)')
    ax2.set_ylim(0, 105)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = traj_f.replace('trajectory.csv', 'analysis.png')
    plt.savefig(out, dpi=120, bbox_inches='tight')
    plt.show()
    print(f'{G}图表已保存: {out}{E}')


def get_latest_run():
    if not os.path.exists(LOG_BASE):
        return None
    runs = sorted([
        d for d in os.listdir(LOG_BASE)
        if d.startswith('run_') and os.path.isdir(os.path.join(LOG_BASE, d))
    ])
    return os.path.join(LOG_BASE, runs[-1]) if runs else None


def main():
    args = sys.argv[1:]

    if '--list' in args:
        list_runs()
        return

    plot = '--plot' in args
    args = [a for a in args if not a.startswith('--')]

    if args:
        run_dir = args[0]
        # 支持序号（来自 --list 输出）
        if run_dir.isdigit():
            runs = sorted([
                d for d in os.listdir(LOG_BASE)
                if d.startswith('run_') and os.path.isdir(os.path.join(LOG_BASE, d))
            ])
            idx = int(run_dir) - 1
            if 0 <= idx < len(runs):
                run_dir = os.path.join(LOG_BASE, runs[idx])
            else:
                print(f'{R}序号超出范围{E}'); return
    else:
        run_dir = get_latest_run()
        if not run_dir:
            print(f'{Y}没有找到任何运行记录。先运行仿真再查看结果。{E}')
            return

    show_run(run_dir, plot=plot)


if __name__ == '__main__':
    main()
