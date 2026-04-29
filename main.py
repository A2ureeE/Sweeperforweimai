#!/usr/bin/env python3
"""
main.py — Entry point for the Sweeperforweimai sweeping robot simulation.

Usage examples
--------------
  # Run with default settings (boustrophedon planner, living-room map)
  python main.py

  # Run with a random map and spiral planner
  python main.py --planner spiral --map random

  # Run with ASCII output only (no Matplotlib window)
  python main.py --no-vis

  # Show help
  python main.py --help
"""

from __future__ import annotations

import argparse
import sys
import time

from src.environment import Environment
from src.simulation import Simulation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweeperforweimai — Sweeping Robot Simulation\n"
        "江苏大学生交通科技大赛企业命题赛道扫地机器人方案",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--planner",
        choices=["boustrophedon", "spiral", "random"],
        default="boustrophedon",
        help="Coverage algorithm to use (default: boustrophedon).",
    )
    parser.add_argument(
        "--map",
        choices=["room", "random"],
        default="room",
        help="Map preset: 'room' = living room with furniture, "
             "'random' = randomly generated obstacles (default: room).",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=30,
        help="Grid rows for the 'random' map (default: 30).",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=30,
        help="Grid columns for the 'random' map (default: 30).",
    )
    parser.add_argument(
        "--obstacle-density",
        type=float,
        default=0.15,
        help="Obstacle density for random map, 0.0–0.5 (default: 0.15).",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=20_000,
        help="Maximum number of simulation steps (default: 20000).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducible random maps (default: 42).",
    )
    parser.add_argument(
        "--vis",
        action="store_true",
        default=False,
        help="Show an animated Matplotlib window (requires matplotlib).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="Print telemetry every 100 steps.",
    )
    return parser.parse_args()


def build_environment(args: argparse.Namespace) -> Environment:
    if args.map == "room":
        return Environment.create_room_with_furniture()
    return Environment.create_random(
        rows=args.rows,
        cols=args.cols,
        obstacle_density=args.obstacle_density,
        seed=args.seed,
    )


def print_banner() -> None:
    print("=" * 60)
    print("  Sweeperforweimai — 扫地机器人仿真系统")
    print("  江苏大学生交通科技大赛 · 企业命题赛道")
    print("=" * 60)


def main() -> int:
    args = parse_args()
    print_banner()

    env = build_environment(args)
    print(
        f"\n地图: {args.map} ({env.rows}×{env.cols}), "
        f"可清洁格数={env.total_cleanable}, "
        f"算法={args.planner}\n"
    )

    sim = Simulation(
        env=env,
        planner=args.planner,
        max_steps=args.max_steps,
        verbose=args.verbose,
    )

    t0 = time.perf_counter()
    result = sim.run(visualise=args.vis)
    elapsed = time.perf_counter() - t0

    print("\n---- 仿真结果 ----")
    for key, val in result.items():
        print(f"  {key:<30} {val}")
    print(f"  {'wall_time_s':<30} {elapsed:.3f}")
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
