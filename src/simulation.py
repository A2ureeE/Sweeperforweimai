"""
simulation.py — Simulation runner and optional ASCII/Matplotlib visualiser.

Usage (programmatic)
--------------------
    from src.simulation import Simulation
    from src.environment import Environment

    env = Environment.create_room_with_furniture()
    sim = Simulation(env, planner="boustrophedon", max_steps=5000)
    result = sim.run()
    print(result)

Usage (with visualisation)
--------------------------
    sim.run(visualise=True)
"""

from __future__ import annotations

import sys
import time
from typing import Dict, Literal, Optional

from .environment import Cell, Environment
from .planner import (
    AStarPlanner,
    BoustrophedonPlanner,
    RandomBouncePlanner,
    SpiralPlanner,
    path_to_steps,
)
from .robot import Robot, RobotState

PlannerName = Literal["boustrophedon", "spiral", "random"]


class Simulation:
    """
    Drives the robot through the environment using the chosen planner.

    Parameters
    ----------
    env        : Environment  — the map.
    planner    : str          — "boustrophedon" | "spiral" | "random".
    max_steps  : int          — hard limit on simulation steps (safety net).
    verbose    : bool         — print step-by-step telemetry.
    """

    def __init__(
        self,
        env: Environment,
        planner: PlannerName = "boustrophedon",
        max_steps: int = 20_000,
        verbose: bool = False,
    ) -> None:
        self.env = env
        self.robot = Robot(env)
        self.max_steps = max_steps
        self.verbose = verbose
        self._planner_name = planner
        self._planner = self._build_planner(planner)
        self._waypoint_iter = self._planner.plan(env, self.robot.position)
        self._current_path: list = []  # (dr, dc) steps toward next waypoint
        self._done = False

    # ------------------------------------------------------------------
    # Planner factory
    # ------------------------------------------------------------------

    def _build_planner(self, name: str):
        if name == "boustrophedon":
            return BoustrophedonPlanner()
        if name == "spiral":
            return SpiralPlanner()
        if name == "random":
            return RandomBouncePlanner()
        raise ValueError(f"Unknown planner: {name!r}. Choose from boustrophedon, spiral, random.")

    # ------------------------------------------------------------------
    # Single-step logic
    # ------------------------------------------------------------------

    def step(self) -> bool:
        """
        Advance the simulation by one robot action.

        Returns True if the simulation should continue, False if it is done.
        """
        robot = self.robot
        env = self.env

        # ---- Handle charging ----
        if robot.at_charger() and robot.needs_to_return():
            fully_charged = robot.charge()
            if fully_charged:
                # Resume coverage after full charge
                self._waypoint_iter = self._build_planner(self._planner_name).plan(
                    env, robot.position
                )
                self._current_path = []
            return True

        # ---- Return to base if needed ----
        if robot.needs_to_return() and not robot.at_charger():
            robot.state = RobotState.RETURNING
            if not self._current_path:
                path = AStarPlanner.plan(env, robot.position, env.charging_pos)
                self._current_path = path_to_steps(path)
            if self._current_path:
                dr, dc = self._current_path.pop(0)
                robot.move(dr, dc)
            return True

        # ---- Normal coverage ----
        if not self._current_path:
            waypoint = self._get_next_waypoint()
            if waypoint is None:
                self._done = True
                return False
            path = AStarPlanner.plan(env, robot.position, waypoint)
            self._current_path = path_to_steps(path)

        if self._current_path:
            dr, dc = self._current_path.pop(0)
            robot.move(dr, dc)
        else:
            # Stuck or waypoint already reached; skip
            self._current_path = []

        return True

    def _get_next_waypoint(self):
        """Pull the next uncleaned waypoint from the planner iterator."""
        while True:
            try:
                wp = next(self._waypoint_iter)
            except StopIteration:
                return None
            r, c = wp
            # Skip already-cleaned cells and obstacles
            if self.env.grid[r, c] == Cell.FREE:
                return wp

    # ------------------------------------------------------------------
    # Run loop
    # ------------------------------------------------------------------

    def run(self, visualise: bool = False) -> Dict:
        """
        Run until coverage is complete or max_steps is reached.

        Parameters
        ----------
        visualise : bool — if True, render an animated Matplotlib plot.

        Returns
        -------
        dict with simulation statistics.
        """
        if visualise:
            return self._run_animated()

        for step_idx in range(self.max_steps):
            should_continue = self.step()
            if self.verbose and step_idx % 100 == 0:
                print(f"Step {step_idx:5d} | {self.robot.status()}")
            if not should_continue:
                break

        return self._summary()

    def _run_animated(self) -> Dict:
        """Matplotlib animated simulation."""
        try:
            import matplotlib.pyplot as plt
            import matplotlib.colors as mcolors
        except ImportError:
            print(
                "[simulation] matplotlib is not installed. "
                "Run: pip install matplotlib",
                file=sys.stderr,
            )
            return self.run(visualise=False)

        cmap = mcolors.ListedColormap(["white", "dimgray", "lightgreen", "gold"])
        bounds = [-0.5, 0.5, 1.5, 2.5, 3.5]
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        fig, ax = plt.subplots(figsize=(8, 8))
        im = ax.imshow(
            self.env.grid,
            cmap=cmap,
            norm=norm,
            interpolation="nearest",
            origin="upper",
        )
        robot_dot, = ax.plot(
            [], [], "ro", markersize=8, label="Robot"
        )
        ax.set_title("Sweeping Robot Simulation")
        ax.legend(loc="upper right")
        plt.tight_layout()
        plt.ion()
        plt.show()

        for step_idx in range(self.max_steps):
            should_continue = self.step()
            if step_idx % 5 == 0:  # refresh every 5 steps for speed
                im.set_data(self.env.grid)
                robot_dot.set_data([self.robot.col], [self.robot.row])
                ax.set_title(
                    f"Step {step_idx} | Coverage {self.env.coverage_ratio * 100:.1f}%"
                    f" | Battery {self.robot.battery:.0f}%"
                )
                plt.pause(0.01)
            if not should_continue:
                break

        plt.ioff()
        plt.title(
            f"Done – Coverage {self.env.coverage_ratio * 100:.1f}% "
            f"in {self.robot.steps} steps"
        )
        plt.show()
        return self._summary()

    # ------------------------------------------------------------------
    # ASCII rendering
    # ------------------------------------------------------------------

    def render_ascii(self) -> str:
        """Return a compact ASCII representation of the current grid state."""
        symbols = {
            Cell.FREE: "·",
            Cell.OBSTACLE: "█",
            Cell.CLEANED: " ",
            Cell.CHARGING: "⚡",
        }
        rows = []
        for r in range(self.env.rows):
            row_chars = []
            for c in range(self.env.cols):
                if (r, c) == self.robot.position:
                    row_chars.append("R")
                else:
                    row_chars.append(symbols.get(self.env.grid[r, c], "?"))
            rows.append("".join(row_chars))
        return "\n".join(rows)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    def _summary(self) -> Dict:
        robot = self.robot
        env = self.env
        return {
            "planner": self._planner_name,
            "grid_size": f"{env.rows}×{env.cols}",
            "total_cleanable": env.total_cleanable,
            "total_cleaned": env.total_cleaned,
            "coverage_pct": round(env.coverage_ratio * 100, 2),
            "steps": robot.steps,
            "cells_cleaned_by_robot": robot.cells_cleaned,
            "total_distance_cells": round(robot.total_distance, 2),
            "final_battery_pct": round(robot.battery / robot.battery_capacity * 100, 1),
            "final_bin_pct": round(robot.bin_level, 1),
            "robot_state": robot.state.name,
        }
