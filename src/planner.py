"""
planner.py — Path-planning algorithms for the sweeping robot.

Algorithms implemented:
  1. BoustrophedonPlanner  — systematic back-and-forth (lawnmower) coverage.
  2. SpiralPlanner         — outward/inward spiral coverage.
  3. RandomBouncePlanner   — iRobot-style random walk with obstacle bouncing.
  4. AStarPlanner          — A* shortest-path planner (used for return-to-base).
"""

from __future__ import annotations

import heapq
import random
from abc import ABC, abstractmethod
from typing import Iterator, List, Optional, Tuple

import numpy as np

from .environment import Cell, Environment


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class BasePlanner(ABC):
    """Abstract base for all coverage planners."""

    @abstractmethod
    def plan(self, env: Environment, start: Tuple[int, int]) -> Iterator[Tuple[int, int]]:
        """
        Yield (row, col) waypoints that the robot should visit.

        The iterator may be infinite (e.g. RandomBouncePlanner) or finite.
        """


# ---------------------------------------------------------------------------
# 1. Boustrophedon (lawnmower / back-and-forth) planner
# ---------------------------------------------------------------------------

class BoustrophedonPlanner(BasePlanner):
    """
    Scans rows left-to-right on even rows, right-to-left on odd rows,
    skipping obstacles.  After each row the robot transitions to the next.
    """

    def plan(
        self,
        env: Environment,
        start: Tuple[int, int],
    ) -> Iterator[Tuple[int, int]]:
        for r in range(env.rows):
            col_range = range(env.cols) if r % 2 == 0 else range(env.cols - 1, -1, -1)
            for c in col_range:
                if env.grid[r, c] != Cell.OBSTACLE:
                    yield (r, c)


# ---------------------------------------------------------------------------
# 2. Spiral planner
# ---------------------------------------------------------------------------

class SpiralPlanner(BasePlanner):
    """
    Generates an inward rectangular spiral starting from the top-left corner.
    Cells blocked by obstacles are skipped.
    """

    def plan(
        self,
        env: Environment,
        start: Tuple[int, int],
    ) -> Iterator[Tuple[int, int]]:
        visited = np.zeros((env.rows, env.cols), dtype=bool)
        top, bottom, left, right = 0, env.rows - 1, 0, env.cols - 1

        while top <= bottom and left <= right:
            # → right along top row
            for c in range(left, right + 1):
                if not visited[top, c] and env.grid[top, c] != Cell.OBSTACLE:
                    visited[top, c] = True
                    yield (top, c)
            top += 1

            # ↓ down along right column
            for r in range(top, bottom + 1):
                if not visited[r, right] and env.grid[r, right] != Cell.OBSTACLE:
                    visited[r, right] = True
                    yield (r, right)
            right -= 1

            # ← left along bottom row
            if top <= bottom:
                for c in range(right, left - 1, -1):
                    if not visited[bottom, c] and env.grid[bottom, c] != Cell.OBSTACLE:
                        visited[bottom, c] = True
                        yield (bottom, c)
                bottom -= 1

            # ↑ up along left column
            if left <= right:
                for r in range(bottom, top - 1, -1):
                    if not visited[r, left] and env.grid[r, left] != Cell.OBSTACLE:
                        visited[r, left] = True
                        yield (r, left)
                left += 1


# ---------------------------------------------------------------------------
# 3. Random-bounce planner  (iRobot Roomba-style)
# ---------------------------------------------------------------------------

class RandomBouncePlanner(BasePlanner):
    """
    Moves in a straight line until hitting an obstacle, then picks a random
    new direction.  Continues indefinitely.

    Parameters
    ----------
    seed : int | None — random seed for reproducibility.
    """

    # Cardinal + diagonal directions
    DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def __init__(self, seed: Optional[int] = None) -> None:
        self._rng = random.Random(seed)

    def plan(
        self,
        env: Environment,
        start: Tuple[int, int],
    ) -> Iterator[Tuple[int, int]]:
        row, col = start
        dr, dc = self._rng.choice(self.DIRECTIONS)
        while True:
            nr, nc = row + dr, col + dc
            if env.is_passable(nr, nc):
                row, col = nr, nc
                yield (row, col)
            else:
                # Bounce: pick a random passable direction
                passable = [
                    (r, c)
                    for r, c in self.DIRECTIONS
                    if env.is_passable(row + r, col + c)
                ]
                if not passable:
                    return  # robot is trapped
                dr, dc = self._rng.choice(passable)


# ---------------------------------------------------------------------------
# 4. A* planner  (used for return-to-base and targeted navigation)
# ---------------------------------------------------------------------------

class AStarPlanner:
    """
    A* shortest path from ``start`` to ``goal`` on the grid.

    Returns a list of (row, col) waypoints including start and goal,
    or an empty list if no path exists.
    """

    @staticmethod
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def plan(
        env: Environment,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Return waypoints (including start and goal), or [] if unreachable."""
        if start == goal:
            return [start]

        open_heap: list = []
        heapq.heappush(open_heap, (0.0, start))
        came_from: dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        g_score: dict[Tuple[int, int], float] = {start: 0.0}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected grid

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                return AStarPlanner._reconstruct(came_from, current)
            r, c = current
            for dr, dc in neighbors:
                neighbor = (r + dr, c + dc)
                if not env.is_passable(*neighbor):
                    continue
                tentative_g = g_score[current] + 1.0
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + AStarPlanner.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))

        return []  # No path found

    @staticmethod
    def _reconstruct(
        came_from: dict,
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        path: List[Tuple[int, int]] = []
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Utility: path to delta-steps
# ---------------------------------------------------------------------------

def path_to_steps(
    path: List[Tuple[int, int]],
) -> List[Tuple[int, int]]:
    """Convert a list of (row, col) waypoints to (dr, dc) movement deltas."""
    steps = []
    for i in range(1, len(path)):
        dr = path[i][0] - path[i - 1][0]
        dc = path[i][1] - path[i - 1][1]
        steps.append((dr, dc))
    return steps
