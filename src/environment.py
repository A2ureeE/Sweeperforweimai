"""
environment.py — Grid-based map representation for the sweeping robot.

The map is a 2-D grid where each cell can be:
  - FREE    : passable, not yet cleaned
  - OBSTACLE: wall / furniture
  - CLEANED : passable, already cleaned
  - CHARGING: the charging station (home base)
"""

from __future__ import annotations

import random
from enum import IntEnum
from typing import List, Optional, Tuple

import numpy as np


class Cell(IntEnum):
    FREE = 0
    OBSTACLE = 1
    CLEANED = 2
    CHARGING = 3


class Environment:
    """
    2-D grid environment for the sweeping robot.

    Parameters
    ----------
    rows : int
        Number of rows (height) of the grid.
    cols : int
        Number of columns (width) of the grid.
    cell_size : float
        Physical size of one cell in metres (default 0.1 m = 10 cm).
    charging_pos : tuple[int, int] | None
        (row, col) of the charging station.  Defaults to (0, 0).
    """

    def __init__(
        self,
        rows: int = 50,
        cols: int = 50,
        cell_size: float = 0.1,
        charging_pos: Optional[Tuple[int, int]] = None,
    ) -> None:
        if rows <= 0 or cols <= 0:
            raise ValueError("rows and cols must be positive integers.")
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size

        self.grid: np.ndarray = np.zeros((rows, cols), dtype=np.int8)

        self.charging_pos: Tuple[int, int] = charging_pos if charging_pos else (0, 0)
        self._place_charging_station(self.charging_pos)

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------

    def _place_charging_station(self, pos: Tuple[int, int]) -> None:
        r, c = pos
        if not self.in_bounds(r, c):
            raise ValueError(f"Charging station position {pos} is out of bounds.")
        self.grid[r, c] = Cell.CHARGING

    def in_bounds(self, row: int, col: int) -> bool:
        """Return True if (row, col) is within the grid."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_passable(self, row: int, col: int) -> bool:
        """Return True if the robot can move into this cell."""
        return self.in_bounds(row, col) and self.grid[row, col] != Cell.OBSTACLE

    def is_cleaned(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and self.grid[row, col] in (
            Cell.CLEANED,
            Cell.CHARGING,
        )

    def mark_cleaned(self, row: int, col: int) -> None:
        """Mark a cell as cleaned (unless it is an obstacle or the charger)."""
        if self.in_bounds(row, col) and self.grid[row, col] == Cell.FREE:
            self.grid[row, col] = Cell.CLEANED

    def add_obstacle(self, row: int, col: int) -> None:
        """Place an obstacle at (row, col).  Cannot overwrite the charging station."""
        if not self.in_bounds(row, col):
            raise ValueError(f"Position ({row}, {col}) is out of bounds.")
        if (row, col) == self.charging_pos:
            raise ValueError("Cannot place obstacle on the charging station.")
        self.grid[row, col] = Cell.OBSTACLE

    def add_rectangular_obstacle(
        self,
        top_left: Tuple[int, int],
        bottom_right: Tuple[int, int],
    ) -> None:
        """Add a rectangular block of obstacles."""
        r0, c0 = top_left
        r1, c1 = bottom_right
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                if (r, c) != self.charging_pos:
                    self.grid[r, c] = Cell.OBSTACLE

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @property
    def total_cleanable(self) -> int:
        """Number of cells that are cleanable (FREE + CLEANED + CHARGING)."""
        return int(np.sum(self.grid != Cell.OBSTACLE))

    @property
    def total_cleaned(self) -> int:
        """Number of cells already cleaned (CLEANED + CHARGING)."""
        return int(
            np.sum((self.grid == Cell.CLEANED) | (self.grid == Cell.CHARGING))
        )

    @property
    def coverage_ratio(self) -> float:
        """Fraction of cleanable area that has been cleaned, in [0, 1]."""
        if self.total_cleanable == 0:
            return 1.0
        return self.total_cleaned / self.total_cleanable

    def uncleaned_cells(self) -> List[Tuple[int, int]]:
        """Return all (row, col) positions that are FREE (not yet cleaned)."""
        positions = np.argwhere(self.grid == Cell.FREE)
        return [(int(r), int(c)) for r, c in positions]

    # ------------------------------------------------------------------
    # Factory helpers
    # ------------------------------------------------------------------

    @classmethod
    def create_random(
        cls,
        rows: int = 30,
        cols: int = 30,
        obstacle_density: float = 0.15,
        seed: Optional[int] = None,
    ) -> "Environment":
        """
        Create an environment with randomly placed rectangular obstacles.

        Parameters
        ----------
        rows, cols : int
            Grid dimensions.
        obstacle_density : float
            Approximate fraction of cells to block (0.0–0.5 recommended).
        seed : int | None
            Random seed for reproducibility.
        """
        rng = random.Random(seed)
        env = cls(rows=rows, cols=cols)
        target = int(rows * cols * obstacle_density)
        placed = 0
        attempts = 0
        while placed < target and attempts < target * 10:
            attempts += 1
            r = rng.randint(0, rows - 1)
            c = rng.randint(0, cols - 1)
            h = rng.randint(1, max(1, rows // 8))
            w = rng.randint(1, max(1, cols // 8))
            r2 = min(r + h, rows - 1)
            c2 = min(c + w, cols - 1)
            new_obst = (r2 - r + 1) * (c2 - c + 1)
            if placed + new_obst > target * 1.2:
                continue
            if (r, c) == env.charging_pos or (r2, c2) == env.charging_pos:
                continue
            env.add_rectangular_obstacle((r, c), (r2, c2))
            placed += new_obst
        return env

    @classmethod
    def create_room_with_furniture(cls) -> "Environment":
        """
        Pre-built 20 × 20 environment that mimics a living room with furniture.
        """
        env = cls(rows=20, cols=20, charging_pos=(0, 0))
        # Sofa
        env.add_rectangular_obstacle((2, 10), (4, 15))
        # Coffee table
        env.add_rectangular_obstacle((6, 11), (7, 14))
        # Wardrobe
        env.add_rectangular_obstacle((0, 17), (5, 19))
        # Dining table
        env.add_rectangular_obstacle((12, 3), (15, 8))
        # Bookshelf
        env.add_rectangular_obstacle((15, 14), (19, 15))
        return env
