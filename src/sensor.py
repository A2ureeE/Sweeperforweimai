"""
sensor.py — Simulated sensors for the sweeping robot.

Sensors provided:
  - BumpSensor   : detects collisions with obstacles at the robot's next cell.
  - CliffSensor  : detects "drop-offs" (edge of the grid).
  - LidarSensor  : 360° distance scan (returns distances to nearest obstacle).
  - DirtSensor   : detects whether the current cell is dirty (FREE).
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

from .environment import Cell, Environment


class BumpSensor:
    """
    Detects whether moving one step in a given direction would hit an obstacle
    or go out of bounds.
    """

    def check(
        self,
        env: Environment,
        row: int,
        col: int,
        dr: int,
        dc: int,
    ) -> bool:
        """
        Return True if moving (dr, dc) from (row, col) is blocked.

        Parameters
        ----------
        env   : Environment
        row   : current row
        col   : current column
        dr    : row delta  (-1, 0, +1)
        dc    : column delta (-1, 0, +1)
        """
        nr, nc = row + dr, col + dc
        return not env.is_passable(nr, nc)


class CliffSensor:
    """
    Detects whether the robot is at the edge of the navigable area.
    A "cliff" is triggered when the next cell is out of bounds.
    """

    def check(
        self,
        env: Environment,
        row: int,
        col: int,
        dr: int,
        dc: int,
    ) -> bool:
        """Return True if moving (dr, dc) would leave the grid."""
        nr, nc = row + dr, col + dc
        return not env.in_bounds(nr, nc)


class LidarSensor:
    """
    Simplified 2-D Lidar that casts rays in ``n_beams`` evenly-spaced directions
    and returns the distance (in cells) to the nearest obstacle or boundary.

    Parameters
    ----------
    n_beams   : int   — number of evenly-spaced rays (default 36 → 10° apart).
    max_range : float — maximum detection range in cells (default 10).
    noise_std : float — Gaussian noise standard deviation added to each reading
                        (0 = noiseless, default 0.05 cell).
    """

    def __init__(
        self,
        n_beams: int = 36,
        max_range: float = 10.0,
        noise_std: float = 0.05,
    ) -> None:
        self.n_beams = n_beams
        self.max_range = max_range
        self.noise_std = noise_std
        angles = np.linspace(0.0, 2 * math.pi, n_beams, endpoint=False)
        self._sin = np.sin(angles)
        self._cos = np.cos(angles)

    def scan(
        self,
        env: Environment,
        row: float,
        col: float,
        rng: np.random.Generator | None = None,
    ) -> np.ndarray:
        """
        Cast all beams and return an array of distances of shape (n_beams,).

        Parameters
        ----------
        env : Environment
        row : float — robot row (can be fractional for sub-cell accuracy)
        col : float — robot column
        rng : numpy Generator for reproducible noise (optional)
        """
        distances = np.full(self.n_beams, self.max_range)
        for i in range(self.n_beams):
            distances[i] = self._cast_ray(env, row, col, self._sin[i], self._cos[i])
        if self.noise_std > 0:
            generator = rng if rng is not None else np.random.default_rng()
            noise = generator.normal(0.0, self.noise_std, self.n_beams)
            distances = np.clip(distances + noise, 0.0, self.max_range)
        return distances

    def _cast_ray(
        self,
        env: Environment,
        row: float,
        col: float,
        sin_a: float,
        cos_a: float,
    ) -> float:
        """Step along one ray until hitting an obstacle or reaching max_range."""
        step = 0.5  # half-cell steps for resolution
        dist = 0.0
        while dist < self.max_range:
            dist += step
            r = int(round(row + dist * sin_a))
            c = int(round(col + dist * cos_a))
            if not env.in_bounds(r, c):
                return dist
            if env.grid[r, c] == Cell.OBSTACLE:
                return dist
        return self.max_range


class DirtSensor:
    """Detects whether the robot's current cell is dirty (not yet cleaned)."""

    def check(self, env: Environment, row: int, col: int) -> bool:
        """Return True if the cell at (row, col) is FREE (dirty)."""
        return env.in_bounds(row, col) and env.grid[row, col] == Cell.FREE
