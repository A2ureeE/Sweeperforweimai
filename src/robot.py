"""
robot.py — Robot state machine and motion controller.

The Robot class models:
  - Position on the grid (row, col) and heading (degrees, 0 = right)
  - Battery level (0–100 %)
  - Dust-bin capacity (0–100 %)
  - Cleaning mode (on / off)
  - State machine: CLEANING → LOW_BATTERY → RETURNING → CHARGING → FULL_BIN
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Optional, Tuple

from .environment import Environment
from .sensor import BumpSensor, CliffSensor, DirtSensor


class RobotState(Enum):
    CLEANING = auto()
    LOW_BATTERY = auto()
    RETURNING = auto()
    CHARGING = auto()
    FULL_BIN = auto()
    DONE = auto()
    ERROR = auto()


# Direction vectors: (delta_row, delta_col)
DIRECTION_VECTORS: dict[str, Tuple[int, int]] = {
    "N": (-1, 0),
    "S": (1, 0),
    "E": (0, 1),
    "W": (0, -1),
    "NE": (-1, 1),
    "NW": (-1, -1),
    "SE": (1, 1),
    "SW": (1, -1),
}

DIRECTION_ANGLE: dict[str, float] = {
    "E": 0.0,
    "NE": 45.0,
    "N": 90.0,
    "NW": 135.0,
    "W": 180.0,
    "SW": 225.0,
    "S": 270.0,
    "SE": 315.0,
}


class Robot:
    """
    Autonomous sweeping robot.

    Parameters
    ----------
    env               : Environment — the map the robot operates in.
    start_pos         : (row, col)  — initial position (default = charging station).
    battery_capacity  : float       — maximum battery in arbitrary units (default 100).
    battery_drain_per_step : float  — battery consumed per movement step (default 0.1).
    battery_charge_per_step: float  — battery recovered per step while charging (default 1.0).
    bin_fill_per_step : float       — dust collected per step on a dirty cell (default 0.3).
    low_battery_threshold : float   — battery level that triggers RETURNING (default 20).
    full_bin_threshold    : float   — bin level that triggers FULL_BIN (default 90).
    """

    def __init__(
        self,
        env: Environment,
        start_pos: Optional[Tuple[int, int]] = None,
        battery_capacity: float = 100.0,
        battery_drain_per_step: float = 0.1,
        battery_charge_per_step: float = 1.0,
        bin_fill_per_step: float = 0.3,
        low_battery_threshold: float = 20.0,
        full_bin_threshold: float = 90.0,
    ) -> None:
        self.env = env
        self.row, self.col = start_pos if start_pos else env.charging_pos
        self.heading: float = 0.0  # degrees; 0 = East
        self.state: RobotState = RobotState.CLEANING

        # Energy / dustbin
        self.battery: float = battery_capacity
        self.battery_capacity = battery_capacity
        self.battery_drain_per_step = battery_drain_per_step
        self.battery_charge_per_step = battery_charge_per_step
        self.bin_level: float = 0.0
        self.bin_fill_per_step = bin_fill_per_step
        self.low_battery_threshold = low_battery_threshold
        self.full_bin_threshold = full_bin_threshold

        # Sensors
        self._bump = BumpSensor()
        self._cliff = CliffSensor()
        self._dirt = DirtSensor()

        # Metrics
        self.steps: int = 0
        self.cells_cleaned: int = 0
        self.total_distance: float = 0.0

    # ------------------------------------------------------------------
    # Public movement API
    # ------------------------------------------------------------------

    def move(self, dr: int, dc: int) -> bool:
        """
        Attempt to move (dr, dc) relative to current position.

        Returns True on success, False if blocked.
        Updates sensors, battery, dust collection, and state.
        """
        if self.state in (RobotState.CHARGING, RobotState.DONE):
            return False

        # Check sensors before moving
        if self._bump.check(self.env, self.row, self.col, dr, dc):
            return False
        if self._cliff.check(self.env, self.row, self.col, dr, dc):
            return False

        # Move
        self.row += dr
        self.col += dc
        self.heading = math.degrees(math.atan2(-dr, dc)) % 360
        self.total_distance += math.hypot(dr, dc)
        self.steps += 1

        # Battery drain
        self.battery = max(0.0, self.battery - self.battery_drain_per_step)

        # Clean current cell
        if self._dirt.check(self.env, self.row, self.col):
            self.env.mark_cleaned(self.row, self.col)
            self.cells_cleaned += 1
            self.bin_level = min(100.0, self.bin_level + self.bin_fill_per_step)

        # Update state
        self._update_state()
        return True

    def move_to(self, target_row: int, target_col: int, path: list) -> bool:
        """
        Follow a pre-computed path (list of (dr, dc) steps) toward target.
        Returns True if any step was taken.
        """
        if not path:
            return False
        dr, dc = path.pop(0)
        return self.move(dr, dc)

    def charge(self) -> bool:
        """Charge for one time step.  Returns True when fully charged."""
        if (self.row, self.col) != self.env.charging_pos:
            return False
        self.state = RobotState.CHARGING
        self.battery = min(self.battery_capacity, self.battery + self.battery_charge_per_step)
        if self.battery >= self.battery_capacity:
            self.bin_level = 0.0  # emptied when returning to base
            self.state = RobotState.CLEANING
            return True
        return False

    # ------------------------------------------------------------------
    # Internal state management
    # ------------------------------------------------------------------

    def _update_state(self) -> None:
        if self.battery <= 0:
            self.state = RobotState.ERROR
            return
        if self.bin_level >= self.full_bin_threshold:
            self.state = RobotState.FULL_BIN
            return
        if self.battery <= self.low_battery_threshold:
            self.state = RobotState.LOW_BATTERY
            return
        if self.state not in (RobotState.RETURNING,):
            self.state = RobotState.CLEANING

    def needs_to_return(self) -> bool:
        """True when the robot should head back to the charging station."""
        return self.state in (
            RobotState.LOW_BATTERY,
            RobotState.FULL_BIN,
            RobotState.RETURNING,
        )

    def at_charger(self) -> bool:
        return (self.row, self.col) == self.env.charging_pos

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @property
    def position(self) -> Tuple[int, int]:
        return (self.row, self.col)

    def status(self) -> dict:
        return {
            "position": (self.row, self.col),
            "heading_deg": round(self.heading, 1),
            "state": self.state.name,
            "battery_pct": round(self.battery / self.battery_capacity * 100, 1),
            "bin_pct": round(self.bin_level, 1),
            "steps": self.steps,
            "cells_cleaned": self.cells_cleaned,
            "coverage_pct": round(self.env.coverage_ratio * 100, 2),
        }
