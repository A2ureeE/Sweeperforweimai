"""Tests for Robot."""

import pytest

from src.environment import Cell, Environment
from src.robot import Robot, RobotState


def make_env(rows=10, cols=10, charging_pos=(0, 0)):
    return Environment(rows=rows, cols=cols, charging_pos=charging_pos)


class TestRobotInitialState:
    def test_starts_at_charging_pos(self):
        env = make_env()
        robot = Robot(env)
        assert robot.position == env.charging_pos

    def test_initial_state_is_cleaning(self):
        env = make_env()
        robot = Robot(env)
        assert robot.state == RobotState.CLEANING

    def test_battery_starts_full(self):
        env = make_env()
        robot = Robot(env)
        assert robot.battery == robot.battery_capacity

    def test_bin_starts_empty(self):
        env = make_env()
        robot = Robot(env)
        assert robot.bin_level == 0.0


class TestRobotMovement:
    def test_move_to_free_cell(self):
        env = make_env()
        robot = Robot(env)
        success = robot.move(0, 1)
        assert success
        assert robot.position == (0, 1)

    def test_blocked_by_obstacle(self):
        env = make_env()
        env.add_obstacle(0, 1)
        robot = Robot(env)
        success = robot.move(0, 1)
        assert not success
        assert robot.position == (0, 0)

    def test_blocked_by_wall(self):
        env = make_env(rows=5, cols=5)
        robot = Robot(env)
        success = robot.move(-1, 0)  # row -1 is out of bounds
        assert not success

    def test_move_cleans_cell(self):
        env = make_env()
        robot = Robot(env)
        assert env.grid[0, 1] == Cell.FREE
        robot.move(0, 1)
        assert env.grid[0, 1] == Cell.CLEANED

    def test_move_drains_battery(self):
        env = make_env()
        robot = Robot(env)
        initial = robot.battery
        robot.move(0, 1)
        assert robot.battery < initial

    def test_steps_counter_increments(self):
        env = make_env()
        robot = Robot(env)
        robot.move(0, 1)
        robot.move(0, 1)
        assert robot.steps == 2


class TestRobotStateTransitions:
    def test_low_battery_triggers_low_battery_state(self):
        env = make_env()
        robot = Robot(env, low_battery_threshold=20.0, battery_drain_per_step=1.0)
        # Drain battery below threshold
        for _ in range(85):
            robot.move(0, 1) or robot.move(0, -1)
        assert robot.battery <= 20.0
        assert robot.state in (RobotState.LOW_BATTERY, RobotState.RETURNING)

    def test_full_bin_triggers_full_bin_state(self):
        env = make_env(rows=20, cols=20)
        robot = Robot(env, bin_fill_per_step=10.0, full_bin_threshold=90.0)
        # Move to fill bin
        c = 1
        for _ in range(15):
            robot.move(0, 1) if c < 19 else robot.move(0, -1)
            c = robot.col
        assert robot.state == RobotState.FULL_BIN

    def test_needs_to_return_when_low_battery(self):
        env = make_env()
        robot = Robot(env, low_battery_threshold=99.9)
        # One move should already trigger low battery
        robot.move(0, 1)
        assert robot.needs_to_return()

    def test_at_charger(self):
        env = make_env()
        robot = Robot(env)
        assert robot.at_charger()
        robot.move(0, 1)
        assert not robot.at_charger()

    def test_charge_increases_battery(self):
        env = make_env()
        robot = Robot(env, battery_drain_per_step=10.0, low_battery_threshold=50.0)
        robot.move(0, 1)
        robot.move(0, -1)  # back to charger
        before = robot.battery
        robot.charge()
        assert robot.battery > before

    def test_charge_empties_bin(self):
        env = make_env(rows=20, cols=20)
        robot = Robot(env, bin_fill_per_step=5.0)
        for _ in range(5):
            robot.move(0, 1)
        assert robot.bin_level > 0
        # Force charging state and fully charge
        robot.row, robot.col = env.charging_pos
        robot.battery = 1.0
        while robot.battery < robot.battery_capacity:
            robot.charge()
        assert robot.bin_level == 0.0


class TestRobotStatus:
    def test_status_returns_dict(self):
        env = make_env()
        robot = Robot(env)
        s = robot.status()
        assert isinstance(s, dict)
        assert "position" in s
        assert "battery_pct" in s
        assert "coverage_pct" in s
