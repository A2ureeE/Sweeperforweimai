"""Tests for Environment."""

import pytest
import numpy as np

from src.environment import Cell, Environment


class TestEnvironmentBasics:
    def test_default_grid_dimensions(self):
        env = Environment(rows=10, cols=15)
        assert env.grid.shape == (10, 15)

    def test_default_charging_station_at_origin(self):
        env = Environment(rows=10, cols=10)
        assert env.grid[0, 0] == Cell.CHARGING

    def test_custom_charging_position(self):
        env = Environment(rows=10, cols=10, charging_pos=(5, 5))
        assert env.grid[5, 5] == Cell.CHARGING

    def test_invalid_charging_position_raises(self):
        with pytest.raises(ValueError):
            Environment(rows=10, cols=10, charging_pos=(20, 20))

    def test_invalid_dimensions_raise(self):
        with pytest.raises(ValueError):
            Environment(rows=0, cols=10)
        with pytest.raises(ValueError):
            Environment(rows=10, cols=-1)

    def test_in_bounds(self):
        env = Environment(rows=5, cols=5)
        assert env.in_bounds(0, 0)
        assert env.in_bounds(4, 4)
        assert not env.in_bounds(-1, 0)
        assert not env.in_bounds(5, 5)

    def test_is_passable(self):
        env = Environment(rows=5, cols=5)
        env.add_obstacle(2, 2)
        assert not env.is_passable(2, 2)
        assert env.is_passable(0, 1)
        assert not env.is_passable(10, 10)  # out of bounds

    def test_add_obstacle(self):
        env = Environment(rows=5, cols=5)
        env.add_obstacle(1, 1)
        assert env.grid[1, 1] == Cell.OBSTACLE

    def test_cannot_add_obstacle_on_charger(self):
        env = Environment(rows=5, cols=5, charging_pos=(2, 2))
        with pytest.raises(ValueError):
            env.add_obstacle(2, 2)

    def test_obstacle_out_of_bounds_raises(self):
        env = Environment(rows=5, cols=5)
        with pytest.raises(ValueError):
            env.add_obstacle(10, 10)


class TestEnvironmentCoverage:
    def test_mark_cleaned(self):
        env = Environment(rows=5, cols=5)
        assert env.grid[1, 1] == Cell.FREE
        env.mark_cleaned(1, 1)
        assert env.grid[1, 1] == Cell.CLEANED

    def test_mark_cleaned_obstacle_stays_obstacle(self):
        env = Environment(rows=5, cols=5)
        env.add_obstacle(3, 3)
        env.mark_cleaned(3, 3)
        assert env.grid[3, 3] == Cell.OBSTACLE

    def test_coverage_ratio_starts_near_zero(self):
        env = Environment(rows=5, cols=5, charging_pos=(0, 0))
        # Only the charging station counts as cleaned
        assert env.coverage_ratio < 0.1

    def test_full_coverage(self):
        env = Environment(rows=3, cols=3, charging_pos=(0, 0))
        for r in range(3):
            for c in range(3):
                if env.grid[r, c] == Cell.FREE:
                    env.mark_cleaned(r, c)
        assert env.coverage_ratio == 1.0

    def test_uncleaned_cells_decreases_after_cleaning(self):
        env = Environment(rows=5, cols=5)
        before = len(env.uncleaned_cells())
        env.mark_cleaned(2, 2)
        after = len(env.uncleaned_cells())
        assert after == before - 1

    def test_rectangular_obstacle(self):
        env = Environment(rows=10, cols=10)
        env.add_rectangular_obstacle((2, 2), (4, 4))
        for r in range(2, 5):
            for c in range(2, 5):
                assert env.grid[r, c] == Cell.OBSTACLE


class TestEnvironmentFactories:
    def test_create_random_produces_obstacles(self):
        env = Environment.create_random(rows=20, cols=20, obstacle_density=0.2, seed=0)
        obstacle_count = int(np.sum(env.grid == Cell.OBSTACLE))
        assert obstacle_count > 0

    def test_create_random_charging_station_intact(self):
        env = Environment.create_random(rows=20, cols=20, seed=7)
        assert env.grid[0, 0] == Cell.CHARGING

    def test_create_room_with_furniture(self):
        env = Environment.create_room_with_furniture()
        assert env.rows == 20
        assert env.cols == 20
        # Charging station must be intact
        assert env.grid[0, 0] == Cell.CHARGING
        # At least one obstacle must exist
        assert int(np.sum(env.grid == Cell.OBSTACLE)) > 0
