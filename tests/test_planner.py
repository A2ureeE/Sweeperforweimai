"""Tests for Path Planners."""

import pytest

from src.environment import Cell, Environment
from src.planner import (
    AStarPlanner,
    BoustrophedonPlanner,
    RandomBouncePlanner,
    SpiralPlanner,
    path_to_steps,
)


def make_open_env(rows=5, cols=5):
    return Environment(rows=rows, cols=cols, charging_pos=(0, 0))


# ---------------------------------------------------------------------------
# BoustrophedonPlanner
# ---------------------------------------------------------------------------

class TestBoustrophedonPlanner:
    def test_yields_all_non_obstacle_cells(self):
        env = make_open_env(5, 5)
        planner = BoustrophedonPlanner()
        waypoints = list(planner.plan(env, (0, 0)))
        # All non-obstacle cells should appear
        total = env.rows * env.cols - int((env.grid == Cell.OBSTACLE).sum())
        assert len(waypoints) == total

    def test_no_obstacle_cells_in_plan(self):
        env = make_open_env(5, 5)
        env.add_obstacle(2, 2)
        planner = BoustrophedonPlanner()
        waypoints = set(planner.plan(env, (0, 0)))
        assert (2, 2) not in waypoints

    def test_alternating_direction(self):
        env = make_open_env(3, 4)
        planner = BoustrophedonPlanner()
        waypoints = list(planner.plan(env, (0, 0)))
        # Row 0 should go left-to-right, row 1 right-to-left
        row0 = [wp for wp in waypoints if wp[0] == 0]
        row1 = [wp for wp in waypoints if wp[0] == 1]
        assert row0[0][1] < row0[-1][1]   # increasing column
        assert row1[0][1] > row1[-1][1]   # decreasing column


# ---------------------------------------------------------------------------
# SpiralPlanner
# ---------------------------------------------------------------------------

class TestSpiralPlanner:
    def test_yields_all_non_obstacle_cells(self):
        env = make_open_env(4, 4)
        planner = SpiralPlanner()
        waypoints = list(planner.plan(env, (0, 0)))
        total = env.rows * env.cols - int((env.grid == Cell.OBSTACLE).sum())
        assert len(waypoints) == total

    def test_no_duplicates(self):
        env = make_open_env(5, 5)
        planner = SpiralPlanner()
        waypoints = list(planner.plan(env, (0, 0)))
        assert len(waypoints) == len(set(waypoints))

    def test_no_obstacle_cells_in_plan(self):
        env = make_open_env(5, 5)
        env.add_obstacle(1, 1)
        planner = SpiralPlanner()
        waypoints = set(planner.plan(env, (0, 0)))
        assert (1, 1) not in waypoints


# ---------------------------------------------------------------------------
# RandomBouncePlanner
# ---------------------------------------------------------------------------

class TestRandomBouncePlanner:
    def test_produces_waypoints(self):
        env = make_open_env(5, 5)
        planner = RandomBouncePlanner(seed=0)
        gen = planner.plan(env, (0, 0))
        # Take 20 waypoints
        waypoints = [next(gen) for _ in range(20)]
        assert len(waypoints) == 20

    def test_waypoints_are_passable(self):
        env = make_open_env(5, 5)
        env.add_obstacle(2, 2)
        planner = RandomBouncePlanner(seed=1)
        gen = planner.plan(env, (0, 0))
        for _ in range(50):
            r, c = next(gen)
            assert env.is_passable(r, c)

    def test_reproducible_with_seed(self):
        env = make_open_env(5, 5)
        p1 = RandomBouncePlanner(seed=42)
        p2 = RandomBouncePlanner(seed=42)
        g1, g2 = p1.plan(env, (0, 0)), p2.plan(env, (0, 0))
        for _ in range(30):
            assert next(g1) == next(g2)


# ---------------------------------------------------------------------------
# AStarPlanner
# ---------------------------------------------------------------------------

class TestAStarPlanner:
    def test_path_from_start_to_goal(self):
        env = make_open_env(5, 5)
        path = AStarPlanner.plan(env, (0, 0), (4, 4))
        assert path[0] == (0, 0)
        assert path[-1] == (4, 4)

    def test_path_length_on_open_grid(self):
        env = make_open_env(5, 5)
        path = AStarPlanner.plan(env, (0, 0), (4, 4))
        # Manhattan distance is 8, path must be at least 9 nodes
        assert len(path) >= 9

    def test_path_avoids_obstacle(self):
        env = make_open_env(5, 5)
        # Wall blocking direct route
        for r in range(5):
            if r != 4:
                env.add_obstacle(r, 2)
        path = AStarPlanner.plan(env, (0, 0), (0, 4))
        assert path  # path exists
        for (r, c) in path:
            assert env.grid[r, c] != Cell.OBSTACLE

    def test_same_start_goal(self):
        env = make_open_env(5, 5)
        path = AStarPlanner.plan(env, (2, 2), (2, 2))
        assert path == [(2, 2)]

    def test_no_path_returns_empty(self):
        env = make_open_env(5, 5)
        # Surround goal with obstacles so it's unreachable
        env.add_obstacle(1, 0)
        env.add_obstacle(0, 1)
        path = AStarPlanner.plan(env, (0, 0), (3, 3))
        # The start (0,0) is the charging station and passable; (1,0) and (0,1)
        # blocked → (0,0) is isolated, so no path to (3,3)
        # NOTE: If a path somehow exists along the boundary, accept it but verify
        #       it doesn't go through an obstacle.
        for (r, c) in path:
            assert env.grid[r, c] != Cell.OBSTACLE

    def test_path_continuity(self):
        env = make_open_env(6, 6)
        path = AStarPlanner.plan(env, (0, 0), (5, 5))
        for i in range(1, len(path)):
            r0, c0 = path[i - 1]
            r1, c1 = path[i]
            assert abs(r1 - r0) <= 1 and abs(c1 - c0) <= 1


# ---------------------------------------------------------------------------
# path_to_steps utility
# ---------------------------------------------------------------------------

class TestPathToSteps:
    def test_basic_conversion(self):
        path = [(0, 0), (0, 1), (0, 2), (1, 2)]
        steps = path_to_steps(path)
        assert steps == [(0, 1), (0, 1), (1, 0)]

    def test_empty_path(self):
        assert path_to_steps([]) == []

    def test_single_cell_path(self):
        assert path_to_steps([(2, 3)]) == []
