"""Unit tests for PlannerNode state machine transitions and recovery flow."""
import math
import unittest
from unittest.mock import MagicMock, patch

from sweeper_planning.planner_node import PlannerState


class FakePlannerNode:
    """Minimal stub that mirrors PlannerNode state-machine fields
    without spinning a ROS node."""

    def __init__(self):
        self._planner_state = PlannerState.NORMAL
        self._controller_recovering = False
        self._detour_clear_pending = False
        self._detour_start_idx = None
        self._bridge_hold_ticks = 0
        self._last_bridge_result = None
        self._recovery_replan_bridge = []
        self._cov_version = 0
        self._cov_version_at_state_entry = 0
        self._detour_exit_t = 0.0
        self._latched_detour_path = None
        self._latched_detour_meta = None
        self.mode = 'COVERAGE'
        self.progress_idx = 0
        self.robot = (0.0, 0.0, 0.0)
        self.obs_memory = {}
        self.dyn_obstacles = []
        self._transition_log = []

    # Re-use real methods by importing them; bind manually.
    def _nearest_obstacle_distance(self, rx, ry):
        best = 999.0
        for ox, oy in self.obs_memory.keys():
            d = math.hypot(rx - ox, ry - oy)
            if d < best:
                best = d
        for wx, wy, *_ in self.dyn_obstacles:
            d = math.hypot(rx - wx, ry - wy)
            if d < best:
                best = d
        return best

    def get_logger(self):
        logger = MagicMock()
        return logger

    def _clear_detour_latch(self, reason=''):
        from sweeper_planning.planner_node import PlannerNode
        PlannerNode._clear_detour_latch(self, reason)

    def _transition_state(self, new_state, reason=''):
        from sweeper_planning.planner_node import PlannerNode
        PlannerNode._transition_state(self, new_state, reason)
        self._transition_log.append((self._planner_state, reason))

    def _update_planner_state(self, rx, ry, ryaw, now):
        from sweeper_planning.planner_node import PlannerNode
        PlannerNode._update_planner_state(self, rx, ry, ryaw, now)


class TestPlannerStateMachine(unittest.TestCase):

    def _make(self):
        return FakePlannerNode()

    # ── Basic transitions ──────────────────────────────────────────────

    def test_normal_to_static_detour(self):
        p = self._make()
        p.mode = 'STATIC_DETOUR'
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertEqual(p._planner_state, PlannerState.STATIC_DETOUR)

    def test_normal_to_dynamic_avoid(self):
        p = self._make()
        p.mode = 'DYNAMIC_AVOID'
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertEqual(p._planner_state, PlannerState.DYNAMIC_AVOID)

    def test_detour_to_rejoin_pending(self):
        p = self._make()
        p._planner_state = PlannerState.STATIC_DETOUR
        p.mode = 'COVERAGE'
        p._update_planner_state(0, 0, 0, 10.0)
        self.assertEqual(p._planner_state, PlannerState.REJOIN_PENDING)

    def test_rejoin_re_enters_detour(self):
        p = self._make()
        p._planner_state = PlannerState.REJOIN_PENDING
        p.mode = 'STATIC_DETOUR'
        p._update_planner_state(0, 0, 0, 10.0)
        self.assertEqual(p._planner_state, PlannerState.STATIC_DETOUR)

    # ── Recovery lifecycle ─────────────────────────────────────────────

    def test_recovery_start_from_normal(self):
        p = self._make()
        p._controller_recovering = True
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertEqual(p._planner_state, PlannerState.RECOVERY_ACTIVE)

    def test_recovery_start_from_detour(self):
        p = self._make()
        p._planner_state = PlannerState.STATIC_DETOUR
        p._detour_clear_pending = True
        p._bridge_hold_ticks = 3
        p._controller_recovering = True
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertEqual(p._planner_state, PlannerState.RECOVERY_ACTIVE)
        self.assertFalse(p._detour_clear_pending)
        self.assertEqual(p._bridge_hold_ticks, 0)

    def test_recovery_done_transitions_to_replan(self):
        p = self._make()
        p._planner_state = PlannerState.RECOVERY_ACTIVE
        p._controller_recovering = False
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertEqual(p._planner_state, PlannerState.POST_RECOVERY_REPLAN)

    def test_recovery_blocks_rejoin(self):
        """REJOIN_PENDING should NOT be reachable while controller recovers."""
        p = self._make()
        p._planner_state = PlannerState.STATIC_DETOUR
        p._controller_recovering = True
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertNotEqual(p._planner_state, PlannerState.REJOIN_PENDING)
        self.assertEqual(p._planner_state, PlannerState.RECOVERY_ACTIVE)

    def test_recovery_stays_active_while_recovering(self):
        p = self._make()
        p._planner_state = PlannerState.RECOVERY_ACTIVE
        p._controller_recovering = True
        p._update_planner_state(0, 0, 0, 2.0)
        self.assertEqual(p._planner_state, PlannerState.RECOVERY_ACTIVE)

    # ── Post-recovery replan stays until explicitly moved ──────────────

    def test_post_replan_stays(self):
        p = self._make()
        p._planner_state = PlannerState.POST_RECOVERY_REPLAN
        p._controller_recovering = False
        p._update_planner_state(0, 0, 0, 5.0)
        self.assertEqual(p._planner_state, PlannerState.POST_RECOVERY_REPLAN)

    # ── Transition logging ─────────────────────────────────────────────

    def test_transition_log_records(self):
        p = self._make()
        p.mode = 'STATIC_DETOUR'
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertTrue(len(p._transition_log) >= 1)
        self.assertEqual(p._transition_log[-1][0], PlannerState.STATIC_DETOUR)

    # ── Detour latch lifecycle ──────────────────────────────────────────

    def test_latch_survives_rejoin_pending(self):
        """Latch should survive REJOIN_PENDING so vehicle follows the smooth
        curve back to ConvergePath instead of snapping to raw coverage slice."""
        p = self._make()
        p._planner_state = PlannerState.STATIC_DETOUR
        p._latched_detour_path = [(0, 0), (1, 1)]
        p._latched_detour_meta = {'obs': (1, 0)}
        p.mode = 'COVERAGE'
        p._update_planner_state(0, 0, 0, 10.0)
        self.assertEqual(p._planner_state, PlannerState.REJOIN_PENDING)
        self.assertIsNotNone(p._latched_detour_path)
        self.assertIsNotNone(p._latched_detour_meta)

    def test_latch_cleared_on_recovery(self):
        """Latch should be cleared when entering RECOVERY_ACTIVE."""
        p = self._make()
        p._planner_state = PlannerState.STATIC_DETOUR
        p._latched_detour_path = [(0, 0), (1, 1)]
        p._latched_detour_meta = {'obs': (1, 0)}
        p._controller_recovering = True
        p._update_planner_state(0, 0, 0, 1.0)
        self.assertEqual(p._planner_state, PlannerState.RECOVERY_ACTIVE)
        self.assertIsNone(p._latched_detour_path)

    def test_latch_cleared_on_normal_after_rejoin(self):
        """Latch should be cleared only when reaching NORMAL (not REJOIN_PENDING).
        Simulate: REJOIN_PENDING -> mode stays COVERAGE -> eventually NORMAL."""
        p = self._make()
        p._planner_state = PlannerState.DYNAMIC_AVOID
        p._latched_detour_path = [(2, 2), (3, 3)]
        p._latched_detour_meta = {'obs': (1, 0)}
        p.mode = 'COVERAGE'
        p._update_planner_state(0, 0, 0, 20.0)
        self.assertEqual(p._planner_state, PlannerState.REJOIN_PENDING)
        self.assertIsNotNone(p._latched_detour_path)
        # Now transition to NORMAL
        p._transition_state(PlannerState.NORMAL, 'rejoin_complete')
        self.assertIsNone(p._latched_detour_path)

    def test_latch_survives_within_same_detour_episode(self):
        """Latch should NOT be cleared while staying in STATIC_DETOUR."""
        p = self._make()
        p._planner_state = PlannerState.STATIC_DETOUR
        p._latched_detour_path = [(0, 0), (1, 1)]
        p._latched_detour_meta = {'obs': (1, 0)}
        p.mode = 'STATIC_DETOUR'
        p._update_planner_state(0, 0, 0, 2.0)
        self.assertEqual(p._planner_state, PlannerState.STATIC_DETOUR)
        self.assertIsNotNone(p._latched_detour_path)


if __name__ == '__main__':
    unittest.main()
