import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

import factory_supervisor as fs
from factory_supervisor import FactorySupervisor, RobotInfo
from config import RobotState
from metrics_collector import MetricsCollector


class PriorityYieldResumeTests(unittest.TestCase):
    def _supervisor(self):
        sup = FactorySupervisor.__new__(FactorySupervisor)
        sup.sim_time = 10.0
        sup.metrics = None
        return sup

    def _robot(self, rid, position, task_priority=1.0, goal=(5.0, 0.0)):
        robot = RobotInfo(rid, position)
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.battery = 100.0
        robot.current_task = SimpleNamespace(priority=task_priority)
        robot.goal_location = goal
        robot.waypoints = [goal]
        robot.current_waypoint_idx = 0
        return robot

    def test_priority_key_prefers_returning_to_charge(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (0.0, 0.0), task_priority=5.0),
            2: self._robot(2, (0.1, 0.0), task_priority=1.0),
        }
        sup.robots[2].state = RobotState.RETURNING_TO_CHARGE
        self.assertGreater(sup._priority_yield_key(2), sup._priority_yield_key(1))
        self.assertFalse(sup._priority_yield_is_exempt(sup.robots[1]))
        self.assertTrue(sup._priority_yield_is_exempt(sup.robots[2]))

    def test_terminal_clearing_owner_precedes_incoming_charger(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-8.5, 0.0), task_priority=0.0),
            2: self._robot(2, (-7.5, 0.0), task_priority=100.0),
        }
        sup.robots[1].terminal_clearance_location = 'CS1'
        sup.robots[1].terminal_egress_location = 'CS1'
        sup.robots[1].terminal_egress_priority_active = True
        sup.robots[2].state = RobotState.RETURNING_TO_CHARGE
        sup.robots[2].goal_location = 'CS1'
        self.assertEqual((1, 2), sup._priority_yield_pair(1, 2))
        self.assertEqual((1, 2), sup._priority_yield_ordered(2, 1))

    def test_side_crossing_forward_robot_waits_for_lateral_robot(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-2.0, 0.0), task_priority=5.0,
                           goal=(2.0, 0.0)),
            2: self._robot(2, (0.0, -2.0), task_priority=1.0,
                           goal=(0.0, 2.0)),
        }
        self.assertEqual('side', sup._classify_conflict_pair(1, 2))
        winner, yielder = sup._side_crossing_pair((1, 2))
        self.assertEqual((2, 1), (winner, yielder))

    def test_same_direction_robot_behind_is_follower(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-2.0, 0.0), task_priority=5.0,
                           goal=(4.0, 0.0)),
            2: self._robot(2, (-1.0, 0.0), task_priority=1.0,
                           goal=(4.0, 0.0)),
        }
        self.assertEqual('same', sup._classify_conflict_pair(1, 2))
        self.assertEqual((1, 2), sup._same_direction_follower((1, 2)))

    def test_priority_key_uses_task_then_id(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (0.0, 0.0), task_priority=1.0, goal=(5.0, 0.0)),
            2: self._robot(2, (0.1, 0.0), task_priority=3.0, goal=(5.0, 0.0)),
        }
        self.assertGreater(sup._priority_yield_key(2), sup._priority_yield_key(1))
        sup.robots[2].current_task.priority = 1.0
        # Goal distance must not override task priority or the final ID
        # tie-break; equal task priorities fall through to robot ID.
        sup.robots[1].goal_location = (1.0, 0.0)
        self.assertGreater(sup._priority_yield_key(2), sup._priority_yield_key(1))

    def test_priority_key_uses_higher_robot_id_on_final_tie(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (0.0, 0.0), task_priority=1.0, goal=(5.0, 0.0)),
            2: self._robot(2, (0.1, 0.0), task_priority=1.0, goal=(5.0, 0.0)),
        }
        self.assertGreater(sup._priority_yield_key(2), sup._priority_yield_key(1))

    def test_priority_aging_is_bounded_and_prevents_equal_priority_starvation(self):
        sup = self._supervisor()
        sup.sim_time = 100.0
        sup.robots = {
            1: self._robot(1, (0.0, 0.0), task_priority=1.0),
            2: self._robot(2, (0.1, 0.0), task_priority=1.0),
        }
        sup.robots[1].wait_started = 40.0
        self.assertGreater(sup._priority_yield_key(1),
                           sup._priority_yield_key(2))
        old_key = sup._priority_yield_key(1)
        sup.sim_time = 10000.0
        self.assertEqual(old_key[1], sup._priority_yield_key(1)[1])

    def test_select_yielder_respects_task_priority(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (0.0, 0.0), task_priority=1.0),
            2: self._robot(2, (0.1, 0.0), task_priority=4.0),
        }
        selection = sup._priority_yield_select((1, 2), [(1, 2, 1.0, 0.5)])
        self.assertEqual((2, 1), selection)

    def test_select_yielder_skips_fully_exempt_component(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (0.0, 0.0)),
            2: self._robot(2, (0.1, 0.0)),
        }
        for robot in sup.robots.values():
            robot.state = RobotState.RETURNING_TO_CHARGE
        self.assertIsNone(sup._priority_yield_select((1, 2), [(1, 2, 1.0, 0.5)]))

    def test_head_on_lateral_standoff_moves_only_yielder(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-1.0, 0.0), task_priority=5.0,
                           goal=(5.0, 0.0)),
            2: self._robot(2, (1.0, 0.0), task_priority=1.0,
                           goal=(-5.0, 0.0)),
        }
        winner_path = list(sup.robots[1].waypoints)
        sup._priority_yield_standoff_candidates = lambda *a, **k: [
            (2.0, 1.2, (1.0, 1.0))]
        sup._static_turning_path = lambda start, target: [target]
        sup._recovery_path_peer_clear = lambda rid, path: True
        sup._priority_yield_path_departure_clear = lambda rid, path: True
        installed = []
        sup._install_runtime_plan = (
            lambda rid, path, delay=0.0, source=None:
            installed.append((rid, path, source)) or True)
        speeds = []
        sup._set_robot_speed_scale = (
            lambda rid, scale: speeds.append((rid, scale)) or True)
        self.assertTrue(sup._priority_yield_lateral_standoff(1, 2, (1, 2)))
        self.assertEqual(winner_path, sup.robots[1].waypoints)
        self.assertEqual([(2, [(1.0, 1.0)], '_priority_yield_resume')],
                         installed)
        self.assertEqual('standoff_enroute',
                         sup.robots[2].priority_yield_state)
        self.assertEqual((1.0, 1.0), sup.robots[2].priority_yield_standoff)
        self.assertFalse(any(rid == 1 for rid, _scale in speeds))

    def test_head_on_dispatch_falls_back_when_both_lateral_sides_fail(self):
        sup = self._supervisor()
        calls = []
        sup._priority_yield_lateral_standoff = (
            lambda *args: calls.append('lateral') or False)
        sup._priority_yield_direct_plan = (
            lambda *args: calls.append('direct') or True)
        self.assertTrue(sup._priority_yield_dispatch_leg(1, 2, (1, 2)))
        self.assertEqual(['lateral', 'direct'], calls)

    def test_lateral_only_candidate_search_never_enters_full_ring(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-1.0, 0.0)),
            2: self._robot(2, (1.0, 0.0)),
        }

        class Coordinator:
            grid = None
            calls = 0

            def _segment_clear(self, start, target):
                self.calls += 1
                return False

        sup.motion_coordinator = Coordinator()
        self.assertEqual([], sup._priority_yield_standoff_candidates(
            1, 2, lateral_only=True))
        self.assertEqual(16, sup.motion_coordinator.calls)

    def test_lateral_standoff_is_not_declared_arrived_by_timer(self):
        sup = self._supervisor()
        robot = self._robot(1, (0.0, 0.0))
        robot.priority_yield_state = 'standoff_enroute'
        robot.priority_yield_started_at = sup.sim_time - 2.0
        robot.priority_yield_standoff = (0.0, 1.0)
        robot.priority_yield_winner = 2
        sup.robots = {1: robot}
        sup._joint_predictive_speed_shield = lambda active: False
        self.assertFalse(sup._priority_yield_resume_scan({1: robot}))
        self.assertEqual('standoff_enroute', robot.priority_yield_state)

    def test_yield_metrics_are_recorded_and_reported(self):
        metrics = MetricsCollector("test", "FCFS", 1)
        events = [
            ("yield_start", 1),
            ("yield_standoff_selected", 1),
            ("yield_wait_start", 1),
            ("yield_wait_cleared", 1),
            ("yield_wait_timeout", 1),
            ("yield_resume_proposed", 1),
            ("yield_resume_rejected", 1),
            ("yield_resume_committed", 1),
            ("yield_resume_rollback", 1),
        ]
        for event_type, rid in events:
            metrics.record_yield_event(event_type, 0.1 * rid, rid, winner=2)
        metrics.record_nonphysical_recovery(1.0, 3, (0.0, 0.0))
        final = metrics.compute_final_metrics({}, {"completed": 0}, {}, 10.0)
        for event_type, _ in events:
            self.assertEqual(1, final[event_type])
        self.assertEqual(1, final["nonphysical_recoveries"])
        self.assertEqual(0, final["safety_event_count"])

    def test_joint_commit_hands_goal_back_to_joint_planner(self):
        sup = self._supervisor()
        sup.robots = {1: self._robot(1, (1.0, 1.0), goal=(4.0, 4.0))}
        robot = sup.robots[1]
        robot.priority_yield_state = "waiting_clear"
        robot.priority_yield_winner = 2
        robot.priority_yield_original_goal = (4.0, 4.0)
        robot.recovery_active = True
        robot.recovery_resume_goal = (4.0, 4.0)
        robot.hold_until = 12.0
        robot.wait_started = 10.0
        robot.route_write_owner = "_priority_yield_resume"
        sup._joint_liveness_needed = False
        sup._next_joint_grid_tick = 20.0
        sup.joint_runtime_enforced = True
        with mock.patch.object(fs, "ENABLE_JOINT_RUNTIME", True):
            self.assertTrue(sup._priority_yield_commit_resume(1))
        self.assertIsNone(robot.priority_yield_state)
        self.assertFalse(robot.recovery_active)
        self.assertEqual((4.0, 4.0), robot.goal_location)
        self.assertEqual("joint_grid_transaction", robot.active_plan_source)
        self.assertEqual([], robot.waypoints)
        self.assertTrue(sup._joint_liveness_needed)
        self.assertEqual(10.0, sup._next_joint_grid_tick)

    def test_only_matching_standoff_leg_owns_arrival(self):
        robot = self._robot(1, (1.0, 1.0), goal=(4.0, 4.0))
        robot.priority_yield_state = "standoff_enroute"
        robot.priority_yield_standoff = (1.1, 1.0)
        self.assertTrue(FactorySupervisor._priority_yield_owns_arrival(robot))
        robot.position = (4.0, 4.0)
        self.assertFalse(FactorySupervisor._priority_yield_owns_arrival(robot))

    def test_waiting_clear_never_owns_later_business_arrival(self):
        robot = self._robot(1, (4.0, 4.0), goal=(4.0, 4.0))
        robot.priority_yield_state = "waiting_clear"
        robot.priority_yield_standoff = (1.0, 1.0)
        self.assertFalse(FactorySupervisor._priority_yield_owns_arrival(robot))

    def test_stale_waiting_clear_is_retired_by_goal_arrival(self):
        sup = self._supervisor()
        robot = RobotInfo(1, (4.0, 4.0))
        robot.priority_yield_state = "waiting_clear"
        robot.priority_yield_winner = 2
        robot.priority_yield_standoff = (1.0, 1.0)
        robot.priority_yield_original_goal = (4.0, 4.0)
        robot.goal_location = (4.0, 4.0)
        robot.route_write_owner = "joint_grid_transaction"
        sup.robots = {1: robot}
        sup._set_robot_speed_scale = lambda rid, scale: None

        sup._handle_goal_reached(1, force=True)

        self.assertIsNone(robot.priority_yield_state)
        self.assertIsNone(robot.priority_yield_winner)
        # Reaching the ordinary handler also retires the completed business
        # leg's writer; retaining it would prove the arrival was still eaten.
        self.assertIsNone(robot.route_write_owner)

    def test_scan_suppresses_legacy_when_feature_off_path_is_unchanged(self):
        sup = self._supervisor()
        active = {}
        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", False):
            with mock.patch.object(
                    sup, "_joint_predictive_speed_shield",
                    return_value=True) as legacy:
                self.assertTrue(sup._joint_collision_scan(active))
                legacy.assert_called_once()

    def test_priority_speed_or_wait_action_does_not_force_joint_replan(self):
        sup = self._supervisor()
        sup._joint_liveness_needed = False
        sup._next_joint_grid_tick = 20.0
        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", True):
            with mock.patch.object(
                    sup, "_priority_yield_resume_scan",
                    return_value=True) as priority_scan:
                with mock.patch.object(
                        sup, "_joint_predictive_speed_shield") as shield:
                    self.assertFalse(sup._joint_collision_scan({}))
        priority_scan.assert_called_once()
        shield.assert_not_called()
        self.assertFalse(sup._joint_liveness_needed)
        self.assertEqual(20.0, sup._next_joint_grid_tick)

    def test_ordinary_speed_shaping_does_not_force_joint_replan(self):
        sup = self._supervisor()
        sup._joint_liveness_needed = False
        sup._next_joint_grid_tick = 20.0
        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", False):
            with mock.patch.object(
                    sup, "_joint_predictive_speed_shield",
                    return_value=False) as shield:
                self.assertFalse(sup._joint_collision_scan({}))
        shield.assert_called_once()
        self.assertFalse(sup._joint_liveness_needed)
        self.assertEqual(20.0, sup._next_joint_grid_tick)


if __name__ == "__main__":
    unittest.main()
