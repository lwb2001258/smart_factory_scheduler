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

    def test_scan_suppresses_legacy_when_feature_off_path_is_unchanged(self):
        sup = self._supervisor()
        active = {}
        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", False):
            with mock.patch.object(
                    sup, "_joint_predictive_speed_shield",
                    return_value=True) as legacy:
                self.assertTrue(sup._joint_collision_scan(active))
                legacy.assert_called_once()


if __name__ == "__main__":
    unittest.main()
