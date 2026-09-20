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
from config import RobotState, TaskStatus
from metrics_collector import MetricsCollector
from task_generator import TransportTask


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

    def test_dock_clearance_robot_has_priority_only_until_one_metre_clear(self):
        sup = self._supervisor()
        clearing = RobotInfo(1, (0.2, 0.0))
        clearing.state = RobotState.RETURNING_HOME
        clearing.goal_location = (2.0, 0.0)
        clearing.waypoints = [(2.0, 0.0)]
        clearing.dock_clearance_origin = (0.0, 0.0)
        peer = self._robot(2, (0.0, 0.5), task_priority=100.0)
        sup.robots = {1: clearing, 2: peer}

        self.assertGreater(sup._priority_yield_key(1),
                           sup._priority_yield_key(2))
        clearing.position = (1.01, 0.0)
        self.assertLess(sup._priority_yield_key(1),
                        sup._priority_yield_key(2))

    def test_dock_clearance_requires_an_active_peer_with_same_goal(self):
        sup = self._supervisor()
        finished = RobotInfo(1, (0.0, 0.0))
        peer = self._robot(2, (1.0, 0.0))
        peer.current_task.pickup_location = "WS1"
        peer.current_task.delivery_location = "S1"
        sup.robots = {1: finished, 2: peer}

        self.assertTrue(sup._dock_clearance_required(1, "WS1"))
        self.assertFalse(sup._dock_clearance_required(1, "WS2"))
        peer.state = RobotState.IDLE
        self.assertFalse(sup._dock_clearance_required(1, "WS1"))

    def test_delivery_completion_immediately_dispatches_pressured_dock_clearance(self):
        sup = self._supervisor()
        delivered = TransportTask(
            task_id=7,
            pickup_location="S1",
            delivery_location="WS1",
            pickup_position=(-3.0, 1.5),
            delivery_position=(-6.0, 3.5),
            arrival_time=0.0,
            status=TaskStatus.IN_PROGRESS,
        )
        incoming = TransportTask(
            task_id=8,
            pickup_location="WS1",
            delivery_location="S2",
            pickup_position=(-6.0, 3.5),
            delivery_position=(-1.0, 1.5),
            arrival_time=1.0,
        )
        clearing = RobotInfo(1, delivered.delivery_position)
        clearing.state = RobotState.EN_ROUTE_DELIVERY
        clearing.current_task = delivered
        clearing.goal_location = delivered.delivery_location
        peer = RobotInfo(2, (-4.0, 3.0))
        peer.state = RobotState.EN_ROUTE_PICKUP
        peer.current_task = incoming
        peer.goal_location = incoming.pickup_location
        peer.waypoints = [incoming.pickup_position]
        sup.robots = {1: clearing, 2: peer}
        sup.metrics = mock.Mock()
        sup.motion_coordinator = mock.Mock()
        sup._get_robot_positions_from_webots = mock.Mock()

        def relocate(robot_id, source):
            sup.robots[robot_id].state = RobotState.RETURNING_HOME
            return True

        sup._relocate_idle_robot = mock.Mock(side_effect=relocate)
        sup._handle_goal_reached(1)

        self.assertEqual(TaskStatus.COMPLETED, delivered.status)
        self.assertIsNone(clearing.current_task)
        self.assertEqual(RobotState.RETURNING_HOME, clearing.state)
        self.assertEqual(delivered.delivery_position,
                         clearing.dock_clearance_origin)
        self.assertEqual("WS1", clearing.dock_clearance_location)
        sup._relocate_idle_robot.assert_called_once_with(
            1, source="_priority_dock_clearance")

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

    def test_side_wait_is_bounded_without_latching_state_when_feature_is_off(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-2.0, 0.0), goal=(2.0, 0.0)),
            2: self._robot(2, (0.0, -2.0), goal=(0.0, 2.0)),
        }
        sup._send_command_to_robot = mock.Mock(return_value=True)
        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", False):
            self.assertTrue(sup._priority_yield_side_wait(
                (1, 2), [(1, 2, 0.5, 0.2)]))
        for robot in sup.robots.values():
            self.assertIsNone(robot.priority_yield_state)
        yielder = sup.robots[1]
        self.assertEqual(0.4, yielder.speed_scale)
        self.assertEqual(11.5, yielder.hold_until)
        self.assertEqual(11.5, yielder.priority_yield_bounded_until)
        self.assertEqual(11.5, yielder.joint_shield_until)
        self.assertEqual(1.0, sup.robots[2].speed_scale)

    def test_bounded_side_wait_expiry_restores_speed_and_wait_metadata(self):
        sup = self._supervisor()
        robot = self._robot(1, (0.0, 0.0))
        robot.priority_yield_bounded_until = 9.0
        robot.hold_until = 9.0
        robot.wait_started = 8.0
        robot.joint_shield_until = 9.0
        robot.speed_scale = 0.4
        sup.robots = {1: robot}

        def set_speed(_robot_id, scale):
            robot.speed_scale = scale
            return True

        sup._set_robot_speed_scale = mock.Mock(side_effect=set_speed)
        sup._priority_yield_expire_bounded_waits(sup.robots)

        self.assertEqual(0.0, robot.priority_yield_bounded_until)
        self.assertEqual(0.0, robot.hold_until)
        self.assertEqual(0.0, robot.wait_started)
        self.assertEqual(1.0, robot.speed_scale)

    def test_side_wait_keeps_state_machine_behavior_when_feature_is_on(self):
        sup = self._supervisor()
        sup.robots = {
            1: self._robot(1, (-2.0, 0.0), goal=(2.0, 0.0)),
            2: self._robot(2, (0.0, -2.0), goal=(0.0, 2.0)),
        }
        sup._send_command_to_robot = mock.Mock(return_value=True)

        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", True):
            self.assertTrue(sup._priority_yield_side_wait(
                (1, 2), [(1, 2, 0.5, 0.2)]))

        yielder = sup.robots[1]
        self.assertEqual("waiting_clear", yielder.priority_yield_state)
        self.assertEqual(2, yielder.priority_yield_winner)
        self.assertEqual(11.5, yielder.priority_yield_wait_deadline)
        self.assertEqual(0.0, yielder.priority_yield_bounded_until)

    def test_bounded_side_wait_expiry_preserves_later_wait(self):
        sup = self._supervisor()
        robot = self._robot(1, (0.0, 0.0))
        robot.priority_yield_bounded_until = 9.0
        robot.hold_until = 12.0
        robot.wait_started = 8.0
        robot.joint_shield_until = 12.0
        robot.speed_scale = 0.4
        sup.robots = {1: robot}
        sup._set_robot_speed_scale = mock.Mock(return_value=True)

        sup._priority_yield_expire_bounded_waits(sup.robots)

        self.assertEqual(12.0, robot.priority_yield_bounded_until)
        self.assertEqual(12.0, robot.hold_until)
        self.assertEqual(8.0, robot.wait_started)
        self.assertEqual(0, sup._set_robot_speed_scale.call_count)

    def test_disabled_stale_yield_does_not_swallow_delivery_arrival(self):
        sup = self._supervisor()
        task = TransportTask(
            task_id=7,
            pickup_location="S1",
            delivery_location="WS1",
            pickup_position=(0.0, 0.0),
            delivery_position=(2.0, 3.0),
            arrival_time=0.0,
            status=TaskStatus.IN_PROGRESS,
        )
        robot = RobotInfo(1, task.delivery_position)
        robot.state = RobotState.EN_ROUTE_DELIVERY
        robot.current_task = task
        robot.goal_location = task.delivery_location
        robot.priority_yield_state = "waiting_clear"
        robot.priority_yield_winner = 2
        robot.priority_yield_original_goal = task.delivery_location
        robot.hold_until = 20.0
        robot.speed_scale = 0.4
        sup.robots = {1: robot}
        sup.metrics = mock.Mock()
        sup.motion_coordinator = mock.Mock()
        sup.motion_coordinator.graph.get_nearest_node.return_value = None
        sup._get_robot_positions_from_webots = mock.Mock()

        def restore_speed(_robot_id, scale):
            robot.speed_scale = scale
            return True

        sup._set_robot_speed_scale = mock.Mock(side_effect=restore_speed)

        with mock.patch.object(fs, "ENABLE_PRIORITY_YIELD_RESUME", False):
            sup._handle_goal_reached(1)

        self.assertEqual(TaskStatus.COMPLETED, task.status)
        self.assertEqual(RobotState.IDLE, robot.state)
        self.assertIsNone(robot.current_task)
        self.assertIsNone(robot.priority_yield_state)
        self.assertEqual(0.0, robot.hold_until)
        self.assertEqual(1.0, robot.speed_scale)
        sup._set_robot_speed_scale.assert_called_once_with(1, 1.0)
        sup.metrics.record_task_completion.assert_called_once_with(
            task, 1, sup.sim_time)


if __name__ == "__main__":
    unittest.main()
