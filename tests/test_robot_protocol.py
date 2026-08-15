import json
import sys
import unittest
from pathlib import Path


ROBOT_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
             "robot_controller")
sys.path.insert(0, str(ROBOT_DIR))

from robot_controller import RobotController, WaypointNavigator


class FakeReceiver:
    def __init__(self, messages):
        self.messages = list(messages)

    def getQueueLength(self):
        return len(self.messages)

    def getString(self):
        return self.messages[0]

    def nextPacket(self):
        self.messages.pop(0)


def command(version, waypoints):
    return json.dumps({
        "target_robot": 1,
        "command": {
            "type": "navigate",
            "path_version": version,
            "all_waypoints": waypoints,
        },
    })


class RobotProtocolTests(unittest.TestCase):
    def test_timed_wait_cell_cannot_advance_before_slot_deadline(self):
        navigator = WaypointNavigator()
        navigator.controller_time = 4.0
        navigator.set_waypoints(
            [(0.0, 0.0), (1.0, 0.0)],
            waypoint_not_before=[5.0, 8.0], partial=True)
        self.assertEqual((0.0, 0.0),
                         navigator.compute_control(0.0, 0.0, 0.0, []))
        self.assertEqual(0, navigator.current_waypoint_idx)
        navigator.controller_time = 5.0
        navigator.compute_control(0.0, 0.0, 0.0, [])
        self.assertEqual(1, navigator.current_waypoint_idx)

    def test_partial_window_endpoint_is_not_reported_as_business_goal(self):
        navigator = WaypointNavigator()
        navigator.controller_time = 10.0
        navigator.set_waypoints(
            [(0.0, 0.0)], waypoint_not_before=[9.0], partial=True)
        self.assertEqual((0.0, 0.0),
                         navigator.compute_control(0.0, 0.0, 0.0, []))
        self.assertTrue(navigator.navigation_active)
        self.assertFalse(navigator.goal_reached)

    def test_joint_barrier_stops_immediately_after_waypoint_advance(self):
        navigator = WaypointNavigator()
        navigator.controller_time = 5.0
        navigator.set_waypoints(
            [(0.0, 0.0), (1.0, 0.0)],
            waypoint_not_before=[5.0, 8.0], partial=True)
        navigator.joint_release_index = 0
        navigator.joint_epoch_wait_deadline = 15.0
        decision = navigator.compute_control(0.0, 0.0, 0.0, [])
        self.assertEqual((0.0, 0.0), decision)
        self.assertEqual(1, navigator.current_waypoint_idx)
        self.assertEqual('joint_epoch_barrier',
                         navigator.planned_wait_reason)
        self.assertGreater(navigator.planned_wait_until,
                           navigator.controller_time)

    def test_expired_joint_barrier_is_not_advertised_as_planned_wait(self):
        navigator = WaypointNavigator()
        navigator.controller_time = 16.0
        navigator.set_waypoints(
            [(0.0, 0.0), (1.0, 0.0)],
            waypoint_not_before=[5.0, 8.0], partial=True)
        navigator.current_waypoint_idx = 1
        navigator.joint_release_index = 0
        navigator.joint_epoch_wait_deadline = 15.0

        self.assertEqual((0.0, 0.0),
                         navigator.compute_control(0.0, 0.0, 0.0, []))
        self.assertEqual(0.0, navigator.planned_wait_until)
        self.assertIsNone(navigator.planned_wait_reason)

    def test_joint_cell_uses_threshold_smaller_than_grid_edge(self):
        navigator = WaypointNavigator()
        navigator.controller_time = 2.0
        navigator.set_waypoints(
            [(0.25, 0.0), (0.50, 0.0)],
            waypoint_not_before=[1.5, 3.0], partial=True)
        navigator.joint_release_index = 0
        navigator.waypoint_threshold = 0.12
        decision = navigator.compute_control(0.0, 0.0, 0.0, [])
        self.assertEqual(0, navigator.current_waypoint_idx)
        self.assertNotEqual((0.0, 0.0), decision)

    def test_status_transmits_joint_planned_wait_evidence(self):
        controller = self.controller_with_messages([])
        controller.position = (1.0, 2.0)
        controller.heading = 0.5
        controller.battery = 80.0
        controller.navigator.planned_wait_until = 12.5
        controller.navigator.planned_wait_reason = 'joint_epoch_barrier'
        controller._active_plan_epoch = 9

        controller._send_status()

        status = json.loads(controller.emitter.payloads[-1].decode('utf-8'))
        self.assertEqual(12.5, status['planned_wait_until'])
        self.assertEqual('joint_epoch_barrier',
                         status['planned_wait_reason'])
        self.assertEqual(9, status['active_plan_epoch'])

    def test_radial_safety_stops_side_peer_before_hard_boundary(self):
        navigator = WaypointNavigator()
        navigator.robot_id = 1
        navigator.set_waypoints([(2.0, 0.0)])
        navigator.peer_positions = {2: (0.0, 0.60)}
        decision = navigator._check_path_conflict_and_emergency(
            0.0, 0.0, 0.0, [])
        self.assertEqual((0.0, 0.0), decision)
        self.assertTrue(navigator._emergency_stopped)
        self.assertTrue(navigator._replan_requested)

    def test_radial_safety_allows_physical_retreat_away_from_peer(self):
        navigator = WaypointNavigator()
        navigator.robot_id = 1
        navigator.set_waypoints([(-1.0, 0.0)])
        navigator.peer_positions = {2: (0.60, 0.0)}
        decision = navigator._check_path_conflict_and_emergency(
            0.0, 0.0, 0.0, [])
        self.assertIsNone(decision)
        self.assertFalse(navigator._emergency_stopped)

    def controller_with_messages(self, messages):
        controller = RobotController.__new__(RobotController)
        controller.robot_id = 1
        controller.navigator = WaypointNavigator()
        controller.navigator.robot_id = 1
        controller.receiver = FakeReceiver(messages)
        controller._set_motor_speeds = lambda left, right: None
        controller._prepared_joint_plans = {}
        controller._active_plan_epoch = 0
        controller._highest_prepared_epoch = 0
        controller._scheduled_joint_plan = None
        controller.emitter = type("Emitter", (), {
            "payloads": None,
            "send": lambda self, p: self.payloads.append(p),
        })()
        controller.emitter.payloads = []
        controller.robot = type("Robot", (), {"getTime": lambda self: 1.0})()
        return controller

    def test_prepare_ack_and_scheduled_activation_use_real_protocol(self):
        prepare = json.dumps({"target_robot": 1, "command": {
            "type": "prepare_plan", "plan_epoch": 11,
            "path_version": 5, "all_waypoints": [[1.0, 0.0]],
            "waypoint_not_before_offsets": [3.0], "partial_plan": True,
        }})
        arm = json.dumps({"target_robot": 1, "command": {
            "type": "arm_plan", "plan_epoch": 11,
        }})
        commit = json.dumps({"target_robot": 1, "command": {
            "type": "commit_plan", "plan_epoch": 11, "activate_at": 2.0,
        }})
        controller = self.controller_with_messages([prepare, arm, commit])
        clock = type("Robot", (), {"now": 1.0,
                    "getTime": lambda self: self.now})()
        controller.robot = clock
        controller._receive_commands()
        ack_types = [json.loads(item.decode("utf-8"))["type"]
                     for item in controller.emitter.payloads]
        self.assertEqual(
            ["PLAN_PREPARED", "PLAN_ARMED", "PLAN_COMMITTED"], ack_types)
        clock.now = 2.0
        controller._activate_scheduled_joint_plan()
        self.assertEqual(5, controller.navigator.path_version)
        self.assertEqual([(1.0, 0.0)], controller.navigator.waypoints)
        self.assertEqual([5.0], controller.navigator.waypoint_not_before)
        self.assertTrue(controller.navigator.plan_is_partial)
        terminal = json.loads(controller.emitter.payloads[-1].decode("utf-8"))
        self.assertEqual("PLAN_ACTIVATED", terminal["type"])

    def test_abort_after_activation_restores_old_execution_snapshot(self):
        prepare = json.dumps({"target_robot": 1, "command": {
            "type": "prepare_plan", "plan_epoch": 11,
            "path_version": 5, "all_waypoints": [[1.0, 0.0]],
        }})
        arm = json.dumps({"target_robot": 1, "command": {
            "type": "arm_plan", "plan_epoch": 11,
        }})
        commit = json.dumps({"target_robot": 1, "command": {
            "type": "commit_plan", "plan_epoch": 11, "activate_at": 2.0,
        }})
        controller = self.controller_with_messages([prepare, arm, commit])
        controller.navigator.path_version = 4
        controller.navigator.set_waypoints([(9.0, 0.0), (8.0, 0.0)])
        controller.navigator.current_waypoint_idx = 0
        controller.navigator.paused_until = 1.7
        controller.navigator._replan_requested = True
        controller.navigator._emergency_stopped = True
        clock = type("Robot", (), {"now": 1.0,
                    "getTime": lambda self: self.now})()
        controller.robot = clock
        controller._receive_commands()
        # The old route continues progressing during the transaction.
        controller.navigator.current_waypoint_idx = 1
        clock.now = 2.0
        controller._activate_scheduled_joint_plan()
        self.assertEqual([(1.0, 0.0)], controller.navigator.waypoints)
        controller.receiver = FakeReceiver([json.dumps({
            "target_robot": 1, "command": {
                "type": "abort_plan", "plan_epoch": 11}})])
        controller._receive_commands()
        self.assertEqual(6, controller.navigator.path_version)
        self.assertEqual([(9.0, 0.0), (8.0, 0.0)],
                         controller.navigator.waypoints)
        self.assertEqual(1, controller.navigator.current_waypoint_idx)
        self.assertEqual(1.7, controller.navigator.paused_until)
        self.assertTrue(controller.navigator._replan_requested)
        self.assertTrue(controller.navigator._emergency_stopped)
        self.assertEqual(0, controller._active_plan_epoch)
        controller.receiver = FakeReceiver([command(5, [[99.0, 99.0]])])
        controller._receive_commands()
        self.assertEqual(6, controller.navigator.path_version)
        self.assertEqual([(9.0, 0.0), (8.0, 0.0)],
                         controller.navigator.waypoints)

    def test_delayed_older_prepare_cannot_overwrite_new_epoch(self):
        def prepare(epoch, version):
            return json.dumps({"target_robot": 1, "command": {
                "type": "prepare_plan", "plan_epoch": epoch,
                "path_version": version,
                "all_waypoints": [[float(epoch), 0.0]],
            }})
        controller = self.controller_with_messages([
            prepare(11, 5), prepare(10, 6)])
        controller._receive_commands()
        self.assertEqual(11, controller._highest_prepared_epoch)
        self.assertEqual({11}, set(controller._prepared_joint_plans))

    def test_legacy_navigate_invalidates_prepared_joint_plan(self):
        prepare = json.dumps({"target_robot": 1, "command": {
            "type": "prepare_plan", "plan_epoch": 11,
            "path_version": 5, "all_waypoints": [[1.0, 0.0]],
        }})
        controller = self.controller_with_messages([
            prepare, command(6, [[2.0, 0.0]])])
        controller._receive_commands()
        self.assertFalse(controller._prepared_joint_plans)
        self.assertEqual(6, controller.navigator.path_version)

    def test_stale_navigation_version_cannot_replace_active_path(self):
        controller = self.controller_with_messages([
            command(4, [[1.0, 0.0]]),
            command(3, [[9.0, 9.0]]),
        ])
        controller._receive_commands()
        self.assertEqual(4, controller.navigator.path_version)
        self.assertEqual([(1.0, 0.0)], controller.navigator.waypoints)

    def test_newer_navigation_version_replaces_active_path(self):
        controller = self.controller_with_messages([
            command(4, [[1.0, 0.0]]),
            command(5, [[2.0, 0.0], [3.0, 0.0]]),
        ])
        controller._receive_commands()
        self.assertEqual(5, controller.navigator.path_version)
        self.assertEqual(
            [(2.0, 0.0), (3.0, 0.0)], controller.navigator.waypoints)

    def test_speed_scale_command_does_not_reset_active_path(self):
        controller = self.controller_with_messages([json.dumps({
            "target_robot": 1,
            "command": {"type": "set_speed_scale", "scale": 0.6},
        })])
        controller.navigator.set_waypoints([(2.0, 0.0)])
        controller.navigator.current_waypoint_idx = 0
        controller._receive_commands()
        self.assertEqual(0.6, controller.navigator.speed_scale)

    def test_predictive_peer_response_never_stops_outside_hard_guard(self):
        navigator = WaypointNavigator()
        navigator.robot_id = 1
        navigator.set_waypoints([(2.0, 0.0)])
        navigator.speed_scale = 1.0
        navigator.peer_samples = {
            2: {
                'position': [0.7, 0.0],
                'velocity': [-0.22, 0.0],
                'sample_time': 1.0,
                'seq': 1,
            }
        }
        factor = navigator._peer_predictive_speed_factor(
            0.0, 0.0, 0.0, 1.0)
        self.assertGreater(factor, 0.0)
        self.assertLess(factor, 1.0)
        self.assertEqual([(2.0, 0.0)], navigator.waypoints)
        self.assertEqual(0, navigator.current_waypoint_idx)


if __name__ == "__main__":
    unittest.main()
