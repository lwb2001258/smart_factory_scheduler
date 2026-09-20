import sys
import json
import math
import unittest
from types import SimpleNamespace
from pathlib import Path


SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

from factory_supervisor import (
    COMMAND_ZERO_RECOVERY_TIMEOUT, FactorySupervisor, RobotInfo,
    PROGRESS_LEASE_SOFT_TIMEOUT)
from config import ALL_LOCATIONS, CHARGING_STATIONS, RobotState
from task_generator import TransportTask
from joint_grid_planner import TimedCell
from motion_safety import MOTION_SAFETY
import training_scenarios as training


class SupervisorPredictionTests(unittest.TestCase):
    def test_fallback_plan_assigns_monotonic_latest_exit_slots(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.robots = {1: SimpleNamespace()}
        supervisor._priority_yield_key = lambda rid: (rid,)
        supervisor._navigation_goal = lambda robot: "G"
        supervisor._goal_coordinates = lambda goal: (1.0, 0.0)
        supervisor.motion_coordinator = SimpleNamespace(
            plan_grid_lifelong=lambda rid, start, goal: [
                start, (0.5, 0.0), (1.0, 0.0)])

        plans, offsets, partial = supervisor._build_joint_fallback_plans(
            {1: ((0.0, 0.0), (1.0, 0.0))})

        self.assertEqual([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)],
                         plans[1])
        self.assertEqual(
            [0.0, 0.0, MOTION_SAFETY.joint_time_slot_s], offsets[1])
        self.assertFalse(partial[1])
        self.assertEqual(len(plans[1]), len(offsets[1]))

    def test_robot_state_exports_existing_controller_motion_diagnostics(self):
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_linear_speed = 0.0
        robot.controller_angular_speed = 0.5
        robot.controller_stop_reason = 'moving'
        robot.controller_local_risk_level = 'clear'
        robot.controller_status_sample_time = 8.25
        state = robot.to_dict()
        self.assertEqual('turning', state['controller_motion_state'])
        self.assertEqual(0.5, state['controller_angular_speed'])
        self.assertEqual(8.25, state['controller_status_sample_time'])

    def test_robot_state_exports_controller_target_for_recording_diagnosis(self):
        robot = RobotInfo(1, (0.1, 0.0))
        robot.waypoints = [(0.25, 0.0), (0.5, 0.0)]
        robot.controller_waypoint_index = 1
        robot.controller_paused_until = 12.0
        robot.controller_joint_wait_until = 13.0
        robot.controller_joint_wait_reason = 'joint_window_endpoint'
        robot.dispatch_not_before = 14.0
        robot.hold_until = 15.0
        state = robot.to_dict()
        self.assertEqual((0.5, 0.0), state['controller_target'])
        self.assertAlmostEqual(0.4, state['controller_target_distance'])
        self.assertEqual(12.0, state['controller_paused_until'])
        self.assertEqual(13.0, state['controller_joint_wait_until'])
        self.assertEqual('joint_window_endpoint',
                         state['controller_joint_wait_reason'])
        self.assertEqual(14.0, state['dispatch_not_before'])
        self.assertEqual(15.0, state['hold_until'])

    def test_robot_state_distinguishes_reported_and_mirrored_target(self):
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(0.25, 0.0)]
        robot.controller_waypoint_index = 0
        robot.controller_reported_target = (1.0, 0.0)
        robot.controller_reported_waypoint_count = 4
        state = robot.to_dict()
        self.assertEqual((0.25, 0.0), state['controller_target'])
        self.assertEqual((1.0, 0.0), state['controller_reported_target'])
        self.assertEqual(4, state['controller_reported_waypoint_count'])
        self.assertTrue(state['controller_target_mismatch'])
        self.assertEqual(1.0, state['controller_reported_target_distance'])

    def _terminal_supervisor(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        owner = RobotInfo(1, ALL_LOCATIONS['WS1'])
        incoming = RobotInfo(2, (0.0, 0.0))
        owner.sample_time = incoming.sample_time = 10.0
        supervisor.robots = {1: owner, 2: incoming}
        return supervisor, owner, incoming

    def test_terminal_owner_survives_task_chaining_until_physically_clear(self):
        supervisor, owner, _ = self._terminal_supervisor()
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        owner.state = RobotState.EN_ROUTE_PICKUP
        owner.goal_location = 'S1'
        self.assertEqual('WS1', owner.terminal_clearance_location)
        supervisor._update_terminal_clearance()
        self.assertEqual('WS1', owner.terminal_clearance_location)
        owner.position = (ALL_LOCATIONS['WS1'][0] + 1.0,
                          ALL_LOCATIONS['WS1'][1])
        owner.sample_time = 10.1
        supervisor.sim_time = 10.1
        supervisor._update_terminal_clearance()
        self.assertEqual('WS1', owner.terminal_clearance_location)
        owner.sample_time = 10.2
        supervisor.sim_time = 10.2
        supervisor._update_terminal_clearance()
        self.assertIsNone(owner.terminal_clearance_location)

    def test_terminal_owner_does_not_release_on_stale_pose(self):
        supervisor, owner, _ = self._terminal_supervisor()
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        owner.position = (ALL_LOCATIONS['WS1'][0] + 2.0,
                          ALL_LOCATIONS['WS1'][1])
        owner.sample_time = 9.0
        supervisor._update_terminal_clearance()
        supervisor._update_terminal_clearance()
        self.assertEqual('WS1', owner.terminal_clearance_location)

    def test_terminal_clearance_uses_hysteresis_and_two_fresh_samples(self):
        supervisor, owner, _ = self._terminal_supervisor()
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        terminal = ALL_LOCATIONS['WS1']
        owner.position = (terminal[0] + 0.64, terminal[1])
        supervisor._update_terminal_clearance()
        self.assertEqual('WS1', owner.terminal_clearance_location)
        owner.position = (terminal[0] + 0.66, terminal[1])
        for now in (10.1, 10.2):
            supervisor.sim_time = owner.sample_time = now
            supervisor._update_terminal_clearance()
        self.assertIsNone(owner.terminal_clearance_location)
        self.assertEqual('WS1', owner.terminal_egress_location)

    def test_terminal_capacity_releases_before_egress_priority(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        owner.current_task = SimpleNamespace(priority=1.0)
        incoming.current_task = SimpleNamespace(priority=100.0)
        incoming.state = RobotState.EN_ROUTE_PICKUP
        incoming.goal_location = 'WS1'
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        terminal = ALL_LOCATIONS['WS1']
        owner.position = (terminal[0] + 0.66, terminal[1])
        for now in (10.1, 10.2):
            supervisor.sim_time = owner.sample_time = now
            supervisor._update_terminal_clearance()
        self.assertIsNone(owner.terminal_clearance_location)
        self.assertEqual('WS1', owner.terminal_egress_location)
        self.assertIsNone(supervisor._terminal_owner('WS1'))
        self.assertEqual((1, 2), supervisor._priority_yield_pair(1, 2))

    def test_terminal_egress_priority_releases_at_outer_boundary(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        owner.current_task = SimpleNamespace(priority=1.0)
        incoming.current_task = SimpleNamespace(priority=100.0)
        incoming.state = RobotState.EN_ROUTE_PICKUP
        incoming.goal_location = 'WS1'
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        terminal = ALL_LOCATIONS['WS1']
        owner.position = (terminal[0] + 0.71, terminal[1])
        for now in (10.1, 10.2):
            owner.sample_time = supervisor.sim_time = now
            supervisor._update_terminal_clearance()
        self.assertIsNone(owner.terminal_egress_location)
        self.assertEqual((2, 1), supervisor._priority_yield_pair(1, 2))

    def test_terminal_evacuation_priority_beats_incoming_task_priority(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        owner.current_task = SimpleNamespace(priority=1.0)
        incoming.current_task = SimpleNamespace(priority=100.0)
        incoming.state = RobotState.EN_ROUTE_PICKUP
        incoming.goal_location = 'WS1'
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        self.assertEqual((1, 2), supervisor._priority_yield_pair(1, 2))

    def test_terminal_egress_does_not_change_uncontended_global_priority(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        owner.current_task = SimpleNamespace(priority=1.0)
        incoming.current_task = SimpleNamespace(priority=100.0)
        incoming.state = RobotState.EN_ROUTE_PICKUP
        incoming.goal_location = 'WS2'
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        self.assertFalse(owner.terminal_egress_priority_active)
        self.assertLess(supervisor._priority_yield_key(1),
                        supervisor._priority_yield_key(2))

    def test_terminal_vicinity_departure_beats_unrelated_high_priority_peer(self):
        supervisor, departing, peer = self._terminal_supervisor()
        departing.current_task = SimpleNamespace(priority=1.0)
        peer.current_task = SimpleNamespace(priority=100.0)
        departing.state = RobotState.EN_ROUTE_PICKUP
        departing.goal_location = 'S1'
        departing.waypoints = [(0.0, 0.0)]
        supervisor._update_terminal_departure_contexts()
        self.assertEqual('WS1', departing.terminal_departure_location)
        self.assertEqual((1, 2), supervisor._priority_yield_pair(1, 2))

    def test_terminal_vicinity_does_not_prioritize_approaching_robot(self):
        supervisor, approaching, peer = self._terminal_supervisor()
        approaching.current_task = SimpleNamespace(priority=1.0)
        peer.current_task = SimpleNamespace(priority=100.0)
        approaching.state = RobotState.EN_ROUTE_PICKUP
        approaching.goal_location = 'WS1'
        approaching.waypoints = [ALL_LOCATIONS['WS1']]
        supervisor._update_terminal_departure_contexts()
        self.assertIsNone(approaching.terminal_departure_location)
        self.assertEqual((2, 1), supervisor._priority_yield_pair(1, 2))

    def test_terminal_vicinity_departure_requires_fresh_pose_not_route(self):
        supervisor, robot, _ = self._terminal_supervisor()
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = 'S1'
        robot.waypoints = [(0.0, 0.0)]
        robot.sample_time = 9.0
        supervisor._update_terminal_departure_contexts()
        self.assertIsNone(robot.terminal_departure_location)
        robot.sample_time = supervisor.sim_time
        robot.waypoints = []
        supervisor._update_terminal_departure_contexts()
        self.assertEqual('WS1', robot.terminal_departure_location)

    def test_charging_station_vicinity_departure_is_detected(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 5.0
        robot = RobotInfo(1, CHARGING_STATIONS['CS1'])
        robot.sample_time = 5.0
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = 'S1'
        robot.waypoints = [(0.0, 0.0)]
        supervisor.robots = {1: robot}
        supervisor._update_terminal_departure_contexts()
        self.assertEqual('CS1', robot.terminal_departure_location)

    def test_two_terminal_departures_use_existing_task_priority(self):
        supervisor, first, second = self._terminal_supervisor()
        first.current_task = SimpleNamespace(priority=1.0)
        second.current_task = SimpleNamespace(priority=100.0)
        first.terminal_departure_location = 'WS1'
        second.terminal_departure_location = 'WS2'
        first.terminal_egress_priority_active = True
        first.terminal_egress_location = 'WS1'
        second.state = RobotState.EN_ROUTE_PICKUP
        second.goal_location = 'WS1'
        self.assertEqual((2, 1), supervisor._priority_yield_pair(1, 2))

    def test_idle_terminal_service_retries_safe_egress_without_new_task(self):
        supervisor, robot, _ = self._terminal_supervisor()
        supervisor.metrics = None
        robot.state = RobotState.IDLE
        robot.goal_location = None
        robot.waypoints = []
        terminal = ALL_LOCATIONS['WS1']
        target = (terminal[0] + 1.0, terminal[1])
        supervisor._joint_staging_goal = lambda *args: target
        supervisor.motion_coordinator = SimpleNamespace(
            plan_grid_lifelong=lambda *args: [target])
        installed = []
        supervisor._install_runtime_plan = (
            lambda rid, path, source=None:
            installed.append((rid, path, source)) or True)
        supervisor._mark_terminal_service_complete(robot, 'WS1')
        supervisor._ensure_terminal_egress_retries()
        self.assertEqual(RobotState.RETURNING_HOME, robot.state)
        self.assertEqual(target, robot.goal_location)
        self.assertEqual([(1, [target], '_terminal_egress_retry')], installed)

    def test_terminal_egress_retry_uses_bounded_backoff(self):
        supervisor, robot, _ = self._terminal_supervisor()
        supervisor.metrics = None
        robot.state = RobotState.IDLE
        robot.waypoints = []
        terminal = ALL_LOCATIONS['WS1']
        target = (terminal[0] + 1.0, terminal[1])
        supervisor._joint_staging_goal = lambda *args: target
        supervisor.motion_coordinator = SimpleNamespace(
            plan_grid_lifelong=lambda *args: None)
        supervisor._mark_terminal_service_complete(robot, 'WS1')
        supervisor._ensure_terminal_egress_retries()
        self.assertEqual(1, robot.terminal_egress_retry_count)
        self.assertEqual(10.5, robot.terminal_egress_retry_at)
        supervisor.sim_time = robot.sample_time = 10.5
        supervisor._ensure_terminal_egress_retries()
        self.assertEqual(2, robot.terminal_egress_retry_count)
        self.assertEqual(11.5, robot.terminal_egress_retry_at)
        supervisor.sim_time = robot.sample_time = 11.5
        supervisor._ensure_terminal_egress_retries()
        self.assertEqual(3, robot.terminal_egress_retry_count)
        self.assertEqual(13.5, robot.terminal_egress_retry_at)

    def test_terminal_egress_retry_rejects_target_still_inside_vicinity(self):
        supervisor, robot, _ = self._terminal_supervisor()
        supervisor.metrics = None
        robot.state = RobotState.IDLE
        terminal = ALL_LOCATIONS['WS1']
        supervisor._joint_staging_goal = lambda *args: (
            terminal[0] + 0.80, terminal[1])
        planned = []
        supervisor.motion_coordinator = SimpleNamespace(
            plan_grid_lifelong=lambda *args: planned.append(args) or [(0, 0)])
        supervisor._mark_terminal_service_complete(robot, 'WS1')
        supervisor._ensure_terminal_egress_retries()
        self.assertEqual([], planned)
        self.assertEqual('no_safe_path', robot.terminal_egress_retry_last_reason)

    def test_departure_retry_survives_inner_lease_release_until_vicinity_clear(self):
        supervisor, robot, _ = self._terminal_supervisor()
        supervisor.metrics = None
        supervisor._mark_terminal_service_complete(robot, 'WS1')
        terminal = ALL_LOCATIONS['WS1']
        robot.state = RobotState.IDLE
        robot.goal_location = None
        robot.waypoints = []
        robot.position = (terminal[0] + 0.75, terminal[1])
        for now in (10.1, 10.2):
            supervisor.sim_time = robot.sample_time = now
            supervisor._update_terminal_departure_contexts()
            supervisor._update_terminal_clearance()
        self.assertIsNone(robot.terminal_egress_location)
        self.assertEqual('WS1', robot.terminal_departure_location)
        self.assertEqual('WS1', robot.terminal_egress_retry_terminal)

    def test_terminal_egress_dispatch_failure_rolls_back_and_retries(self):
        supervisor, robot, _ = self._terminal_supervisor()
        supervisor.metrics = None
        robot.state = RobotState.IDLE
        terminal = ALL_LOCATIONS['WS1']
        target = (terminal[0] + 1.0, terminal[1])
        supervisor._joint_staging_goal = lambda *args: target
        rolled_back = []
        supervisor.motion_coordinator = SimpleNamespace(
            plan_grid_lifelong=lambda *args: [target],
            rollback_robot_plan=lambda rid: rolled_back.append(rid))
        supervisor._install_runtime_plan = lambda *args, **kwargs: False
        supervisor._mark_terminal_service_complete(robot, 'WS1')
        supervisor._ensure_terminal_egress_retries()
        self.assertEqual([1], rolled_back)
        self.assertEqual(RobotState.IDLE, robot.state)
        self.assertEqual('dispatch_failed', robot.terminal_egress_retry_last_reason)
        self.assertEqual(10.5, robot.terminal_egress_retry_at)

    def test_charging_terminal_is_owned_until_swap_robot_clears(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 5.0
        owner = RobotInfo(1, CHARGING_STATIONS['CS1'])
        incoming = RobotInfo(2, (-7.0, 0.0))
        owner.sample_time = incoming.sample_time = 5.0
        supervisor.robots = {1: owner, 2: incoming}
        supervisor._mark_terminal_service_complete(owner, 'CS1')
        self.assertEqual(1, supervisor._terminal_owner('CS1'))
        self.assertEqual(1, supervisor._terminal_owner('CS1', 2))

    def test_existing_clearing_owner_wins_abnormal_overlap(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        incoming.position = ALL_LOCATIONS['WS1']
        incoming.current_task = SimpleNamespace(priority=100.0)
        owner.current_task = SimpleNamespace(priority=1.0)
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        self.assertEqual(1, supervisor._terminal_owner('WS1'))

    def test_rolling_goal_arbitration_stages_incoming_and_evacuation(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        owner.goal_location = 'S1'
        owner.state = RobotState.EN_ROUTE_PICKUP
        incoming.goal_location = 'WS1'
        incoming.state = RobotState.EN_ROUTE_PICKUP
        supervisor._terminal_staging_target = lambda rid, terminal: (
            (-5.0, 3.0) if rid == 1 else (-4.0, 3.0))
        supervisor.motion_coordinator = SimpleNamespace(grid=SimpleNamespace(
            world_to_grid=lambda *point: (0, 0),
            in_bounds=lambda *cell: True, is_free=lambda *cell: True))
        self.assertEqual((1.0, 1.5),
                         supervisor._terminal_arbitrated_goal(
                             1, (1.0, 1.5)))
        self.assertEqual((-4.0, 3.0),
                         supervisor._terminal_arbitrated_goal(
                             2, ALL_LOCATIONS['WS1']))

    def test_terminal_staging_is_cached_for_stable_owner_epoch(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        terminal = ALL_LOCATIONS['WS1']
        incoming.position = (terminal[0] + 1.0, terminal[1])
        calls = []
        supervisor._joint_staging_goal = lambda *args: (
            calls.append(args) or (-5.0, 3.0))
        first = supervisor._terminal_staging_target(2, 'WS1')
        second = supervisor._terminal_staging_target(2, 'WS1')
        self.assertEqual(first, second)
        self.assertEqual(1, len(calls))
        owner.terminal_clearance_epoch += 1
        supervisor._terminal_staging_target(2, 'WS1')
        self.assertEqual(2, len(calls))

    def test_terminal_staging_does_not_reroute_distant_incoming_robot(self):
        supervisor, owner, incoming = self._terminal_supervisor()
        supervisor._mark_terminal_service_complete(owner, 'WS1')
        incoming.position = (0.0, 0.0)
        self.assertEqual('WS1', supervisor._terminal_admission_target(2, 'WS1'))
        self.assertEqual(ALL_LOCATIONS['WS1'],
                         supervisor._terminal_staging_target(2, 'WS1'))

    def test_progress_lease_soft_timeout_preserves_hard_planning_window(self):
        from config import JOINT_STALL_RELOCATION_TIMEOUT
        self.assertEqual(5.0, PROGRESS_LEASE_SOFT_TIMEOUT)
        self.assertLess(PROGRESS_LEASE_SOFT_TIMEOUT,
                        JOINT_STALL_RELOCATION_TIMEOUT)
        self.assertEqual(5.25, COMMAND_ZERO_RECOVERY_TIMEOUT)
        self.assertLess(COMMAND_ZERO_RECOVERY_TIMEOUT,
                        JOINT_STALL_RELOCATION_TIMEOUT)

    def test_uncommanded_zero_lease_ignores_pose_and_epoch_churn(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_status_sample_time = 10.0
        robot.controller_stop_reason = 'local_planner_zero_replan'
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        self.assertEqual(10.0, robot.uncommanded_zero_since)
        robot.position = (0.20, 0.0)
        robot.active_plan_epoch = 99
        robot.controller_status_sample_time = 15.0
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        self.assertEqual(10.0, robot.uncommanded_zero_since)

    def test_uncommanded_zero_lease_rearms_only_after_observed_motion(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_status_sample_time = 10.0
        robot.controller_stop_reason = 'unknown'
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        robot.uncommanded_zero_recovery_dispatched = True
        robot.uncommanded_zero_next_recovery_at = 30.0
        robot.controller_status_sample_time = 11.0
        robot.controller_linear_speed = 0.10
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        self.assertIsNone(robot.uncommanded_zero_since)
        self.assertFalse(robot.uncommanded_zero_recovery_dispatched)
        self.assertEqual(0.0, robot.uncommanded_zero_next_recovery_at)

    def test_command_zero_lease_is_fail_closed_without_wait_evidence(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_status_sample_time = 20.0
        robot.controller_stop_reason = 'joint_advance_grant'
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        self.assertEqual(20.0, robot.uncommanded_zero_since)
        robot.controller_joint_wait_reason = 'joint_advance_grant'
        robot.controller_joint_wait_until = 20.5
        robot.controller_status_sample_time = 21.0
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        self.assertEqual(20.0, robot.uncommanded_zero_since)
        robot.controller_joint_wait_until = 21.5
        supervisor._update_uncommanded_zero_lease(
            robot, {'navigating': True, 'emergency_braking': False})
        self.assertIsNone(robot.uncommanded_zero_since)

    def test_progress_lease_ignores_epoch_changes_and_pose_jitter(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 1.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.goal_location = (1.0, 0.0)
        supervisor._update_physical_progress_lease(robot)
        generation = robot.progress_lease_generation
        robot.active_plan_epoch = 99
        robot.path_version = 200
        supervisor.sim_time = 2.0
        robot.position = (0.02, 0.0)
        supervisor._update_physical_progress_lease(robot)
        self.assertEqual(generation, robot.progress_lease_generation)
        self.assertEqual(1.0, robot.progress_lease_last_progress_at)
        supervisor.sim_time = 3.0
        robot.position = (0.06, 0.0)
        supervisor._update_physical_progress_lease(robot)
        self.assertEqual(generation, robot.progress_lease_generation)
        self.assertEqual(1.0, robot.progress_lease_last_progress_at)
        supervisor.sim_time = 4.0
        robot.position = (0.11, 0.0)
        supervisor._update_physical_progress_lease(robot)
        self.assertEqual(generation, robot.progress_lease_generation)
        self.assertEqual(4.0, robot.progress_lease_last_progress_at)

    def test_logical_stall_clear_does_not_renew_physical_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 20.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.progress_lease_goal = (1.0, 0.0)
        robot.progress_lease_last_progress_at = 10.0
        robot.progress_lease_escalation_stage = 2
        supervisor._clear_joint_stall_state(robot)
        self.assertEqual(10.0, robot.progress_lease_last_progress_at)
        self.assertEqual(2, robot.progress_lease_escalation_stage)
        robot.active_plan_epoch = 100
        robot.path_version = 200
        self.assertEqual(10.0, supervisor._update_hard_stall_clock(robot))

    def test_physical_recovery_gate_requires_expired_cross_epoch_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.progress_lease_goal = (1.0, 0.0)
        robot.progress_lease_last_progress_at = 10.0
        robot.progress_lease_escalation_stage = 1
        robot._deadlock_stuck_since = 0.0
        supervisor.sim_time = 17.9
        self.assertFalse(supervisor._physical_progress_lease_hard_due(robot))
        robot.progress_lease_escalation_stage = 2
        supervisor.sim_time = 18.0
        self.assertTrue(supervisor._physical_progress_lease_hard_due(robot))
        # Logical route/epoch churn cannot change the physical decision.
        robot.active_plan_epoch = 500
        robot.path_version = 900
        self.assertTrue(supervisor._physical_progress_lease_hard_due(robot))
        supervisor._mark_physical_recovery_dispatched(robot)
        self.assertEqual(3, robot.progress_lease_escalation_stage)
        self.assertFalse(supervisor._physical_progress_lease_hard_due(robot))

    def test_command_zero_hard_gate_ignores_recent_pose_progress(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 18.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.progress_lease_goal = (1.0, 0.0)
        robot.progress_lease_last_progress_at = 17.5
        robot.uncommanded_zero_since = (
            supervisor.sim_time - COMMAND_ZERO_RECOVERY_TIMEOUT + 0.01)
        self.assertFalse(supervisor._physical_progress_lease_hard_due(robot))
        self.assertFalse(supervisor._uncommanded_zero_hard_due(robot))
        robot.uncommanded_zero_since = (
            supervisor.sim_time - COMMAND_ZERO_RECOVERY_TIMEOUT)
        self.assertTrue(supervisor._uncommanded_zero_hard_due(robot))
        self.assertTrue(supervisor._recovery_hard_due(robot))
        supervisor._mark_physical_recovery_dispatched(robot)
        self.assertFalse(supervisor._uncommanded_zero_hard_due(robot))
        self.assertFalse(supervisor._recovery_hard_due(robot))
        supervisor.sim_time = 20.0
        self.assertTrue(supervisor._uncommanded_zero_hard_due(robot))

    def _grant_supervisor(self, peer_position=(2.0, 2.0)):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        owner = RobotInfo(1, (0.0, 0.0))
        peer = RobotInfo(2, peer_position)
        for robot in (owner, peer):
            robot.sample_time = 10.0
        owner.active_plan_source = 'joint_grid_transaction'
        owner.active_plan_epoch = 7
        owner.active_joint_started_at = 10.0
        owner.active_joint_offsets = (0.0,)
        owner.controller_waypoint_index = 0
        owner.waypoints = [(0.25, 0.0)]
        supervisor.robots = {1: owner, 2: peer}
        supervisor.motion_coordinator = SimpleNamespace(
            turning_zones=lambda: {},
            resource_reservations=SimpleNamespace(owner_at=lambda *a, **k: None))
        return supervisor, owner, peer

    def test_advance_grant_requires_clear_swept_segment(self):
        supervisor, owner, _ = self._grant_supervisor()
        sent = []
        supervisor._send_command_to_robot = lambda rid, cmd: (
            sent.append((rid, cmd)) or True)
        supervisor._maybe_issue_advance_grant(owner)
        self.assertEqual(1, len(sent))
        self.assertEqual('advance_grant', sent[0][1]['type'])
        self.assertEqual(7, sent[0][1]['plan_epoch'])
        self.assertEqual(0, sent[0][1]['waypoint_index'])
        self.assertGreater(sent[0][1]['valid_until'], supervisor.sim_time)
        supervisor._maybe_issue_advance_grant(owner)
        self.assertEqual(1, len(sent))

    def test_advance_grant_rejects_entry_until_terminal_owner_clears(self):
        ws = ALL_LOCATIONS['WS1']
        supervisor, incoming, owner = self._grant_supervisor(
            peer_position=ws)
        incoming.position = (ws[0], ws[1] - 1.0)
        incoming.waypoints = [(ws[0], ws[1])]
        incoming.goal_location = 'WS1'
        incoming.state = RobotState.EN_ROUTE_PICKUP
        owner.sample_time = supervisor.sim_time
        owner.terminal_clearance_location = 'WS1'
        owner.terminal_egress_location = 'WS1'
        valid, evidence, peer_id = supervisor._validate_advance_grant(
            incoming, 0)
        self.assertFalse(valid)
        self.assertEqual('terminal_occupied:WS1', evidence)
        self.assertEqual(owner.robot_id, peer_id)

    def test_terminal_owner_can_receive_egress_grant(self):
        ws = ALL_LOCATIONS['WS1']
        supervisor, owner, peer = self._grant_supervisor(
            peer_position=(ws[0] + 3.0, ws[1]))
        owner.position = ws
        owner.waypoints = [(ws[0], ws[1] - 1.0)]
        owner.terminal_clearance_location = 'WS1'
        owner.terminal_egress_location = 'WS1'
        valid, evidence, peer_id = supervisor._validate_advance_grant(owner, 0)
        self.assertTrue(valid, evidence)
        self.assertIsNone(peer_id)

    def test_terminal_owner_cannot_follow_regressing_egress_segment(self):
        ws = ALL_LOCATIONS['WS1']
        supervisor, owner, peer = self._grant_supervisor(
            peer_position=(ws[0] + 3.0, ws[1]))
        owner.position = (ws[0], ws[1] + 0.40)
        owner.waypoints = [(ws[0], ws[1] + 0.20)]
        owner.terminal_clearance_location = 'WS1'
        owner.terminal_egress_location = 'WS1'
        valid, evidence, _ = supervisor._validate_advance_grant(owner, 0)
        self.assertFalse(valid)
        self.assertEqual('terminal_egress_regression:WS1', evidence)

    def test_advance_grant_preauthorizes_next_clear_segment(self):
        supervisor, owner, _ = self._grant_supervisor()
        owner.waypoints = [(0.25, 0.0), (0.50, 0.0)]
        owner.active_joint_offsets = (0.0, 1.0)
        sent = []
        supervisor._send_command_to_robot = lambda rid, cmd: (
            sent.append(cmd) or True)
        supervisor._maybe_issue_advance_grant(owner)
        self.assertEqual([0, 1], [item['waypoint_index'] for item in sent])
        self.assertLess(sent[0]['grant_seq'], sent[1]['grant_seq'])
        supervisor._maybe_issue_advance_grant(owner)
        self.assertEqual(2, len(sent))

    def test_advance_grant_rejects_swept_occupancy_and_stale_pose(self):
        supervisor, owner, peer = self._grant_supervisor((0.4, 0.0))
        valid, reason, peer_id = supervisor._validate_advance_grant(owner, 0)
        self.assertFalse(valid)
        self.assertIn('swept_occupancy', reason)
        self.assertEqual(2, peer_id)
        peer.position = (0.80, 0.0)
        peer.sample_time = 9.0
        valid, reason, peer_id = supervisor._validate_advance_grant(owner, 0)
        self.assertFalse(valid)
        self.assertIn('stale_relevant_peer', reason)
        self.assertEqual(2, peer_id)


    def test_advance_grant_ignores_spatially_irrelevant_stale_peer(self):
        supervisor, owner, peer = self._grant_supervisor((3.0, 3.0))
        peer.sample_time = 9.0
        valid, reason, peer_id = supervisor._validate_advance_grant(owner, 0)
        self.assertTrue(valid)
        self.assertEqual('vertex_edge_swept_clear', reason)
        self.assertIsNone(peer_id)

    def test_advance_grant_rejects_reserved_turning_zone(self):
        supervisor, owner, _ = self._grant_supervisor()
        supervisor.motion_coordinator = SimpleNamespace(
            turning_zones=lambda: {'I1': (0.1, 0.0)},
            resource_reservations=SimpleNamespace(
                owner_at=lambda *args, **kwargs: 2))
        valid, reason, peer_id = supervisor._validate_advance_grant(owner, 0)
        self.assertFalse(valid)
        self.assertEqual('zone_reserved:I1', reason)
        self.assertEqual(2, peer_id)

    def test_joint_physical_progress_uses_pose_not_controller_index(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 2.5
        supervisor.timestep = 16
        samples = []
        supervisor.metrics = SimpleNamespace(
            record_joint_cell_traversal=lambda value: samples.append(value))
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 4
        robot.waypoints = [(0.25, 0.0), (0.50, 0.0)]
        robot._physical_waypoint_started_at = 1.0
        robot._physical_waypoint_start_position = (0.0, 0.0)
        robot.controller_waypoint_index = 0

        robot._physical_waypoint_departed_at = 1.0
        robot.position = (0.24, 0.0)
        supervisor._record_joint_physical_progress(robot)
        self.assertEqual([1.5], samples)
        self.assertEqual(1, robot._physical_waypoint_index)
        supervisor._record_joint_physical_progress(robot)
        self.assertEqual([1.5], samples)

        robot.position = (0.10, 0.0)
        supervisor._record_joint_physical_progress(robot)
        self.assertEqual([1.5], samples)

    def test_joint_physical_progress_excludes_prior_scheduled_wait(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 5.0
        supervisor.timestep = 16
        samples = []
        supervisor.metrics = SimpleNamespace(
            record_joint_cell_traversal=lambda value: samples.append(value))
        robot = RobotInfo(1, (0.25, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 4
        robot.active_joint_started_at = 1.0
        robot.active_joint_offsets = (3.0, 6.0)
        robot.waypoints = [(0.25, 0.0), (0.50, 0.0)]
        robot._physical_waypoint_index = 1
        robot._physical_waypoint_started_at = 2.0
        robot._physical_waypoint_start_position = (0.25, 0.0)
        robot._physical_waypoint_departed_at = 4.0
        robot.position = (0.50, 0.0)

        supervisor._record_joint_physical_progress(robot)
        self.assertEqual([1.0], samples)

    def test_joint_physical_progress_excludes_short_grid_connector(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 2.5
        supervisor.timestep = 16
        samples = []
        supervisor.metrics = SimpleNamespace(
            record_joint_cell_traversal=lambda value: samples.append(value))
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 4
        robot.waypoints = [(0.10, 0.0)]
        robot._physical_waypoint_started_at = 1.0
        robot._physical_waypoint_start_position = (0.0, 0.0)
        robot._physical_waypoint_departed_at = 1.5
        robot.position = (0.10, 0.0)

        supervisor._record_joint_physical_progress(robot)
        self.assertEqual([], samples)
        self.assertEqual(1, robot._physical_waypoint_index)

    def test_joint_physical_progress_does_not_consume_waypoint_at_plan_tolerance(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 2.0
        supervisor.timestep = 16
        supervisor.metrics = SimpleNamespace(
            record_joint_cell_traversal=lambda value: None)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 4
        robot.waypoints = [(0.10, 0.0)]

        supervisor._record_joint_physical_progress(robot)
        self.assertEqual(0, robot._physical_waypoint_index)
        self.assertIsNone(robot._physical_waypoint_departed_at)

    def test_low_battery_cannot_requeue_onboard_cargo(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        task = TransportTask(
            1, "A", "B", (0.0, 0.0), (1.0, 0.0), 0.0,
            cargo_state="onboard")
        robot.current_task = task
        robot.state = RobotState.EN_ROUTE_DELIVERY
        supervisor.robots = {1: robot}
        self.assertFalse(supervisor._requeue_task_for_low_battery(
            1, cancel=True))
        self.assertIs(robot.current_task, task)
        self.assertEqual("onboard", task.cargo_state)

    def test_pickup_service_commits_delivery_state_once_before_route(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, ALL_LOCATIONS['S1'])
        task = TransportTask(
            1, 'S1', 'WS1', ALL_LOCATIONS['S1'], ALL_LOCATIONS['WS1'], 0.0)
        robot.current_task = task
        robot.state = RobotState.EN_ROUTE_PICKUP
        supervisor.robots = {1: robot}
        events = []

        class Event:
            def to_dict(self):
                return {'event': 'pickup_reached'}

        supervisor.rl_event_ledger = SimpleNamespace(
            append=lambda *args, **kwargs: events.append(args) or Event())
        supervisor.metrics = SimpleNamespace(record_rl_event=lambda event: None)
        self.assertTrue(supervisor._commit_pickup_service(robot))
        self.assertFalse(supervisor._commit_pickup_service(robot))
        self.assertEqual('onboard', task.cargo_state)
        self.assertEqual(10.0, task.pickup_time)
        self.assertEqual(RobotState.EN_ROUTE_DELIVERY, robot.state)
        self.assertEqual('WS1', robot.goal_location)
        self.assertEqual(1, len(events))

    def test_joint_wait_protocol_rejects_nonfinite_oversized_and_unknown(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor.metrics = None
        robot = RobotInfo(1, (0.0, 0.0))
        robot.path_version = 7
        robot.active_plan_epoch = 3
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_joint_started_at = 9.0
        robot.active_joint_wait_deadline = 20.5
        txn = SimpleNamespace(
            state='activated', epoch=3, members={1}, activate_at=9.0,
            waypoint_offsets={1: [1.5]})
        supervisor._joint_plan_transaction = txn

        audited = []
        supervisor.metrics = SimpleNamespace(
            record_safety_event=lambda event: audited.append(event))
        for until, reason in (
                (float('inf'), 'joint_epoch_barrier'),
                (1e9, 'joint_epoch_barrier'),
                (11.0, 'untrusted_wait'),
                (0.0, 'joint_epoch_barrier'),
                (-1.0, 'joint_epoch_barrier')):
            self.assertEqual(
                (0.0, None), supervisor._validated_joint_wait(robot, {
                    'planned_wait_until': until,
                    'planned_wait_reason': reason,
                }))
        self.assertEqual(5, len(audited))

        self.assertEqual(
            (11.0, 'joint_epoch_barrier'),
            supervisor._validated_joint_wait(robot, {
                'planned_wait_until': 11.0,
                'planned_wait_reason': 'joint_epoch_barrier',
            }))

    def test_active_epoch_wait_remains_valid_during_next_txn_prepare(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor.metrics = None
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 3
        robot.active_joint_started_at = 9.0
        robot.active_joint_wait_deadline = 20.5
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='preparing', epoch=4, members={1}, activate_at=None,
            waypoint_offsets={1: [1.5]})

        self.assertEqual(
            (11.0, 'joint_epoch_barrier'),
            supervisor._validated_joint_wait(robot, {
                'planned_wait_until': 11.0,
                'planned_wait_reason': 'joint_epoch_barrier',
            }))

    def test_valid_wait_that_expired_in_transit_clears_without_violation(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.1
        supervisor.metrics = None
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 3
        robot.active_joint_started_at = 9.0
        robot.active_joint_wait_deadline = 20.5

        self.assertEqual(
            (0.0, None), supervisor._validated_joint_wait(robot, {
                'planned_wait_until': 10.0,
                'planned_wait_reason': 'joint_slot_deadline',
            }))

    def test_supervisor_never_guesses_joint_grid_waypoint_progress(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(0.25, 0.0), (0.50, 0.0)]
        robot.active_plan_source = 'joint_grid_transaction'
        supervisor.robots = {1: robot}
        supervisor._simulate_movement(1, 0.1)
        self.assertEqual(0, robot.current_waypoint_idx)

    def test_joint_prefix_cannot_replace_authoritative_home_goal(self):
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.RETURNING_HOME
        robot.goal_location = (7.0, 3.0)
        robot.waypoints = [(0.25, 0.0), (0.50, 0.0)]
        robot.active_plan_source = 'joint_grid_transaction'
        self.assertEqual((7.0, 3.0),
                         FactorySupervisor._navigation_goal(robot))

    def test_charge_arrival_is_rejected_away_from_authoritative_station(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.RETURNING_TO_CHARGE
        robot.goal_location = 'CS1'
        supervisor.robots = {1: robot}
        supervisor._handle_goal_reached(1)
        self.assertEqual(RobotState.RETURNING_TO_CHARGE, robot.state)

    def test_joint_candidate_is_dispatched_for_every_active_robot(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 2.0
        supervisor._joint_plan_transaction = None
        supervisor._next_joint_grid_log = 99.0
        supervisor.robots = {
            1: RobotInfo(1, (0.0, 0.0)),
            2: RobotInfo(2, (2.0, 0.0)),
        }
        goals = {1: (1.0, 0.0), 2: (3.0, 0.0)}
        supervisor._has_active_navigation = lambda robot: True
        supervisor._navigation_goal = lambda robot: goals[robot.robot_id]

        class Grid:
            @staticmethod
            def grid_to_world(col, row): return (float(col), float(row))

        candidate = SimpleNamespace(
            paths={
                1: [TimedCell((0, 0), 0), TimedCell((1, 0), 1)],
                2: [TimedCell((2, 0), 0), TimedCell((3, 0), 1)],
            }, planning_seconds=0.001, expanded_nodes=2,
            time_slot_seconds=3.0)
        supervisor.motion_coordinator = SimpleNamespace(
            grid=Grid(),
            plan_joint_grid_candidate=lambda agents, max_seconds, **kwargs: candidate)
        captured = []
        supervisor._begin_joint_plan_transaction = (
            lambda plans, **kwargs: captured.append((plans, kwargs)) or True)

        supervisor._refresh_joint_grid_candidate()

        self.assertEqual(
            {1: [(0.1, 0.0), (1.0, 0.0)],
             2: [(2.1, 0.0), (3.0, 0.0)]}, captured[0][0])
        self.assertEqual({1: [0.0, 0.0], 2: [0.0, 0.0]},
                         captured[0][1]['waypoint_offsets'])
        self.assertEqual({1: False, 2: False},
                         captured[0][1]['partial_plans'])

    def test_robot_info_exposes_active_epoch_and_bounds_pending_wait(self):
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_epoch = 9
        robot.active_plan_source = 'joint_grid_transaction'
        robot.pending_waypoints = [(1.0, 0.0)]
        robot.dispatch_not_before = 5.0
        robot.sample_time = 4.0
        state = robot.to_dict()
        self.assertEqual(9, state['plan_epoch'])
        self.assertTrue(state['planned_wait'])
        self.assertEqual('dispatch_delay', state['wait_reason'])
        robot.sample_time = 6.0
        self.assertFalse(robot.to_dict()['planned_wait'])

    def test_omitted_successor_does_not_hold_committed_predecessor(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.active_plan_source = 'joint_grid_transaction'
        robot.active_plan_epoch = 4
        robot.waypoints = [(0.5, 0.0), (1.0, 0.0)]
        robot.controller_waypoint_index = 0
        supervisor.robots = {1: robot}
        holds = []
        supervisor._hold_robot = lambda rid, duration: holds.append((rid, duration))

        self.assertFalse(supervisor._hold_only_without_committed_route(1, 0.6))
        self.assertEqual([], holds)

    def test_omitted_robot_without_committed_route_is_bounded_held(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.robots = {1: RobotInfo(1, (0.0, 0.0))}
        holds = []
        supervisor._hold_robot = lambda rid, duration: holds.append((rid, duration))

        self.assertTrue(supervisor._hold_only_without_committed_route(1, 0.6))
        self.assertEqual([(1, 0.6)], holds)

    def test_robot_info_exposes_bounded_joint_controller_wait(self):
        robot = RobotInfo(1, (0.0, 0.0))
        robot.sample_time = 10.0
        robot.controller_joint_wait_until = 10.5
        robot.controller_joint_wait_reason = 'joint_epoch_barrier'
        state = robot.to_dict()
        self.assertTrue(state['planned_wait'])
        self.assertEqual('joint_epoch_barrier', state['wait_reason'])
        robot.sample_time = 10.6
        self.assertFalse(robot.to_dict()['planned_wait'])

    def test_runtime_route_writer_lease_rejects_competing_source(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_plan_epoch = 1
        supervisor._joint_plan_transaction = None
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(1.0, 0.0)]
        supervisor.robots = {1: robot}

        class Coordinator:
            rollbacks = 0
            def get_dispatch_delay(self, rid): return 0.0
            def commit_robot_plan(self, rid): pass
            def rollback_robot_plan(self, rid): self.rollbacks += 1
            def record_runtime_replan(self): pass

        supervisor.motion_coordinator = Coordinator()
        supervisor.metrics = None
        supervisor._send_command_to_robot = lambda *args: True
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(2.0, 0.0)], source="_command_reverse"))
        self.assertFalse(supervisor._install_runtime_plan(
            1, [(3.0, 0.0)], source="_proactive_path_conflict_scan"))
        self.assertEqual([(2.0, 0.0)], robot.waypoints)
        supervisor.sim_time = 13.0
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(3.0, 0.0)], source="_proactive_path_conflict_scan"))

    def test_emergency_writer_preempts_proactive_owner(self):
        supervisor = self._transaction_supervisor()
        supervisor._next_plan_epoch = 1
        supervisor.motion_coordinator.get_dispatch_delay = lambda rid: 0.0
        supervisor.motion_coordinator.commit_robot_plan = lambda rid: None
        supervisor.motion_coordinator.rollback_robot_plan = lambda rid: None
        supervisor.motion_coordinator.record_runtime_replan = lambda: None
        robot = supervisor.robots[1]
        robot.route_write_owner = '_proactive_path_conflict_scan'
        robot.route_write_until = 99.0
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(-1.0, 0.0)], source='_command_reverse'))
        self.assertEqual('_command_reverse', robot.route_write_owner)

    def test_equivalent_runtime_route_is_successful_noop_not_dispatch(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(1.0, 0.0)]
        robot.route_write_owner = '_command_reverse'
        robot.route_write_until = 13.0
        supervisor.robots = {1: robot}

        class Coordinator:
            def rollback_robot_plan(self, rid): pass

        supervisor.motion_coordinator = Coordinator()
        supervisor._last_plan_dispatch_performed = True
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(1.0, 0.0)], source='_command_reverse'))
        self.assertFalse(supervisor._last_plan_dispatch_performed)

    def test_direct_dispatch_cannot_bypass_higher_priority_owner(self):
        supervisor = self._transaction_supervisor()
        supervisor._next_plan_epoch = 1
        supervisor.motion_coordinator.get_dispatch_delay = lambda rid: 0.0
        robot = supervisor.robots[1]
        robot.route_write_owner = '_command_reverse'
        robot.route_write_until = 99.0
        before = len(supervisor.emitter.payloads)
        self.assertFalse(supervisor._dispatch_plan(
            1, [(3.0, 0.0)], source='_proactive_path_conflict_scan'))
        self.assertEqual(before, len(supervisor.emitter.payloads))

    def test_stale_delayed_pending_is_cleared_before_owner_gate(self):
        supervisor = self._transaction_supervisor()
        supervisor.motion_coordinator.get_dispatch_delay = lambda rid: 0.0
        robot = supervisor.robots[1]
        robot.pending_waypoints = [(3.0, 0.0)]
        robot.pending_plan_source = 'old_writer'
        robot.pending_plan_epoch = 4
        robot.pending_route_generation = 1
        robot.pending_previous_waypoints = [(9.0, 0.0)]
        robot.dispatch_not_before = 5.0
        robot.route_write_generation = 2
        robot.route_write_owner = '_command_reverse'
        robot.route_write_until = 99.0
        self.assertFalse(supervisor._dispatch_plan(
            1, [(3.0, 0.0)], delay=0.0, source='old_writer'))
        self.assertIsNone(robot.pending_waypoints)
        self.assertIsNone(robot.pending_previous_waypoints)
        self.assertEqual(0.0, robot.dispatch_not_before)

    def test_emergency_aborts_active_joint_and_dispatches_same_call(self):
        supervisor = self._transaction_supervisor()
        supervisor._next_plan_epoch = 1
        supervisor.motion_coordinator.get_dispatch_delay = lambda rid: 0.0
        supervisor.motion_coordinator.commit_robot_plan = lambda rid: None
        self.assertTrue(supervisor._begin_joint_plan_transaction({
            1: [(1.0, 0.0)], 2: [(2.0, 0.0)]}))
        self.assertTrue(supervisor._dispatch_plan(
            1, [(-1.0, 0.0)], delay=0.0, source='_command_reverse'))
        self.assertEqual('aborted', supervisor._joint_plan_transaction.state)
        self.assertEqual('_command_reverse',
                         supervisor.robots[1].route_write_owner)

    def test_enforced_joint_runtime_rejects_legacy_physical_route_writer(self):
        supervisor = self._transaction_supervisor()
        supervisor.joint_runtime_enforced = True
        supervisor._next_joint_grid_tick = 5.0
        rejected = []
        supervisor.metrics = SimpleNamespace(
            record_unauthorized_route_write=lambda *args: rejected.append(args),
            record_route_dispatch=lambda *args: None)
        supervisor.motion_coordinator.rollback_robot_plan = lambda rid: None
        supervisor.motion_coordinator.commit_robot_plan = lambda rid: None

        self.assertTrue(supervisor._dispatch_plan(
            1, [(-1.0, 0.0)], delay=0.0, source='_command_reverse'))
        self.assertEqual(0, len(rejected))
        self.assertTrue(supervisor.emitter.payloads)
        self.assertEqual('_command_reverse',
                         supervisor.robots[1].route_write_owner)
        self.assertEqual(5.0, supervisor._next_joint_grid_tick)

    @staticmethod
    def _transaction_supervisor():
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.robots = {
            1: RobotInfo(1, (0.0, 0.0)),
            2: RobotInfo(2, (1.0, 0.0)),
        }
        supervisor.robots[1].waypoints = [(9.0, 0.0)]
        supervisor.robots[2].waypoints = [(9.0, 1.0)]
        supervisor.sim_time = 1.0
        supervisor._next_plan_epoch = 10
        supervisor._joint_plan_transaction = None
        supervisor.metrics = None
        supervisor.motion_coordinator = type("Coordinator", (), {
            "joint_transactions_attempted": 0,
            "joint_transactions_activated": 0,
            "joint_transactions_aborted": 0,
        })()

        class Emitter:
            def __init__(self):
                self.payloads = []

            def send(self, payload):
                self.payloads.append(payload)

        supervisor.emitter = Emitter()
        return supervisor

    def test_joint_transaction_does_not_install_before_all_ack(self):
        supervisor = self._transaction_supervisor()
        plans = {1: [(1.0, 0.0)], 2: [(2.0, 0.0)]}
        self.assertTrue(supervisor._begin_joint_plan_transaction(plans))
        txn = supervisor._joint_plan_transaction
        txn.acknowledge(1, True)
        supervisor._advance_joint_plan_transaction()
        self.assertEqual([(9.0, 0.0)], supervisor.robots[1].waypoints)
        txn.acknowledge(2, True)
        supervisor._advance_joint_plan_transaction()
        self.assertEqual("arming", txn.state)
        txn.acknowledge_armed(1)
        txn.acknowledge_armed(2)
        supervisor._advance_joint_plan_transaction()
        self.assertEqual("committing", txn.state)
        txn.acknowledge_committed(1)
        txn.acknowledge_committed(2)
        supervisor._advance_joint_plan_transaction()
        self.assertEqual("committed", txn.state)
        txn.acknowledge_activated(1)
        txn.acknowledge_activated(2)
        supervisor._advance_joint_plan_transaction()
        self.assertEqual("activated", txn.state)
        self.assertEqual([(1.0, 0.0)], supervisor.robots[1].waypoints)
        self.assertEqual([(2.0, 0.0)], supervisor.robots[2].waypoints)
        self.assertEqual(11, supervisor._next_plan_epoch)

    def test_joint_barrier_never_partially_releases_later_slots(self):
        supervisor = self._transaction_supervisor()
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='activated', epoch=10, members={1, 2},
            released_waypoint_index=0)
        for robot in supervisor.robots.values():
            robot.controller_active_plan_epoch = 10
        supervisor.robots[1].controller_waypoint_index = 1
        supervisor.robots[2].controller_waypoint_index = 1
        supervisor._advance_joint_execution_barrier()
        self.assertFalse(supervisor.emitter.payloads)

    def test_business_leg_dispatch_retires_activated_joint_epoch(self):
        supervisor = self._transaction_supervisor()
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='activated', members={1, 2})
        for robot in supervisor.robots.values():
            robot.route_write_owner = 'joint_grid_transaction'
            robot.route_write_until = 99.0
        supervisor.motion_coordinator.get_dispatch_delay = lambda rid: 0.0
        supervisor.motion_coordinator.commit_robot_plan = lambda rid: None
        self.assertTrue(supervisor._dispatch_plan(
            1, [(3.0, 0.0)], delay=0.0,
            source='_handle_goal_reached'))
        self.assertIsNone(supervisor._joint_plan_transaction)
        self.assertIsNone(supervisor.robots[2].route_write_owner)
        # Business intent is accepted but no single-robot navigate is sent.
        self.assertFalse(supervisor.emitter.payloads)

    def test_joint_edge_timeout_keeps_epoch_when_successor_fails(self):
        supervisor = self._transaction_supervisor()
        supervisor.sim_time = 12.0
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='activated', epoch=10, members={1, 2}, activate_at=0.0,
            plans={1: [(1, 0), (2, 0), (3, 0)],
                   2: [(1, 1), (2, 1), (3, 1)]},
            waypoint_offsets={1: [1.5, 3.0, 4.5],
                              2: [1.5, 3.0, 4.5]})
        for robot in supervisor.robots.values():
            robot.controller_active_plan_epoch = 10
            robot.controller_waypoint_index = 0
            robot.route_write_owner = 'joint_grid_transaction'
        supervisor._has_active_navigation = lambda robot: True
        supervisor._navigation_goal = lambda robot: (4.0, robot.robot_id)
        supervisor.motion_coordinator.plan_joint_grid_candidate = (
            lambda agents, max_seconds, **kwargs: None)
        supervisor._refresh_joint_grid_candidate()
        self.assertEqual(10, supervisor._joint_plan_transaction.epoch)
        self.assertTrue(all(
            robot.route_write_owner == 'joint_grid_transaction'
            for robot in supervisor.robots.values()))
        commands = [json.loads(payload.decode('utf-8'))['command']['type']
                    for payload in supervisor.emitter.payloads]
        self.assertEqual([], commands)

    def test_joint_transaction_timeout_preserves_every_old_plan(self):
        supervisor = self._transaction_supervisor()
        old = {rid: list(robot.waypoints)
               for rid, robot in supervisor.robots.items()}
        supervisor._begin_joint_plan_transaction(
            {1: [(1.0, 0.0)], 2: [(2.0, 0.0)]}, timeout=0.5)
        supervisor._joint_plan_transaction.acknowledge(1, True)
        supervisor.sim_time = 1.5
        supervisor._advance_joint_plan_transaction()
        self.assertEqual("aborted", supervisor._joint_plan_transaction.state)
        self.assertEqual(old[1], supervisor.robots[1].waypoints)
        self.assertEqual(old[2], supervisor.robots[2].waypoints)

    def test_joint_abort_is_broadcast_only_once(self):
        supervisor = self._transaction_supervisor()
        supervisor._begin_joint_plan_transaction(
            {1: [(1.0, 0.0)], 2: [(2.0, 0.0)]}, timeout=0.1)
        supervisor.sim_time = 1.1
        supervisor._advance_joint_plan_transaction()
        count_after_abort = len(supervisor.emitter.payloads)
        supervisor._advance_joint_plan_transaction()
        self.assertEqual(count_after_abort, len(supervisor.emitter.payloads))

    def test_delayed_plan_preserves_source_and_epoch_until_activation(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_plan_epoch = 7
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(1.0, 0.0)]
        supervisor.robots = {1: robot}

        class Coordinator:
            @staticmethod
            def get_dispatch_delay(rid):
                return 0.0
            @staticmethod
            def commit_robot_plan(rid):
                pass
            @staticmethod
            def rollback_robot_plan(rid):
                pass
            @staticmethod
            def record_runtime_replan():
                pass

        class Metrics:
            def __init__(self):
                self.events = []
            def record_route_dispatch(self, *args):
                self.events.append(args)

        supervisor.motion_coordinator = Coordinator()
        supervisor.metrics = Metrics()
        supervisor._send_command_to_robot = lambda *args: True
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(2.0, 0.0)], delay=2.0, source="joint_planner"))
        self.assertEqual("joint_planner", robot.pending_plan_source)
        self.assertEqual(7, robot.pending_plan_epoch)
        self.assertEqual([(2.0, 0.0)], robot.waypoints)
        supervisor.sim_time = 12.0
        supervisor._dispatch_delayed_robots()
        self.assertIsNone(robot.pending_waypoints)
        self.assertEqual("joint_planner", supervisor.metrics.events[0][3])
        self.assertEqual(7, supervisor.metrics.events[0][2])

    def test_delayed_activation_failure_restores_previous_plan(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_plan_epoch = 1
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(1.0, 0.0)]
        supervisor.robots = {1: robot}

        class Coordinator:
            def get_dispatch_delay(self, rid): return 0.0
            def commit_robot_plan(self, rid): pass
            def rollback_robot_plan(self, rid): pass
            def record_runtime_replan(self): pass

        sends = iter((True, False))
        supervisor.motion_coordinator = Coordinator()
        supervisor.metrics = None
        supervisor._send_command_to_robot = lambda *args: next(sends)
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(2.0, 0.0)], delay=2.0, source="joint_planner"))
        supervisor.sim_time = 12.0
        supervisor._dispatch_delayed_robots()
        self.assertEqual([(1.0, 0.0)], robot.waypoints)
        self.assertIsNone(robot.pending_waypoints)

    def test_implicit_delay_activation_failure_restores_previous_plan(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_plan_epoch = 1
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(1.0, 0.0)]
        supervisor.robots = {1: robot}

        class Coordinator:
            def get_dispatch_delay(self, rid): return 2.0
            def commit_robot_plan(self, rid): pass
            def rollback_robot_plan(self, rid): pass
            def record_runtime_replan(self): pass

        sends = iter((True, False))
        supervisor.motion_coordinator = Coordinator()
        supervisor.metrics = None
        supervisor._send_command_to_robot = lambda *args: next(sends)
        self.assertTrue(supervisor._install_runtime_plan(
            1, [(2.0, 0.0)], source="joint_planner"))
        supervisor.sim_time = 12.0
        supervisor._dispatch_delayed_robots()
        self.assertEqual([(1.0, 0.0)], robot.waypoints)
        self.assertIsNone(robot.pending_waypoints)

    def test_short_trajectory_scan_resolves_active_conflict_episode(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 5.0
        supervisor.robots = {}

        class Metrics:
            scans = []
            def record_conflict_scan(self, conflicts, sim_time):
                self.scans.append((conflicts, sim_time))

        supervisor.metrics = Metrics()
        supervisor._proactive_path_conflict_scan()
        self.assertEqual([([], 5.0)], supervisor.metrics.scans)

    def test_escape_search_uses_open_space_beyond_half_metre(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 20.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (3.0, 0.0)
        robot.waypoints = [(3.0, 0.0)]
        peer = RobotInfo(2, (0.6, 0.0))
        supervisor.robots = {1: robot, 2: peer}

        class Coordinator:
            @staticmethod
            def _segment_clear(start, target):
                return True

            @staticmethod
            def plan_grid_lifelong(rid, start, target):
                if __import__('math').hypot(
                        target[0] - start[0], target[1] - start[1]) < 1.0:
                    return None
                return [target]

        supervisor.motion_coordinator = Coordinator()
        supervisor._navigation_goal = lambda rb: rb.goal_location
        installed = []
        supervisor._last_plan_dispatch_performed = True
        supervisor._install_runtime_plan = (
            lambda rid, path, delay=0.0: installed.append(path) or True)
        self.assertTrue(supervisor._command_reverse(1, 0.3))
        self.assertGreaterEqual(
            __import__('math').hypot(installed[0][0][0], installed[0][0][1]),
            1.0)

    def test_recovery_target_displacement_contract_boundaries(self):
        robot = RobotInfo(1, (0.0, 0.0))
        self.assertFalse(
            FactorySupervisor._recovery_target_has_effective_displacement(
                robot, (0.34, 0.0)))
        self.assertFalse(
            FactorySupervisor._recovery_target_has_effective_displacement(
                robot, (0.35, 0.0)))
        self.assertFalse(
            FactorySupervisor._recovery_target_has_effective_displacement(
                robot, (0.49, 0.0)))
        self.assertTrue(
            FactorySupervisor._recovery_target_has_effective_displacement(
                robot, (0.50, 0.0)))

    def test_command_reverse_never_dispatches_subthreshold_target(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 0.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (2.0, 0.0)
        supervisor.robots = {1: robot}

        class Coordinator:
            grid = None

            @staticmethod
            def _segment_clear(start, target): return True

            @staticmethod
            def plan_grid_lifelong(rid, start, target): return [target]

        supervisor.motion_coordinator = Coordinator()
        targets = []
        supervisor._last_plan_dispatch_performed = True
        supervisor._install_runtime_plan = (
            lambda rid, path, delay=0.0: targets.append(path[-1]) or True)
        self.assertTrue(supervisor._command_reverse(1, 0.3))
        self.assertGreaterEqual(
            __import__('math').hypot(targets[0][0], targets[0][1]), 0.50)

    def test_escape_dispatch_acquires_bounded_execution_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 20.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (3.0, 0.0)
        robot.waypoints = [(3.0, 0.0)]
        supervisor.robots = {1: robot}

        class Coordinator:
            grid = None

            @staticmethod
            def _segment_clear(start, target): return True

            @staticmethod
            def plan_grid_lifelong(rid, start, target): return [target]

        supervisor.motion_coordinator = Coordinator()
        supervisor._last_plan_dispatch_performed = True
        supervisor._install_runtime_plan = lambda *args, **kwargs: True
        self.assertTrue(supervisor._command_reverse(1, 0.3))
        self.assertTrue(robot.recovery_active)
        self.assertEqual(0, robot.recovery_execution_generation)
        self.assertEqual(0.0, robot.recovery_execution_until)

    def test_active_recovery_lease_suppresses_duplicate_watchdog(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 25.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (3.0, 0.0)
        robot.waypoints = [(1.0, 0.0)]
        robot.recovery_active = True
        robot.active_plan_source = '_command_reverse'
        robot.recovery_execution_business_goal = (3.0, 0.0)
        robot.recovery_execution_until = 30.0
        self.assertFalse(supervisor._robot_requires_recovery_monitoring(robot))
        supervisor.sim_time = 30.0
        self.assertTrue(supervisor._robot_requires_recovery_monitoring(robot))

    def test_goal_change_invalidates_recovery_execution_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 25.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (4.0, 0.0)
        robot.waypoints = [(1.0, 0.0)]
        robot.recovery_active = True
        robot.recovery_execution_business_goal = (3.0, 0.0)
        robot.recovery_execution_until = 30.0
        self.assertFalse(supervisor._recovery_execution_lease_active(robot))
        self.assertTrue(supervisor._robot_requires_recovery_monitoring(robot))

    def test_joint_stall_dispatch_acquires_execution_lease_without_escape_flag(self):
        supervisor = self._transaction_supervisor()
        supervisor._next_plan_epoch = 1
        supervisor.motion_coordinator.get_dispatch_delay = lambda rid: 0.0
        supervisor.motion_coordinator.commit_robot_plan = lambda rid: None
        supervisor.motion_coordinator.record_runtime_replan = lambda: None
        robot = supervisor.robots[1]
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (3.0, 0.0)
        self.assertTrue(supervisor._dispatch_plan(
            1, [(1.0, 0.0), (3.0, 0.0)], delay=0.0,
            source='_joint_stall_recovery', is_runtime_replan=True))
        self.assertFalse(robot.recovery_active)
        self.assertFalse(supervisor._recovery_execution_lease_active(robot))
        self.assertIsNone(robot.recovery_execution_target)

    def test_command_reverse_is_idempotent_during_execution_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (3.0, 0.0)
        robot.active_plan_source = '_command_reverse'
        robot.recovery_execution_business_goal = (3.0, 0.0)
        robot.recovery_execution_until = 20.0
        supervisor.robots = {1: robot}
        self.assertTrue(supervisor._recovery_execution_lease_active(robot))

    def test_joint_watchdog_skips_active_recovery_execution(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.system_ready = True
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = (3.0, 0.0)
        robot.waypoints = [(1.0, 0.0)]
        robot.active_plan_source = '_joint_stall_recovery'
        robot.recovery_execution_business_goal = (3.0, 0.0)
        robot.recovery_execution_until = 20.0
        supervisor.robots = {1: robot}
        supervisor._joint_collision_scan = lambda active: False
        calls = []
        supervisor._update_hard_stall_clock = lambda rb: calls.append(rb) or None
        supervisor._joint_runtime_watchdog()
        self.assertEqual([robot], calls)

    def test_recovery_reserved_cells_cover_compressed_segments(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(0.75, 0.0), (0.75, 0.5)]

        class Grid:
            @staticmethod
            def world_to_grid(x, y): return (round(x / 0.25), round(y / 0.25))

        class Coordinator:
            grid = Grid()

        supervisor.motion_coordinator = Coordinator()
        self.assertEqual(
            {(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2)},
            supervisor._recovery_reserved_cells(robot))

    def test_recovery_reservation_band_advances_instead_of_sealing_route(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(2.0, 0.0)]

        class Grid:
            @staticmethod
            def world_to_grid(x, y): return (round(x / 0.25), round(y / 0.25))

        class Planner:
            time_slot_seconds = 3.0
            horizon_slots = 4

        class Coordinator:
            grid = Grid()
            joint_grid_planner = Planner()

        supervisor.motion_coordinator = Coordinator()
        bands = supervisor._recovery_reserved_cells_by_slot(robot)
        self.assertIn((0, 0), bands[0])
        self.assertNotIn((8, 0), bands[0])
        self.assertNotEqual(bands[0], bands[2])
        self.assertIn((8, 0), bands[4])

    def test_joint_activation_clears_expired_recovery_identity(self):
        supervisor = self._transaction_supervisor()
        robot = supervisor.robots[1]
        robot.recovery_active = True
        robot.recovery_resume_goal = (3.0, 0.0)
        robot.recovery_execution_target = (1.0, 0.0)
        robot.recovery_execution_business_goal = (3.0, 0.0)
        robot.recovery_execution_until = -1.0
        self.assertTrue(supervisor._begin_joint_plan_transaction({
            1: [(1.0, 0.0)], 2: [(2.0, 1.0)]}))
        txn = supervisor._joint_plan_transaction
        for rid in txn.members:
            txn.acknowledge(rid, True)
        supervisor._advance_joint_plan_transaction()
        for rid in txn.members:
            txn.acknowledge_armed(rid)
        supervisor._advance_joint_plan_transaction()
        for rid in txn.members:
            txn.acknowledge_committed(rid)
        supervisor._advance_joint_plan_transaction()
        for rid in txn.members:
            txn.acknowledge_activated(rid)
        supervisor.sim_time = txn.activate_at
        supervisor._advance_joint_plan_transaction()
        self.assertFalse(robot.recovery_active)
        self.assertIsNone(robot.recovery_execution_target)
        self.assertIsNone(robot.recovery_execution_business_goal)
        self.assertEqual(0.0, robot.recovery_execution_until)

    def test_legal_wait_is_not_expected_to_move(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.waypoints = [(1.0, 0.0)]
        robot.pending_waypoints = [(1.0, 0.0)]
        robot.dispatch_not_before = 15.0
        self.assertFalse(supervisor._robot_should_be_moving(robot))
        robot.pending_waypoints = None
        robot.dispatch_not_before = 0.0
        robot.hold_until = 12.0
        self.assertFalse(supervisor._robot_should_be_moving(robot))
        robot.hold_until = 0.0
        self.assertTrue(supervisor._robot_should_be_moving(robot))

    def test_start_delay_keeps_global_samples_stationary(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        trajectory = supervisor._predict_trajectory(
            (0.0, 0.0), [(2.0, 0.0)], speed=1.0,
            horizon=4.0, dt=0.5, start_delay=2.0)
        first_four = trajectory[:4]
        self.assertEqual(
            [(0.0, 0.0)] * 4,
            [(x, y) for x, y, _ in first_four],
        )
        self.assertGreater(trajectory[4][0], 0.0)

    def test_zero_delay_moves_on_first_sample(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        trajectory = supervisor._predict_trajectory(
            (0.0, 0.0), [(2.0, 0.0)], speed=1.0,
            horizon=2.0, dt=0.5)
        self.assertAlmostEqual(0.5, trajectory[0][0])

    def test_emergency_brake_still_requires_recovery_monitoring(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.waypoints = [(1.0, 0.0)]
        robot.emergency_braking = True
        self.assertFalse(supervisor._robot_should_be_moving(robot))
        self.assertTrue(supervisor._robot_requires_recovery_monitoring(robot))

    def test_emergency_replan_is_processed_before_motion_filter(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor.robots = {1: RobotInfo(1, (0.0, 0.0))}
        robot = supervisor.robots[1]
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.waypoints = [(1.0, 0.0)]
        robot.goal_location = (1.0, 0.0)
        robot.emergency_braking = True
        robot._replan_requested = True
        robot._last_replan_time = 0.0

        class Coordinator:
            def plan_grid_lifelong(self, *args):
                return [(0.0, 0.5), (1.0, 0.0)]

        supervisor.motion_coordinator = Coordinator()
        supervisor._install_runtime_plan = lambda *args, **kwargs: True
        supervisor._log_replan = lambda *args, **kwargs: None
        supervisor._monitor_progress_and_replan()
        self.assertFalse(robot._replan_requested)
        self.assertEqual(10.0, robot._last_replan_time)

    def test_replan_request_survives_cooldown(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor.robots = {1: RobotInfo(1, (0.0, 0.0))}
        robot = supervisor.robots[1]
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.waypoints = [(1.0, 0.0)]
        robot.goal_location = (1.0, 0.0)
        robot.emergency_braking = True
        robot._replan_requested = True
        robot._last_replan_time = 9.0
        supervisor._monitor_progress_and_replan()
        self.assertTrue(robot._replan_requested)

    def test_long_stall_event_updates_metrics_once_per_window(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 12.0

        class Coordinator:
            deadlocks_detected = 0

        class Metrics:
            def __init__(self):
                self.events = []

            def record_deadlock(self, group, sim_time):
                self.events.append((group, sim_time))

        supervisor.motion_coordinator = Coordinator()
        supervisor.metrics = Metrics()
        self.assertTrue(supervisor._record_long_stall_event({3, 1}))
        self.assertFalse(supervisor._record_long_stall_event({1, 3}))
        self.assertEqual(1, supervisor.motion_coordinator.deadlocks_detected)
        self.assertEqual([([1, 3], 12.0)], supervisor.metrics.events)

    def test_escape_arrival_resumes_goal_without_business_transition(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 20.0
        supervisor.robots = {1: RobotInfo(1, (0.5, 0.0))}
        robot = supervisor.robots[1]
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.waypoints = [(0.5, 0.0)]
        robot.current_waypoint_idx = 1
        robot.recovery_active = True
        robot.recovery_resume_goal = (2.0, 0.0)

        class Coordinator:
            def plan_grid_lifelong(self, rid, position, goal):
                return [(1.0, 0.0), tuple(goal)]

        supervisor.motion_coordinator = Coordinator()
        installed = []
        supervisor._install_runtime_plan = (
            lambda rid, path, delay=0.0, source=None:
            installed.append((rid, path)) or True)
        supervisor._handle_goal_reached(1)
        self.assertEqual(RobotState.EN_ROUTE_PICKUP, robot.state)
        self.assertFalse(robot.recovery_active)
        self.assertEqual([(1, [(1.0, 0.0), (2.0, 0.0)])], installed)

    def test_close_emergency_pair_uses_priority_yield_key(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor.robots = {
            1: RobotInfo(1, (0.0, 0.0)),
            2: RobotInfo(2, (0.6, 0.0)),
        }
        for robot in supervisor.robots.values():
            robot.state = RobotState.EN_ROUTE_PICKUP
            robot.waypoints = [(2.0, 0.0)]
            robot.emergency_braking = True
            robot._replan_requested = True
        escaped = []
        supervisor._command_reverse = (
            lambda rid, distance: escaped.append(rid) or True)
        self.assertTrue(supervisor._coordinate_emergency_pair())
        # Equal task priority falls back to higher robot ID as right-of-way,
        # so the lower-ID robot is the yielder.
        self.assertEqual([1], escaped)
        self.assertEqual('winner',
                         supervisor.robots[2].recovery_session_role)
        self.assertEqual('yielder',
                         supervisor.robots[1].recovery_session_role)
        self.assertFalse(supervisor.robots[1]._replan_requested)
        self.assertFalse(supervisor.robots[2]._replan_requested)

    def test_speed_scale_supervisor_command_is_sent_without_path_change(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.robots = {1: RobotInfo(1, (0.0, 0.0))}

        class Emitter:
            def __init__(self):
                self.payload = None

            def send(self, payload):
                self.payload = payload

        supervisor.emitter = Emitter()
        self.assertTrue(supervisor._set_robot_speed_scale(1, 0.6))
        payload = __import__('json').loads(
            supervisor.emitter.payload.decode('utf-8'))
        self.assertEqual('set_speed_scale', payload['command']['type'])
        self.assertEqual(0.6, payload['command']['scale'])

    def test_dynamic_shield_stops_stale_moving_robot_and_audits(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 1.0
        stale = RobotInfo(1, (0.0, 0.0))
        peer = RobotInfo(2, (3.0, 0.0))
        stale.waypoints = [(1.0, 0.0)]
        stale.sample_time = 0.0
        peer.sample_time = 1.0
        supervisor.robots = {1: stale, 2: peer}
        sent = []
        supervisor._set_robot_speed_scale = (
            lambda rid, scale: sent.append((rid, scale)) or True)

        class Metrics:
            def __init__(self): self.events = []
            def record_safety_event(self, event): self.events.append(event)
        supervisor.metrics = Metrics()
        supervisor._apply_dynamic_safety_shield()
        self.assertIn((1, 0.0), sent)
        self.assertTrue(stale._replan_requested)
        self.assertEqual('emergency', stale._safety_shield_level)
        self.assertEqual('dynamic_safety_shield',
                         supervisor.metrics.events[0].event_type)

    def test_dynamic_shield_restores_preexisting_coordinator_scale(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 1.0
        a = RobotInfo(1, (0.0, 0.0))
        b = RobotInfo(2, (3.0, 0.0))
        a.speed_scale = 0.6
        a._safety_shield_scale = 0.0
        a._safety_shield_base_scale = 0.6
        a.sample_time = b.sample_time = 1.0
        supervisor.robots = {1: a, 2: b}
        sent = []
        supervisor._set_robot_speed_scale = (
            lambda rid, scale: sent.append((rid, scale)) or True)
        supervisor.metrics = None
        supervisor._apply_dynamic_safety_shield()
        self.assertIn((1, 0.6), sent)

    def test_dynamic_shield_uses_contract_prediction_clearance(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 1.0
        a = RobotInfo(1, (0.0, 0.0))
        b = RobotInfo(2, (0.8, 0.0))
        a.sample_time = b.sample_time = 1.0
        supervisor.robots = {1: a, 2: b}
        sent = []
        supervisor._set_robot_speed_scale = (
            lambda rid, scale: sent.append((rid, scale)) or True)
        supervisor.metrics = None
        supervisor._apply_dynamic_safety_shield()
        self.assertIn((1, 0.6), sent)
        self.assertIn((2, 0.6), sent)

    def test_failed_pose_read_does_not_refresh_freshness(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.num_robots = 1
        supervisor.sim_time = 2.0
        supervisor.step_count = 1
        robot = RobotInfo(1, (0.0, 0.0))
        robot.sample_time = 1.0
        supervisor.robots = {1: robot}

        class BrokenNode:
            def getField(self, _name): raise RuntimeError('injected delay')
        supervisor.robot_nodes = {1: BrokenNode()}
        supervisor._get_robot_positions_from_webots()
        self.assertEqual(1.0, robot.sample_time)

    def test_dynamic_shield_holds_restriction_to_prevent_chatter(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 1.1
        a = RobotInfo(1, (0.0, 0.0))
        b = RobotInfo(2, (3.0, 0.0))
        a.sample_time = b.sample_time = 1.1
        a._safety_shield_scale = 0.6
        a._safety_shield_level = 'caution'
        a._safety_shield_until = 1.5
        a._safety_shield_base_scale = 1.0
        supervisor.robots = {1: a, 2: b}
        sent = []
        supervisor._set_robot_speed_scale = (
            lambda rid, scale: sent.append((rid, scale)) or True)
        supervisor.metrics = None
        supervisor._apply_dynamic_safety_shield()
        self.assertNotIn((1, 1.0), sent)

    def test_joint_escape_cooldown_suppresses_repeat_route_write(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot._joint_escape_until = 12.0
        supervisor.robots = {1: robot}
        calls = []
        supervisor._joint_escape_robot = (
            lambda rid: calls.append(rid) or True)
        self.assertFalse(supervisor._joint_escalate_stalled_robots([1]))
        self.assertEqual([], calls)

    def test_proactive_scan_without_conflicts_does_not_reference_pair(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor.robots = {}
        supervisor._proactive_path_conflict_scan()

    def test_conflict_components_merge_three_robot_chain(self):
        conflicts = [
            (1, 2, 2.0, 0.5),
            (2, 3, 3.0, 0.5),
            (7, 8, 1.0, 0.4),
        ]
        self.assertEqual(
            [(1, 2, 3), (7, 8)],
            FactorySupervisor._conflict_components(conflicts))

    def test_trajectory_conflict_filter_includes_external_peer(self):
        trajectories = {
            1: [(0.0, 0.0, 0.5)],
            2: [(2.0, 0.0, 0.5)],
            3: [(0.5, 0.0, 0.5)],
        }
        conflicts = FactorySupervisor._trajectory_conflicts(
            trajectories, 0.75, only_robot_ids={1, 2})
        self.assertEqual([(1, 3, 0.5, 0.5)], conflicts)

    def test_joint_speed_transaction_rolls_back_sent_members(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.robots = {
            1: RobotInfo(1, (0.0, 0.0)),
            2: RobotInfo(2, (1.0, 0.0)),
        }
        sent = []

        def set_scale(rid, scale):
            sent.append((rid, scale))
            if rid == 2 and scale == 0.7:
                return False
            supervisor.robots[rid].speed_scale = scale
            return True

        supervisor._set_robot_speed_scale = set_scale
        self.assertFalse(supervisor._apply_joint_speed_profile(
            {1: 0.7, 2: 0.7}))
        self.assertEqual([(1, 0.7), (2, 0.7), (1, 1.0)], sent)
        self.assertEqual(1.0, supervisor.robots[1].speed_scale)

    def test_joint_speed_profile_can_adjust_multiple_group_members(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 0.0
        supervisor.robots = {
            rid: RobotInfo(rid, (0.0, 0.0)) for rid in (1, 2, 3)
        }
        for robot in supervisor.robots.values():
            robot.waypoints = [(1.0, 0.0)]

        def synthetic(rid, scale, horizon=10.0, dt=0.5):
            x = 0.0 if scale > 0.9 else rid * 2.0
            return [(x, 0.0, 0.5)]

        supervisor._trajectory_for_scale = synthetic
        base = {rid: [(0.0, 0.0, 0.5)] for rid in (1, 2, 3)}
        profile = supervisor._find_joint_speed_profile(
            (1, 2, 3), base, minimum_distance=0.75)
        self.assertIsNotNone(profile)
        self.assertEqual(2, sum(scale < 1.0 for scale in profile.values()))
        self.assertGreaterEqual(min(profile.values()), 0.85)


class AsyncCostOracleTests(unittest.TestCase):
    def test_runtime_bounded_miss_never_invokes_astar(self):
        oracle = training.FactoryAStarCostOracle(
            {1: {'position': (0.0, 0.0)}}, 1,
            bounded_cache_misses=True)
        oracle.cache.clear()
        original = oracle.coordinator.grid_planner.plan
        oracle.coordinator.grid_planner.plan = lambda *a, **k: (
            self.fail('A* ran in bounded scheduler cost path'))
        try:
            value = oracle.segment((0.01, 0.01), (4.0, 0.0))
            self.assertTrue(math.isfinite(value))
        finally:
            oracle.coordinator.grid_planner.plan = original

    def test_async_cache_miss_returns_without_running_astar_inline(self):
        oracle = training.FactoryAStarCostOracle(
            {1: {'position': (0.0, 0.0)}}, 1,
            async_cache_misses=True)
        oracle.cache.clear()
        original = oracle.coordinator.grid_planner.plan
        started = []
        blocker = __import__('threading').Event()

        def delayed(*args, **kwargs):
            started.append(True)
            blocker.wait(1.0)
            return original(*args, **kwargs)

        oracle.coordinator.grid_planner.plan = delayed
        try:
            value = oracle.segment((0.01, 0.01), (4.0, 0.0))
            self.assertTrue(math.isfinite(value))
            self.assertGreater(value, 0.0)
        finally:
            blocker.set()
            oracle.coordinator.grid_planner.plan = original


    def test_joint_transaction_detects_same_goal_unstarted_member(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robots = {1: RobotInfo(1, (0.0, 0.0)),
                  2: RobotInfo(2, (1.0, 0.0))}
        for rid, goal in ((1, 'WS1'), (2, 'WS2')):
            robots[rid].state = RobotState.EN_ROUTE_PICKUP
            robots[rid].goal_location = goal
            robots[rid].waypoints = [(2.0, 0.0)]
        supervisor.robots = robots
        txn = SimpleNamespace(
            members={1, 2}, plans={1: [(2.0, 0.0)], 2: [(2.0, 0.0)]},
            activation_positions={1: (0.0, 0.0), 2: (1.0, 0.0)},
            activation_goals={1: 'WS1', 2: 'WS2'}, epoch=3)
        self.assertTrue(supervisor._joint_transaction_has_unstarted_members(txn))

    def test_joint_transaction_rolls_after_every_member_departs(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robots = {1: RobotInfo(1, (0.06, 0.0)),
                  2: RobotInfo(2, (1.06, 0.0))}
        for rid, goal in ((1, 'WS1'), (2, 'WS2')):
            robots[rid].state = RobotState.EN_ROUTE_PICKUP
            robots[rid].goal_location = goal
            robots[rid].waypoints = [(2.0, 0.0)]
        supervisor.robots = robots
        txn = SimpleNamespace(
            members={1, 2}, plans={1: [(2.0, 0.0)], 2: [(2.0, 0.0)]},
            activation_positions={1: (0.0, 0.0), 2: (1.0, 0.0)},
            activation_goals={1: 'WS1', 2: 'WS2'}, epoch=3)
        self.assertFalse(supervisor._joint_transaction_has_unstarted_members(txn))

    def test_joint_transaction_goal_change_bypasses_stability_hold(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = 'WS2'
        robot.waypoints = [(2.0, 0.0)]
        supervisor.robots = {1: robot}
        txn = SimpleNamespace(
            members={1}, plans={1: [(2.0, 0.0)]},
            activation_positions={1: (0.0, 0.0)},
            activation_goals={1: 'WS1'}, epoch=3)
        self.assertFalse(supervisor._joint_transaction_has_unstarted_members(txn))

    def test_joint_refresh_interval_tracks_prefix_with_bounded_margin(self):
        self.assertEqual(2.0, FactorySupervisor._joint_refresh_interval(
            SimpleNamespace(waypoint_offsets={})))
        self.assertEqual(2.0, FactorySupervisor._joint_refresh_interval(
            SimpleNamespace(waypoint_offsets={1: [0.0, 3.0]})))
        self.assertEqual(4.5, FactorySupervisor._joint_refresh_interval(
            SimpleNamespace(waypoint_offsets={1: [0.0, 6.0]})))
        self.assertEqual(6.0, FactorySupervisor._joint_refresh_interval(
            SimpleNamespace(waypoint_offsets={1: [0.0, 12.0]})))
        self.assertEqual(3.5, FactorySupervisor._joint_refresh_interval(
            SimpleNamespace(waypoint_offsets={
                1: [0.0, 12.0], 2: [0.0, 5.0]})))

    def test_joint_transaction_goal_match_is_explicit(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.goal_location = 'WS1'
        supervisor.robots = {1: robot}
        txn = SimpleNamespace(members={1}, activation_goals={1: 'WS1'})
        self.assertTrue(supervisor._joint_transaction_goals_match(txn))
        robot.goal_location = 'WS2'
        self.assertFalse(supervisor._joint_transaction_goals_match(txn))

    def test_joint_refresh_does_not_retire_unstarted_valid_route(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 3.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = 'WS1'
        robot.waypoints = [(2.0, 0.0)]
        supervisor.robots = {1: robot}
        txn = SimpleNamespace(
            state='activated', members={1}, plans={1: [(2.0, 0.0)]},
            activation_positions={1: (0.0, 0.0)},
            activation_goals={1: 'WS1'}, epoch=3, activate_at=0.0)
        supervisor._joint_plan_transaction = txn
        supervisor._joint_liveness_needed = False
        retired = []
        supervisor._retire_joint_transaction = (
            lambda *args, **kwargs: retired.append(args))
        supervisor._refresh_joint_grid_candidate()
        self.assertEqual([], retired)

    def test_joint_refresh_liveness_request_bypasses_stability_hold(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 3.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.state = RobotState.EN_ROUTE_PICKUP
        robot.goal_location = 'WS1'
        robot.waypoints = [(2.0, 0.0)]
        supervisor.robots = {1: robot}
        txn = SimpleNamespace(
            state='activated', members={1}, plans={1: [(2.0, 0.0)]},
            activation_positions={1: (0.0, 0.0)},
            activation_goals={1: 'WS1'}, epoch=3, activate_at=0.0)
        supervisor._joint_plan_transaction = txn
        supervisor._joint_liveness_needed = True

        class Planned(Exception):
            pass

        supervisor._retire_joint_transaction = lambda *args, **kwargs: None
        supervisor._has_active_navigation = lambda robot: True
        supervisor._goal_coordinates = lambda goal: (2.0, 0.0)
        supervisor._navigation_goal = lambda robot: robot.goal_location
        supervisor._terminal_arbitrated_goal = lambda rid, goal: goal
        supervisor._priority_yield_key = lambda rid: rid
        supervisor._joint_candidate_snapshot = lambda agents: ()
        supervisor._recovery_execution_lease_active = lambda robot: False
        supervisor._joint_staging_goal = lambda *args: args[0]
        supervisor.motion_coordinator = SimpleNamespace(
            plan_joint_grid_candidate=lambda *args, **kwargs: (
                (_ for _ in ()).throw(Planned())))
        with self.assertRaises(Planned):
            supervisor._refresh_joint_grid_candidate()

    def test_new_route_less_component_bypasses_active_prefix_stability_hold(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 1.0
        first = RobotInfo(1, (0.0, 0.0))
        second = RobotInfo(2, (1.0, 0.0))
        for robot, goal in ((first, 'WS1'), (second, 'WS2')):
            robot.state = RobotState.EN_ROUTE_PICKUP
            robot.goal_location = goal
            robot.waypoints = [(2.0, float(robot.robot_id))]
        supervisor.robots = {1: first, 2: second}
        supervisor._pending_joint_plan_robot_ids = {2}
        supervisor._joint_liveness_needed = False
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='activated', members={1}, plans={1: [(2.0, 1.0)]},
            activation_positions={1: (0.0, 0.0)},
            activation_goals={1: 'WS1'}, epoch=3, activate_at=0.5,
            waypoint_offsets={1: [0.0, 8.0]})

        class Planned(Exception):
            pass

        supervisor._has_active_navigation = lambda robot: True
        supervisor._goal_coordinates = lambda goal: (2.0, 2.0)
        supervisor._navigation_goal = lambda robot: robot.goal_location
        supervisor._terminal_arbitrated_goal = lambda rid, goal: goal
        supervisor._priority_yield_key = lambda rid: rid
        supervisor._joint_candidate_snapshot = lambda agents: ()
        supervisor._recovery_execution_lease_active = lambda robot: False
        supervisor._recovery_reserved_cells_by_slot = lambda robot: {}
        supervisor._joint_staging_goal = lambda *args: args[0]
        supervisor.motion_coordinator = SimpleNamespace(
            plan_joint_grid_candidate=lambda *args, **kwargs: (
                (_ for _ in ()).throw(Planned())))
        with self.assertRaises(Planned):
            supervisor._refresh_joint_grid_candidate()

    def test_equivalent_joint_prefix_is_kept_with_reservation_margin(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 2.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(0.25, 0.0), (0.5, 0.0), (0.75, 0.0)]
        robot.current_waypoint_idx = 0
        supervisor.robots = {1: robot}

        class Grid:
            @staticmethod
            def world_to_grid(x, y):
                return round(x / 0.25), round(y / 0.25)

        supervisor.motion_coordinator = SimpleNamespace(
            grid=Grid(), joint_grid_planner=SimpleNamespace(
                time_slot_seconds=3.0))
        txn = SimpleNamespace(
            members={1}, activate_at=0.0,
            waypoint_offsets={1: [1.0, 4.0, 7.0]})
        self.assertTrue(supervisor._joint_candidate_preserves_active_prefix(
            txn, {1: [(0.25, 0.0), (0.5, 0.0), (0.75, 0.0)]}))

    def test_equivalent_joint_prefix_is_not_kept_near_expiry(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 3.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(0.25, 0.0), (0.5, 0.0)]
        supervisor.robots = {1: robot}

        class Grid:
            @staticmethod
            def world_to_grid(x, y):
                return round(x / 0.25), round(y / 0.25)

        supervisor.motion_coordinator = SimpleNamespace(
            grid=Grid(), joint_grid_planner=SimpleNamespace(
                time_slot_seconds=1.0))
        txn = SimpleNamespace(
            members={1}, activate_at=0.0,
            waypoint_offsets={1: [3.0, 4.0]})
        self.assertFalse(supervisor._joint_candidate_preserves_active_prefix(
            txn, {1: [(0.25, 0.0), (0.5, 0.0)]}))

    def test_changed_joint_prefix_is_never_suppressed(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 0.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.waypoints = [(0.25, 0.0), (0.5, 0.0)]
        supervisor.robots = {1: robot}

        class Grid:
            @staticmethod
            def world_to_grid(x, y):
                return round(x / 0.25), round(y / 0.25)

        supervisor.motion_coordinator = SimpleNamespace(
            grid=Grid(), joint_grid_planner=SimpleNamespace(
                time_slot_seconds=3.0))
        txn = SimpleNamespace(
            members={1}, activate_at=0.0,
            waypoint_offsets={1: [3.0, 6.0]})
        self.assertFalse(supervisor._joint_candidate_preserves_active_prefix(
            txn, {1: [(0.0, 0.25), (0.0, 0.5)]}))

    def test_failed_successor_preflight_preserves_active_transaction(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 4.0
        robot = RobotInfo(1, (0.0, 0.0))
        supervisor.robots = {1: robot}
        active = SimpleNamespace(state='activated', members={1}, epoch=8)
        supervisor._joint_plan_transaction = active
        supervisor._next_plan_epoch = 9
        self.assertFalse(supervisor._begin_joint_plan_transaction(
            {1: []}, replace_transaction=active))
        self.assertIs(active, supervisor._joint_plan_transaction)

    def test_successor_retires_active_only_after_preflight(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 4.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.route_write_owner = 'joint_grid_transaction'
        supervisor.robots = {1: robot}
        active = SimpleNamespace(state='activated', members={1}, epoch=8)
        supervisor._joint_plan_transaction = active
        supervisor._next_plan_epoch = 9
        supervisor.motion_coordinator = SimpleNamespace(
            joint_transactions_attempted=0)
        supervisor._waypoints_are_stationary = lambda *args: False
        supervisor._route_write_allowed = lambda *args: True
        supervisor._send_command_to_robot = lambda *args: True
        supervisor._advance_joint_plan_transaction = lambda: None
        retired = []
        original_retire = supervisor._retire_joint_transaction
        supervisor._retire_joint_transaction = lambda txn, reason: (
            retired.append((txn, reason)), original_retire(txn, reason))[-1]
        self.assertTrue(supervisor._begin_joint_plan_transaction(
            {1: [(1.0, 0.0)]}, replace_transaction=active))
        self.assertEqual([(active, 'validated_successor')], retired)
        self.assertEqual(9, supervisor._joint_plan_transaction.epoch)

    def test_bounded_joint_wait_pauses_physical_stall_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_joint_wait_until = 11.0
        robot.controller_joint_wait_reason = 'joint_window_endpoint'
        self.assertTrue(supervisor._validated_runtime_wait(robot))

    def test_expired_joint_wait_does_not_pause_physical_stall_lease(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 12.0
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_joint_wait_until = 11.0
        robot.controller_joint_wait_reason = 'joint_window_endpoint'
        self.assertFalse(supervisor._validated_runtime_wait(robot))

    def test_joint_endpoint_request_is_once_per_epoch_and_reason(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_joint_grid_tick = 12.0
        supervisor.metrics = SimpleNamespace(
            record_joint_window_gap=lambda *args: None)
        robot = RobotInfo(1, (0.0, 0.0))
        robot.controller_joint_wait_reason = 'joint_window_endpoint'
        robot.active_joint_wait_deadline = 11.0
        robot.controller_active_plan_epoch = 4
        robot.controller_waypoint_index = 1
        supervisor.robots = {1: robot}
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='activated', epoch=4, members={1},
            plans={1: [(0.0, 0.0)]})
        self.assertTrue(supervisor._request_joint_endpoint_once(robot, 4))
        self.assertFalse(supervisor._request_joint_endpoint_once(robot, 4))
        self.assertEqual(10.1, supervisor._next_joint_grid_tick)

    def test_joint_endpoint_waits_for_all_transaction_members(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_joint_grid_tick = 12.0
        supervisor.metrics = SimpleNamespace(
            record_joint_window_gap=lambda *args: None)
        first = RobotInfo(1, (1.0, 0.0))
        second = RobotInfo(2, (0.0, 0.0))
        for robot in (first, second):
            robot.controller_joint_wait_reason = 'joint_window_endpoint'
            robot.active_joint_wait_deadline = 11.0
            robot.controller_active_plan_epoch = 4
        first.controller_waypoint_index = 1
        supervisor.robots = {1: first, 2: second}
        supervisor._joint_plan_transaction = SimpleNamespace(
            state='activated', epoch=4, members={1, 2},
            plans={1: [(1.0, 0.0)], 2: [(1.0, 0.0)]})
        self.assertFalse(supervisor._request_joint_endpoint_once(first, 4))
        self.assertEqual(12.0, supervisor._next_joint_grid_tick)
        second.position = (1.0, 0.0)
        second.controller_waypoint_index = 1
        self.assertTrue(supervisor._request_joint_endpoint_once(second, 4))
        self.assertEqual(10.1, supervisor._next_joint_grid_tick)

    def test_fresh_plan_request_preserves_activated_transaction(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_joint_grid_tick = 12.0
        supervisor._joint_replan_cooldown_until = 0.0
        active = SimpleNamespace(state='activated', members={1}, epoch=4)
        supervisor._joint_plan_transaction = active
        supervisor._joint_candidate_failures = 3
        supervisor._log_replan = lambda *args: None
        self.assertTrue(supervisor._joint_request_fresh_plan(
            {1}, 'test_liveness'))
        self.assertIs(active, supervisor._joint_plan_transaction)
        self.assertEqual(10.0, supervisor._next_joint_grid_tick)

    def test_joint_planning_event_is_consumed_without_periodic_reschedule(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor._next_joint_grid_tick = 10.0
        supervisor._refresh_joint_grid_candidate = lambda: None
        supervisor._consume_joint_planning_event()
        self.assertTrue(math.isinf(supervisor._next_joint_grid_tick))

    def test_joint_planning_event_preserves_reschedule_from_refresh(self):
        supervisor = FactorySupervisor.__new__(FactorySupervisor)
        supervisor.sim_time = 10.0
        supervisor._next_joint_grid_tick = 10.0
        supervisor._refresh_joint_grid_candidate = lambda: setattr(
            supervisor, '_next_joint_grid_tick', 10.5)
        supervisor._consume_joint_planning_event()
        self.assertEqual(10.5, supervisor._next_joint_grid_tick)

    def test_joint_plan_request_reasons_have_stable_gate_classes(self):
        classify = FactorySupervisor._joint_plan_request_class
        self.assertEqual('SAFETY', classify('predicted_collision'))
        self.assertEqual('SAFETY', classify('priority_yield_direct_failed'))
        self.assertEqual('LIVENESS', classify(
            'physical_progress_lease_soft_deadline'))
        self.assertEqual('LIVENESS', classify('priority_yield_timeout'))
        self.assertEqual('BUSINESS', classify('terminal_member_change'))
        self.assertEqual('ROLLING', classify('ordinary_refresh'))

    def test_joint_watchdog_reason_preserves_highest_severity_cause(self):
        reason = FactorySupervisor._joint_watchdog_request_reason
        self.assertEqual('joint_watchdog_emergency', reason(
            [1], [2], [3], [(4, 8.0)]))
        self.assertEqual('joint_watchdog_hard_motion', reason(
            [], [2], [3], [(4, 8.0)]))
        self.assertEqual('joint_watchdog_route_less', reason(
            [], [2], [3], []))
        self.assertEqual('joint_watchdog_stale_wait', reason(
            [], [2], [], []))


if __name__ == "__main__":
    unittest.main()
