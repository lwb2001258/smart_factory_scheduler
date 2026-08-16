import sys
import json
import unittest
from types import SimpleNamespace
from pathlib import Path


SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

from factory_supervisor import FactorySupervisor, RobotInfo
from config import RobotState
from joint_grid_planner import TimedCell


class SupervisorPredictionTests(unittest.TestCase):
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

    def test_joint_edge_timeout_retires_epoch_without_holding_group(self):
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
        self.assertIsNone(supervisor._joint_plan_transaction)
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


if __name__ == "__main__":
    unittest.main()
