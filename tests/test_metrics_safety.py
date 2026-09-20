import sys
import json
import unittest
from pathlib import Path


SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

from metrics_collector import MetricsCollector
from config import RobotState


class MetricsSafetyTests(unittest.TestCase):

    def test_motion_continuity_classifies_final_commands(self):
        metrics = MetricsCollector("test", "test", 1)
        base = {
            'navigating': True, 'emergency_braking': False,
            'commanded_linear_speed': 0.0,
            'commanded_angular_speed': 0.0,
        }
        for seq, sample_time in enumerate((0.0, 0.16, 0.32, 0.48, 0.64), 1):
            metrics.record_motion_telemetry(
                sample_time, 1,
                dict(base, status_seq=seq, status_sample_time=sample_time))
        metrics.record_motion_telemetry(0.80, 1, dict(
            base, status_seq=6, status_sample_time=0.80,
            commanded_linear_speed=0.10))
        state = metrics.motion_continuity_by_robot[1]
        self.assertAlmostEqual(0.80, state['eligible_seconds'])
        self.assertAlmostEqual(0.16, state['moving_seconds'])
        self.assertAlmostEqual(0.64,
                               state['unblocked_zero_speed_seconds'])
        self.assertEqual(1, state['unblocked_stop_episodes'])

    def test_motion_continuity_does_not_bridge_missing_samples(self):
        metrics = MetricsCollector("test", "test", 1)
        def sample(seq, when, linear=0.0):
            return {
                'status_seq': seq, 'status_sample_time': when,
                'navigating': True, 'emergency_braking': False,
                'commanded_linear_speed': linear,
                'commanded_angular_speed': 0.0,
            }
        metrics.record_motion_telemetry(0.0, 1, sample(1, 0.0))
        metrics.record_motion_telemetry(0.16, 1, sample(2, 0.16))
        metrics.record_motion_telemetry(0.64, 1, sample(4, 0.64))
        metrics.record_motion_telemetry(0.80, 1, sample(5, 0.80, 0.1))
        state = metrics.motion_continuity_by_robot[1]
        self.assertEqual(1, state['status_drops'])
        self.assertEqual(0, state['unblocked_stop_episodes'])
        self.assertAlmostEqual(0.32, state['eligible_seconds'])

    def test_validated_wait_cuts_zero_episode_with_unchanged_motor_state(self):
        metrics = MetricsCollector("test", "test", 1)
        def sample(seq, when, wait=False, moving=False):
            status = {
                'status_seq': seq, 'status_sample_time': when,
                'navigating': True, 'emergency_braking': False,
                'commanded_linear_speed': 0.1 if moving else 0.0,
                'commanded_angular_speed': 0.0,
                'command_motion_changed_at': 0.0 if not moving else when,
            }
            if wait:
                status['wait_validator_evidence'] = {
                    'validated': True, 'valid_until': when + 1.0}
            return status
        metrics.record_motion_telemetry(0.0, 1, sample(1, 0.0))
        metrics.record_motion_telemetry(0.16, 1, sample(2, 0.16))
        metrics.record_motion_telemetry(0.32, 1, sample(3, 0.32, wait=True))
        for seq, when in enumerate((0.48, 0.64, 0.80, 0.96, 1.12), 4):
            metrics.record_motion_telemetry(when, 1, sample(seq, when))
        metrics.record_motion_telemetry(1.28, 1, sample(9, 1.28, moving=True))
        event = metrics.motion_continuity_events[-1]
        self.assertAlmostEqual(0.32, event['started_at'])
        self.assertAlmostEqual(0.96, event['duration'])

    def test_motion_continuity_rejects_malformed_wait_evidence(self):
        metrics = MetricsCollector("test", "test", 1)
        base = {
            'navigating': True, 'emergency_braking': False,
            'commanded_linear_speed': 0.0,
            'commanded_angular_speed': 0.0,
            'wait_validator_evidence': {
                'validated': True, 'valid_until': 'not-a-time'},
        }
        metrics.record_motion_telemetry(
            0.0, 1, dict(base, status_seq=1, status_sample_time=0.0))
        metrics.record_motion_telemetry(
            0.16, 1, dict(base, status_seq=2, status_sample_time=0.16))
        state = metrics.motion_continuity_by_robot[1]
        self.assertEqual(0.0, state['valid_wait_seconds'])
        self.assertAlmostEqual(0.16,
                               state['unblocked_zero_speed_seconds'])

    def test_progress_lease_events_have_independent_summary_counts(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_progress_lease_event(3.0, 1, 4, 'soft_replan', 3.0)
        metrics.record_progress_lease_event(8.0, 1, 4, 'hard_escape', 8.0)
        metrics.record_progress_lease_event(
            9.0, 2, 5, 'hard_zero_escape', 8.0)

        class Robot:
            idle_time = 0.0
            total_distance = 0.0
            tasks_completed = 0

        summary = metrics.compute_final_metrics(
            {1: Robot()}, {'completed': 0}, {}, 10.0)
        self.assertEqual(1, summary['progress_lease_soft_deadlines'])
        self.assertEqual(2, summary['progress_lease_hard_deadlines'])
        self.assertEqual(0, metrics.safety_event_count)

    def test_joint_timing_metrics_are_auditable_and_gap_is_deduplicated(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_joint_cell_traversal(2.5)
        metrics.record_joint_cell_traversal(3.5)
        metrics.record_joint_window_gap(10.0, 1, 7, 9.0)
        metrics.record_joint_window_gap(11.0, 1, 7, 9.0)
        metrics.record_expired_reservation_movement(12.0, 1, 7, 2, 11.0)
        self.assertEqual(metrics.joint_cell_traversal_seconds, [2.5, 3.5])
        self.assertEqual(len(metrics.joint_window_gap_events), 1)
        self.assertEqual(len(metrics.reservation_expired_movement_events), 1)
    def test_task_motion_without_progress_is_audited_after_ten_seconds(self):
        metrics = MetricsCollector("test", "test", 1)
        state = {1: {
            "state": RobotState.EN_ROUTE_PICKUP,
            "position": (1.0, 1.0), "planned_wait": False,
            "path_version": 7, "plan_epoch": 11,
        }}
        metrics.record_step(0.0, state, {}, {})
        metrics.record_step(10.0, state, {}, {})
        self.assertEqual(1, metrics.unplanned_stop_count)
        self.assertEqual(7, metrics.unplanned_stop_events[0]["path_version"])

    def test_explicit_planned_wait_is_not_reported_as_stop(self):
        metrics = MetricsCollector("test", "test", 1)
        state = {1: {
            "state": RobotState.EN_ROUTE_DELIVERY,
            "position": (1.0, 1.0), "planned_wait": True,
        }}
        metrics.record_step(0.0, state, {}, {})
        metrics.record_step(20.0, state, {}, {})
        self.assertEqual(0, metrics.unplanned_stop_count)

    def test_planned_wait_is_one_audited_episode_then_stop_timer_restarts(self):
        metrics = MetricsCollector("test", "test", 1)
        waiting = {1: {
            "state": RobotState.EN_ROUTE_DELIVERY,
            "position": (1.0, 1.0), "planned_wait": True,
            "wait_reason": "joint_epoch_barrier", "wait_deadline": 5.0,
            "path_version": 7, "plan_epoch": 11,
            "plan_source": "joint_grid", "task_id": 3,
        }}
        moving = {1: dict(waiting[1], planned_wait=False)}
        metrics.record_step(0.0, waiting, {}, {})
        metrics.record_step(2.0, waiting, {}, {})
        metrics.record_step(3.0, moving, {}, {})

        self.assertEqual(1, metrics.planned_wait_count)
        self.assertEqual(1, len(metrics.planned_wait_events))
        event = metrics.planned_wait_events[0]
        self.assertEqual(3.0, event["duration"])
        self.assertEqual("resumed", event["outcome"])
        self.assertEqual(11, event["plan_epoch"])
        self.assertEqual("joint_grid", event["plan_source"])
        self.assertEqual(0, metrics.unplanned_stop_count)

        metrics.record_step(13.0, moving, {}, {})
        self.assertEqual(1, metrics.unplanned_stop_count)

    def test_planned_wait_deadline_expiration_is_audited(self):
        metrics = MetricsCollector("test", "test", 1)
        waiting = {1: {
            "state": RobotState.EN_ROUTE_PICKUP,
            "position": (0.0, 0.0), "planned_wait": True,
            "wait_reason": "joint_window_endpoint", "wait_deadline": 2.0,
        }}
        resumed = {1: dict(waiting[1], planned_wait=False)}
        metrics.record_step(0.0, waiting, {}, {})
        metrics.record_step(3.0, resumed, {}, {})
        self.assertEqual("deadline_expired",
                         metrics.planned_wait_events[0]["outcome"])
        summary = metrics.compute_final_metrics({}, {}, {}, 3.0)
        self.assertEqual(1, summary["planned_wait_episodes"])
        self.assertEqual(3.0, summary["planned_wait_total_seconds"])

    def test_adjacent_waits_from_different_epochs_are_separate_episodes(self):
        metrics = MetricsCollector("test", "test", 1)
        first = {1: {
            "state": RobotState.EN_ROUTE_PICKUP,
            "position": (0.0, 0.0), "planned_wait": True,
            "wait_reason": "joint_epoch_barrier", "wait_deadline": 5.0,
            "task_id": 4, "plan_epoch": 10, "plan_source": "joint_grid",
            "path_version": 20,
        }}
        second = {1: dict(
            first[1], plan_epoch=11, path_version=21,
            wait_reason="joint_slot_deadline", wait_deadline=8.0)}
        metrics.record_step(0.0, first, {}, {})
        metrics.record_step(2.0, second, {}, {})

        self.assertEqual(2, metrics.planned_wait_count)
        self.assertEqual(1, len(metrics.planned_wait_events))
        self.assertEqual("superseded",
                         metrics.planned_wait_events[0]["outcome"])
        self.assertEqual(10, metrics.planned_wait_events[0]["plan_epoch"])
        self.assertEqual(11,
                         metrics._active_planned_waits[1]["plan_epoch"])
        self.assertEqual(2.0,
                         metrics.planned_wait_events[0]["duration"])

    def test_charging_trip_is_not_counted_as_task_stop(self):
        metrics = MetricsCollector("test", "test", 1)
        state = {1: {
            "state": RobotState.RETURNING_TO_CHARGE,
            "position": (1.0, 1.0), "planned_wait": False,
            "task_id": None,
        }}
        metrics.record_step(0.0, state, {}, {})
        metrics.record_step(20.0, state, {}, {})
        self.assertEqual(0, metrics.unplanned_stop_count)

    def test_single_robot_step_serializes_as_strict_json(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_step(0.0, {1: {
            "state": RobotState.IDLE, "position": (0.0, 0.0),
        }}, {}, {})
        payload = json.dumps(
            [record.__dict__ for record in metrics.step_records],
            allow_nan=False)
        self.assertIsNone(json.loads(payload)[0]["min_pair_distance"])

    def test_step_snapshot_preserves_low_rate_controller_diagnostics(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_step(4.0, {1: {
            "state": RobotState.EN_ROUTE_PICKUP,
            "position": (0.0, 0.0),
            "controller_motion_state": "zero",
            "controller_linear_speed": 0.0,
            "controller_angular_speed": 0.0,
            "controller_measured_left_wheel_speed": 1.25,
            "controller_measured_right_wheel_speed": -0.75,
            "controller_stop_reason": "emergency",
            "controller_local_risk_level": "emergency",
            "controller_status_sample_time": 3.99,
            "heading": 1.25,
            "controller_target": (0.5, 0.25),
            "controller_target_distance": 0.559,
            "controller_reported_target": (0.75, 0.25),
            "controller_reported_target_distance": 0.791,
            "controller_reported_waypoint_count": 6,
            "controller_target_mismatch": True,
            "controller_paused_until": 4.5,
            "controller_joint_wait_until": 5.0,
            "controller_joint_wait_reason": "joint_window_endpoint",
            "dispatch_not_before": 5.5,
            "hold_until": 6.0,
        }}, {}, {})
        state = metrics.step_records[0].robot_task_states["1"]
        self.assertEqual("zero", state["controller_motion_state"])
        self.assertEqual("emergency", state["controller_stop_reason"])
        self.assertEqual(3.99, state["controller_status_sample_time"])
        self.assertEqual(1.25, state["controller_measured_left_wheel_speed"])
        self.assertEqual(-0.75, state["controller_measured_right_wheel_speed"])
        self.assertEqual([0.5, 0.25], state["controller_target"])
        self.assertEqual([0.75, 0.25], state["controller_reported_target"])
        self.assertEqual(0.791,
                         state["controller_reported_target_distance"])
        self.assertEqual(6, state["controller_reported_waypoint_count"])
        self.assertTrue(state["controller_target_mismatch"])
        self.assertEqual(1.25, state["heading"])
        self.assertEqual(4.5, state["controller_paused_until"])
        self.assertEqual(5.0, state["controller_joint_wait_until"])
        self.assertEqual("joint_window_endpoint",
                         state["controller_joint_wait_reason"])
        self.assertEqual(5.5, state["dispatch_not_before"])
        self.assertEqual(6.0, state["hold_until"])

    def test_replan_and_escape_have_independent_cumulative_audit(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_replan(
            3.0, 1, "predictor", 7, "succeeded", True)
        metrics.record_escape(4.0, 1, "_command_reverse", (2.0, 3.0))
        summary = metrics.compute_final_metrics({}, {}, {}, 10.0)
        self.assertEqual(1, summary["audited_replans"])
        self.assertEqual(1, summary["physical_escapes"])

    def test_cross_source_route_override_is_audited(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_route_dispatch(1, 1, 10, "assignment", 3, 5.0)
        metrics.record_route_dispatch(
            1, 2, 11, "proactive_scan", 4, 5.5)
        self.assertEqual(2, len(metrics.route_dispatch_events))
        self.assertEqual(1, len(metrics.route_override_events))
        self.assertEqual("assignment",
                         metrics.route_override_events[0]["previous_source"])

    def test_route_dispatch_preserves_activation_diagnostics(self):
        metrics = MetricsCollector("test", "test", 1)
        diagnostics = {
            'planning_position': (0.0, 0.0),
            'first_waypoint': (0.25, 0.0),
            'activation_to_first_m': 0.3,
        }
        metrics.record_route_dispatch(
            1, 2, 3, 'joint_grid_transaction', 4, 5.0,
            diagnostics=diagnostics)
        self.assertEqual(
            diagnostics,
            metrics.route_dispatch_events[-1]['diagnostics'])

    def test_unauthorized_route_write_has_independent_audit(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_unauthorized_route_write(
            2.0, 1, '_command_reverse', 7, 11)
        summary = metrics.compute_final_metrics({}, {}, {}, 3.0)
        self.assertEqual(1, summary['unauthorized_route_writes'])
        self.assertEqual('rejected',
                         metrics.unauthorized_route_write_events[0]['decision'])

    def test_same_source_rapid_route_override_is_audited(self):
        metrics = MetricsCollector("test", "test", 1)
        metrics.record_route_dispatch(1, 1, 10, "predictor", 3, 5.0)
        metrics.record_route_dispatch(1, 2, 11, "predictor", 4, 5.5)
        self.assertEqual(1, len(metrics.route_override_events))

    def test_conflict_scan_tracks_one_episode_until_resolution(self):
        metrics = MetricsCollector("test", "test", 2)
        metrics.record_conflict_scan([(1, 2, 3.0, 0.6)], 1.0)
        metrics.record_conflict_scan([(1, 2, 2.5, 0.5)], 1.25)
        self.assertEqual(1, len(metrics.conflict_events))
        metrics.record_conflict_scan([], 1.5)
        self.assertEqual("resolved", metrics.conflict_events[0]["status"])

    def test_all_simultaneous_distance_violations_are_recorded(self):
        metrics = MetricsCollector("test", "test", 3)
        states = {
            1: {"state": "active", "position": (0.0, 0.0)},
            2: {"state": "active", "position": (0.4, 0.0)},
            3: {"state": "active", "position": (0.8, 0.0)},
        }
        metrics.record_step(1.0, states, {}, {})
        self.assertEqual(2, len(metrics.safety_events))

    def test_safety_kpis_survive_step_record_trimming(self):
        metrics = MetricsCollector("test", "test", 2)
        states = {
            1: {"state": "active", "position": (0.0, 0.0)},
            2: {"state": "active", "position": (0.4, 0.0)},
        }
        metrics.record_step(1.0, states, {}, {})
        metrics.step_records.clear()
        summary = metrics.compute_final_metrics({}, {}, {}, 2.0)
        self.assertAlmostEqual(0.4, summary["min_pair_distance"])
        self.assertEqual(1, summary["pair_distance_violations"])

    def test_distance_violation_creates_auditable_safety_event(self):
        metrics = MetricsCollector("test", "test", 2)
        states = {
            1: {"state": "active", "position": (0.0, 0.0), "battery": 90},
            2: {"state": "active", "position": (0.49, 0.0), "battery": 90},
        }
        metrics.record_step(12.0, states, {}, {})
        self.assertEqual(1, len(metrics.safety_events))
        self.assertEqual("pair_distance_violation",
                         metrics.safety_events[0]["event_type"])
        self.assertAlmostEqual(0.49,
                               metrics.safety_events[0]["minimum_distance"])

    def test_liveness_metrics_report_completion_plateaus(self):
        metrics = MetricsCollector("test", "test", 2)
        metrics.task_completions = [
            {"completion_time": 20.0, "completion_duration": 10.0,
             "waiting_time": 1.0},
            {"completion_time": 50.0, "completion_duration": 20.0,
             "waiting_time": 2.0},
        ]

        class Robot:
            idle_time = 0.0
            total_distance = 0.0

            def __init__(self, completed):
                self.tasks_completed = completed

        summary = metrics.compute_final_metrics(
            {1: Robot(2), 2: Robot(0)}, {"completed": 2}, {}, 100.0)
        self.assertEqual(50.0, summary["longest_completion_plateau_seconds"])
        self.assertEqual(50.0, summary["final_completion_plateau_seconds"])
        self.assertEqual(1, summary["robots_without_completed_tasks"])


if __name__ == "__main__":
    unittest.main()
def test_terminal_handoff_events_are_auditable_in_summary():
    metrics = MetricsCollector("test", "FCFS", 2)
    metrics.record_terminal_handoff(
        "service_complete", 1.0, 1, "WS1", 3, owner_id=1)
    metrics.record_terminal_handoff(
        "inbound_denied", 1.1, 2, "WS1", 3, owner_id=1)
    metrics.record_terminal_handoff(
        "physically_clear", 2.0, 1, "WS1", 3, owner_id=1)
    metrics.record_terminal_handoff(
        "inbound_granted", 2.1, 2, "WS1", 0)
    summary = metrics.compute_final_metrics({}, {}, {}, 10.0)
    assert summary["terminal_service_complete"] == 1
    assert summary["terminal_physically_clear"] == 1
    assert summary["terminal_inbound_denied"] == 1
    assert summary["terminal_inbound_granted"] == 1
    assert len(metrics.terminal_handoff_events) == 4


def test_terminal_egress_retry_events_preserve_reason_and_attempt():
    metrics = MetricsCollector('C', 'FCFS', 1)
    metrics.record_terminal_handoff(
        'egress_retry_failed', 10.0, 1, 'WS1', 3,
        owner_id=1, reason='no_safe_path', attempt=2)
    metrics.record_terminal_handoff(
        'egress_retry_dispatched', 11.0, 1, 'WS1', 3,
        owner_id=1, attempt=3)
    summary = metrics.compute_final_metrics({}, {}, {}, 12.0)
    assert summary['terminal_egress_retry_failed'] == 1
    assert summary['terminal_egress_retry_dispatched'] == 1
    assert metrics.terminal_handoff_events[0]['reason'] == 'no_safe_path'
    assert metrics.terminal_handoff_events[0]['attempt'] == 2


def test_runtime_wall_metrics_report_percentiles_and_samples():
    metrics = MetricsCollector('C', 'FCFS', 1)
    for seconds in (0.001, 0.002, 0.003, 0.004):
        metrics.record_supervisor_step_wall(seconds)
    for seconds in (0.010, 0.020, 0.030):
        metrics.record_joint_planning_wall(seconds)
    summary = metrics.compute_final_metrics({}, {}, {}, 10.0)
    assert summary['supervisor_step_samples'] == 4
    assert summary['supervisor_step_wall_p50_ms'] == 2.0
    assert summary['supervisor_step_wall_p99_ms'] == 4.0
    assert summary['joint_planning_samples'] == 3
    assert summary['joint_planning_wall_p95_ms'] == 30.0


def test_route_sources_and_override_transitions_reconcile():
    metrics = MetricsCollector('C', 'FCFS', 1)
    metrics.record_route_dispatch(1, 1, 1, 'joint', 2, 1.0)
    metrics.record_route_dispatch(1, 2, 2, 'egress', 1, 1.5)
    metrics.record_route_dispatch(1, 3, 3, 'joint', 2, 3.0)
    summary = metrics.compute_final_metrics({}, {}, {}, 10.0)
    assert summary['route_dispatches'] == 3
    assert summary['route_dispatches_by_source'] == {
        'joint': 2, 'egress': 1}
    assert summary['rapid_route_overrides'] == 1
    assert summary['route_overrides_by_transition'] == {
        'joint->egress': 1}


def test_communication_metrics_reconcile_counts_bytes_and_latency():
    metrics = MetricsCollector('C', 'FCFS', 1)
    metrics.record_communication('peer', 0.001, messages=8, byte_count=800)
    metrics.record_communication('peer', 0.003, messages=8, byte_count=900)
    summary = metrics.compute_final_metrics({}, {}, {}, 10.0)
    peer = summary['communication_metrics']['peer']
    assert peer['samples'] == 2
    assert peer['messages'] == 16
    assert peer['bytes'] == 1700
    assert peer['wall_p50_ms'] == 1.0
    assert peer['wall_p99_ms'] == 3.0


def test_joint_plan_request_provenance_reconciles():
    metrics = MetricsCollector('C', 'FCFS', 1)
    metrics.record_joint_plan_request('joint_runtime_watchdog', 'LIVENESS')
    metrics.record_joint_plan_request('joint_runtime_watchdog', 'LIVENESS')
    metrics.record_joint_plan_request('predicted_collision', 'SAFETY')
    summary = metrics.compute_final_metrics({}, {}, {}, 10.0)
    assert summary['joint_plan_requests_by_reason'] == {
        'joint_runtime_watchdog': 2, 'predicted_collision': 1}
    assert summary['joint_plan_requests_by_class'] == {
        'LIVENESS': 2, 'SAFETY': 1}
