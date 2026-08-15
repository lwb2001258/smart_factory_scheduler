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
