import json
from pathlib import Path

from scripts.analyze_webots_stops import (
    analyze, classify, concurrent_stop_summary, detect_stops,
)


def test_detects_stationary_interval_and_motion_boundary():
    animation = {"frames": [
        {"time": 0, "updates": [{"id": 10, "translation": "0 0 0"}]},
        {"time": 2500, "updates": [{"id": 10, "translation": "0.01 0 0"}]},
        {"time": 3000, "updates": [{"id": 10, "translation": "0.2 0 0"}]},
    ]}
    stops = detect_stops(animation, {10: 1}, 2.0, 0.03)
    assert len(stops) == 1
    assert stops[0]["robot_id"] == 1
    assert stops[0]["duration"] == 2.5
    assert stops[0]["pose_stationary"] is False
    assert stops[0]["max_translation_change_m"] == 0.01


def test_rotation_only_updates_distinguish_turning_from_complete_stop():
    animation = {"frames": [
        {"time": 0, "updates": [{
            "id": 10, "translation": "0 0 0", "rotation": "0 0 1 0"}]},
        {"time": 1000, "updates": [{"id": 10, "rotation": "0 0 1 0.4"}]},
        {"time": 2500, "updates": [{"id": 10, "rotation": "0 0 1 1.2"}]},
        {"time": 3000, "updates": [{"id": 10, "translation": "0.2 0 0"}]},
    ]}
    stops = detect_stops(animation, {10: 1}, 2.0, 0.03)
    assert len(stops) == 1
    assert stops[0]["rotated_in_place"] is True
    assert stops[0]["pose_stationary"] is False
    assert stops[0]["max_yaw_change_rad"] == 1.2


def test_pose_stationary_requires_sub_threshold_translation():
    animation = {"frames": [
        {"time": 0, "updates": [{"id": 10, "translation": "0 0 0"}]},
        {"time": 2500, "updates": [{"id": 10, "translation": "0.003 0 0"}]},
        {"time": 3000, "updates": [{"id": 10, "translation": "0.2 0 0"}]},
    ]}
    stops = detect_stops(animation, {10: 1}, 2.0, 0.03)
    assert len(stops) == 1
    assert stops[0]["ended_at"] == 2.5
    assert stops[0]["pose_stationary"] is True
    assert stops[0]["max_translation_change_m"] == 0.003


def test_correlates_active_stop_with_unplanned_event(tmp_path: Path):
    animation_path = tmp_path / "run.json"
    x3d_path = tmp_path / "run.x3d"
    experiment_path = tmp_path / "experiment.json"
    animation_path.write_text(json.dumps({"frames": [
        {"time": 0, "updates": [{"id": 10, "translation": "0 0 0"}]},
        {"time": 3000, "updates": [{"id": 10, "translation": "0 0 0"}]},
    ]}), encoding="utf-8")
    x3d_path.write_text("<Robot id='n10' name='robot_1'>", encoding="utf-8")
    experiment_path.write_text(json.dumps({
        "time_series": [{"sim_time": 2, "robot_task_states": {"1": {
            "state": "en_route_pickup", "task_id": 7, "planned_wait": False}}}],
        "unplanned_stop_events": [{"sim_time": 2.5, "robot_id": 1}],
    }), encoding="utf-8")
    report = analyze(animation_path, experiment_path, x3d_path, 2.0, 0.03)
    assert report["summary"]["active_stop_intervals"] == 1
    assert report["stops"][0]["classification"] == "unplanned_stop"
    assert report["stops"][0]["evidence"][0]["_event_stream"] == "unplanned_stop_events"


def test_open_interval_event_and_returning_robot_are_supported(tmp_path: Path):
    animation_path = tmp_path / "run.json"
    x3d_path = tmp_path / "run.x3d"
    experiment_path = tmp_path / "experiment.json"
    animation_path.write_text(json.dumps({"frames": [
        {"time": 0, "updates": [{"id": 10, "translation": "0 0 0"}]},
        {"time": 3000, "updates": [{"id": 10, "translation": "0 0 0"}]},
    ]}), encoding="utf-8")
    x3d_path.write_text("<Robot id='n10' name='robot_1'>", encoding="utf-8")
    experiment_path.write_text(json.dumps({
        "time_series": [{"sim_time": 2, "robot_task_states": {"1": {
            "state": "returning_home", "task_id": None, "planned_wait": True,
            "wait_reason": "joint_window_endpoint"}}}],
        "planned_wait_events": [{"robot_id": 1, "started_at": 1, "last_seen_at": 3, "ended_at": None}],
    }), encoding="utf-8")
    report = analyze(animation_path, experiment_path, x3d_path, 2.0, 0.03)
    assert report["summary"]["active_stop_intervals"] == 1
    assert report["stops"][0]["classification"] == "planned_wait:joint_window_endpoint"


def test_concurrent_mass_stop_summary_counts_only_motion_expected():
    stops = [
        {"robot_id": 1, "started_at": 1, "ended_at": 5, "motion_expected": True},
        {"robot_id": 2, "started_at": 2, "ended_at": 6, "motion_expected": True},
        {"robot_id": 3, "started_at": 3, "ended_at": 4, "motion_expected": True},
        {"robot_id": 4, "started_at": 0, "ended_at": 9, "motion_expected": False},
    ]
    summary = concurrent_stop_summary(stops, threshold=3)
    assert summary["max_concurrent_motion_expected_stops"] == 3
    assert summary["mass_stop_seconds"] == 1.0
    assert summary["mass_stop_intervals"][0]["robot_ids"] == [1, 2, 3]


def test_abnormal_concurrency_excludes_validated_waits_and_recovery():
    stops = [
        {"robot_id": 1, "started_at": 1, "ended_at": 5,
         "motion_expected": True, "classification": "unexplained_active_stop"},
        {"robot_id": 2, "started_at": 1, "ended_at": 5,
         "motion_expected": True, "classification": "unplanned_stop"},
        {"robot_id": 3, "started_at": 1, "ended_at": 5,
         "motion_expected": True, "classification": "planned_wait:controller_hold"},
    ]
    summary = concurrent_stop_summary(stops, threshold=3, abnormal_only=True)
    assert summary["max_concurrent_motion_expected_stops"] == 2
    assert summary["mass_stop_seconds"] == 0


def test_pose_stationary_concurrency_excludes_turning_robot():
    stops = [
        {"robot_id": 1, "started_at": 1, "ended_at": 5,
         "motion_expected": True, "classification": "unplanned_stop",
         "pose_stationary": True},
        {"robot_id": 2, "started_at": 1, "ended_at": 5,
         "motion_expected": True, "classification": "unplanned_stop",
         "pose_stationary": True},
        {"robot_id": 3, "started_at": 1, "ended_at": 5,
         "motion_expected": True, "classification": "unplanned_stop",
         "pose_stationary": False},
    ]
    summary = concurrent_stop_summary(
        stops, threshold=3, abnormal_only=True, pose_stationary_only=True)
    assert summary["max_concurrent_motion_expected_stops"] == 2
    assert summary["mass_stop_seconds"] == 0


def test_classifies_commanded_motion_without_physical_progress():
    state = {
        "state": "en_route_pickup",
        "controller_motion_state": "moving",
        "controller_stop_reason": "moving",
    }
    assert classify(state, [], {"pose_stationary": True}) == (
        "commanded_without_pose_progress")


def test_classifies_controller_zero_reasons_without_calling_them_unexplained():
    state = {
        "state": "en_route_delivery",
        "controller_motion_state": "zero",
        "controller_stop_reason": "advance_hold:swept_occupancy:5",
    }
    assert classify(state, [], {"pose_stationary": True}) == (
        "controller_advance_hold:swept_occupancy:5")
    state["controller_stop_reason"] = "emergency"
    assert classify(state, [], {"pose_stationary": True}) == (
        "controller_emergency_stop")
