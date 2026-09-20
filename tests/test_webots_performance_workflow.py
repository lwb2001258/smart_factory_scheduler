import importlib.util
from pathlib import Path


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "run_webots_performance_workflow.py")
SPEC = importlib.util.spec_from_file_location("performance_workflow", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def metrics(**overrides):
    values = {
        "artifacts": {"html": "recording.html"},
        "pair_distance_violations": 0,
        "sim_to_wall_ratio": 1.2,
        "duration": 600.0,
        "recorded_sim_duration": 600.016,
        "throughput_per_minute": 1.5,
        "longest_completion_plateau_seconds": 80.0,
        "final_completion_plateau_seconds": 45.0,
        "robots_without_completed_tasks": 2,
        "reservation_deadline_stop_episodes": 0,
        "active_zero_speed_ratio": 0.10,
        "mean_active_linear_speed_mps": 0.14,
        "motion_state_transitions_per_robot_minute": 1.0,
        "shield_speed_changes_per_robot_minute": 10.0,
        "abnormal_mass_stop_seconds": 2.0,
        "unexplained_active_intervals": 4,
        "rotating_in_place_intervals": 3,
        "unplanned_task_stops": 0,
        "clear_motion_duty_cycle": 0.98,
        "tasks_completed": 5,
        "min_pair_distance": 0.60,
    }
    values.update(overrides)
    return values


def test_gate_requires_recording_safety_and_simulator_ratio():
    report = MODULE.evaluate_gates(metrics(), None, 1.0)
    assert report["passed"]
    assert not MODULE.evaluate_gates(
        metrics(sim_to_wall_ratio=0.99), None, 1.0)["passed"]
    assert not MODULE.evaluate_gates(
        metrics(pair_distance_violations=1), None, 1.0)["passed"]
    assert not MODULE.evaluate_gates(
        metrics(artifacts={}), None, 1.0)["passed"]


def test_gate_requires_long_recording_and_absolute_fleet_efficiency():
    for name, value in (
        ("duration", 299.0),
        ("recorded_sim_duration", 299.0),
        ("throughput_per_minute", 0.99),
        ("longest_completion_plateau_seconds", 150.01),
        ("final_completion_plateau_seconds", 120.01),
        ("robots_without_completed_tasks", 5),
        ("reservation_deadline_stop_episodes", 3),
        ("active_zero_speed_ratio", 0.181),
        ("motion_state_transitions_per_robot_minute", 2.51),
        ("shield_speed_changes_per_robot_minute", 30.01),
    ):
        assert not MODULE.evaluate_gates(
            metrics(**{name: value}), None, 1.0)["passed"], name

    assert not MODULE.evaluate_gates(
        metrics(duration=600.0, recorded_sim_duration=599.0),
        None, 1.0)["passed"]


def test_duration_validator_enforces_five_to_ten_minute_window():
    assert MODULE.validate_duration(300.0) == 300.0
    assert MODULE.validate_duration(600.0) == 600.0
    for duration in (299.999, 600.001, float("nan")):
        try:
            MODULE.validate_duration(duration)
            assert False, duration
        except RuntimeError:
            pass


def test_reservation_deadline_counter_counts_transitions_not_samples():
    def state(reason):
        return {"controller_stop_reason": reason}

    result = {"time_series": [
        {"robot_task_states": {"1": state("moving"),
                               "2": state("moving")}},
        {"robot_task_states": {"1": state(
            "emergency:reservation_deadline_expired"),
                               "2": state("moving")}},
        {"robot_task_states": {"1": state(
            "emergency:reservation_deadline_expired"),
                               "2": state(
            "emergency:reservation_deadline_expired")}},
        {"robot_task_states": {"1": state("moving"),
                               "2": state("moving")}},
        {"robot_task_states": {"1": state(
            "emergency:reservation_deadline_expired")}},
    ]}
    assert MODULE._reservation_deadline_stop_episodes(result) == 3


def test_motion_stability_metrics_count_active_stop_start_transitions():
    def sample(now, first_state, second_state="idle"):
        def robot(state):
            moving = state == "moving"
            return {
                "state": ("en_route_pickup" if state != "idle" else
                          "idle"),
                "controller_motion_state": state,
                "controller_linear_speed": 0.2 if moving else 0.0,
            }
        return {"sim_time": now, "robot_task_states": {
            "1": robot(first_state), "2": robot(second_state)}}

    result = {
        "experiment_info": {"sim_duration": 60.0, "num_robots": 2},
        "time_series": [
            sample(0.0, "moving"), sample(20.0, "zero"),
            sample(40.0, "moving"), sample(60.0, "idle")],
        "safety_events": [
            {"event_type": "dynamic_safety_shield",
             "decision": "caution:speed_scale=0.6"},
            {"event_type": "dynamic_safety_shield", "decision": "rejected"},
        ],
    }
    values = MODULE._motion_stability_metrics(result)
    assert values["motion_state_transitions"] == 2.0
    assert values["active_zero_speed_ratio"] == 1.0 / 3.0
    assert values["mean_active_linear_speed_mps"] == 0.4 / 3.0
    assert values["motion_state_transitions_per_robot_minute"] == 2.0
    assert values["shield_speed_changes_per_robot_minute"] == 0.5


def test_gate_rejects_any_robot_efficiency_regression():
    baseline = metrics()
    assert MODULE.evaluate_gates(
        metrics(), baseline, 1.0, "workflow")["passed"]
    for name, worse in (
        ("abnormal_mass_stop_seconds", 2.8),
        ("unexplained_active_intervals", 10),
        ("rotating_in_place_intervals", 9),
        ("unplanned_task_stops", 1),
        ("clear_motion_duty_cycle", 0.95),
        ("tasks_completed", 4),
        ("min_pair_distance", 0.49),
        ("throughput_per_minute", 0.99),
        ("longest_completion_plateau_seconds", 151.0),
        ("final_completion_plateau_seconds", 121.0),
        ("robots_without_completed_tasks", 5),
        ("reservation_deadline_stop_episodes", 3),
        ("active_zero_speed_ratio", 0.181),
        ("motion_state_transitions_per_robot_minute", 2.51),
        ("shield_speed_changes_per_robot_minute", 30.01),
    ):
        assert not MODULE.evaluate_gates(
            metrics(**{name: worse}), baseline, 1.0, "workflow")["passed"], name


def test_behavior_change_must_have_material_improvement():
    baseline = metrics()
    unchanged = MODULE.evaluate_gates(metrics(), baseline, 1.0, "behavior")
    assert not unchanged["passed"]
    improved = MODULE.evaluate_gates(
        metrics(abnormal_mass_stop_seconds=1.2), baseline, 1.0, "behavior")
    assert improved["passed"]
    assert improved["material_improvements"]["abnormal_mass_stop"]


def test_baseline_comparison_requires_same_recording_duration():
    baseline = metrics(duration=300.0)
    report = MODULE.evaluate_gates(
        metrics(duration=600.0), baseline, 1.0, "workflow")
    assert not report["passed"]
    assert not report["checks"]["baseline_duration_matches"]


def test_reviews_are_mandatory_and_must_explicitly_pass(tmp_path):
    missing = tmp_path / "missing.md"
    try:
        MODULE.validate_review(missing, "R1")
        assert False, "missing review must fail"
    except RuntimeError:
        pass
    short = tmp_path / "short.md"
    short.write_text("Verdict: PASS", encoding="utf-8")
    try:
        MODULE.validate_review(short, "R1")
        assert False, "empty ceremonial review must fail"
    except RuntimeError:
        pass
    review = tmp_path / "review.md"
    review.write_text(
        "Scope: safety, state machine, performance and rollback. "
        "No blocking I/O or protocol changes were found. Verdict: PASS",
        encoding="utf-8")
    assert "Verdict: PASS" in MODULE.validate_review(review, "R2")
