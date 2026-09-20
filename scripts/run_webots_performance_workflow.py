"""Run one reproducible robot-efficiency and Webots-performance gate.

The same 5–10 minute Webots run provides the recording, robot-efficiency
evidence, and simulator wall-time evidence. Every artifact is selected from
files created after launch and copied into one step directory.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
MIN_RECORDING_DURATION_SECONDS = 300.0
MAX_RECORDING_DURATION_SECONDS = 600.0
sys.path.insert(0, str(ROOT / "scripts"))

from analyze_webots_stops import analyze  # noqa: E402
from run_experiments import run_single_experiment  # noqa: E402


def _newest(pattern: str, launched_at: float) -> Path:
    candidates = [
        path for path in RESULTS.glob(pattern)
        if path.stat().st_mtime >= launched_at - 1.0
    ]
    if not candidates:
        raise RuntimeError(f"no fresh artifact matching {pattern}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _copy_animation(stem: Path, output_dir: Path) -> dict[str, str]:
    copied = {}
    for suffix in (".html", ".json", ".x3d", ".css"):
        source = stem.with_suffix(suffix)
        if not source.is_file():
            raise RuntimeError(f"recording artifact missing: {source}")
        target = output_dir / source.name
        shutil.copy2(source, target)
        copied[suffix[1:]] = target.name
    return copied


def _metric(result: dict[str, Any], name: str, default: Any = None) -> Any:
    return result.get("summary_metrics", {}).get(name, default)


def _load_baseline(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    return data.get("metrics", data)


def _reservation_deadline_stop_episodes(result: dict[str, Any]) -> int:
    """Count transitions into reservation-deadline stops per robot.

    Counting episodes rather than status samples prevents a long stop from
    looking worse merely because telemetry was sampled more frequently.
    """
    active: dict[int, bool] = {}
    episodes = 0
    for sample in result.get("time_series", ()):
        for raw_id, state in (sample.get("robot_task_states", {}) or {}).items():
            robot_id = int(raw_id)
            stopped = (
                state.get("controller_stop_reason") ==
                "emergency:reservation_deadline_expired")
            if stopped and not active.get(robot_id, False):
                episodes += 1
            active[robot_id] = stopped
    return episodes


def _motion_stability_metrics(result: dict[str, Any]) -> dict[str, float]:
    """Derive stop/start and speed-command churn from recorded telemetry."""
    series = result.get("time_series", ()) or ()
    active_states = {
        "en_route_pickup", "en_route_delivery", "returning_home",
        "returning_to_charge",
    }
    last_moving: dict[int, bool] = {}
    robot_ids: set[int] = set()
    transitions = 0
    active_samples = 0
    zero_samples = 0
    linear_speed_sum = 0.0
    for sample in series:
        states = sample.get("robot_task_states", {}) or {}
        seen = set()
        for raw_id, state in states.items():
            robot_id = int(raw_id)
            seen.add(robot_id)
            if state.get("state") not in active_states:
                last_moving.pop(robot_id, None)
                continue
            robot_ids.add(robot_id)
            active_samples += 1
            moving = state.get("controller_motion_state") in (
                "moving", "turning")
            if not moving:
                zero_samples += 1
            linear_speed_sum += abs(float(
                state.get("controller_linear_speed", 0.0)))
            if robot_id in last_moving and last_moving[robot_id] != moving:
                transitions += 1
            last_moving[robot_id] = moving
        for robot_id in tuple(last_moving):
            if robot_id not in seen:
                last_moving.pop(robot_id, None)

    if len(series) >= 2:
        window_seconds = max(
            0.0, float(series[-1].get("sim_time", 0.0)) -
            float(series[0].get("sim_time", 0.0)))
    else:
        window_seconds = 0.0
    robot_minutes = len(robot_ids) * window_seconds / 60.0
    shield_changes = sum(
        1 for event in (result.get("safety_events", ()) or ())
        if event.get("event_type") == "dynamic_safety_shield" and
        event.get("decision") != "rejected")
    run_duration = float(result.get("experiment_info", {}).get(
        "sim_duration", 0.0))
    num_robots = int(result.get("experiment_info", {}).get(
        "num_robots", len(robot_ids) or 1))
    fleet_minutes = num_robots * run_duration / 60.0
    return {
        "active_zero_speed_ratio": (
            zero_samples / active_samples if active_samples else 1.0),
        "mean_active_linear_speed_mps": (
            linear_speed_sum / active_samples if active_samples else 0.0),
        "motion_state_transitions": float(transitions),
        "motion_state_transitions_per_robot_minute": (
            transitions / robot_minutes if robot_minutes > 0.0 else
            math.inf),
        "shield_speed_changes_per_robot_minute": (
            shield_changes / fleet_minutes if fleet_minutes > 0.0 else
            math.inf),
    }


def validate_duration(duration: float) -> float:
    """Require recordings long enough to expose fleet-wide liveness faults."""
    if not math.isfinite(duration) or not (
            MIN_RECORDING_DURATION_SECONDS <= duration <=
            MAX_RECORDING_DURATION_SECONDS):
        raise RuntimeError(
            "Webots validation duration must be between 300 and 600 seconds")
    return duration


def validate_review(path: Path, label: str) -> str:
    """Require an explicit, non-empty PASS review before launching Webots."""
    if not path.is_file():
        raise RuntimeError(f"{label} review file missing: {path}")
    text = path.read_text(encoding="utf-8")
    normalized = " ".join(text.upper().split())
    if len(text.strip()) < 80 or "VERDICT: PASS" not in normalized:
        raise RuntimeError(
            f"{label} must contain substantive findings and 'Verdict: PASS'")
    return text


def evaluate_gates(metrics: dict[str, Any], baseline: dict[str, Any] | None,
                   minimum_ratio: float, change_kind: str = "behavior",
                   minimum_throughput: float = 1.0,
                   maximum_longest_plateau: float = 150.0,
                   maximum_final_plateau: float = 120.0,
                   maximum_robots_without_completions: int = 4,
                   maximum_reservation_deadline_episodes: int = 2,
                   maximum_active_zero_speed_ratio: float = 0.18,
                   maximum_motion_transitions_per_minute: float = 2.5,
                   maximum_shield_changes_per_minute: float = 30.0,
                   require_long_duration: bool = True) -> dict[str, Any]:
    checks = {
        "recording_exists": bool(metrics["artifacts"].get("html")),
        "pair_distance_violations_zero":
            metrics["pair_distance_violations"] == 0,
        "sim_to_wall_ratio": metrics["sim_to_wall_ratio"] >= minimum_ratio,
        "recording_duration_5_to_10_minutes": (
            not require_long_duration or
            MIN_RECORDING_DURATION_SECONDS <= metrics["duration"] <=
            MAX_RECORDING_DURATION_SECONDS and
            MIN_RECORDING_DURATION_SECONDS <=
            metrics["recorded_sim_duration"] <=
            MAX_RECORDING_DURATION_SECONDS + 1.0),
        "recording_reached_requested_duration": (
            metrics["recorded_sim_duration"] + 0.05 >=
            metrics["duration"]),
        "throughput_absolute_minimum": (
            metrics["throughput_per_minute"] >= minimum_throughput),
        "longest_completion_plateau_bounded": (
            metrics["longest_completion_plateau_seconds"] <=
            maximum_longest_plateau),
        "final_completion_plateau_bounded": (
            metrics["final_completion_plateau_seconds"] <=
            maximum_final_plateau),
        "fleet_task_participation": (
            metrics["robots_without_completed_tasks"] <=
            maximum_robots_without_completions),
        "reservation_deadline_stops_bounded": (
            metrics["reservation_deadline_stop_episodes"] <=
            maximum_reservation_deadline_episodes),
        "active_zero_speed_ratio_bounded": (
            metrics["active_zero_speed_ratio"] <=
            maximum_active_zero_speed_ratio),
        "motion_stop_start_churn_bounded": (
            metrics["motion_state_transitions_per_robot_minute"] <=
            maximum_motion_transitions_per_minute),
        "shield_speed_churn_bounded": (
            metrics["shield_speed_changes_per_robot_minute"] <=
            maximum_shield_changes_per_minute),
    }
    if baseline:
        checks["baseline_duration_matches"] = abs(
            metrics["duration"] - baseline.get("duration", -math.inf)) <= 1.0
        # Repeated same-seed Webots physics/search runs are not bit-identical.
        # These budgets were derived from three unchanged-code recordings and
        # prevent noise from passing a real behaviour change: behaviour steps
        # must additionally improve at least one primary KPI beyond the same
        # budget below.
        abnormal_budget = max(0.75, baseline.get(
            "abnormal_mass_stop_seconds", 0.0) * 0.15)
        interval_budget = max(5, math.ceil(baseline.get(
            "unexplained_active_intervals", 0) * 0.15))
        rotation_budget = max(5, math.ceil(baseline.get(
            "rotating_in_place_intervals", 0) * 0.15))
        checks["abnormal_mass_stop_seconds_within_noise"] = (
            metrics["abnormal_mass_stop_seconds"] <=
            baseline["abnormal_mass_stop_seconds"] + abnormal_budget)
        checks["unexplained_active_intervals_within_noise"] = (
            metrics["unexplained_active_intervals"] <=
            baseline["unexplained_active_intervals"] + interval_budget)
        checks["rotating_in_place_intervals_within_noise"] = (
            metrics["rotating_in_place_intervals"] <=
            baseline["rotating_in_place_intervals"] + rotation_budget)
        checks["unplanned_task_stops_not_worse"] = (
            metrics["unplanned_task_stops"] <=
            baseline["unplanned_task_stops"])
        if baseline.get("clear_motion_duty_cycle") is not None:
            checks["clear_motion_duty_cycle_within_noise"] = (
                metrics["clear_motion_duty_cycle"] >=
                baseline["clear_motion_duty_cycle"] - 0.02)
        if baseline.get("tasks_completed") is not None:
            checks["tasks_completed_not_worse"] = (
                metrics["tasks_completed"] >= baseline["tasks_completed"])
        checks["minimum_pair_distance_absolute"] = (
            metrics["min_pair_distance"] is not None and
            metrics["min_pair_distance"] >= 0.50)
        if "active_zero_speed_ratio" in baseline:
            checks["active_zero_speed_ratio_not_worse"] = (
                metrics["active_zero_speed_ratio"] <=
                baseline["active_zero_speed_ratio"] + 0.02)
        if "motion_state_transitions_per_robot_minute" in baseline:
            checks["motion_stop_start_churn_not_worse"] = (
                metrics["motion_state_transitions_per_robot_minute"] <=
                baseline["motion_state_transitions_per_robot_minute"] + 0.3)
        if "shield_speed_changes_per_robot_minute" in baseline:
            checks["shield_speed_churn_not_worse"] = (
                metrics["shield_speed_changes_per_robot_minute"] <=
                baseline["shield_speed_changes_per_robot_minute"] + 5.0)
        if change_kind == "behavior":
            improvements = {
                "tasks": metrics["tasks_completed"] >
                    baseline.get("tasks_completed", 0),
                "throughput": metrics["throughput_per_minute"] >=
                    baseline.get("throughput_per_minute", 0.0) + 0.10,
                "completion_plateau": metrics[
                    "longest_completion_plateau_seconds"] <=
                    baseline.get("longest_completion_plateau_seconds",
                                 math.inf) - 15.0,
                "fleet_participation": metrics[
                    "robots_without_completed_tasks"] < baseline.get(
                        "robots_without_completed_tasks", math.inf),
                "reservation_deadline_stops": metrics[
                    "reservation_deadline_stop_episodes"] < baseline.get(
                        "reservation_deadline_stop_episodes", math.inf),
                "active_zero_speed": metrics[
                    "active_zero_speed_ratio"] <= baseline.get(
                        "active_zero_speed_ratio", math.inf) - 0.03,
                "motion_stop_start_churn": metrics[
                    "motion_state_transitions_per_robot_minute"] <=
                    baseline.get(
                        "motion_state_transitions_per_robot_minute",
                        math.inf) - 0.5,
                "abnormal_mass_stop": metrics[
                    "abnormal_mass_stop_seconds"] <=
                    baseline["abnormal_mass_stop_seconds"] - abnormal_budget,
                "unexplained_active": metrics[
                    "unexplained_active_intervals"] <=
                    baseline["unexplained_active_intervals"] - interval_budget,
                "rotating_in_place": metrics[
                    "rotating_in_place_intervals"] <=
                    baseline["rotating_in_place_intervals"] - rotation_budget,
                "motion_duty": metrics["clear_motion_duty_cycle"] >=
                    baseline["clear_motion_duty_cycle"] + 0.01,
            }
            checks["material_robot_efficiency_improvement"] = any(
                improvements.values())
        else:
            improvements = {}
    return {"passed": all(checks.values()), "checks": checks,
            "material_improvements": improvements if baseline else {}}


def run_step(args: argparse.Namespace) -> dict[str, Any]:
    validate_duration(args.duration)
    output_dir = RESULTS / "performance_workflow" / args.step
    output_dir.mkdir(parents=True, exist_ok=False)
    review_r1 = validate_review(args.review_r1, "R1")
    review_r2 = validate_review(args.review_r2, "R2")
    (output_dir / "review_r1.md").write_text(review_r1, encoding="utf-8")
    (output_dir / "review_r2.md").write_text(review_r2, encoding="utf-8")
    previous = {
        name: os.environ.get(name) for name in (
            "SMART_FACTORY_SIM_DURATION",
            "SMART_FACTORY_DIAGNOSTIC_RECORDING",
        )
    }
    os.environ["SMART_FACTORY_SIM_DURATION"] = str(args.duration)
    try:
        os.environ["SMART_FACTORY_DIAGNOSTIC_RECORDING"] = "1"
        behaviour_started = time.time()
        behaviour = run_single_experiment(
            args.scenario, args.scheduler, args.seed, args.webots)
        if behaviour is None:
            raise RuntimeError("Webots recording run failed")
        behaviour_result = _newest(
            f"experiment_{args.scenario}_{args.scheduler}_*.json",
            behaviour_started)
        animation_json = _newest(
            f"webots_diagnostic_{args.scenario}_{args.scheduler}_*.json",
            behaviour_started)
        artifacts = _copy_animation(animation_json.with_suffix(""), output_dir)
        behaviour_copy = output_dir / "behaviour_result.json"
        shutil.copy2(behaviour_result, behaviour_copy)
        stop_report = analyze(
            output_dir / artifacts["json"], behaviour_copy,
            output_dir / artifacts["x3d"], args.min_stop_seconds,
            args.movement_epsilon_m)
        (output_dir / "stop_analysis.json").write_text(
            json.dumps(stop_report, indent=2), encoding="utf-8")

    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value

    summary = stop_report["summary"]
    continuity = _metric(behaviour, "motion_continuity", {}) or {}
    stability = _motion_stability_metrics(behaviour)
    execution = behaviour.get("execution_metrics", {})
    metrics = {
        "scenario": args.scenario,
        "scheduler": args.scheduler,
        "seed": args.seed,
        "duration": args.duration,
        "recorded_sim_duration": float(
            behaviour.get("experiment_info", {}).get(
                "sim_duration", 0.0)),
        "tasks_completed": _metric(behaviour, "total_tasks_completed", 0),
        "tasks_generated": _metric(behaviour, "total_tasks_generated", 0),
        "throughput_per_minute": _metric(
            behaviour, "throughput_per_minute", 0.0),
        "longest_completion_plateau_seconds": _metric(
            behaviour, "longest_completion_plateau_seconds", args.duration),
        "final_completion_plateau_seconds": _metric(
            behaviour, "final_completion_plateau_seconds", args.duration),
        "robots_without_completed_tasks": _metric(
            behaviour, "robots_without_completed_tasks", 8),
        "reservation_deadline_stop_episodes": (
            _reservation_deadline_stop_episodes(behaviour)),
        **stability,
        "total_distance_all_robots": _metric(
            behaviour, "total_distance_all_robots", 0.0),
        "route_dispatches": _metric(behaviour, "route_dispatches", 0),
        "rapid_route_overrides": _metric(
            behaviour, "rapid_route_overrides", 0),
        "audited_replans": _metric(behaviour, "audited_replans", 0),
        "unplanned_task_stops": _metric(
            behaviour, "unplanned_task_stops", 0),
        "min_pair_distance": _metric(behaviour, "min_pair_distance"),
        "pair_distance_violations": _metric(
            behaviour, "pair_distance_violations", 0),
        "clear_motion_duty_cycle": continuity.get(
            "clear_motion_duty_cycle", 0.0),
        "unexplained_active_intervals": summary[
            "unexplained_active_intervals"],
        "abnormal_mass_stop_seconds": summary[
            "abnormal_mass_stop_seconds"],
        "rotating_in_place_intervals": summary[
            "rotating_in_place_intervals"],
        "sim_to_wall_ratio": execution.get("sim_to_wall_ratio", 0.0),
        "webots_wall_seconds": execution.get("webots_wall_seconds"),
        "supervisor_step_wall_p99_ms": _metric(
            behaviour, "supervisor_step_wall_p99_ms"),
        "joint_planning_wall_p99_ms": _metric(
            behaviour, "joint_planning_wall_p99_ms"),
        "performance_source": "recording_run",
        "artifacts": artifacts,
    }
    report = {
        "step": args.step,
        "metrics": metrics,
        "gate": evaluate_gates(
            metrics, _load_baseline(args.baseline), args.minimum_ratio,
            args.change_kind, args.minimum_throughput,
            args.maximum_longest_plateau,
            args.maximum_final_plateau,
            args.maximum_robots_without_completions,
            args.maximum_reservation_deadline_episodes,
            args.maximum_active_zero_speed_ratio,
            args.maximum_motion_transitions_per_minute,
            args.maximum_shield_changes_per_minute),
    }
    (output_dir / "workflow_summary.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", required=True)
    parser.add_argument("--scenario", default="C", choices=("A", "B", "C"))
    parser.add_argument("--scheduler", default="FCFS")
    parser.add_argument("--seed", type=int, default=44)
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--webots", required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--review-r1", type=Path, required=True)
    parser.add_argument("--review-r2", type=Path, required=True)
    parser.add_argument("--minimum-ratio", type=float, default=1.0)
    parser.add_argument("--minimum-throughput", type=float, default=1.0)
    parser.add_argument("--maximum-longest-plateau", type=float,
                        default=150.0)
    parser.add_argument("--maximum-final-plateau", type=float, default=120.0)
    parser.add_argument("--maximum-robots-without-completions", type=int,
                        default=4)
    parser.add_argument("--maximum-reservation-deadline-episodes", type=int,
                        default=2)
    parser.add_argument("--maximum-active-zero-speed-ratio", type=float,
                        default=0.18)
    parser.add_argument("--maximum-motion-transitions-per-minute", type=float,
                        default=2.5)
    parser.add_argument("--maximum-shield-changes-per-minute", type=float,
                        default=30.0)
    parser.add_argument("--change-kind", choices=("behavior", "observability",
                                                  "workflow"),
                        default="behavior")
    parser.add_argument("--min-stop-seconds", type=float, default=2.0)
    parser.add_argument("--movement-epsilon-m", type=float, default=0.03)
    args = parser.parse_args()
    report = run_step(args)
    print(json.dumps(report, indent=2))
    return 0 if report["gate"]["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
