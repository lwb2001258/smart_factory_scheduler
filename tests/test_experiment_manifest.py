import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from config import SCENARIOS
from config import initial_battery_for_robot
from experiment_manifest import generate_experiment_manifest


def test_manifest_is_deterministic_and_seed_sensitive():
    left = generate_experiment_manifest("C", 42, 60.0)
    right = generate_experiment_manifest("C", 42, 60.0)
    other = generate_experiment_manifest("C", 43, 60.0)
    assert left == right
    assert left.fingerprint() == right.fingerprint()
    assert left.fingerprint() != other.fingerprint()


def test_manifest_matches_scenario_fleet_and_first_tick_arrival():
    for scenario, cfg in SCENARIOS.items():
        manifest = generate_experiment_manifest(scenario, 1, 60.0)
        assert len(manifest.robots) == cfg["num_robots"]
        assert [item[0] for item in manifest.robots] == list(
            range(1, cfg["num_robots"] + 1))
        assert manifest.tasks[0].arrival_time == manifest.timestep_seconds


def test_manifest_tasks_are_chronological_and_unique():
    manifest = generate_experiment_manifest("C", 7, 1800.0)
    assert [task.task_id for task in manifest.tasks] == list(
        range(1, len(manifest.tasks) + 1))
    assert list(task.arrival_time for task in manifest.tasks) == sorted(
        task.arrival_time for task in manifest.tasks)
    assert manifest.version == "factory-manifest-v2-deadline"
    assert all(task.deadline is not None and task.deadline > task.arrival_time
               for task in manifest.tasks)
    assert all(task.priority_rank in {100, 200, 300, 400}
               for task in manifest.tasks)


def test_initial_battery_is_mode_independent_pure_function():
    manifest = generate_experiment_manifest("C", 42, 1.0)
    for rid, _, battery in manifest.robots:
        assert battery == initial_battery_for_robot(42, rid)
    assert initial_battery_for_robot(42, 1) != initial_battery_for_robot(42, 2)
