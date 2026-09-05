import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from experiment_manifest import generate_experiment_manifest
from training_scenarios import manifest_scenario


def test_training_snapshot_uses_exact_manifest_inputs():
    for scenario in "ABC":
        manifest = generate_experiment_manifest(scenario, 123, 1800.0)
        robots, tasks, context = manifest_scenario(123, scenario, max_tasks=12)
        assert [(rid, state["position"], state["battery"])
                for rid, state in robots.items()] == list(manifest.robots)
        assert [(task.task_id, task.arrival_time, task.pickup_location,
                 task.delivery_location, task.priority) for task in tasks] == [
            (task.task_id, task.arrival_time, task.pickup_location,
             task.delivery_location, task.priority)
            for task in manifest.tasks[:12]]
        assert context.configuration["manifest_fingerprint"] == (
            manifest.fingerprint())

