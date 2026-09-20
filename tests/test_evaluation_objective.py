import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))
sys.path.insert(0, str(ROOT / "scripts"))

from evaluation_objective import SelectionMetrics, algorithm_selection_score
from train_scheduler import validation_selection_score


def test_shared_objective_has_consistent_directions():
    base = SelectionMetrics(0.8, 50, 20, 100, 30, 0, 0, 2, 10)
    score = algorithm_selection_score(base)
    assert algorithm_selection_score(SelectionMetrics(**{
        **base.__dict__, "completion_rate": 0.9})) > score
    for field in ("mean_completion_time", "mean_waiting_time",
                  "mean_makespan", "mean_distance", "invalid_actions",
                  "native_failures", "p95_latency_ms"):
        assert algorithm_selection_score(SelectionMetrics(**{
            **base.__dict__, field: getattr(base, field) + 1})) < score


def test_value_trainer_adapter_uses_canonical_objective():
    metrics = {"completion_rate": 0.75, "mean_completion_time": 40.0,
               "mean_waiting_time": 12.0, "mean_makespan": 150.0,
               "mean_distance": 25.0, "invalid_actions": 1,
               "mean_reward": 9.0, "mean_completed": 15.0}
    expected = algorithm_selection_score(SelectionMetrics(
        0.75, 40.0, 12.0, 150.0, 25.0, 1, mean_reward=9.0))
    assert validation_selection_score(metrics) == expected
