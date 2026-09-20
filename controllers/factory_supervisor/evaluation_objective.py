"""Canonical, algorithm-independent model-selection objective."""

from dataclasses import asdict, dataclass


OBJECTIVE_VERSION = "scheduler-selection-v1"


@dataclass(frozen=True)
class SelectionMetrics:
    completion_rate: float
    mean_completion_time: float = 0.0
    mean_waiting_time: float = 0.0
    mean_makespan: float = 0.0
    mean_distance: float = 0.0
    invalid_actions: int = 0
    native_failures: int = 0
    p95_latency_ms: float = 0.0
    mean_reward: float = 0.0


def algorithm_selection_score(metrics: SelectionMetrics) -> float:
    """Score every candidate with one direction and one set of weights.

    Completion is the dominant safety gate. Operational cost and policy
    failures break ties; reward has deliberately tiny influence because its
    scale is configurable.
    """
    return float(
        10000.0 * metrics.completion_rate
        - 0.50 * metrics.mean_completion_time
        - 0.25 * metrics.mean_waiting_time
        - 0.10 * metrics.mean_makespan
        - 0.05 * metrics.mean_distance
        - 1000.0 * metrics.invalid_actions
        - 1000.0 * metrics.native_failures
        - 0.02 * metrics.p95_latency_ms
        + 0.01 * metrics.mean_reward)


def objective_metadata() -> dict:
    return {"version": OBJECTIVE_VERSION,
            "higher_is_better": True,
            "metrics": list(asdict(SelectionMetrics(0.0))),
            "completion_dominates": True}
