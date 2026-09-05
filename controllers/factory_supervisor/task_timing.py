"""Pure task timing and deadline calculations shared by all schedulers."""

from dataclasses import dataclass
import math
from typing import Optional


DEFAULT_LINEAR_SPEED = 0.22


@dataclass(frozen=True)
class TaskTimingEstimate:
    empty_distance: float
    loaded_distance: float
    travel_seconds: float
    service_seconds: float
    congestion_buffer_seconds: float
    uncertainty_buffer_seconds: float
    expected_completion: float
    conservative_completion: float
    soft_slack: Optional[float]
    hard_slack: Optional[float]
    predicted_tardiness: Optional[float]


def _optional_slack(deadline, completion):
    if deadline is None:
        return None
    value = float(deadline)
    if not math.isfinite(value):
        raise ValueError("task deadline must be finite or None")
    return value - completion


def estimate_task_timing(task, *, current_time: float,
                         empty_distance: float, loaded_distance: float,
                         speed: float = DEFAULT_LINEAR_SPEED,
                         congestion_buffer_seconds: float = 0.0,
                         uncertainty_ratio: float = 0.15,
                         minimum_uncertainty_seconds: float = 5.0
                         ) -> TaskTimingEstimate:
    """Estimate delivery-service completion without using future state."""
    now = float(current_time)
    empty = float(empty_distance)
    loaded = float(loaded_distance)
    velocity = float(speed)
    congestion = max(0.0, float(congestion_buffer_seconds))
    if not all(math.isfinite(value) for value in (now, empty, loaded, velocity)):
        raise ValueError("task timing inputs must be finite")
    if empty < 0 or loaded < 0 or velocity <= 0:
        raise ValueError("distances must be non-negative and speed positive")
    pickup_service = max(0.0, float(getattr(task, "pickup_service_time", 0.0)))
    delivery_service = max(
        0.0, float(getattr(task, "delivery_service_time", 0.0)))
    travel_seconds = (empty + loaded) / velocity
    service_seconds = pickup_service + delivery_service
    expected = now + travel_seconds + service_seconds + congestion
    uncertainty = max(
        max(0.0, float(minimum_uncertainty_seconds)),
        travel_seconds * max(0.0, float(uncertainty_ratio)))
    conservative = expected + uncertainty
    soft_slack = _optional_slack(
        getattr(task, "target_completion_time", None), expected)
    hard_slack = _optional_slack(getattr(task, "deadline", None), conservative)
    predicted_tardiness = (
        None if hard_slack is None else max(0.0, -hard_slack))
    return TaskTimingEstimate(
        empty, loaded, travel_seconds, service_seconds, congestion,
        uncertainty, expected, conservative, soft_slack, hard_slack,
        predicted_tardiness)


def actual_tardiness(task, completion_time: Optional[float] = None
                     ) -> Optional[float]:
    """Return actual tardiness; tasks without a deadline return None."""
    deadline = getattr(task, "deadline", None)
    completed = (completion_time if completion_time is not None else
                 getattr(task, "completion_time", None))
    if deadline is None or completed is None:
        return None
    return max(0.0, float(completed) - float(deadline))

