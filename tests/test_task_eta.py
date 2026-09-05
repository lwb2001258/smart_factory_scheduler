import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from task_generator import TransportTask
from task_timing import actual_tardiness, estimate_task_timing


def make_task(**overrides):
    values = dict(
        task_id=1, pickup_location="A", delivery_location="B",
        pickup_position=(0.0, 0.0), delivery_position=(1.0, 0.0),
        arrival_time=0.0, pickup_service_time=2.0,
        delivery_service_time=3.0, target_completion_time=20.0,
        deadline=25.0)
    values.update(overrides)
    return TransportTask(**values)


def test_eta_components_and_slack_are_not_combined_costs():
    result = estimate_task_timing(
        make_task(), current_time=5.0, empty_distance=2.0,
        loaded_distance=3.0, speed=1.0,
        minimum_uncertainty_seconds=2.0)
    assert result.travel_seconds == 5.0
    assert result.service_seconds == 5.0
    assert result.expected_completion == 15.0
    assert result.conservative_completion == 17.0
    assert result.soft_slack == 5.0
    assert result.hard_slack == 8.0
    assert result.predicted_tardiness == 0.0


def test_no_deadline_remains_none_not_zero():
    result = estimate_task_timing(
        make_task(target_completion_time=None, deadline=None),
        current_time=0.0, empty_distance=1.0, loaded_distance=1.0,
        speed=1.0)
    assert result.soft_slack is None
    assert result.hard_slack is None
    assert result.predicted_tardiness is None


def test_eta_rejects_unreachable_and_actual_tardiness_is_separate():
    with pytest.raises(ValueError):
        estimate_task_timing(make_task(), current_time=0.0,
                             empty_distance=math.inf, loaded_distance=1.0)
    item = make_task(completion_time=30.0)
    assert actual_tardiness(item) == 5.0

