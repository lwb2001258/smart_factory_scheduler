import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from task_generator import TransportTask


def task(**overrides):
    values = dict(
        task_id=1, pickup_location="A", delivery_location="B",
        pickup_position=(0.0, 0.0), delivery_position=(1.0, 0.0),
        arrival_time=0.0)
    values.update(overrides)
    return TransportTask(**values)


def test_task_without_deadline_has_no_false_tardiness():
    item = task(completion_time=100.0)
    assert item.tardiness is None


def test_task_tardiness_and_business_weight_are_explicit():
    item = task(deadline=80.0, completion_time=95.0, priority_rank=300)
    assert item.tardiness == 15.0
    assert item.business_weight == 2.5


def test_cargo_state_defaults_to_not_picked():
    assert task().cargo_state == "not_picked"

