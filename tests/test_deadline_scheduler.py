import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from config import RobotState
from schedulers import HungarianScheduler, SchedulingContext
from task_generator import TransportTask


def make_task(task_id, *, deadline, priority_rank):
    return TransportTask(
        task_id, "A", "B", (1.0, 0.0), (2.0, 0.0), 0.0,
        priority=priority_rank / 200.0, priority_rank=priority_rank,
        deadline=deadline, deadline_type="hard")


def test_near_deadline_beats_higher_rank_with_wide_slack():
    robots = {1: {"position": (0.0, 0.0), "state": RobotState.IDLE,
                  "battery": 100.0, "current_task": None}}
    near = make_task(1, deadline=25.0, priority_rank=200)
    wide_critical = make_task(2, deadline=1000.0, priority_rank=400)
    result = HungarianScheduler().assign(
        [wide_critical, near], robots, SchedulingContext(current_time=0.0))
    assert result.assignments[0].task.task_id == near.task_id


def test_no_deadline_task_remains_feasible():
    robots = {1: {"position": (0.0, 0.0), "state": RobotState.IDLE,
                  "battery": 100.0, "current_task": None}}
    item = make_task(1, deadline=None, priority_rank=100)
    result = HungarianScheduler().assign([item], robots, SchedulingContext())
    assert result.is_feasible
    assert result.assignments[0].timing.hard_slack is None

