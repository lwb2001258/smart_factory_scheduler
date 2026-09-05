"""Deterministic A/B/C inputs shared by training, standalone and Webots."""

from dataclasses import asdict, dataclass
import hashlib
import json
from typing import List, Tuple

from config import (INITIAL_BATTERY_MAX, INITIAL_BATTERY_MIN, PARKING_SPOTS,
                    SCENARIOS, TIMESTEP, initial_battery_for_robot)
from task_generator import TaskGenerator


@dataclass(frozen=True)
class ManifestTask:
    task_id: int
    arrival_time: float
    pickup_location: str
    delivery_location: str
    priority: float
    priority_rank: int = 200
    target_completion_time: float | None = None
    deadline: float | None = None
    deadline_type: str = "none"
    deadline_source: str = "none"
    late_penalty_per_second: float = 1.0
    pickup_service_time: float = 0.0
    delivery_service_time: float = 0.0


@dataclass(frozen=True)
class ExperimentManifest:
    version: str
    scenario: str
    seed: int
    duration: float
    timestep_seconds: float
    robots: Tuple[Tuple[int, Tuple[float, float], float], ...]
    tasks: Tuple[ManifestTask, ...]

    def to_dict(self) -> dict:
        return asdict(self)

    def fingerprint(self) -> str:
        payload = json.dumps(
            self.to_dict(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def generate_experiment_manifest(scenario: str, seed: int,
                                 duration: float) -> ExperimentManifest:
    cfg = SCENARIOS[scenario]
    dt = TIMESTEP / 1000.0
    robots = tuple(
        (rid, tuple(PARKING_SPOTS[rid]), initial_battery_for_robot(
            seed, rid, INITIAL_BATTERY_MIN, INITIAL_BATTERY_MAX))
        for rid in range(1, cfg["num_robots"] + 1))
    generator = TaskGenerator(
        cfg["task_interval"], seed=seed,
        initial_task_immediately=cfg.get("initial_task_immediately", False))
    tasks: List[ManifestTask] = []
    tick = 1
    while tick * dt <= duration + 1e-12:
        now = tick * dt
        task = generator.update(now)
        if task is not None:
            tasks.append(ManifestTask(
                task.task_id, task.arrival_time, task.pickup_location,
                task.delivery_location, task.priority, task.priority_rank,
                task.target_completion_time, task.deadline,
                task.deadline_type, task.deadline_source,
                task.late_penalty_per_second, task.pickup_service_time,
                task.delivery_service_time))
        tick += 1
    return ExperimentManifest(
        "factory-manifest-v2-deadline", scenario, int(seed), float(duration), dt,
        robots, tuple(tasks))
