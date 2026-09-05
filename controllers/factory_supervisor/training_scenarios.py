"""Factory-grounded scenario generation shared by standalone trainers."""

import math
from typing import Dict, List, Tuple

import numpy as np

from config import (PARKING_SPOTS, REST_NODES, STORAGE_AREAS, WAYPOINTS,
                    WORKSTATIONS, RobotState)
from grid_planner import OccupancyGrid
from motion_coordinator import MotionCoordinator
from schedulers import SchedulingContext
from task_generator import TransportTask
from experiment_manifest import generate_experiment_manifest


_STATIC_COORDINATOR = None
_SEGMENT_CACHE = {}


def factory_free_positions() -> List[Tuple[float, float]]:
    """Return deduplicated parking/corridor positions that are grid-free.

    A graph/rest node can still lie inside an inflated grid keep-out. Check
    the same occupancy grid used by A* instead of trusting configuration
    comments when choosing physical robot centres for training.
    """
    values = list(PARKING_SPOTS.values())
    values.extend(WAYPOINTS[node] for node in REST_NODES if node in WAYPOINTS)
    candidates = list(dict.fromkeys((float(x), float(y)) for x, y in values))
    grid = OccupancyGrid(num_active_robots=8)
    positions = [point for point in candidates
                 if grid.is_free(*grid.world_to_grid(*point))]
    if len(positions) < 8:
        raise RuntimeError(
            "Factory layout has fewer than 8 verified free training starts")
    return positions


def _task_pair(rng):
    storage = list(STORAGE_AREAS)
    workstations = list(WORKSTATIONS)
    draw = float(rng.random())
    if draw < 0.40:
        return str(rng.choice(storage)), str(rng.choice(workstations))
    if draw < 0.75:
        return str(rng.choice(workstations)), str(rng.choice(storage))
    pickup = str(rng.choice(workstations))
    delivery = str(rng.choice([name for name in workstations
                               if name != pickup]))
    return pickup, delivery


def path_length(path, start):
    if path is None:
        return math.inf
    points = [tuple(start)] + [tuple(point) for point in path]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


class FactoryAStarCostOracle:
    """Cached static Grid-A* travel costs for scheduler observations/masks."""

    def __init__(self, robot_states: Dict[int, dict], num_robots: int):
        global _STATIC_COORDINATOR
        self.robot_states = robot_states
        if _STATIC_COORDINATOR is None:
            _STATIC_COORDINATOR = MotionCoordinator(num_active_robots=8)
        self.coordinator = _STATIC_COORDINATOR
        self.cache = _SEGMENT_CACHE

    def bind_robot_states(self, robot_states: Dict[int, dict]):
        """Rebind to the live/deep-copied simulator snapshot."""
        self.robot_states = robot_states
        return self

    def segment(self, start, goal):
        start = (float(start[0]), float(start[1]))
        goal = (float(goal[0]), float(goal[1]))
        key = (start, goal)
        if key not in self.cache:
            path = self.coordinator.grid_planner.plan(start, goal, smooth=True)
            self.cache[key] = path_length(path, start)
        return self.cache[key]

    def __call__(self, robot_id: int, task: TransportTask) -> float:
        empty = self.segment(
            self.robot_states[robot_id]["position"], task.pickup_position)
        loaded = self.segment(task.pickup_position, task.delivery_position)
        total = empty + loaded
        return total if math.isfinite(total) else math.inf


def factory_scenario(seed: int, max_robots=8, max_tasks=20,
                     min_robots=2, max_generated_tasks=12):
    """Generate a valid factory snapshot with Grid-A* pair costs.

    Robot positions come only from verified parking/rest nodes. Task endpoints
    use the same WS/S docks as Webots. Unreachable pairs return infinity and
    are consequently removed by the scheduler action mask.
    """
    rng = np.random.default_rng(seed)
    robot_count = int(rng.integers(min_robots, max_robots + 1))
    task_upper = min(max_tasks, max_generated_tasks)
    task_count = int(rng.integers(2, task_upper + 1))
    positions = factory_free_positions()
    chosen = rng.choice(len(positions), size=robot_count, replace=False)
    robots = {
        rid: {"position": positions[int(chosen[rid - 1])],
              "state": RobotState.IDLE,
              "battery": float(rng.uniform(30.0, 100.0)),
              "current_task": None, "tasks_completed": 0,
              "total_distance": 0.0}
        for rid in range(1, robot_count + 1)
    }
    tasks = []
    mean_interval = 30.0 if robot_count <= 3 else (15.0 if robot_count <= 5
                                                   else 8.0)
    arrival_time = 0.0
    for index in range(task_count):
        pickup, delivery = _task_pair(rng)
        locations = {**STORAGE_AREAS, **WORKSTATIONS}
        if index:
            arrival_time += float(rng.exponential(mean_interval))
        priority_rank = int(rng.choice([100, 200, 200, 200, 300, 400]))
        tasks.append(TransportTask(
            index + 1, pickup, delivery, locations[pickup],
            locations[delivery], arrival_time,
            priority=priority_rank / 200.0,
            priority_rank=priority_rank,
            target_completion_time=arrival_time + 240.0,
            deadline=arrival_time + 300.0,
            deadline_type="hard", deadline_source="training_sla"))
    oracle = FactoryAStarCostOracle(robots, robot_count)
    return robots, tasks, SchedulingContext(
        current_time=0.0, path_cost_provider=oracle,
        configuration={"training_geometry": "factory-grid-astar-v3-online"})


def manifest_scenario(seed: int, scenario: str, max_tasks: int = 20):
    """Build a training snapshot from the exact A/B/C evaluation domain."""
    manifest = generate_experiment_manifest(scenario, seed, duration=1800.0)
    robots = {
        rid: {"position": tuple(position), "state": RobotState.IDLE,
              "battery": float(battery), "current_task": None,
              "tasks_completed": 0, "total_distance": 0.0}
        for rid, position, battery in manifest.robots
    }
    locations = {**STORAGE_AREAS, **WORKSTATIONS}
    tasks = [TransportTask(
        item.task_id, item.pickup_location, item.delivery_location,
        locations[item.pickup_location], locations[item.delivery_location],
        item.arrival_time, priority=item.priority,
        priority_rank=item.priority_rank,
        target_completion_time=item.target_completion_time,
        deadline=item.deadline, deadline_type=item.deadline_type,
        deadline_source=item.deadline_source,
        late_penalty_per_second=item.late_penalty_per_second,
        pickup_service_time=item.pickup_service_time,
        delivery_service_time=item.delivery_service_time)
        for item in manifest.tasks[:max_tasks]]
    oracle = FactoryAStarCostOracle(robots, len(robots))
    return robots, tasks, SchedulingContext(
        current_time=0.0, path_cost_provider=oracle,
        configuration={
            "training_geometry": "factory-grid-astar-v3-online",
            "manifest_version": manifest.version,
            "manifest_fingerprint": manifest.fingerprint(),
            "scenario": scenario,
        })
