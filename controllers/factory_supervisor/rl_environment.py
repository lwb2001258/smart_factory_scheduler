"""Versioned, Webots-independent environment contract for RL schedulers."""

from copy import deepcopy
from dataclasses import dataclass, field, replace
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from config import (BATTERY_CAPACITY, BATTERY_DRAIN_RATE, CHARGING_STATIONS,
                    FULL_BATTERY_THRESHOLD, MAX_ROBOTS, MIN_TASK_BATTERY,
                    RL_ENVIRONMENT_VERSION, RobotState, TaskStatus)
from rl_contract import RL_SCHEDULING_CONTRACT
from rl_event_ledger import RLEventLedger, reward_for_event
from schedulers import (
    Assignment, SchedulingContext, build_cost_matrix, estimate_pair_timing,
    validate_assignment,
)
from task_generator import TransportTask
from time_discount import elapsed_bootstrap_discount


ENVIRONMENT_VERSION = RL_ENVIRONMENT_VERSION
ABSTRACT_LINEAR_SPEED = 0.22  # robot_controller's executable speed limit


@dataclass(frozen=True)
class RLEnvironmentConfig:
    max_robots: int = MAX_ROBOTS
    max_tasks: int = 20
    max_steps_per_episode: int = 64
    position_scale_x: float = 10.0
    position_scale_y: float = 8.0
    time_scale: float = 120.0
    distance_scale: float = 30.0


@dataclass(frozen=True)
class RewardConfig:
    completion: float = 5.0
    on_time: float = 3.0
    hard_breach_once: float = -8.0
    tardiness: float = -2.0
    valid_assignment: float = 0.0
    empty_distance: float = -0.10
    wait_increment: float = -0.10
    age_rescue: float = 0.20
    age_scale_seconds: float = 120.0
    max_age_units: float = 2.0
    time_scale_seconds: float = 120.0
    distance_scale_metres: float = 30.0
    max_tardiness_units: float = 3.0
    reassignment: float = -0.25
    replan: float = -0.05
    deadlock_recovery: float = -1.0
    failed_retryable: float = -2.0
    failed_final: float = -8.0
    post_pickup_abort: float = -12.0
    invalid_action: float = -5.0
    avoidable_wait: float = -1.0
    forced_wait: float = 0.0
    collision: float = -100.0

    # Compatibility aliases for old reporting code; no raw priority reward.
    @property
    def task_completion(self):
        return self.completion

    @property
    def no_op(self):
        return self.avoidable_wait


class SchedulingEnvironment:
    """Snapshot encoder plus a small abstract assignment environment.

    Webots mode never commits domain state; scheduler adapters use observation
    and mask only. Abstract mode operates on private deep copies for training.
    """

    ROBOT_FEATURES = RL_SCHEDULING_CONTRACT.robot_features
    TASK_FEATURES = RL_SCHEDULING_CONTRACT.task_features
    GLOBAL_FEATURES = RL_SCHEDULING_CONTRACT.global_features

    def __init__(self, config: Optional[RLEnvironmentConfig] = None,
                 reward: Optional[RewardConfig] = None,
                 simulation_mode: str = "abstract"):
        if simulation_mode not in {"abstract", "webots"}:
            raise ValueError("simulation_mode must be abstract or webots")
        self.config = config or RLEnvironmentConfig()
        self.reward_config = reward or RewardConfig()
        self.simulation_mode = simulation_mode
        self.action_dim = self.config.max_robots * self.config.max_tasks + 1
        self.no_op_action = self.action_dim - 1
        self.observation_dim = (
            self.GLOBAL_FEATURES
            + self.config.max_robots * self.ROBOT_FEATURES
            + self.config.max_tasks * self.TASK_FEATURES
            + self.config.max_robots + self.config.max_tasks
        )
        self._robots: Dict[int, dict] = {}
        self._tasks: List[TransportTask] = []
        self._context = SchedulingContext()
        self._robot_slots: List[int] = []
        self._task_slots: List[TransportTask] = []
        self._step = 0
        self._completed_ids = set()
        self._cost_matrix = None
        self.rng = np.random.default_rng(0)
        self.event_ledger = RLEventLedger()
        self._breached_ids = set()
        self._decision_sequence = 0
        self.last_elapsed_seconds = 0.0
        self.last_bootstrap_discount = 1.0
        self._terminal_settled = False

    def contract_metadata(self) -> dict:
        """Return immutable model-interface metadata for audit/provenance."""
        metadata = RL_SCHEDULING_CONTRACT.metadata()
        metadata["fingerprint"] = RL_SCHEDULING_CONTRACT.fingerprint()
        if (self.config.max_robots != RL_SCHEDULING_CONTRACT.max_robots or
                self.config.max_tasks != RL_SCHEDULING_CONTRACT.max_tasks):
            metadata["runtime_override"] = {
                "max_robots": self.config.max_robots,
                "max_tasks": self.config.max_tasks,
                "observation_dim": self.observation_dim,
                "action_dim": self.action_dim,
                "no_op_action": self.no_op_action,
            }
        return metadata

    def reset(self, robot_states: Optional[Dict[int, dict]] = None,
              tasks: Optional[List[TransportTask]] = None,
              context: Optional[SchedulingContext] = None,
              seed: Optional[int] = None) -> Tuple[np.ndarray, dict]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if self.simulation_mode == "abstract":
            self._robots = deepcopy(robot_states or {})
            self._tasks = deepcopy(tasks or [])
        else:
            self._robots = robot_states or {}
            self._tasks = tasks or []
        self._context = context or SchedulingContext()
        provider = self._context.path_cost_provider
        bind = getattr(provider, "bind_robot_states", None)
        if callable(bind):
            bind(self._robots)
        self._step = 0
        self._completed_ids.clear()
        self.event_ledger = RLEventLedger()
        self._breached_ids.clear()
        self._decision_sequence = 0
        self.last_elapsed_seconds = 0.0
        self.last_bootstrap_discount = 1.0
        self._terminal_settled = False
        self._refresh_slots()
        self._cost_matrix = None
        # Mirror Supervisor's pre-dispatch battery guard for an initial
        # snapshot that contains an idle low-battery robot.
        for rid, state in self._robots.items():
            if (state.get("state") == RobotState.IDLE and
                    state.get("current_task") is None and
                    float(state.get("battery", BATTERY_CAPACITY))
                    < MIN_TASK_BATTERY):
                self._schedule_charge_if_required(
                    rid, float(self._context.current_time))
        return self.observe(), {"environment_version": ENVIRONMENT_VERSION}

    def set_snapshot(self, robot_states: Dict[int, dict],
                     tasks: List[TransportTask],
                     context: Optional[SchedulingContext] = None) -> np.ndarray:
        return self.reset(robot_states, tasks, context)[0]

    def _refresh_slots(self) -> None:
        self._robot_slots = sorted(self._robots)[:self.config.max_robots]
        now = float(self._context.current_time)
        self._task_slots = sorted(
            (task for task in self._tasks
             if task.status == TaskStatus.PENDING
             and float(task.arrival_time) <= now + 1e-9),
            key=lambda task: (task.task_id, task.arrival_time)
        )[:self.config.max_tasks]

    def encode_action(self, robot_slot: int, task_slot: int) -> int:
        if not (0 <= robot_slot < self.config.max_robots):
            raise ValueError("robot_slot out of range")
        if not (0 <= task_slot < self.config.max_tasks):
            raise ValueError("task_slot out of range")
        return robot_slot * self.config.max_tasks + task_slot

    def decode_action(self, action: int) -> Optional[Tuple[int, int]]:
        if action == self.no_op_action:
            return None
        if not (0 <= action < self.no_op_action):
            raise ValueError("action out of range")
        return divmod(action, self.config.max_tasks)

    def get_action_mask(self) -> np.ndarray:
        self._refresh_slots()
        self._cost_matrix = None
        mask = np.zeros(self.action_dim, dtype=bool)
        if self._robot_slots and self._task_slots:
            matrix = self._matrix()
            robot_index = {rid: index for index, rid in enumerate(matrix.robot_ids)}
            task_index = {
                task.task_id: index for index, task in enumerate(matrix.tasks)}
            for rslot, rid in enumerate(self._robot_slots):
                ri = robot_index.get(rid)
                if ri is None:
                    continue
                for tslot, task in enumerate(self._task_slots):
                    ti = task_index.get(task.task_id)
                    if ti is not None and matrix.feasible[ri, ti]:
                        mask[self.encode_action(rslot, tslot)] = True
        # NO_OP is a safety action, but is unavailable when useful work exists.
        mask[self.no_op_action] = not bool(mask[:-1].any())
        return mask

    def _matrix(self):
        if self._cost_matrix is None:
            self._cost_matrix = build_cost_matrix(
                self._task_slots, self._robots, self._context)
        return self._cost_matrix

    def assignment_for_action(self, action: int) -> Optional[Assignment]:
        decoded = self.decode_action(action)
        if decoded is None:
            return None
        rslot, tslot = decoded
        if rslot >= len(self._robot_slots) or tslot >= len(self._task_slots):
            return None
        rid = self._robot_slots[rslot]
        task = self._task_slots[tslot]
        matrix = self._matrix()
        try:
            ri = matrix.robot_ids.index(rid)
            ti = tuple(item.task_id for item in matrix.tasks).index(task.task_id)
        except ValueError:
            return None
        cost = float(matrix.values[ri, ti])
        if not matrix.feasible[ri, ti] or not np.isfinite(cost):
            return None
        return Assignment(rid, task, cost)

    def observe(self) -> np.ndarray:
        self._refresh_slots()
        self._cost_matrix = None
        cfg = self.config
        pending = len(self._task_slots)
        idle = sum(
            state.get("state") == RobotState.IDLE
            and state.get("current_task") is None
            for state in self._robots.values()
        )
        congestion = self._context.congestion_map
        mean_congestion = float(np.mean(congestion)) if congestion else 0.0
        mask = self.get_action_mask()
        at_risk = 0
        breached = 0
        for task in self._task_slots:
            if task.deadline is None:
                continue
            if float(task.deadline) < float(self._context.current_time):
                breached += 1
                continue
            estimates = []
            for rid in self._robot_slots:
                try:
                    estimates.append(estimate_pair_timing(
                        rid, task, self._robots, self._context))
                except (TypeError, ValueError, OverflowError):
                    continue
            best_slack = max(
                (item.hard_slack for item in estimates
                 if item.hard_slack is not None), default=None)
            if best_slack is not None and best_slack <= 60.0:
                at_risk += 1
        global_features = [
            np.clip(self._context.current_time / cfg.time_scale, 0, 10),
            pending / max(1, cfg.max_tasks),
            idle / max(1, cfg.max_robots),
            np.clip(mean_congestion, 0, 10),
            float(mask[:-1].mean()) if self.no_op_action else 0.0,
            self._step / max(1, cfg.max_steps_per_episode),
            at_risk / max(1, cfg.max_tasks),
            breached / max(1, cfg.max_tasks),
        ]
        robot_values = []
        robot_mask = []
        for slot in range(cfg.max_robots):
            if slot < len(self._robot_slots):
                state = self._robots[self._robot_slots[slot]]
                x, y = state.get("position", (0.0, 0.0))
                robot_values.extend([
                    x / cfg.position_scale_x, y / cfg.position_scale_y,
                    float(state.get("state") == RobotState.IDLE),
                    np.clip(float(state.get("battery", 100.0)) / 100.0, 0, 1),
                    float(state.get("current_task") is not None),
                    float(state.get("faulted", False) or state.get("failed", False)),
                    min(float(state.get("tasks_completed", 0)) / 20.0, 10),
                    min(float(state.get("total_distance", 0.0)) / 200.0, 10),
                ])
                robot_mask.append(1.0)
            else:
                robot_values.extend([0.0] * self.ROBOT_FEATURES)
                robot_mask.append(0.0)
        task_values = []
        task_mask = []
        for slot in range(cfg.max_tasks):
            if slot < len(self._task_slots):
                task = self._task_slots[slot]
                px, py = task.pickup_position
                dx, dy = task.delivery_position
                wait = max(0.0, self._context.current_time - task.arrival_time)
                matrix = self._matrix()
                ti = next((index for index, item in enumerate(matrix.tasks)
                           if item.task_id == task.task_id), None)
                feasible = (
                    matrix.feasible[:, ti] if ti is not None
                    else np.zeros(0, dtype=bool))
                feasible_fraction = float(feasible.mean()) if feasible.size else 0.0
                costs = (
                    matrix.values[:, ti][feasible] if ti is not None
                    else np.zeros(0))
                expected = float(costs.min()) if costs.size else 0.0
                timing_candidates = []
                for ri, rid in enumerate(matrix.robot_ids):
                    if ti is None or not matrix.feasible[ri, ti]:
                        continue
                    try:
                        timing_candidates.append(estimate_pair_timing(
                            rid, task, self._robots, self._context))
                    except (TypeError, ValueError, OverflowError):
                        continue
                timing = min(
                    timing_candidates,
                    key=lambda item: item.conservative_completion,
                    default=None)
                soft_slack = 0.0 if timing is None or timing.soft_slack is None else (
                    timing.soft_slack / cfg.time_scale)
                hard_slack = 0.0 if timing is None or timing.hard_slack is None else (
                    timing.hard_slack / cfg.time_scale)
                predicted_late = 0.0 if timing is None else float(
                    timing.predicted_tardiness or 0.0) / cfg.time_scale
                task_values.extend([
                    px / cfg.position_scale_x, py / cfg.position_scale_y,
                    dx / cfg.position_scale_x, dy / cfg.position_scale_y,
                    np.clip(float(task.priority) / 2.0, 0, 10),
                    min(wait / cfg.time_scale, 10),
                    float(task.status == TaskStatus.PENDING),
                    feasible_fraction,
                    min(expected / cfg.distance_scale, 10),
                    float(task.deadline is not None),
                    np.clip(soft_slack, -10, 10),
                    np.clip(hard_slack, -10, 10),
                    np.clip(predicted_late, 0, 10),
                ])
                task_mask.append(1.0)
            else:
                task_values.extend([0.0] * self.TASK_FEATURES)
                task_mask.append(0.0)
        result = np.asarray(
            global_features + robot_values + task_values
            + robot_mask + task_mask, dtype=np.float32)
        if result.shape != (self.observation_dim,) or not np.isfinite(result).all():
            raise ValueError("invalid RL observation")
        return result

    def step(self, action: int):
        if self.simulation_mode != "abstract":
            raise RuntimeError("step is only available in abstract mode")
        mask = self.get_action_mask()
        self.last_elapsed_seconds = 0.0
        self.last_bootstrap_discount = 1.0
        self._step += 1
        if not (0 <= action < self.action_dim) or not mask[action]:
            event = self.event_ledger.append(
                "invalid_action", self._context.current_time, action=action)
            truncated = self._step >= self.config.max_steps_per_episode
            reward = reward_for_event(event, self.reward_config)
            reward += self._settle_terminal_pending(False, truncated)
            return self.observe(), reward, False, truncated, {
                "invalid_action": True}
        if action == self.no_op_action:
            had_active = self._has_active_executions()
            had_future = self._has_future_arrivals()
            event_start = len(self.event_ledger.events)
            completed = self._advance_to_next_completion()
            terminated = self._is_terminal_state()
            if completed:
                reward = sum(
                    reward_for_event(item, self.reward_config)
                    for item in self.event_ledger.events[event_start:])
            else:
                event = self.event_ledger.append(
                    "no_op", self._context.current_time,
                    productive_wait=bool(had_active or had_future))
                reward = reward_for_event(event, self.reward_config)
            self._cost_matrix = None
            truncated = (not terminated and
                         self._step >= self.config.max_steps_per_episode)
            reward += self._settle_terminal_pending(terminated, truncated)
            return self.observe(), float(reward), terminated, truncated, {
                "no_op": True, "completed_this_step": completed,
                "completed_count": len(self._completed_ids)}
        assignment = self.assignment_for_action(action)
        if assignment is None:
            event = self.event_ledger.append(
                "invalid_action", self._context.current_time, action=action,
                reason="assignment_decode_failed")
            truncated = self._step >= self.config.max_steps_per_episode
            reward = reward_for_event(event, self.reward_config)
            reward += self._settle_terminal_pending(False, truncated)
            return self.observe(), reward, False, truncated, {
                "invalid_action": True}
        task = assignment.task
        robot = self._robots[assignment.robot_id]
        wait = max(0.0, self._context.current_time - task.arrival_time)
        assignment_event = self.event_ledger.append(
            "assignment_committed", self._context.current_time,
            robot_id=assignment.robot_id, task_id=task.task_id,
            priority_rank=int(task.priority_rank), waiting_seconds=wait,
            empty_distance=float(assignment.empty_distance or 0.0),
            estimated_cost=float(assignment.estimated_cost or 0.0),
            decision_id=self._next_decision_id())
        reward = reward_for_event(assignment_event, self.reward_config)
        # Commit the dispatch exactly as FactorySupervisor does. Completion is
        # a later discrete event, so multiple robots can remain active at the
        # same time instead of one robot completing every task instantaneously.
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot = assignment.robot_id
        task.assignment_time = self._context.current_time
        robot["state"] = RobotState.EN_ROUTE_PICKUP
        robot["current_task"] = task
        robot["has_task"] = True
        robot["goal_location"] = task.pickup_location

        pickup_distance = max(0.0, float(
            assignment.empty_distance if assignment.empty_distance is not None
            else self._pickup_leg_distance(assignment)))
        loaded_distance = max(0.0, float(assignment.loaded_distance or 0.0))
        travel_distance = pickup_distance + loaded_distance
        travel_seconds = (travel_distance / ABSTRACT_LINEAR_SPEED
                          + task.pickup_service_time
                          + task.delivery_service_time)
        pickup_seconds = pickup_distance / ABSTRACT_LINEAR_SPEED
        robot["_abstract_execution"] = {
            "kind": "task",
            "task": task,
            "pickup_time": self._context.current_time + pickup_seconds,
            "completion_time": self._context.current_time + travel_seconds,
            "distance": travel_distance,
            "decision_id": assignment_event.values["decision_id"],
        }
        self._cost_matrix = None

        # When the fleet is saturated, advance to the next physical completion
        # event. This is the event-driven equivalent of Webots continuing to
        # step while no idle robot is available.
        completed = 0
        event_start = len(self.event_ledger.events)
        if not self.get_action_mask()[:-1].any():
            completed = self._advance_to_next_completion()
            if completed:
                new_events = self.event_ledger.events[event_start:]
                reward += sum(reward_for_event(item, self.reward_config)
                              for item in new_events)
        terminated = self._is_terminal_state()
        truncated = self._step >= self.config.max_steps_per_episode
        reward += self._settle_terminal_pending(terminated, truncated)
        return self.observe(), float(reward), bool(terminated), bool(truncated), {
            "assignment": (assignment.robot_id, task.task_id),
            "completed_this_step": completed,
            "completed_count": len(self._completed_ids),
        }

    def _advance_to_next_completion(self) -> int:
        """Advance to the next task arrival or robot completion event."""
        executions = [
            (float(state["_abstract_execution"]["completion_time"]), rid)
            for rid, state in self._robots.items()
            if state.get("_abstract_execution") is not None]
        now = float(self._context.current_time)
        future_arrivals = [
            float(task.arrival_time) for task in self._tasks
            if task.status == TaskStatus.PENDING
            and float(task.arrival_time) > now + 1e-9]
        event_times = [item[0] for item in executions] + future_arrivals
        if not event_times:
            return 0
        event_time = min(event_times)
        elapsed = max(0.0, event_time - now)
        self.last_elapsed_seconds = elapsed
        self.last_bootstrap_discount = elapsed_bootstrap_discount(elapsed)
        pending_count = sum(
            task.status == TaskStatus.PENDING and task.arrival_time <= now + 1e-9
            for task in self._tasks)
        if elapsed > 0.0 and pending_count:
            self.event_ledger.append(
                "queue_wait_advanced", event_time,
                pending_wait_increment=elapsed * pending_count,
                elapsed_seconds=elapsed)
        self._context = replace(self._context, current_time=event_time)
        completed = 0
        for _, rid in executions:
            robot = self._robots[rid]
            execution = robot.get("_abstract_execution")
            if (execution is None or
                    float(execution["completion_time"]) > event_time + 1e-9):
                continue
            if execution.get("kind") == "charge":
                journey_seconds = float(execution["journey_seconds"])
                distance = float(execution["distance"])
                robot["state"] = RobotState.CHARGING
                robot["position"] = tuple(execution["position"])
                robot["battery"] = max(
                    0.0, float(robot["battery"])
                    - BATTERY_DRAIN_RATE * journey_seconds)
                robot["total_distance"] = (
                    float(robot.get("total_distance", 0.0)) + distance)
                robot["battery"] = float(self.rng.uniform(
                    FULL_BATTERY_THRESHOLD, BATTERY_CAPACITY))
                robot["state"] = RobotState.IDLE
                robot["goal_location"] = None
                robot.pop("_abstract_execution", None)
                continue
            task = execution["task"]
            task.pickup_time = float(execution["pickup_time"])
            task.cargo_state = "onboard"
            task.status = TaskStatus.IN_PROGRESS
            robot["state"] = RobotState.EN_ROUTE_DELIVERY
            task.completion_time = event_time
            task.status = TaskStatus.COMPLETED
            task.cargo_state = "delivered"
            robot["position"] = tuple(task.delivery_position)
            travel_distance = float(execution["distance"])
            travel_seconds = max(0.0, event_time - float(task.assignment_time))
            robot["battery"] = max(
                0.0, float(robot.get("battery", BATTERY_CAPACITY))
                - BATTERY_DRAIN_RATE * travel_seconds)
            robot["total_distance"] = (
                float(robot.get("total_distance", 0.0)) + travel_distance)
            robot["tasks_completed"] = int(
                robot.get("tasks_completed", 0)) + 1
            robot["current_task"] = None
            robot["has_task"] = False
            robot["goal_location"] = None
            robot["state"] = RobotState.IDLE
            robot.pop("_abstract_execution", None)
            self._completed_ids.add(task.task_id)
            completed += 1
            tardiness = task.tardiness
            on_time = tardiness is None or tardiness <= 1e-9
            if (task.deadline is not None and not on_time and
                    task.task_id not in self._breached_ids):
                self._breached_ids.add(task.task_id)
                self.event_ledger.append(
                    "deadline_breached", event_time, robot_id=rid,
                    task_id=task.task_id,
                    business_weight=task.business_weight,
                    decision_id=execution.get("decision_id"))
            self.event_ledger.append(
                "task_completed_on_time" if on_time else "task_completed_late",
                event_time, robot_id=rid, task_id=task.task_id,
                count=1, on_time=on_time,
                tardiness_seconds=float(tardiness or 0.0),
                business_weight=task.business_weight,
                decision_id=execution.get("decision_id"))
            self._schedule_charge_if_required(rid, event_time)
        self._cost_matrix = None
        return completed

    def _has_active_executions(self) -> bool:
        return any(state.get("_abstract_execution") is not None
                   for state in self._robots.values())

    def _has_future_arrivals(self) -> bool:
        now = float(self._context.current_time)
        return any(task.status == TaskStatus.PENDING
                   and float(task.arrival_time) > now + 1e-9
                   for task in self._tasks)

    def _is_terminal_state(self) -> bool:
        pending = any(task.status == TaskStatus.PENDING for task in self._tasks)
        return not pending and not self._has_active_executions()

    def _pickup_leg_distance(self, assignment: Assignment) -> float:
        provider = self._context.path_cost_provider
        segment = getattr(provider, "segment", None)
        if callable(segment):
            return float(segment(
                self._robots[assignment.robot_id]["position"],
                assignment.task.pickup_position))
        start = self._robots[assignment.robot_id]["position"]
        goal = assignment.task.pickup_position
        return math.hypot(goal[0] - start[0], goal[1] - start[1])

    def _schedule_charge_if_required(self, robot_id: int,
                                     current_time: float) -> bool:
        robot = self._robots[robot_id]
        if float(robot.get("battery", BATTERY_CAPACITY)) >= MIN_TASK_BATTERY:
            return False
        provider = self._context.path_cost_provider
        segment = getattr(provider, "segment", None)
        candidates = []
        if callable(segment):
            for name, position in CHARGING_STATIONS.items():
                distance = float(segment(robot["position"], position))
                if np.isfinite(distance):
                    candidates.append((distance, name, position))
        if not candidates:
            for name, position in CHARGING_STATIONS.items():
                distance = math.hypot(
                    position[0] - robot["position"][0],
                    position[1] - robot["position"][1])
                candidates.append((distance, name, position))
        distance, station, position = min(candidates, key=lambda item: item[0])
        journey_seconds = distance / ABSTRACT_LINEAR_SPEED
        robot["state"] = RobotState.RETURNING_TO_CHARGE
        robot["goal_location"] = station
        robot["_abstract_execution"] = {
            "kind": "charge",
            "completion_time": current_time + journey_seconds + 5.0,
            "journey_seconds": journey_seconds,
            "distance": distance,
            "station": station,
            "position": tuple(position),
        }
        return True

    def is_terminal(self) -> bool:
        return self._is_terminal_state()

    def _next_decision_id(self) -> str:
        self._decision_sequence += 1
        return f"decision-{self._decision_sequence}"

    def _settle_terminal_pending(self, terminated: bool,
                                 truncated: bool) -> float:
        if self._terminal_settled or not (terminated or truncated):
            return 0.0
        self._terminal_settled = True
        remaining = [task for task in self._tasks
                     if task.status != TaskStatus.COMPLETED]
        if not remaining:
            return 0.0
        loss = sum(float(task.business_weight) for task in remaining)
        event = self.event_ledger.append(
            "episode_terminated_with_pending", self._context.current_time,
            remaining_count=len(remaining), remaining_business_loss=loss)
        return reward_for_event(event, self.reward_config)
