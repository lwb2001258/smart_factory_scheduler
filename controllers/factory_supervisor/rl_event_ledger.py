"""Versioned RL event ledger and side-effect-free reward replay."""

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Iterable, List


LEDGER_VERSION = "rl-event-ledger-v2-deadline"


@dataclass(frozen=True)
class RLEvent:
    event_type: str
    sim_time: float
    robot_id: int = 0
    task_id: int = 0
    values: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        result = asdict(self)
        result["ledger_version"] = LEDGER_VERSION
        return result


class RLEventLedger:
    def __init__(self):
        self.events: List[RLEvent] = []
        self._sequence = 0

    def append(self, event_type: str, sim_time: float, *, robot_id: int = 0,
               task_id: int = 0, **values) -> RLEvent:
        self._sequence += 1
        values.setdefault("event_sequence", self._sequence)
        event = RLEvent(
            str(event_type), float(sim_time), int(robot_id), int(task_id),
            dict(values))
        self.events.append(event)
        return event

    def snapshot(self) -> List[dict]:
        return [event.to_dict() for event in self.events]


def reward_for_event(event: RLEvent, config) -> float:
    """Compute reward from one canonical event without mutating state."""
    values = event.values
    if event.event_type == "invalid_action":
        return float(config.invalid_action)
    if event.event_type in {"no_op", "wait"}:
        return (float(config.forced_wait) if values.get("productive_wait", False)
                else float(config.avoidable_wait))
    if event.event_type in {"task_completed", "task_completed_on_time",
                            "task_completed_late"}:
        count = float(values.get("count", 1))
        weight = max(0.0, float(values.get("business_weight", 1.0)))
        reward = float(config.completion) * count
        if values.get("on_time", False):
            reward += float(config.on_time) * weight
        tardiness_units = min(
            max(0.0, float(values.get("tardiness_seconds", 0.0))) /
            max(float(config.time_scale_seconds), 1e-6),
            float(config.max_tardiness_units))
        reward += float(config.tardiness) * tardiness_units * weight
        return float(reward)
    if event.event_type == "assignment_committed":
        age_units = min(
            max(0.0, float(values.get("waiting_seconds", 0.0))) /
            max(float(config.age_scale_seconds), 1e-6),
            float(config.max_age_units))
        distance_units = min(
            max(0.0, float(values.get("empty_distance", 0.0))) /
            max(float(config.distance_scale_metres), 1e-6), 3.0)
        return float(
            config.valid_assignment
            + config.age_rescue * age_units
            + config.empty_distance * distance_units)
    if event.event_type == "queue_wait_advanced":
        units = min(
            max(0.0, float(values.get("pending_wait_increment", 0.0))) /
            max(float(config.time_scale_seconds), 1e-6), 10.0)
        return float(config.wait_increment) * units
    if event.event_type == "deadline_breached":
        return (float(config.hard_breach_once) *
                max(0.0, float(values.get("business_weight", 1.0))))
    if event.event_type == "assignment_reassigned":
        return float(config.reassignment)
    if event.event_type == "replan":
        return float(config.replan)
    if event.event_type == "deadlock_recovery":
        return float(config.deadlock_recovery)
    if event.event_type == "task_failed_retryable":
        return float(config.failed_retryable)
    if event.event_type == "task_failed_final":
        return float(config.failed_final)
    if event.event_type == "post_pickup_abort":
        return float(config.post_pickup_abort)
    if event.event_type == "episode_terminated_with_pending":
        return -max(0.0, float(values.get("remaining_business_loss", 0.0)))
    if event.event_type == "collision":
        return float(config.collision) * float(values.get("count", 1))
    if event.event_type == "deadlock":
        return float(config.deadlock_recovery) * float(values.get("count", 1))
    # Operational events such as pickup, charge, wait and replan are recorded
    # even when the current reward profile intentionally gives them zero.
    return 0.0


def replay_reward(events: Iterable[RLEvent], config) -> float:
    return float(sum(reward_for_event(event, config) for event in events))
