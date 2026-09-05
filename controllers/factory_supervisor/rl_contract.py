"""Versioned observation/action contract shared by RL training and runtime."""

from dataclasses import asdict, dataclass
import hashlib
import json

from config import MAX_ROBOTS, RL_ENVIRONMENT_VERSION


@dataclass(frozen=True)
class RLSchedulingContract:
    version: str = RL_ENVIRONMENT_VERSION
    max_robots: int = MAX_ROBOTS
    max_tasks: int = 20
    robot_features: int = 8
    task_features: int = 13
    global_features: int = 8
    robot_order: str = "ascending_robot_id"
    task_order: str = "task_id_then_arrival_time"
    action_semantics: str = "robot_slot*max_tasks+task_slot; final=NO_OP"
    no_op_rule: str = "WAIT legal only when no feasible pair in v7"
    feasibility_contract: str = "deadline-v1+build_cost_matrix+validate_assignment"
    reward_contract: str = "deadline-event-reward-v2"
    discount_contract: str = "elapsed-seconds-bootstrap-v1"

    @property
    def observation_dim(self) -> int:
        return (self.global_features
                + self.max_robots * self.robot_features
                + self.max_tasks * self.task_features
                + self.max_robots + self.max_tasks)

    @property
    def action_dim(self) -> int:
        return self.max_robots * self.max_tasks + 1

    @property
    def no_op_action(self) -> int:
        return self.action_dim - 1

    def metadata(self) -> dict:
        result = asdict(self)
        result.update({
            "observation_dim": self.observation_dim,
            "action_dim": self.action_dim,
            "no_op_action": self.no_op_action,
        })
        return result

    def fingerprint(self) -> str:
        payload = json.dumps(
            self.metadata(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()


RL_SCHEDULING_CONTRACT = RLSchedulingContract()
