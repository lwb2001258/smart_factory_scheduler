"""Authoritative RL scheduler and checkpoint metadata registry.

This module is deliberately independent from scheduler construction so model
artifacts can be audited before Webots or the standalone simulator starts.
Checkpoint files are trusted project artifacts; DQN currently uses pickle and
must never be loaded from an untrusted source.
"""

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import pickle
from typing import Any, Dict, Optional

import numpy as np

from config import RL_ENVIRONMENT_VERSION
from rl_contract import RL_SCHEDULING_CONTRACT


PAIRWISE_STATE_DIM = RL_SCHEDULING_CONTRACT.observation_dim
PAIRWISE_ACTION_DIM = RL_SCHEDULING_CONTRACT.action_dim
LEGACY_PPO_STATE_DIM = 126
LEGACY_PPO_ACTION_DIM = 8


@dataclass(frozen=True)
class RLAlgorithmSpec:
    name: str
    implementation: str
    checkpoint_format: str
    action_contract: str
    status: str = "implemented"


ALGORITHM_REGISTRY: Dict[str, RLAlgorithmSpec] = {
    "SARSA": RLAlgorithmSpec(
        "SARSA", "tabular on-policy SARSA(0)", "json",
        "masked robot-task pair + NO_OP"),
    "DQN": RLAlgorithmSpec(
        "DQN", "NumPy Double-DQN", "pickle",
        "masked robot-task pair + NO_OP"),
    "PPO_RL": RLAlgorithmSpec(
        "PPO_RL", "NumPy clipped PPO", "npz",
        "masked robot-task pair + NO_OP"),
    "SARSA_LAMBDA": RLAlgorithmSpec(
        "SARSA_LAMBDA", "tabular on-policy SARSA(lambda)", "json",
        "masked robot-task pair + NO_OP"),
    "RAINBOW_DQN": RLAlgorithmSpec(
        "RAINBOW_DQN", "C51 Rainbow: Double+Dueling+PER+n-step+Noisy", "npz",
        "masked robot-task pair + NO_OP"),
    "A2C": RLAlgorithmSpec(
        "A2C", "masked Advantage Actor-Critic", "npz",
        "masked robot-task pair + NO_OP"),
    "DISCRETE_SAC": RLAlgorithmSpec(
        "DISCRETE_SAC", "masked discrete Soft Actor-Critic with twin Q", "npz",
        "masked robot-task pair + NO_OP"),
    "QR_DQN": RLAlgorithmSpec(
        "QR_DQN", "masked quantile-regression Double-DQN", "npz",
        "masked robot-task pair + NO_OP"),
    # Kept in the registry to make the distinction from SARSA explicit.
    "Q_LEARNING": RLAlgorithmSpec(
        "Q_LEARNING",
        "Q-learning family is used through Double-DQN; no standalone tabular scheduler",
        "none", "not independently selectable",
        status="implemented_via_dqn"),
}


@dataclass(frozen=True)
class CheckpointAudit:
    requested_algorithm: str
    checkpoint_algorithm: str
    path: str
    sha256: str
    environment_version: str
    state_dim: Optional[int]
    action_dim: int
    action_contract: str
    contract_fingerprint: str
    checkpoint_contract_fingerprint: Optional[str]
    checkpoint_contract_verified: bool
    legacy: bool
    valid: bool

    def to_dict(self) -> dict:
        return asdict(self)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _scalar(value: Any) -> Any:
    array = np.asarray(value)
    if array.size != 1:
        raise ValueError("checkpoint metadata value is not scalar")
    return array.reshape(-1)[0].item()


def _read_metadata(algorithm: str, path: Path) -> dict:
    if algorithm in {"SARSA", "SARSA_LAMBDA"}:
        return json.loads(path.read_text(encoding="utf-8"))
    if algorithm == "DQN":
        # The repository's DQN persistence format is pickle. Only audit
        # checkpoints produced by this project and obtained from a trusted
        # source.
        with path.open("rb") as stream:
            return pickle.load(stream)
    if algorithm in {"PPO_RL", "RAINBOW_DQN", "A2C", "DISCRETE_SAC", "QR_DQN"}:
        with np.load(path, allow_pickle=False) as payload:
            keys = ("algorithm", "environment_version", "state_dim", "action_dim",
                    "no_op_action", "contract_fingerprint", "config_json",
                    "seed", "training_step")
            return {key: _scalar(payload[key]) for key in keys
                    if key in payload.files}
    raise ValueError(f"unsupported RL algorithm: {algorithm}")


def audit_checkpoint(algorithm: str, checkpoint_path: str, *,
                     allow_legacy_ppo: bool = False) -> CheckpointAudit:
    """Validate checkpoint identity and return reproducibility metadata."""
    requested = str(algorithm).upper()
    if requested not in ALGORITHM_REGISTRY:
        raise ValueError(f"unknown RL algorithm: {algorithm}")
    if ALGORITHM_REGISTRY[requested].status != "implemented":
        raise ValueError(
            f"RL algorithm is not independently deployable: {requested}")
    path = Path(checkpoint_path).resolve()
    if not path.is_file():
        raise ValueError(f"checkpoint does not exist: {path}")
    metadata = _read_metadata(requested, path)
    actual = str(metadata.get("algorithm", requested)).upper()
    environment = str(metadata.get("environment_version", ""))
    state_dim_value = metadata.get("state_dim")
    state_dim = int(state_dim_value) if state_dim_value is not None else None
    action_dim = int(metadata.get("action_dim", -1))
    if actual != requested:
        raise ValueError(
            f"checkpoint algorithm mismatch: requested {requested}, got {actual}")
    if environment != RL_ENVIRONMENT_VERSION:
        raise ValueError(
            f"environment version mismatch: expected {RL_ENVIRONMENT_VERSION}, "
            f"got {environment or '<missing>'}")

    legacy = False
    if requested in {"SARSA", "SARSA_LAMBDA", "DQN"}:
        if action_dim != PAIRWISE_ACTION_DIM:
            raise ValueError("checkpoint action dimension is not pairwise-v1")
        if requested == "DQN" and state_dim != PAIRWISE_STATE_DIM:
            raise ValueError("DQN state dimension is not pairwise-v1")
    elif requested == "PPO_RL":
        legacy = (state_dim, action_dim) == (
            LEGACY_PPO_STATE_DIM, LEGACY_PPO_ACTION_DIM)
        pairwise = (state_dim, action_dim) == (
            PAIRWISE_STATE_DIM, PAIRWISE_ACTION_DIM)
        if legacy and not allow_legacy_ppo:
            raise ValueError(
                "legacy 126-state/8-action PPO is not valid for new evaluations")
        if not pairwise and not legacy:
            raise ValueError("unknown PPO observation/action contract")
    else:
        if (state_dim, action_dim) != (PAIRWISE_STATE_DIM, PAIRWISE_ACTION_DIM):
            raise ValueError("checkpoint observation/action dimension is not pairwise-v1")
        if int(metadata.get("no_op_action", -1)) != RL_SCHEDULING_CONTRACT.no_op_action:
            raise ValueError("checkpoint NO_OP action mismatch")

    runtime_fingerprint = RL_SCHEDULING_CONTRACT.fingerprint()
    embedded_fingerprint = metadata.get("contract_fingerprint")
    if (embedded_fingerprint is not None and
            str(embedded_fingerprint) != runtime_fingerprint):
        raise ValueError("checkpoint contract fingerprint mismatch")
    return CheckpointAudit(
        requested_algorithm=requested,
        checkpoint_algorithm=actual,
        path=str(path),
        sha256=_sha256(path),
        environment_version=environment,
        state_dim=state_dim,
        action_dim=action_dim,
        action_contract=("legacy robot-only" if legacy
                         else "pairwise robot-task + NO_OP"),
        contract_fingerprint=runtime_fingerprint,
        checkpoint_contract_fingerprint=(
            str(embedded_fingerprint) if embedded_fingerprint is not None
            else None),
        checkpoint_contract_verified=(embedded_fingerprint is not None),
        legacy=legacy,
        valid=True,
    )


def registry_snapshot() -> dict:
    return {name: asdict(spec) for name, spec in ALGORITHM_REGISTRY.items()}
