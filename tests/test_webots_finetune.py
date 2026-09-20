import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))
sys.path.insert(0, str(ROOT / "scripts"))

from rl_agents import DQNAgent, DQNConfig
from rl_agents import SarsaAgent
from advanced_rl_agents import (A2CAgent, DiscreteSACAgent, QRDQNAgent,
                                RainbowDQNAgent, SarsaLambdaAgent)
from schedulers import PPONetwork
from rl_contract import RL_SCHEDULING_CONTRACT
from finetune_from_webots import _build_agent, audit_transitions, finetune
from webots_dataset import DATASET_VERSION, write_jsonl


def transition():
    return {"dataset_version": DATASET_VERSION, "decision_id": "d1",
            "state": [0.0] * RL_SCHEDULING_CONTRACT.observation_dim,
            "action": 0, "reward": 1.0,
            "action_mask": [True] * RL_SCHEDULING_CONTRACT.action_dim,
            "next_state": [0.1] * RL_SCHEDULING_CONTRACT.observation_dim,
            "done": True,
            "next_action_mask": [True] * RL_SCHEDULING_CONTRACT.action_dim,
            "elapsed_seconds": 1.0, "bootstrap_discount": 0.0}


def test_audit_rejects_wrong_dataset_version():
    row = transition()
    row["dataset_version"] = "old"
    with pytest.raises(ValueError, match="version"):
        audit_transitions([row])


def test_audit_rejects_terminal_bootstrap_leak():
    row = transition()
    row["bootstrap_discount"] = 0.9
    with pytest.raises(ValueError, match="zero bootstrap"):
        audit_transitions([row])


def test_dqn_finetune_creates_separate_audited_candidate(tmp_path):
    base = tmp_path / "base" / "model.pkl"
    agent = DQNAgent(RL_SCHEDULING_CONTRACT.observation_dim,
                     RL_SCHEDULING_CONTRACT.action_dim,
                     RL_SCHEDULING_CONTRACT.no_op_action,
                     DQNConfig(batch_size=1, warmup_steps=1), seed=3)
    agent.save(str(base))
    dataset = write_jsonl([transition()], tmp_path / "data.jsonl")
    report = finetune(base, [dataset], tmp_path / "candidate",
                      updates=2, seed=3)
    assert Path(report["candidate"]).exists()
    assert report["updates_applied"] == 2
    assert report["base_sha256"] != report["candidate_sha256"]
    saved = json.loads((tmp_path / "candidate" /
                        "finetune_report.json").read_text())
    assert saved["status"] == "candidate_requires_webots_validation"
    reloaded, _ = _build_agent(
        "DQN", Path(report["candidate"]), 1e-5, seed=4)
    assert reloaded is not None


@pytest.mark.parametrize("algorithm,extension", [
    ("SARSA", ".json"), ("PPO", ".npz"),
    ("SARSA_LAMBDA", ".json"), ("RAINBOW_DQN", ".npz"),
    ("A2C", ".npz"), ("DISCRETE_SAC", ".npz"), ("QR_DQN", ".npz")])
def test_every_non_dqn_algorithm_can_finetune_webots_data(
        tmp_path, algorithm, extension):
    dims = (RL_SCHEDULING_CONTRACT.observation_dim,
            RL_SCHEDULING_CONTRACT.action_dim,
            RL_SCHEDULING_CONTRACT.no_op_action)
    constructors = {
        "SARSA": lambda: SarsaAgent(dims[1], dims[2]),
        "PPO": lambda: PPONetwork(dims[0], dims[1], hidden_size=8),
        "SARSA_LAMBDA": lambda: SarsaLambdaAgent(dims[1], dims[2]),
        "RAINBOW_DQN": lambda: RainbowDQNAgent(*dims),
        "A2C": lambda: A2CAgent(*dims),
        "DISCRETE_SAC": lambda: DiscreteSACAgent(*dims),
        "QR_DQN": lambda: QRDQNAgent(*dims),
    }
    base = tmp_path / f"base{extension}"
    constructors[algorithm]().save(str(base))
    dataset = write_jsonl([transition()], tmp_path / "data.jsonl")
    report = finetune(base, [dataset], tmp_path / "candidate",
                      algorithm=algorithm, updates=2, seed=7)
    assert report["algorithm"] == algorithm
    assert report["updates_applied"] == 2
    assert Path(report["candidate"]).is_file()
    assert report["base_sha256"] != report["candidate_sha256"]
    reloaded, _ = _build_agent(
        algorithm, Path(report["candidate"]), 1e-5, seed=8)
    assert reloaded is not None
