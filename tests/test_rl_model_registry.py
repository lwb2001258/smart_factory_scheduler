import json
import pickle
import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from config import RL_ENVIRONMENT_VERSION
from rl_model_registry import (ALGORITHM_REGISTRY, audit_checkpoint,
                               registry_snapshot)
from rl_contract import RL_SCHEDULING_CONTRACT


def test_registry_does_not_mislabel_sarsa_as_q_learning():
    snapshot = registry_snapshot()
    assert snapshot["SARSA"]["status"] == "implemented"
    assert snapshot["Q_LEARNING"]["status"] == "implemented_via_dqn"
    assert "Double-DQN" in snapshot["Q_LEARNING"]["implementation"]
    assert "on-policy" in snapshot["SARSA"]["implementation"]


def test_audit_pairwise_dqn(tmp_path):
    path = tmp_path / "model.pkl"
    with path.open("wb") as stream:
        pickle.dump({
            "algorithm": "DQN",
            "environment_version": RL_ENVIRONMENT_VERSION,
            "state_dim": RL_SCHEDULING_CONTRACT.observation_dim,
            "action_dim": 161,
        }, stream)
    result = audit_checkpoint("DQN", str(path))
    assert result.valid and not result.legacy
    assert not result.checkpoint_contract_verified
    assert len(result.sha256) == 64


def test_audit_rejects_legacy_ppo_by_default(tmp_path):
    path = tmp_path / "legacy.npz"
    np.savez(path, environment_version=np.asarray([RL_ENVIRONMENT_VERSION]),
             state_dim=np.asarray([126]), action_dim=np.asarray([8]))
    with pytest.raises(ValueError, match="legacy"):
        audit_checkpoint("PPO_RL", str(path))
    assert audit_checkpoint(
        "PPO_RL", str(path), allow_legacy_ppo=True).legacy


def test_audit_rejects_wrong_environment(tmp_path):
    path = tmp_path / "sarsa.json"
    path.write_text(json.dumps({
        "algorithm": "SARSA", "environment_version": "obsolete",
        "action_dim": 161,
    }), encoding="utf-8")
    with pytest.raises(ValueError, match="environment version"):
        audit_checkpoint("SARSA", str(path))


def test_audit_verifies_embedded_contract_fingerprint(tmp_path):
    from rl_contract import RL_SCHEDULING_CONTRACT
    path = tmp_path / "sarsa.json"
    path.write_text(json.dumps({
        "algorithm": "SARSA",
        "environment_version": RL_ENVIRONMENT_VERSION,
        "action_dim": 161,
        "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint(),
    }), encoding="utf-8")
    result = audit_checkpoint("SARSA", str(path))
    assert result.checkpoint_contract_verified
    assert result.checkpoint_contract_fingerprint == result.contract_fingerprint


def test_q_learning_cannot_be_audited_as_implemented(tmp_path):
    path = tmp_path / "q.json"
    path.write_text("{}", encoding="utf-8")
    assert ALGORITHM_REGISTRY["Q_LEARNING"].status == "implemented_via_dqn"
    with pytest.raises(ValueError, match="not independently deployable"):
        audit_checkpoint("Q_LEARNING", str(path))


def test_old_pairwise_champion_checkpoints_are_rejected_after_contract_bump():
    paths = {
        "DQN": ROOT / "results/hyperparameter_search/optimized_v2/best/dqn/best_validation.pkl",
        "SARSA": ROOT / "results/hyperparameter_search/optimized_v2/best/sarsa/best_validation.json",
        "PPO_RL": ROOT / "results/ppo_auto_optimized_full_seed1/best/ppo/ppo_model_best_validation.npz",
    }
    for algorithm, path in paths.items():
        if path.exists():
            with pytest.raises(ValueError, match=(
                    "environment version|observation/action dimension")):
                audit_checkpoint(algorithm, str(path))
