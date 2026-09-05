import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from evaluation_gate import validate_evaluation_result


def valid_result():
    return {
        "experiment_info": {
            "scheduler": "DQN",
            "provenance": {
                "sha256": "a" * 64, "environment_version": "v",
                "contract_fingerprint": "b" * 64,
                "manifest_fingerprint": "c" * 64,
                "manifest_version": "factory-manifest-v1",
                "run_mode": "webots", "seed": 42,
                "checkpoint_contract_verified": True,
            },
        },
        "summary_metrics": {
            "scheduler_commits_by_algorithm": {"DQN": 10},
            "rl_fallback_decisions": 0,
        },
    }


def test_publication_gate_accepts_native_result():
    assert validate_evaluation_result(valid_result(), "DQN")["valid"]


def test_publication_gate_rejects_missing_provenance():
    result = valid_result()
    del result["experiment_info"]["provenance"]["manifest_fingerprint"]
    with pytest.raises(ValueError, match="provenance"):
        validate_evaluation_result(result, "DQN")


def test_publication_gate_rejects_fallback_commit():
    result = valid_result()
    result["summary_metrics"]["scheduler_commits_by_algorithm"][
        "DQN_FALLBACK_HUNGARIAN"] = 1
    with pytest.raises(ValueError, match="fallback"):
        validate_evaluation_result(result, "DQN")


def test_publication_gate_rejects_legacy_unverified_checkpoint():
    result = valid_result()
    result["experiment_info"]["provenance"][
        "checkpoint_contract_verified"] = False
    with pytest.raises(ValueError, match="verified RL contract"):
        validate_evaluation_result(result, "DQN")
