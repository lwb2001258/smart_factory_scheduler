import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from run_experiments import (run_single_experiment,
                             console_safe,
                             resolve_checkpoint_path,
                             validate_native_rl_result)


def test_native_rl_experiment_requires_checkpoint():
    with pytest.raises(ValueError, match="requires a checkpoint"):
        run_single_experiment(
            "A", "DQN", 1, webots_path="definitely-not-webots")


def test_checkpoint_path_is_absolute_for_webots_controller(tmp_path,
                                                            monkeypatch):
    checkpoint = tmp_path / "model.pkl"
    checkpoint.write_bytes(b"checkpoint")
    monkeypatch.chdir(tmp_path)
    assert resolve_checkpoint_path("model.pkl") == str(checkpoint.resolve())


def test_console_safe_handles_replacement_character():
    assert isinstance(console_safe("controller \ufffd output"), str)


def test_non_rl_experiment_does_not_require_checkpoint(monkeypatch):
    called = {}

    def fake_standalone(*args, **kwargs):
        called["args"] = args

    monkeypatch.setattr("run_experiments.run_standalone_simulation",
                        fake_standalone)
    monkeypatch.setattr("run_experiments.load_latest_results",
                        lambda *args: {"ok": True})
    result = run_single_experiment(
        "A", "FCFS", 1, webots_path="definitely-not-webots")
    assert result == {"ok": True}
    assert called["args"][:3] == ("A", "FCFS", 1)


def test_native_result_gate_accepts_uncontaminated_policy():
    validate_native_rl_result({"summary_metrics": {
        "rl_policy_decisions": 4,
        "rl_fallback_decisions": 0,
        "scheduler_fallbacks": 0,
        "scheduler_commits_by_algorithm": {"DQN": 3},
    }}, "DQN")


@pytest.mark.parametrize("metrics", [
    {"rl_policy_decisions": 0,
     "scheduler_commits_by_algorithm": {}},
    {"rl_policy_decisions": 3, "rl_fallback_decisions": 1,
     "scheduler_commits_by_algorithm": {"SARSA": 2}},
    {"rl_policy_decisions": 3, "scheduler_fallbacks": 1,
     "scheduler_commits_by_algorithm": {
         "PPO_RL": 2, "PPO_RL_FALLBACK_HUNGARIAN": 1}},
])
def test_native_result_gate_rejects_unverifiable_or_fallback_results(metrics):
    with pytest.raises(RuntimeError):
        validate_native_rl_result({"summary_metrics": metrics}, "SARSA")
