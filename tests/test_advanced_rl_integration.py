import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "controllers" / "factory_supervisor"
sys.path.insert(0, str(SUPERVISOR))

from advanced_rl_agents import (A2CAgent, DiscreteSACAgent, QRDQNAgent,
                                RainbowConfig, RainbowDQNAgent,
                                SarsaLambdaAgent)
from rl_contract import RL_SCHEDULING_CONTRACT
from rl_model_registry import audit_checkpoint, registry_snapshot
from schedulers import ModelValidationError, create_scheduler
from schedulers import SchedulerResult
from rl_schedulers import RLSchedulerSafetyWrapper


AGENTS = {
    "SARSA_LAMBDA": lambda: SarsaLambdaAgent(161, 160),
    "A2C": lambda: A2CAgent(RL_SCHEDULING_CONTRACT.observation_dim, 161, 160),
    "DISCRETE_SAC": lambda: DiscreteSACAgent(
        RL_SCHEDULING_CONTRACT.observation_dim, 161, 160),
    "QR_DQN": lambda: QRDQNAgent(
        RL_SCHEDULING_CONTRACT.observation_dim, 161, 160),
    "RAINBOW_DQN": lambda: RainbowDQNAgent(
        RL_SCHEDULING_CONTRACT.observation_dim, 161, 160,
        RainbowConfig(atoms=11)),
}


def test_benign_empty_feasible_graph_is_not_counted_as_rl_fallback():
    class Policy:
        name = "TEST_RL"

        def assign(self, *_args, **_kwargs):
            return SchedulerResult(
                algorithm_name=self.name, computation_time=0.001,
                diagnostics={"reason": "no_feasible_pair"})

        def reset(self):
            pass

    wrapper = RLSchedulerSafetyWrapper(Policy())
    result = wrapper.assign([], {}, None)
    assert not result.is_feasible
    assert result.diagnostics["benign_no_decision"]
    assert wrapper.fallback_decisions == 0
    assert wrapper.consecutive_failures == 0


@pytest.mark.parametrize("algorithm", AGENTS)
def test_registry_audit_and_factory_load_advanced_checkpoint(algorithm, tmp_path):
    extension = "json" if algorithm == "SARSA_LAMBDA" else "npz"
    path = tmp_path / f"model.{extension}"
    AGENTS[algorithm]().save(path)
    audit = audit_checkpoint(algorithm, str(path))
    assert audit.valid and audit.checkpoint_contract_verified
    scheduler = create_scheduler(algorithm, str(path), allow_safe_fallback=False)
    assert scheduler.name == algorithm
    assert scheduler.policy.environment.observation_dim == RL_SCHEDULING_CONTRACT.observation_dim


def test_advanced_factory_fails_closed_without_checkpoint():
    with pytest.raises(ModelValidationError):
        create_scheduler("A2C", None, allow_safe_fallback=False)


def test_registry_lists_every_advanced_algorithm():
    snapshot = registry_snapshot()
    assert set(AGENTS).issubset(snapshot)
