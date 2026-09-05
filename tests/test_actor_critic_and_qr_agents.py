import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from advanced_rl_agents import A2CAgent, DiscreteSACAgent, QRDQNAgent
from schedulers import ModelValidationError


@pytest.mark.parametrize("agent_type", [A2CAgent, DiscreteSACAgent, QRDQNAgent])
def test_advanced_agent_respects_mask_and_roundtrips(agent_type, tmp_path):
    agent = agent_type(6, 5, 4, seed=7)
    state = np.linspace(0.0, 1.0, 6, dtype=np.float32)
    mask = np.array([False, True, False, True, False])
    for _ in range(20):
        assert agent.select_action(state, mask, training=True) in (1, 3)
    path = tmp_path / "agent.npz"
    agent.save(path)
    loaded = agent_type(6, 5, 4, seed=99)
    loaded.load(path)
    assert loaded.select_action(state, mask, training=False) == agent.select_action(
        state, mask, training=False)


@pytest.mark.parametrize("agent_type", [A2CAgent, DiscreteSACAgent, QRDQNAgent])
def test_advanced_agent_rejects_contract_dimension_mismatch(agent_type, tmp_path):
    path = tmp_path / "agent.npz"
    agent_type(6, 5, 4).save(path)
    with pytest.raises(ModelValidationError):
        agent_type(7, 5, 4).load(path)


def test_a2c_terminal_update_is_finite_and_does_not_bootstrap():
    agent = A2CAgent(3, 3, 2, seed=1)
    state = np.array([1.0, 0.0, 0.0], np.float32)
    agent.params["value_w"][:] = [2.0, 10.0, 10.0]
    result = agent.update(state, 0, 1.0, np.full(3, 100.0), True,
                          [True, True, False])
    assert result["advantage"] == pytest.approx(-1.0)
    assert all(np.isfinite(value).all() for value in agent.params.values())


def test_discrete_sac_excludes_illegal_action_from_target():
    agent = DiscreteSACAgent(2, 3, 2, seed=2)
    agent.target["q1_b"][:] = [1.0, 2.0, 1e6]
    agent.target["q2_b"][:] = [1.0, 2.0, 1e6]
    result = agent.update(np.zeros(2), 0, 0.0, np.zeros(2), False,
                          [True, True, False], [True, True, False])
    assert result["target"] < 10.0


def test_qr_dqn_terminal_target_and_illegal_greedy_action():
    agent = QRDQNAgent(2, 3, 2, seed=3)
    agent.params["b"][2] = 1e6
    assert agent.select_action(np.zeros(2), [True, True, False]) in (0, 1)
    result = agent.update(np.zeros(2), 0, 4.0, np.ones(2), True,
                          [True, True, False], [True, True, False])
    assert result["target_mean"] == pytest.approx(4.0)
    assert all(np.isfinite(value).all() for value in agent.params.values())
