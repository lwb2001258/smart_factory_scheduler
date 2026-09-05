import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from advanced_rl_agents import SarsaLambdaAgent, SarsaLambdaConfig


def test_sarsa_lambda_propagates_td_error_to_previous_state():
    agent = SarsaLambdaAgent(
        3, 2, SarsaLambdaConfig(learning_rate=1.0, gamma=1.0,
                                trace_lambda=0.5, epsilon_start=0.0))
    first, second = (0,), (1,)
    agent.update(first, 0, 0.0, second, 1, False)
    agent.update(second, 1, 2.0, second, 2, True)
    assert agent.values(second)[1] == 2.0
    assert agent.values(first)[0] == 1.0
    assert agent.traces == {}


def test_sarsa_lambda_masks_actions_and_roundtrips_checkpoint(tmp_path):
    agent = SarsaLambdaAgent(4, 3, seed=5)
    state = (1, 2)
    agent.values(state)[:] = [0, 100, 3, 0]
    assert agent.select_action(state, [True, False, True, False], False) == 2
    path = tmp_path / "model.json"
    agent.save(path)
    loaded = SarsaLambdaAgent(4, 3)
    loaded.load(path)
    np.testing.assert_array_equal(loaded.values(state), agent.values(state))

