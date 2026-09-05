import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from advanced_rl_agents import RainbowConfig, RainbowDQNAgent


def make_agent(**overrides):
    config = RainbowConfig(atoms=11, v_min=-5, v_max=5, n_steps=3,
                           replay_capacity=20, **overrides)
    return RainbowDQNAgent(4, 3, 2, config, seed=4)


def test_rainbow_requires_all_named_components():
    for component in ("double", "dueling", "prioritized_replay",
                      "distributional", "noisy"):
        with pytest.raises(ValueError):
            make_agent(**{component: False})


def test_rainbow_mask_dueling_distribution_and_noisy_action():
    agent = make_agent()
    state = np.ones(4, np.float32)
    distributions = agent.distributions(state)
    assert distributions.shape == (3, 11)
    np.testing.assert_allclose(distributions.sum(axis=1), 1.0, atol=1e-6)
    for _ in range(10):
        assert agent.select_action(state, [True, False, False], training=True) == 0


def test_rainbow_nstep_terminal_flush_per_and_training():
    agent = make_agent()
    mask = np.array([True, True, False])
    assert agent.observe(np.zeros(4), 0, 1, np.ones(4), False, mask) == 0
    assert agent.observe(np.ones(4), 1, 2, np.ones(4)*2, True, mask) == 2
    assert len(agent.replay) == 2
    first = agent.replay.rows[0]
    assert first.reward == pytest.approx(1+agent.config.gamma*2)
    assert first.bootstrap_discount == 0
    before = agent.params["value_w"].copy()
    result = agent.train_batch(2)
    assert np.isfinite(result["distributional_loss"])
    assert not np.array_equal(before, agent.params["value_w"])
    assert np.isfinite(agent.replay.priorities[:2]).all()


def test_rainbow_double_selection_is_masked_and_checkpoint_roundtrip(tmp_path):
    agent = make_agent()
    agent.params["adv_b"][2, -1] = 1e4
    assert agent.select_action(np.zeros(4), [True, True, False]) in (0, 1)
    path = tmp_path / "rainbow.npz"
    agent.save(path)
    loaded = make_agent()
    loaded.load(path)
    np.testing.assert_allclose(loaded.q_values(np.ones(4)), agent.q_values(np.ones(4)))
    with np.load(path, allow_pickle=False) as data:
        config = str(data["config_json"][0])
        assert all(name in config for name in ("double", "dueling", "prioritized_replay",
                                               "distributional", "noisy"))
