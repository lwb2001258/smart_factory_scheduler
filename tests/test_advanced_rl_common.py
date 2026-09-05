import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from advanced_rl_common import (Adam, NStepAccumulator, PrioritizedReplay,
                                masked_softmax)


def test_masked_softmax_excludes_illegal_actions_and_is_shift_stable():
    mask = np.array([True, False, True])
    first = masked_softmax(np.array([1000.0, 99999.0, 999.0]), mask)
    second = masked_softmax(np.array([1.0, -5.0, 0.0]), mask)
    np.testing.assert_allclose(first, second)
    assert first[1] == 0 and first.sum() == pytest.approx(1.0)


def test_n_step_accumulator_flushes_terminal_tail_without_bootstrap():
    acc = NStepAccumulator(3, gamma=0.5)
    mask = np.array([True, False])
    assert acc.add([0], 0, 1, [1], False, mask) == []
    assert acc.add([1], 0, 2, [2], False, mask) == []
    rows = acc.add([2], 0, 4, [3], True, mask)
    assert [row.reward for row in rows] == pytest.approx([3.0, 4.0, 4.0])
    assert all(row.done for row in rows)
    assert all(row.bootstrap_discount == 0.0 for row in rows)
    assert len(acc.pending) == 0


def test_prioritized_replay_returns_normalized_importance_weights():
    replay = PrioritizedReplay(capacity=4, seed=1)
    for index in range(4):
        replay.add(index, priority=index + 1)
    rows, indices, weights = replay.sample(3, beta=0.7)
    assert len(rows) == len(indices) == len(weights) == 3
    assert np.isfinite(weights).all() and weights.max() == pytest.approx(1.0)
    replay.update_priorities(indices, np.ones(3) * 2)


def test_adam_clips_and_rejects_nonfinite_gradients():
    params = {"w": np.ones(2, np.float32)}
    optimizer = Adam(params, learning_rate=0.1, max_grad_norm=1.0)
    optimizer.step(params, {"w": np.array([100.0, 100.0])})
    assert np.isfinite(params["w"]).all()
    with pytest.raises(ValueError, match="invalid gradient"):
        optimizer.step(params, {"w": np.array([np.nan, 0.0])})
