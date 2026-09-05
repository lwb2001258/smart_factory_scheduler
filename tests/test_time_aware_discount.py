import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from advanced_rl_common import NStepAccumulator
from time_discount import elapsed_bootstrap_discount


def test_elapsed_discount_uses_real_time_and_terminal_zero():
    assert elapsed_bootstrap_discount(0.0) == 1.0
    assert elapsed_bootstrap_discount(10.0) == pytest.approx(0.99)
    assert elapsed_bootstrap_discount(100.0) == pytest.approx(0.99 ** 10)
    assert elapsed_bootstrap_discount(10.0, terminal=True) == 0.0


def test_nstep_multiplies_per_transition_discounts():
    acc = NStepAccumulator(2, gamma=0.5)
    mask = np.ones(2, dtype=bool)
    assert acc.add([0], 0, 1.0, [1], False, mask,
                   bootstrap_discount=0.9) == []
    rows = acc.add([1], 1, 2.0, [2], False, mask,
                   bootstrap_discount=0.8)
    assert len(rows) == 1
    assert rows[0].reward == pytest.approx(1.0 + 0.9 * 2.0)
    assert rows[0].bootstrap_discount == pytest.approx(0.9 * 0.8)


def test_invalid_discount_is_rejected():
    acc = NStepAccumulator(1)
    with pytest.raises(ValueError):
        acc.add([0], 0, 0.0, [1], False, [True],
                bootstrap_discount=1.1)

