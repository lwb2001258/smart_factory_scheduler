import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from rl_environment import RewardConfig
from rl_event_ledger import RLEventLedger, replay_reward


def test_late_task_has_one_breach_and_one_tardiness_settlement():
    cfg = RewardConfig()
    ledger = RLEventLedger()
    ledger.append("deadline_breached", 100.0, task_id=1,
                  business_weight=2.5)
    ledger.append("task_completed_late", 120.0, task_id=1,
                  business_weight=2.5, on_time=False,
                  tardiness_seconds=60.0)
    expected = (cfg.hard_breach_once * 2.5 + cfg.completion
                + cfg.tardiness * 0.5 * 2.5)
    assert replay_reward(ledger.events, cfg) == pytest.approx(expected)


def test_wait_cost_outweighs_bounded_age_rescue_when_delay_is_large():
    cfg = RewardConfig()
    ledger = RLEventLedger()
    ledger.append("queue_wait_advanced", 600.0,
                  pending_wait_increment=600.0)
    ledger.append("assignment_committed", 600.0,
                  waiting_seconds=600.0, empty_distance=0.0)
    assert replay_reward(ledger.events, cfg) < 0.0


def test_terminal_pending_loss_cannot_be_avoided_by_not_completing():
    cfg = RewardConfig()
    ledger = RLEventLedger()
    ledger.append("episode_terminated_with_pending", 100.0,
                  remaining_count=2, remaining_business_loss=5.0)
    assert replay_reward(ledger.events, cfg) == -5.0
