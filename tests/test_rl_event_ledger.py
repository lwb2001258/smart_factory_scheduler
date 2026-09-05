import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from rl_environment import RewardConfig, SchedulingEnvironment
from rl_event_ledger import RLEventLedger, replay_reward, reward_for_event
from training_scenarios import factory_scenario


def test_assignment_reward_is_pure_and_replayable():
    cfg = RewardConfig()
    ledger = RLEventLedger()
    event = ledger.append(
        "assignment_committed", 10.0, robot_id=2, task_id=3,
        priority_rank=300, waiting_seconds=60.0, empty_distance=5.0)
    expected = (cfg.valid_assignment + cfg.age_rescue * 0.5
                + cfg.empty_distance * (5.0 / cfg.distance_scale_metres))
    assert reward_for_event(event, cfg) == pytest.approx(expected)
    assert replay_reward(ledger.events, cfg) == pytest.approx(expected)


def test_operational_events_are_recorded_but_reward_neutral():
    cfg = RewardConfig()
    ledger = RLEventLedger()
    for event_type in ("pickup_reached", "charging_started",
                       "planned_wait"):
        ledger.append(event_type, 1.0)
    assert replay_reward(ledger.events, cfg) == 0.0


def test_online_environment_reward_equals_offline_event_replay():
    robots, tasks, context = factory_scenario(
        91, min_robots=3, max_robots=3, max_generated_tasks=5)
    env = SchedulingEnvironment(simulation_mode="abstract")
    state, _ = env.reset(robots, tasks, context, seed=91)
    online = 0.0
    terminated = truncated = False
    while not (terminated or truncated):
        action = int(env.get_action_mask().nonzero()[0][0])
        state, reward, terminated, truncated, _ = env.step(action)
        online += reward
    assert online == pytest.approx(
        replay_reward(env.event_ledger.events, env.reward_config))
