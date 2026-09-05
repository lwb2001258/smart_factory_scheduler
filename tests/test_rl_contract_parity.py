import sys
from pathlib import Path

import numpy as np
import json
import pickle


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from rl_contract import RL_SCHEDULING_CONTRACT
from rl_environment import SchedulingEnvironment
from rl_agents import DQNAgent, SarsaAgent
from schedulers import PPONetwork
from training_scenarios import factory_scenario


def test_contract_dimensions_match_environment():
    env = SchedulingEnvironment()
    assert env.observation_dim == RL_SCHEDULING_CONTRACT.observation_dim == 360
    assert env.action_dim == RL_SCHEDULING_CONTRACT.action_dim == 161
    assert env.no_op_action == RL_SCHEDULING_CONTRACT.no_op_action == 160
    assert len(RL_SCHEDULING_CONTRACT.fingerprint()) == 64


def test_action_codec_is_bijective_for_every_pair():
    env = SchedulingEnvironment()
    seen = set()
    for robot_slot in range(env.config.max_robots):
        for task_slot in range(env.config.max_tasks):
            action = env.encode_action(robot_slot, task_slot)
            assert env.decode_action(action) == (robot_slot, task_slot)
            seen.add(action)
    assert seen == set(range(env.no_op_action))
    assert env.decode_action(env.no_op_action) is None


def test_100_factory_snapshots_match_abstract_and_webots_contracts():
    """Contract acceptance gate required by workflow step 2."""
    for seed in range(100):
        robots, tasks, context = factory_scenario(seed)
        abstract = SchedulingEnvironment(simulation_mode="abstract")
        webots = SchedulingEnvironment(simulation_mode="webots")
        abstract_state, _ = abstract.reset(robots, tasks, context, seed=seed)
        webots_state = webots.set_snapshot(robots, tasks, context)
        np.testing.assert_array_equal(abstract_state, webots_state)
        np.testing.assert_array_equal(
            abstract.get_action_mask(), webots.get_action_mask())
        assert abstract._robot_slots == webots._robot_slots
        assert [item.task_id for item in abstract._task_slots] == [
            item.task_id for item in webots._task_slots]
        for action in np.flatnonzero(abstract.get_action_mask()):
            left = abstract.assignment_for_action(int(action))
            right = webots.assignment_for_action(int(action))
            if left is None or right is None:
                assert left is right is None
            else:
                assert (left.robot_id, left.task.task_id, left.estimated_cost) == (
                    right.robot_id, right.task.task_id, right.estimated_cost)


def test_new_checkpoints_embed_contract_fingerprint(tmp_path):
    expected = RL_SCHEDULING_CONTRACT.fingerprint()

    sarsa_path = tmp_path / "sarsa.json"
    SarsaAgent(161, 160).save(str(sarsa_path))
    assert json.loads(sarsa_path.read_text(
        encoding="utf-8"))["contract_fingerprint"] == expected

    dqn_path = tmp_path / "dqn.pkl"
    DQNAgent(360, 161, 160).save(str(dqn_path))
    with dqn_path.open("rb") as stream:
        assert pickle.load(stream)["contract_fingerprint"] == expected

    ppo_path = tmp_path / "ppo.npz"
    PPONetwork(360, 161, 16).save(str(ppo_path))
    with np.load(ppo_path, allow_pickle=False) as payload:
        assert int(payload["schema_version"][0]) == 4
        assert str(payload["contract_fingerprint"][0]) == expected
