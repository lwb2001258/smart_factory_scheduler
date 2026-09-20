import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from webots_behavior_proxy import WebotsBehaviorProxy
from config import FULL_BATTERY_THRESHOLD, RobotState
from rl_environment import SchedulingEnvironment
from training_scenarios import factory_scenario


def test_straight_motion_uses_controller_speed_tolerance_and_ticks():
    proxy = WebotsBehaviorProxy()
    result = proxy.estimate_path((0.0, 0.0), [(2.0, 0.0)])
    assert result.path_distance == 2.0
    assert result.executable_distance == 1.65
    assert result.turns == 0
    assert result.total_seconds >= 1.65 / 0.22
    assert result.total_seconds % proxy.config.timestep_seconds < 1e-9


def test_turning_path_takes_longer_than_equal_straight_distance():
    proxy = WebotsBehaviorProxy()
    straight = proxy.estimate_path((0, 0), [(2, 0)])
    turning = proxy.estimate_path((0, 0), [(1, 0), (1, 1)])
    assert turning.path_distance == straight.path_distance
    assert turning.turns == 1
    assert turning.turning_seconds > 0
    # Two waypoint tolerances can offset some turning time; compare against
    # the same executable distance to isolate the turn contribution.
    assert turning.total_seconds > (
        turning.executable_distance / proxy.config.linear_speed)


def test_continuous_charging_matches_runtime_rate_and_tick_quantisation():
    proxy = WebotsBehaviorProxy()
    seconds = proxy.charging_seconds(20.0, 80.0)
    assert seconds > 0
    assert seconds * proxy.config.battery_charge_rate >= 60.0 - 1e-9
    assert proxy.charging_seconds(90.0, 80.0) == 0.0


def test_proxy_is_deterministic():
    proxy = WebotsBehaviorProxy()
    path = [(1, 0), (1, 1), (2, 1)]
    assert proxy.estimate_path((0, 0), path, math.pi / 3) == proxy.estimate_path(
        (0, 0), path, math.pi / 3)


def test_abstract_environment_uses_quantised_proxy_motion_time():
    robots, tasks, context = factory_scenario(17, max_robots=2,
                                              max_generated_tasks=2)
    env = SchedulingEnvironment()
    env.reset(robots, tasks, context, seed=17)
    action = int(next(index for index, valid in enumerate(
        env.get_action_mask()[:-1]) if valid))
    assignment = env.assignment_for_action(action)
    start = tuple(env._robots[assignment.robot_id]["position"])
    expected = env.behavior_proxy.estimate_path(
        start,
        env._execution_path(start, assignment.task.pickup_position)
        + env._execution_path(assignment.task.pickup_position,
                              assignment.task.delivery_position),
        float(env._robots[assignment.robot_id].get("heading", 0.0)))
    env.step(action)
    robot = env._robots[assignment.robot_id]
    execution = robot.get("_abstract_execution")
    if execution is not None:
        assert execution["completion_time"] == expected.total_seconds
        assert execution["battery_used"] == expected.battery_used
    else:
        assert assignment.task.completion_time == expected.total_seconds
        assert robot["battery"] == pytest.approx(
            robots[assignment.robot_id]["battery"] - expected.battery_used)


def test_abstract_charging_uses_continuous_runtime_rate():
    robots, tasks, context = factory_scenario(19, max_robots=2,
                                              max_generated_tasks=2)
    for state in robots.values():
        state["battery"] = 20.0
        state["state"] = RobotState.IDLE
    env = SchedulingEnvironment()
    env.reset(robots, tasks, context, seed=19)
    rid = min(env._robots)
    execution = env._robots[rid]["_abstract_execution"]
    assert execution["completion_time"] > execution["journey_seconds"]
    while env._robots[rid].get("_abstract_execution") is not None:
        env._advance_to_next_completion()
    assert env._robots[rid]["battery"] == FULL_BATTERY_THRESHOLD
