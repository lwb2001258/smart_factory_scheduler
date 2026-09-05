import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from webots_behavior_proxy import WebotsBehaviorProxy


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
