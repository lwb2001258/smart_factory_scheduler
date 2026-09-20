import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers"))

from motion_safety import MotionSafetyConfig, load_motion_safety_config


def test_defaults_are_monotonic_and_stopping_distance_includes_latency():
    config = MotionSafetyConfig()
    assert config.hard_collision_distance_m <= config.coordinated_stop_distance_m
    assert config.coordinated_stop_distance_m <= config.planning_clearance_m
    assert config.planning_clearance_m <= config.prediction_clearance_m
    assert config.stopping_distance_m == pytest.approx(0.0924)
    assert config.joint_time_slot_s >= config.worst_case_cardinal_slot_s


def test_fingerprint_changes_with_safety_contract():
    assert (MotionSafetyConfig().fingerprint() !=
            MotionSafetyConfig(reaction_latency_s=0.21).fingerprint())


def test_invalid_order_and_stale_age_fail_closed():
    with pytest.raises(ValueError, match="monotonic"):
        MotionSafetyConfig(coordinated_stop_distance_m=0.49)
    with pytest.raises(ValueError, match="stale stop"):
        MotionSafetyConfig(stale_stop_age_s=0.10)
    with pytest.raises(ValueError, match="motion bound"):
        MotionSafetyConfig(joint_time_slot_s=2.0)
    with pytest.raises(ValueError, match="positive"):
        MotionSafetyConfig(maximum_speed_mps=float("nan"))
    with pytest.raises(ValueError, match="positive"):
        MotionSafetyConfig(prediction_clearance_m=float("inf"))


def test_environment_override_is_parsed(monkeypatch):
    monkeypatch.setenv("SMART_FACTORY_SAFETY_MAXIMUM_SPEED_MPS", "0.20")
    monkeypatch.setenv("SMART_FACTORY_SAFETY_JOINT_TIME_SLOT_S", "3.6")
    assert load_motion_safety_config().maximum_speed_mps == pytest.approx(0.20)
    monkeypatch.setenv("SMART_FACTORY_SAFETY_MAXIMUM_SPEED_MPS", "bad")
    with pytest.raises(ValueError, match="finite number"):
        load_motion_safety_config()
