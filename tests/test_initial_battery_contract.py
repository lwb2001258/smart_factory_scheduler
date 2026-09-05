import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))
sys.path.insert(0, str(ROOT / "controllers" / "robot_controller"))

from config import initial_battery_for_robot as supervisor_battery
from robot_controller import initial_battery_for_robot as controller_battery


def test_supervisor_and_robot_controller_initial_battery_match():
    for seed in (1, 42, 123, 1024):
        for robot_id in range(1, 9):
            assert supervisor_battery(seed, robot_id) == controller_battery(
                seed, robot_id)

