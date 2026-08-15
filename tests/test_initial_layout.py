import re
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR_DIR = ROOT / "controllers" / "factory_supervisor"
sys.path.insert(0, str(SUPERVISOR_DIR))

from config import CHARGING_STATIONS, PARKING_SPOTS


class InitialLayoutTests(unittest.TestCase):
    def test_world_robot_positions_match_parking_source_of_truth(self):
        world = (ROOT / "worlds" / "smart_factory.wbt").read_text(
            encoding="utf-8")
        for robot_id, expected in PARKING_SPOTS.items():
            match = re.search(
                rf"DEF ROBOT_{robot_id} Robot \{{\s*translation "
                rf"([-\d.]+) ([-\d.]+)", world)
            self.assertIsNotNone(match, robot_id)
            actual = (float(match.group(1)), float(match.group(2)))
            self.assertEqual(tuple(expected), actual)

    def test_extra_robots_are_near_but_not_on_charging_pads(self):
        for robot_id, station in ((7, "CS1"), (8, "CS2")):
            home = PARKING_SPOTS[robot_id]
            charge = CHARGING_STATIONS[station]
            distance = ((home[0] - charge[0]) ** 2 +
                        (home[1] - charge[1]) ** 2) ** 0.5
            self.assertGreaterEqual(distance, 1.0)
            self.assertLessEqual(distance, 1.5)

    def test_visual_dividers_are_floor_markings_below_lidar(self):
        world = (ROOT / "worlds" / "smart_factory.wbt").read_text(
            encoding="utf-8")
        for divider_id in range(1, 5):
            marker = world.index(f'name "divider_{divider_id}"')
            translations = re.findall(
                r"translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)",
                world[max(0, marker - 400):marker])
            self.assertTrue(translations, divider_id)
            z = float(translations[-1][2])
            self.assertLess(z, 0.01)


if __name__ == "__main__":
    unittest.main()
