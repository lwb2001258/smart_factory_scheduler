import sys
import time
import unittest
from pathlib import Path

SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

from grid_planner import OccupancyGrid
from joint_grid_planner import JointGridPlanner


class JointGridPlannerTests(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid()
        self.planner = JointGridPlanner(
            self.grid, horizon_slots=4, time_slot_seconds=3.0,
            separation_cells=4)

    def assert_joint_plan(self, agents):
        plan = self.planner.plan(agents, max_seconds=1.0)
        self.assertIsNotNone(plan)
        self.assertEqual(set(agents), set(plan.paths))
        self.assertTrue(self.planner.validate(plan, 4))
        return plan

    def test_opposite_direction_robots_are_jointly_separated(self):
        self.assert_joint_plan({
            1: ((-7.0, 3.0), (7.0, 3.0)),
            2: ((7.0, 2.0), (-7.0, 2.0)),
        })

    def test_squared_grid_conflict_test_matches_world_euclidean_distance(self):
        planner = JointGridPlanner(
            self.grid, horizon_slots=4, time_slot_seconds=3.0,
            separation_cells=3, minimum_distance_m=0.70)
        for dc in range(-5, 6):
            for dr in range(-5, 6):
                first = (20, 20)
                second = (20 + dc, 20 + dr)
                first_xy = self.grid.grid_to_world(*first)
                second_xy = self.grid.grid_to_world(*second)
                expected = (
                    (first_xy[0] - second_xy[0]) ** 2 +
                    (first_xy[1] - second_xy[1]) ** 2
                ) ** 0.5 <= 0.70
                self.assertEqual(expected, planner._cells_conflict(first, second))
        self.assertIn((2, 1), planner._conflict_offsets)
        self.assertNotIn((2, 2), planner._conflict_offsets)

    def test_true_same_lane_head_on_is_resolved_in_space_time(self):
        self.assert_joint_plan({
            1: ((-1.0, 3.0), (1.0, 3.0)),
            2: ((1.0, 3.0), (-1.0, 3.0)),
        })

    def test_only_bounded_endpoint_connector_relaxes_inflation(self):
        center = self.grid.world_to_grid(0.0, -4.0)
        connector = self.planner._endpoint_cells(center)
        self.assertLessEqual(len(connector), 4)
        self.assertIn(center, connector)
        self.assertFalse(any(
            self.grid.cells[row][col] == 1 for col, row in connector))

    def test_crossing_robots_are_jointly_separated(self):
        self.assert_joint_plan({
            1: ((-7.0, 3.0), (7.0, 3.0)),
            2: ((0.0, -4.0), (0.0, 4.0)),
        })

    def test_three_robot_chain_is_planned_as_one_batch(self):
        plan = self.assert_joint_plan({
            1: ((-7.0, 3.0), (7.0, 3.0)),
            2: ((0.0, -4.0), (0.0, 4.0)),
            3: ((7.0, 2.0), (-7.0, 2.0)),
        })
        self.assertEqual(3, len(plan.order))

    def test_eight_robot_candidate_respects_runtime_budget(self):
        agents = {
            1: ((-7.0, 3.0), (7.0, 3.0)),
            2: ((7.0, 2.0), (-7.0, 2.0)),
            3: ((-7.0, -3.0), (7.0, -3.0)),
            4: ((7.0, -2.0), (-7.0, -2.0)),
            5: ((-8.0, 0.0), (0.0, 4.0)),
            6: ((8.0, 0.0), (0.0, -4.0)),
            7: ((-4.0, 4.0), (4.0, -4.0)),
            8: ((4.0, 4.0), (-4.0, -4.0)),
        }
        started = time.perf_counter()
        plan = self.planner.plan(agents, max_seconds=0.20)
        elapsed = time.perf_counter() - started
        self.assertLess(elapsed, 0.23)
        self.assertIsNotNone(plan)
        self.assertTrue(self.planner.validate(plan, 4))

    def test_recovery_corridor_cells_are_reserved_for_entire_horizon(self):
        planner = JointGridPlanner(
            self.grid, horizon_slots=4, time_slot_seconds=3.0,
            separation_cells=1)
        blocked = {self.grid.world_to_grid(0.0, 3.0)}
        plan = planner.plan(
            {1: ((-1.0, 3.0), (1.0, 3.0))}, max_seconds=1.0,
            blocked_cells=blocked)
        self.assertIsNotNone(plan)
        for step in plan.paths[1]:
            self.assertTrue(all(
                not planner._cells_conflict(step.cell, cell)
                for cell in blocked))

    def test_recovery_occupancy_moves_forward_by_time_slot(self):
        planner = JointGridPlanner(
            self.grid, horizon_slots=4, time_slot_seconds=3.0,
            separation_cells=1)
        blocked_by_slot = {
            0: {self.grid.world_to_grid(-0.5, 3.0)},
            1: {self.grid.world_to_grid(0.0, 3.0)},
            2: {self.grid.world_to_grid(0.5, 3.0)},
        }
        plan = planner.plan(
            {1: ((-1.0, 2.5), (1.0, 2.5))}, max_seconds=1.0,
            blocked_cells_by_slot=blocked_by_slot)
        self.assertIsNotNone(plan)
        for step in plan.paths[1]:
            self.assertTrue(all(
                not planner._cells_conflict(step.cell, cell)
                for cell in blocked_by_slot.get(step.time_slot, set())))

if __name__ == '__main__':
    unittest.main()
