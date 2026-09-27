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
    def test_precomputed_vertex_mask_matches_cell_conflict_semantics(self):
        for minimum_distance in (None, 0.70, 0.75):
            planner = JointGridPlanner(
                self.grid, separation_cells=3,
                minimum_distance_m=minimum_distance)
            offsets = set(planner._vertex_conflict_offsets)
            radius = planner._conflict_radius_cells()
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    self.assertEqual(
                        planner._cells_conflict((10, 10),
                                                (10 + dc, 10 + dr)),
                        (dc, dr) in offsets)

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

    def test_expansion_budget_is_a_deterministic_search_stop(self):
        plan = self.planner.plan(
            {1: ((-7.0, 3.0), (7.0, 3.0))},
            max_seconds=1.0, max_expansions=1)
        self.assertIsNone(plan)
        self.assertEqual("expansion_budget", self.planner.last_failure_reason)
        self.assertEqual(1, self.planner.last_expanded_nodes)
        self.assertEqual(1, self.planner.last_orders_evaluated)


if __name__ == '__main__':
    unittest.main()
