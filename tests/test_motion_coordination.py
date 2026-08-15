import math
import sys
import unittest
from unittest.mock import patch
from pathlib import Path


SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

from motion_coordinator import MotionCoordinator


def rasterize(coordinator, start, waypoints, spacing=0.10):
    cells = set()
    points = [start] + list(waypoints)
    for first, second in zip(points, points[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        samples = max(2, int(distance / spacing) + 1)
        for index in range(samples + 1):
            ratio = index / samples
            point = (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
            cells.add(coordinator.grid.world_to_grid(*point))
    return cells


class MotionCoordinationRegressionTests(unittest.TestCase):
    def test_joint_grid_validation_rejection_is_counted_as_failure(self):
        coordinator = MotionCoordinator(2)
        agents = {
            1: ((-1.0, 3.0), (1.0, 3.0)),
            2: ((1.0, 3.0), (-1.0, 3.0)),
        }
        with patch.object(coordinator.joint_grid_planner, "validate",
                          return_value=False):
            # A rejected primary plan is allowed to fall back to the short
            # rolling planner; it is no longer a hard failure because that
            # fallback preserves fleet progress in dense scenario C.
            self.assertIsNotNone(coordinator.plan_joint_grid_candidate(
                agents, max_seconds=1.0))
        statistics = coordinator.get_statistics()
        self.assertEqual(1, statistics["joint_grid_candidates_attempted"])
        self.assertEqual(1, statistics["joint_grid_candidates_validated"])
        self.assertEqual(0, statistics["joint_grid_candidates_timed_out"])
        self.assertEqual(0, statistics["joint_grid_candidates_failed"])

    def test_joint_grid_candidate_covers_and_validates_all_agents(self):
        coordinator = MotionCoordinator(3)
        agents = {
            1: ((-7.0, 3.0), (7.0, 3.0)),
            2: ((0.0, -4.0), (0.0, 4.0)),
            3: ((7.0, 2.0), (-7.0, 2.0)),
        }
        candidate = coordinator.plan_joint_grid_candidate(
            agents, max_seconds=1.0)
        self.assertIsNotNone(candidate)
        self.assertEqual(set(agents), set(candidate.paths))
        self.assertTrue(coordinator.joint_grid_planner.validate(candidate, 4))
        self.assertEqual(1, coordinator.joint_grid_candidates_attempted)
        self.assertEqual(1, coordinator.joint_grid_candidates_validated)
        statistics = coordinator.get_statistics()
        self.assertEqual(1, statistics["joint_grid_candidates_attempted"])
        self.assertEqual(1, statistics["joint_grid_candidates_validated"])

    def test_near_term_spatial_prefix_is_clipped(self):
        points = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0)]
        prefix = MotionCoordinator._polyline_prefix(points, 4.0)
        self.assertEqual([(0.0, 0.0), (3.0, 0.0), (4.0, 0.0)], prefix)

    def test_joint_activation_gate_delays_crossing_trajectory(self):
        coordinator = MotionCoordinator(2)
        delay = coordinator._joint_trajectory_delay(
            (-1.0, 0.0), [(1.0, 0.0)],
            [(0.0, -1.0), (0.0, 1.0)])
        self.assertIsNotNone(delay)
        self.assertGreater(delay, 0.0)

    def test_joint_activation_gate_accepts_separated_trajectory(self):
        coordinator = MotionCoordinator(2)
        delay = coordinator._joint_trajectory_delay(
            (-1.0, 0.0), [(1.0, 0.0)],
            [(-1.0, 2.0), (1.0, 2.0)])
        self.assertEqual(0.0, delay)

    def test_runtime_replan_counter_has_no_plan_cleanup_side_effect(self):
        coordinator = MotionCoordinator(2)
        coordinator.robot_paths[1] = ["active-plan"]
        before = coordinator.total_replans
        coordinator.record_runtime_replan()
        self.assertEqual(before + 1, coordinator.total_replans)
        self.assertEqual(["active-plan"], coordinator.robot_paths[1])

    def test_final_candidate_passes_independent_conflict_recheck(self):
        coordinator = MotionCoordinator(2)
        start = (-4.0, 3.0)
        path = coordinator.plan_grid_lifelong(1, start, "WS3")
        self.assertTrue(path)
        self.assertTrue(coordinator.validate_candidate_plan(1, start, path))

        reservation = coordinator.resource_reservations.snapshot_owner(1)[0]
        coordinator.resource_reservations._items.append(type(reservation)(
            2, reservation.resource, reservation.start, reservation.end))
        self.assertFalse(coordinator.validate_candidate_plan(1, start, path))

    def test_failed_replan_preserves_active_reservations(self):
        coordinator = MotionCoordinator(3)
        path = coordinator.plan_grid_lifelong(1, (-4.0, 3.0), "WS3")
        self.assertTrue(path)
        old_grid = set(coordinator._grid_path_reservations[1])
        old_coordinates = list(coordinator.coordinate_paths[1])
        old_timed = tuple(
            item for item in coordinator.resource_reservations.snapshot()
            if item.owner == 1
        )

        coordinator.grid_planner.plan = lambda *args, **kwargs: None
        coordinator._plan_space_time_detour = lambda *args, **kwargs: None
        result = coordinator.plan_grid_lifelong(1, (-3.8, 3.0), "WS1")

        self.assertIsNone(result)
        self.assertEqual(old_grid, coordinator._grid_path_reservations[1])
        self.assertEqual(old_coordinates, coordinator.coordinate_paths[1])
        self.assertEqual(
            old_timed,
            tuple(item for item in coordinator.resource_reservations.snapshot()
                  if item.owner == 1),
        )

    def test_replan_exception_preserves_active_reservations(self):
        coordinator = MotionCoordinator(3)
        self.assertTrue(
            coordinator.plan_grid_lifelong(1, (-4.0, 3.0), "WS3"))
        old_grid = set(coordinator._grid_path_reservations[1])
        old_coordinates = list(coordinator.coordinate_paths[1])
        old_timed = tuple(
            item for item in coordinator.resource_reservations.snapshot()
            if item.owner == 1
        )

        def fail(*args, **kwargs):
            raise RuntimeError("synthetic planner failure")

        coordinator.grid_planner.plan = fail
        with self.assertRaisesRegex(RuntimeError, "synthetic planner failure"):
            coordinator.plan_grid_lifelong(1, (-3.8, 3.0), "WS1")

        self.assertEqual(old_grid, coordinator._grid_path_reservations[1])
        self.assertEqual(old_coordinates, coordinator.coordinate_paths[1])
        self.assertEqual(
            old_timed,
            tuple(item for item in coordinator.resource_reservations.snapshot()
                  if item.owner == 1),
        )

    def test_final_path_cells_are_reserved(self):
        cases = (
            (1, (-4.0, 3.0), "WS3"),
            (2, (-4.0, -3.0), "WS6"),
            (5, (-7.0, 0.0), "WS2"),
            (6, (7.0, 0.0), "WS5"),
        )
        for robot_id, start, goal in cases:
            with self.subTest(robot_id=robot_id, goal=goal):
                coordinator = MotionCoordinator(8)
                path = coordinator.plan_grid_lifelong(
                    robot_id, start, goal)
                self.assertTrue(path)
                actual = rasterize(coordinator, start, path)
                reserved = coordinator._grid_path_reservations[robot_id]
                self.assertFalse(actual - reserved)

    def test_temporal_delay_never_decreases_existing_delay(self):
        coordinator = MotionCoordinator(2)
        shared = [(0.0, 0.0), (2.0, 0.0)]
        coordinator.coordinate_paths[1] = shared
        coordinator.coordinate_paths[2] = shared
        coordinator._dispatch_delays[2] = 20.0

        coordinator._inject_delay_if_temporal_conflict(
            2, (0.0, 0.0), [(2.0, 0.0)])

        self.assertGreaterEqual(coordinator._dispatch_delays[2], 20.0)

    def test_timed_reservation_respects_minimum_dispatch_delay(self):
        coordinator = MotionCoordinator(2)
        coordinator._dispatch_delays[2] = 7.25
        path = coordinator.plan_grid_lifelong(2, (-4.0, -3.0), "WS6")
        self.assertTrue(path)
        reservations = [
            item for item in coordinator.resource_reservations.snapshot()
            if item.owner == 2
        ]
        self.assertTrue(reservations)
        self.assertGreaterEqual(
            min(item.start for item in reservations),
            coordinator.current_time_seconds + 7.25,
        )

    def test_absolute_simulation_time_prunes_expired_reservations(self):
        coordinator = MotionCoordinator(2)
        coordinator.set_sim_time(10.0)
        self.assertTrue(coordinator.plan_grid_lifelong(
            1, (-4.0, 3.0), "WS1"))
        coordinator.commit_robot_plan(1)
        self.assertTrue(coordinator.resource_reservations.snapshot_owner(1))
        coordinator.set_sim_time(1000.0)
        self.assertFalse(coordinator.resource_reservations.snapshot_owner(1))

    def test_refresh_reanchors_timing_to_actual_progress_and_hold(self):
        coordinator = MotionCoordinator(2)
        coordinator.set_sim_time(50.0)
        remaining = [(0.0, 3.0), (6.0, 3.5)]
        self.assertTrue(coordinator.refresh_active_plan_timing(
            1, (-2.0, 3.0), remaining, not_before=55.0))
        reservations = coordinator.resource_reservations.snapshot_owner(1)
        self.assertTrue(reservations)
        self.assertGreaterEqual(min(item.start for item in reservations), 55.0)

    def test_dispatch_rollback_restores_previous_plan(self):
        coordinator = MotionCoordinator(3)
        first = coordinator.plan_grid_lifelong(1, (-4.0, 3.0), "WS1")
        self.assertTrue(first)
        coordinator.commit_robot_plan(1)
        old_grid = set(coordinator._grid_path_reservations[1])
        old_coordinates = list(coordinator.coordinate_paths[1])
        old_timed = coordinator.resource_reservations.snapshot_owner(1)

        second = coordinator.plan_grid_lifelong(1, (-3.5, 3.0), "WS3")
        self.assertTrue(second)
        self.assertNotEqual(old_coordinates, coordinator.coordinate_paths[1])
        coordinator.rollback_robot_plan(1)

        self.assertEqual(old_grid, coordinator._grid_path_reservations[1])
        self.assertEqual(old_coordinates, coordinator.coordinate_paths[1])
        self.assertEqual(
            old_timed, coordinator.resource_reservations.snapshot_owner(1))


if __name__ == "__main__":
    unittest.main()
