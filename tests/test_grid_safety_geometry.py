import sys
from pathlib import Path


SUPERVISOR = (Path(__file__).resolve().parents[1] /
              "controllers" / "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR))

from grid_planner import (
    CELL_FREE, CELL_INFLATED, CELL_OBSTACLE, GridAStar, OccupancyGrid)
from motion_coordinator import MotionCoordinator


def test_endpoint_relaxation_returns_only_shortest_cardinal_connector():
    grid = OccupancyGrid()
    grid.cells = [[CELL_OBSTACLE] * grid.cols for _ in range(grid.rows)]
    center = (10, 10)
    grid.cells[10][10] = CELL_INFLATED
    grid.cells[10][11] = CELL_INFLATED
    grid.cells[10][12] = CELL_FREE
    # An unrelated inflated neighbour must not be opened with the connector.
    grid.cells[11][10] = CELL_INFLATED
    planner = GridAStar(grid)
    assert planner._endpoint_connector(center, 2) == [(10, 10), (11, 10)]


def test_endpoint_connector_never_relaxes_real_obstacle():
    grid = OccupancyGrid()
    planner = GridAStar(grid)
    obstacle = next(
        (col, row) for row in range(grid.rows) for col in range(grid.cols)
        if grid.cells[row][col] == CELL_OBSTACLE)
    assert planner._endpoint_connector(obstacle, 4) == []


def test_motion_coordinator_segment_check_uses_configured_obstacle_source():
    coordinator = MotionCoordinator.__new__(MotionCoordinator)
    assert not coordinator._segment_clear((-4.0, 0.0), (-2.0, 0.0))
    assert coordinator._segment_clear((-8.0, 3.0), (-7.0, 3.0))
