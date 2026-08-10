"""
Multi-Robot Motion Coordinator with Prioritised Planning and Conflict-Based Search.

Implements:
1. A* path planning on the factory graph
2. Prioritised planning for multi-robot coordination
3. Conflict detection and resolution (space-time conflicts)
4. Deadlock detection and resolution via priority-based yielding
"""

import heapq
import math
import time
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass, field
from config import (
    WAYPOINTS, GRAPH_EDGES, LOCATION_TO_NODE,
    ROBOT_RADIUS, GOAL_TOLERANCE, ALL_LOCATIONS,
    WORKSTATIONS, STORAGE_AREAS, CHARGING_STATIONS,
    OBSTACLE_BOXES, line_intersects_obstacles, REST_NODES,
    SPACE_TIME_DETOUR_BUDGET_SECONDS,
    SPACE_TIME_DETOUR_MAX_EXPANSIONS,
)
from cbs_planner import CBSPlanner, LifelongPlanner
from grid_planner import OccupancyGrid, GridAStar, subsample_path
from safety_coordination import ReservationManager, priority_key


@dataclass(order=True)
class PriorityItem:
    """Priority queue item for A* search."""
    priority: float
    item: object = field(compare=False)


@dataclass
class PathStep:
    """A single step in a robot's planned path."""
    node: str
    position: Tuple[float, float]
    time_step: int  # discrete time step
    
    
@dataclass
class SpaceTimeConflict:
    """Represents a conflict between two robots in space-time."""
    robot_a: int
    robot_b: int
    node: str
    time_step: int
    conflict_type: str  # "vertex" or "edge"


class FactoryGraph:
    """Graph representation of the factory corridor network for path planning."""
    
    def __init__(self):
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.edges: Dict[str, List[str]] = {}
        self.edge_weights: Dict[Tuple[str, str], float] = {}
        self._build_graph()

    def _build_graph(self):
        """Build the navigation graph from configuration."""
        # Add all waypoint nodes
        for name, pos in WAYPOINTS.items():
            self.nodes[name] = pos
            self.edges[name] = []

        # Add location nodes that map to existing waypoints
        # These are handled via LOCATION_TO_NODE mapping

        # Add edges (bidirectional)
        for (a, b) in GRAPH_EDGES:
            if a in self.nodes and b in self.nodes:
                self.edges[a].append(b)
                self.edges[b].append(a)
                dist = self._distance(self.nodes[a], self.nodes[b])
                self.edge_weights[(a, b)] = dist
                self.edge_weights[(b, a)] = dist

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def get_nearest_node(self, position: Tuple[float, float]) -> str:
        """
        Find the nearest graph node REACHABLE from `position` by a
        straight line that does not cross any inflated obstacle.
        
        Strategy:
          1. Sort all graph nodes by Euclidean distance.
          2. For each candidate (closest first), check if the straight
             line from `position` to the node intersects any inflated
             obstacle box (shelves / workstation tables).
          3. Return the first collision-free candidate.
          4. Fallback: if every node has an obstructed straight path
             (extremely rare — would mean robot is fully enclosed),
             return the absolute closest one anyway, and let DWA /
             sidestep handle the local manoeuvre.
        
        This guards against the case where the robot has drifted off
        the corridor (e.g. after a sidestep manoeuvre, after avoiding
        a peer, or after a transient GPS jitter) and the straight-line
        first-hop to the nearest graph node would otherwise cut
        through a shelf or table.
        """
        # Sort all nodes by Euclidean distance to the query position
        candidates = sorted(
            self.nodes.items(),
            key=lambda kv: self._distance(position, kv[1])
        )
        # Return first candidate whose straight-line first-hop is clear
        for name, pos in candidates:
            if not line_intersects_obstacles(position, pos):
                return name
        # Fallback: nothing was clear, return closest anyway
        return candidates[0][0] if candidates else None

    def get_node_position(self, node: str) -> Tuple[float, float]:
        """Get the (x, y) position of a graph node."""
        return self.nodes[node]

    def get_location_node(self, location_name: str) -> str:
        """Get the graph node associated with a task location."""
        return LOCATION_TO_NODE.get(location_name, self.get_nearest_node(
            ALL_LOCATIONS.get(location_name, (0, 0))
        ))


class AStarPlanner:
    """A* path planner on the factory graph."""

    def __init__(self, graph: FactoryGraph):
        self.graph = graph

    def plan(self, start_node: str, goal_node: str,
             blocked_nodes: Optional[Set[str]] = None,
             blocked_edges: Optional[Set[Tuple[str, str]]] = None
             ) -> Optional[List[str]]:
        """
        Find shortest path from start to goal using A*.
        
        Args:
            start_node: Starting graph node.
            goal_node: Target graph node.
            blocked_nodes: Nodes to avoid (occupied by other robots at specific times).
            blocked_edges: Edges to avoid.
            
        Returns:
            List of node names forming the path, or None if no path found.
        """
        if start_node == goal_node:
            return [start_node]

        if blocked_nodes is None:
            blocked_nodes = set()
        if blocked_edges is None:
            blocked_edges = set()

        open_set = []
        heapq.heappush(open_set, PriorityItem(0, start_node))
        
        came_from: Dict[str, str] = {}
        g_score: Dict[str, float] = {start_node: 0}
        f_score: Dict[str, float] = {start_node: self._heuristic(start_node, goal_node)}
        
        closed_set: Set[str] = set()

        while open_set:
            current_item = heapq.heappop(open_set)
            current = current_item.item
            
            if current == goal_node:
                return self._reconstruct_path(came_from, current)
            
            if current in closed_set:
                continue
            closed_set.add(current)

            for neighbor in self.graph.edges.get(current, []):
                if neighbor in blocked_nodes:
                    continue
                if (current, neighbor) in blocked_edges:
                    continue
                    
                tentative_g = g_score[current] + self.graph.edge_weights.get(
                    (current, neighbor), float('inf')
                )
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_node)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, PriorityItem(f, neighbor))

        return None  # No path found

    def _heuristic(self, node: str, goal: str) -> float:
        """Euclidean distance heuristic for A*."""
        p1 = self.graph.nodes[node]
        p2 = self.graph.nodes[goal]
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _reconstruct_path(self, came_from: Dict[str, str], current: str) -> List[str]:
        """Reconstruct path from A* search results."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


class MotionCoordinator:
    """
    Multi-robot motion coordinator using prioritised planning with
    conflict detection and resolution.
    
    Architecture:
    - Centralised planning: computes paths for all robots
    - Prioritised: higher-priority robots plan first
    - Conflict resolution: re-plans lower-priority robots when conflicts detected
    - Deadlock detection: identifies circular wait conditions
    """

    def __init__(self, num_active_robots: int = 8):
        self.num_active_robots = num_active_robots
        self.graph = FactoryGraph()
        self.planner = AStarPlanner(self.graph)
        # CBS multi-agent planner (uses the same factory graph)
        self.cbs = CBSPlanner(
            graph_nodes=self.graph.nodes,
            graph_adj=self.graph.edges,
        )
        # Lifelong (online, Cooperative A*) planner — preferred for
        # the continuous-task-stream use-case. CBS is reserved for
        # batch / offline replanning on demand.
        self.lifelong = LifelongPlanner(
            graph_nodes=self.graph.nodes,
            graph_adj=self.graph.edges,
            lookahead_horizon=120,
        )
        
        # ── Grid-based free-space planner ──
        # Replaces corridor-only routing with full 2D grid (0.25m/cell)
        # so robots can take shortest paths around shelves/tables instead
        # of being forced through hand-placed corridor nodes.
        self.grid = OccupancyGrid(num_active_robots=self.num_active_robots if hasattr(self, 'num_active_robots') else 8)
        self.grid_planner = GridAStar(self.grid)
        
        # Grid path-cell reservations: maps robot_id → set of (col, row) cells
        # currently reserved by that robot's active path. When planning for
        # another robot, these cells (plus a 1-cell inflation) are treated
        # as TEMPORARY OBSTACLES so A* routes around active peers.
        self._grid_path_reservations = {}
        self._dispatch_delays: Dict[int, float] = {}
        
        # Current robot paths: robot_id -> list of PathSteps
        self.robot_paths: Dict[int, List[PathStep]] = {}
        # Grid planner paths use world coordinates and must not be mixed with
        # the topological PathStep representation above.
        self.coordinate_paths: Dict[int, List[Tuple[float, float]]] = {}
        
        # Robot priorities: lower number = higher priority
        self.robot_priorities: Dict[int, int] = {}
        
        # Space-time reservation table: (node, time_step) -> robot_id
        self.reservations: Dict[Tuple[str, int], int] = {}
        self.resource_reservations = ReservationManager()
        
        # Conflict history for analysis
        self.conflicts_detected: List[SpaceTimeConflict] = []
        self.conflicts_resolved: int = 0
        self.deadlocks_detected: int = 0
        self.total_replans: int = 0
        self._deadlock_break_count = 0
        self._priority_order = list(range(1, num_active_robots + 1))

    def set_priorities(self, robot_ids: List[int]):
        """
        Set robot priorities. Robots with active tasks get higher priority.
        Ties are broken by robot ID (lower ID = higher priority).
        """
        for i, rid in enumerate(robot_ids):
            self.robot_priorities[rid] = i


    def _segment_clear(self, p1, p2) -> bool:
        """True iff the straight segment p1→p2 doesn't hit any
        physical obstacle (shelves + workstation tables, inflated
        by ROBOT_RADIUS). Used to filter start-node candidates so
        the robot doesn't have to drive through an obstacle to
        reach the first waypoint."""
        # Inflated obstacle list (matches scripts/verify_layout.py)
        r = ROBOT_RADIUS
        obstacles = [
            (-3.0, 0.0, 0.5 + r, 1.0 + r),  # shelf_1
            (-1.0, 0.0, 0.5 + r, 1.0 + r),  # shelf_2
            ( 1.0, 0.0, 0.5 + r, 1.0 + r),  # shelf_3
            ( 3.0, 0.0, 0.5 + r, 1.0 + r),  # shelf_4
            (-6.0,  5.0, 1.1 + r, 0.85 + r),  # WS1 table
            ( 0.0,  5.0, 1.1 + r, 0.85 + r),  # WS2 table
            ( 6.0,  5.0, 1.1 + r, 0.85 + r),  # WS3 table
            (-6.0, -5.0, 1.1 + r, 0.85 + r),  # WS4 table
            ( 0.0, -5.0, 1.1 + r, 0.85 + r),  # WS5 table
            ( 6.0, -5.0, 1.1 + r, 0.85 + r),  # WS6 table
        ]
        x1, y1 = p1
        x2, y2 = p2
        for cx, cy, hx, hy in obstacles:
            minx, maxx = cx - hx, cx + hx
            miny, maxy = cy - hy, cy + hy
            dx, dy = x2 - x1, y2 - y1
            t0, t1 = 0.0, 1.0
            ok = True
            for p, q in [(-dx, x1 - minx), (dx, maxx - x1),
                         (-dy, y1 - miny), (dy, maxy - y1)]:
                if abs(p) < 1e-12:
                    if q < 0:
                        ok = False
                        break
                else:
                    r_t = q / p
                    if p < 0:
                        if r_t > t1:
                            ok = False
                            break
                        if r_t > t0:
                            t0 = r_t
                    else:
                        if r_t < t0:
                            ok = False
                            break
                        if r_t < t1:
                            t1 = r_t
            if ok and t0 < t1:
                return False
        return True

    def plan_path_for_robot(self, robot_id: int, 
                             current_position: Tuple[float, float],
                             goal_location: str) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path for a single robot considering other robots' paths.
        
        Args:
            robot_id: The robot's ID.
            current_position: Robot's current (x, y) position on factory floor.
            goal_location: Target location name (e.g., "WS1", "S3").
            
        Returns:
            List of (x, y) waypoints to follow, or None if no path found.
        """
        # ----------------------------------------------------------
        # Find START node — three-tier strategy:
        #   1) If robot is AT a known task location (just finished
        #      pickup), use LOCATION_TO_NODE for that location.
        #   2) Otherwise, choose the *closest* graph node, breaking
        #      ties by which one is closer to the goal (avoids the
        #      "north-detour" bug where robot goes the wrong way
        #      because two nodes are equidistant).
        #   3) Reject candidates whose first segment from
        #      current_position is not collision-free.
        # ----------------------------------------------------------
        # Resolve goal position once (used for tie-breaking)
        _goal_pos = ALL_LOCATIONS.get(goal_location,
                    CHARGING_STATIONS.get(goal_location, (0, 0)))

        start_node = None
        # Tier 1: exact-location lookup
        for _loc_name, _loc_pos in ALL_LOCATIONS.items():
            if (abs(current_position[0] - _loc_pos[0]) < 0.4 and
                    abs(current_position[1] - _loc_pos[1]) < 0.4 and
                    _loc_name in LOCATION_TO_NODE):
                start_node = LOCATION_TO_NODE[_loc_name]
                break

        # Tier 2: goal-aware nearest collision-free node
        if start_node is None:
            best_score = float('inf')
            best_name = None
            for _n_name, _n_pos in self.graph.nodes.items():
                # First segment must be collision-free
                if not self._segment_clear(current_position, _n_pos):
                    continue
                # Composite cost = dist(robot→node) + dist(node→goal)
                # This prefers nodes that lie *toward* the goal.
                d_in = math.hypot(current_position[0] - _n_pos[0],
                                   current_position[1] - _n_pos[1])
                d_out = math.hypot(_n_pos[0] - _goal_pos[0],
                                    _n_pos[1] - _goal_pos[1])
                score = d_in + d_out
                if score < best_score:
                    best_score = score
                    best_name = _n_name
            start_node = best_name or self.graph.get_nearest_node(current_position)
        
        # Find goal node
        if goal_location in LOCATION_TO_NODE:
            goal_node = LOCATION_TO_NODE[goal_location]
        else:
            # Try to find in all locations
            goal_pos = ALL_LOCATIONS.get(goal_location, 
                        CHARGING_STATIONS.get(goal_location, (0, 0)))
            goal_node = self.graph.get_nearest_node(goal_pos)

        # Get blocked nodes from other robots' current reservations
        my_priority = self.robot_priorities.get(robot_id, robot_id)
        blocked_nodes = set()
        blocked_edges = set()
        
        # Higher priority robots (lower priority number) block nodes
        for other_id, path in self.robot_paths.items():
            if other_id == robot_id:
                continue
            other_priority = self.robot_priorities.get(other_id, other_id)
            if other_priority < my_priority:
                # This robot has higher priority, avoid its path nodes
                for step in path[:3]:  # Only consider near-future steps
                    # Keep compatibility with paths produced before the
                    # coordinate/topological stores were separated.
                    node = getattr(step, "node", None)
                    if node is not None:
                        blocked_nodes.add(node)

        # Plan path with A*
        path_nodes = self.planner.plan(start_node, goal_node, 
                                        blocked_nodes, blocked_edges)
        
        if path_nodes is None:
            # Try again without blocks (fallback)
            path_nodes = self.planner.plan(start_node, goal_node)
            if path_nodes is None:
                return None

        # Convert to path steps with time information
        path_steps = []
        for i, node in enumerate(path_nodes):
            pos = self.graph.get_node_position(node)
            path_steps.append(PathStep(node=node, position=pos, time_step=i))

        # Store the planned path
        self.robot_paths[robot_id] = path_steps

        # Update reservations
        self._update_reservations(robot_id, path_steps)

        # Convert to waypoint positions.
        # Strip the first node if it equals current_position (avoids
        # the redundant "current → current" segment).
        waypoints = [step.position for step in path_steps]
        if waypoints and (
                abs(waypoints[0][0] - current_position[0]) < 0.05 and
                abs(waypoints[0][1] - current_position[1]) < 0.05):
            waypoints.pop(0)
        
        # Add the actual goal position as the final waypoint
        goal_pos = ALL_LOCATIONS.get(goal_location,
                    CHARGING_STATIONS.get(goal_location))
        if goal_pos and waypoints:
            waypoints.append(goal_pos)

        return waypoints

    def plan_all_paths(self, robot_states: Dict[int, dict]) -> Dict[int, List[Tuple[float, float]]]:
        """
        Plan paths for all robots using prioritised planning.
        
        Args:
            robot_states: Dict of robot_id -> {position, goal_location, state, has_task}
            
        Returns:
            Dict of robot_id -> list of waypoints
        """
        # Clear existing plans
        self.robot_paths.clear()
        self.reservations.clear()

        # Sort robots by priority
        active_robots = [(rid, state) for rid, state in robot_states.items()
                         if state.get('goal_location') is not None]
        
        # Prioritise: robots with tasks first, then by distance to goal
        def sort_key(item):
            rid, state = item
            priority = self.robot_priorities.get(rid, rid)
            has_task = 1 if state.get('has_task', False) else 0
            return (-has_task, priority)  # task-bearing robots first
        
        active_robots.sort(key=sort_key)

        # Plan paths in priority order
        all_paths = {}
        for robot_id, state in active_robots:
            path = self.plan_path_for_robot(
                robot_id,
                state['position'],
                state['goal_location']
            )
            if path:
                all_paths[robot_id] = path

        # Detect and resolve conflicts
        self._detect_and_resolve_conflicts(robot_states)

        return all_paths

    def _update_reservations(self, robot_id: int, path_steps: List[PathStep]):
        """Update the space-time reservation table for a robot's planned path."""
        # Remove old reservations for this robot
        to_remove = [key for key, rid in self.reservations.items() if rid == robot_id]
        for key in to_remove:
            del self.reservations[key]
        
        # Add new reservations
        for step in path_steps:
            key = (step.node, step.time_step)
            if key in self.reservations and self.reservations[key] != robot_id:
                # Conflict detected
                other_id = self.reservations[key]
                self.conflicts_detected.append(SpaceTimeConflict(
                    robot_a=robot_id,
                    robot_b=other_id,
                    node=step.node,
                    time_step=step.time_step,
                    conflict_type="vertex"
                ))
            self.reservations[key] = robot_id
        timed = []
        for step in path_steps:
            timed.append((("node", step.node), step.time_step, step.time_step + 1))
        for a, b in zip(path_steps, path_steps[1:]):
            timed.append((("edge", a.node, b.node), a.time_step, b.time_step))
        self.resource_reservations.reserve_batch(robot_id, timed)

    def _detect_and_resolve_conflicts(self, robot_states: Dict[int, dict]):
        """
        Detect conflicts between planned paths and resolve them.
        Uses priority-based resolution: lower-priority robot re-plans.
        """
        conflicts = self._find_conflicts()
        
        for conflict in conflicts:
            self.conflicts_detected.append(conflict)
            
            # Determine which robot to re-plan (lower priority)
            pri_a = self.robot_priorities.get(conflict.robot_a, conflict.robot_a)
            pri_b = self.robot_priorities.get(conflict.robot_b, conflict.robot_b)
            
            replan_robot = conflict.robot_b if pri_a < pri_b else conflict.robot_a
            
            # Re-plan with additional constraints
            if replan_robot in robot_states:
                state = robot_states[replan_robot]
                if state.get('goal_location'):
                    # Add wait step: robot waits at current node
                    if replan_robot in self.robot_paths and self.robot_paths[replan_robot]:
                        # Insert a wait step
                        current_step = self.robot_paths[replan_robot][0]
                        wait_step = PathStep(
                            node=current_step.node,
                            position=current_step.position,
                            time_step=current_step.time_step
                        )
                        self.robot_paths[replan_robot].insert(0, wait_step)
                        self.total_replans += 1
            
            self.conflicts_resolved += 1

    def _find_conflicts(self) -> List[SpaceTimeConflict]:
        """Find all vertex and edge conflicts in current path plans."""
        conflicts = []
        robot_ids = list(self.robot_paths.keys())
        
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                rid_a = robot_ids[i]
                rid_b = robot_ids[j]
                path_a = self.robot_paths[rid_a]
                path_b = self.robot_paths[rid_b]
                
                # Check vertex conflicts
                max_t = min(len(path_a), len(path_b))
                for t in range(max_t):
                    if path_a[t].node == path_b[t].node:
                        conflicts.append(SpaceTimeConflict(
                            robot_a=rid_a,
                            robot_b=rid_b,
                            node=path_a[t].node,
                            time_step=t,
                            conflict_type="vertex"
                        ))
                
                # Check edge conflicts (swap conflicts)
                for t in range(max_t - 1):
                    if (path_a[t].node == path_b[t + 1].node and 
                        path_a[t + 1].node == path_b[t].node):
                        conflicts.append(SpaceTimeConflict(
                            robot_a=rid_a,
                            robot_b=rid_b,
                            node=path_a[t].node,
                            time_step=t,
                            conflict_type="edge"
                        ))
        
        return conflicts

    def detect_deadlock(self, robot_positions: Dict[int, str],
                        robot_goals: Dict[int, str]) -> List[List[int]]:
        """
        Detect deadlocks using cycle detection in the wait-for graph.
        
        A deadlock occurs when robot A is waiting for robot B, which is
        waiting for robot C, which is waiting for robot A.
        
        Returns:
            List of deadlock cycles (each cycle is a list of robot IDs).
        """
        # Build wait-for graph
        wait_for: Dict[int, int] = {}
        
        for rid, pos_node in robot_positions.items():
            if rid not in robot_goals:
                continue
            goal = robot_goals[rid]
            
            # Check if any robot is blocking this robot's next step
            if rid in self.robot_paths and len(self.robot_paths[rid]) > 1:
                step = self.robot_paths[rid][1]
                # Handle both PathStep objects (with .node) and raw tuples
                if hasattr(step, 'node'):
                    next_node = step.node
                elif isinstance(step, (list, tuple)) and len(step) >= 2:
                    next_node = step  # raw (x, y) tuple
                else:
                    continue
                for other_rid, other_pos in robot_positions.items():
                    if other_rid == rid:
                        continue
                    # Compare positions (handle both node names and coordinates)
                    if isinstance(next_node, str) and other_pos == next_node:
                        wait_for[rid] = other_rid
                        break
                    elif isinstance(next_node, (list, tuple)):
                        # Compare as coordinates with tolerance
                        if isinstance(other_pos, (list, tuple)) and len(other_pos) >= 2:
                            import math
                            d = math.hypot(next_node[0]-other_pos[0], 
                                          next_node[1]-other_pos[1])
                            if d < 0.5:
                                wait_for[rid] = other_rid
                                break

        # Detect cycles using DFS
        deadlocks = []
        visited = set()
        
        for start_rid in wait_for:
            if start_rid in visited:
                continue
            
            path = []
            current = start_rid
            path_set = set()
            
            while current is not None and current not in visited:
                if current in path_set:
                    # Found a cycle
                    cycle_start = path.index(current)
                    cycle = path[cycle_start:]
                    deadlocks.append(cycle)
                    self.deadlocks_detected += 1
                    break
                
                path.append(current)
                path_set.add(current)
                current = wait_for.get(current)
            
            visited.update(path_set)

        return deadlocks

    def resolve_deadlock(self, deadlock_cycle: List[int]):
        """
        Resolve a deadlock by forcing the lowest-priority robot to yield.
        The yielding robot backs up or waits at a safe location.
        """
        if not deadlock_cycle:
            return
        
        # Find lowest priority robot in the cycle
        lowest_pri_robot = max(
            deadlock_cycle,
            key=lambda rid: self.robot_priorities.get(rid, rid))
        
        # Clear this robot's path and re-plan later
        if lowest_pri_robot in self.robot_paths:
            self.robot_paths[lowest_pri_robot] = []
        self.resource_reservations.release(lowest_pri_robot)
        return lowest_pri_robot

    def replan_all_with_cbs(self,
                            robot_targets: Dict[int, Tuple[Tuple[float, float], str]],
                            max_iterations: int = 500,
                            max_time: int = 100,
                            ) -> Dict[int, List[Tuple[float, float]]]:
        """
        Re-plan paths for ALL active robots using Conflict-Based Search.

        This is the *strict* multi-robot planner: it guarantees the
        returned paths are pairwise conflict-free (no two robots ever
        occupy the same node at the same time-step, and no robot
        ever swaps positions with another between consecutive steps).

        Use this whenever the set of active robots / goals changes
        materially (new task assignment, robot finishing a task,
        battery-return, etc.).

        Args:
            robot_targets: {robot_id: (current_position, goal_location_name)}
                           Only include robots that currently HAVE a goal.
            max_iterations: cap on CT node expansions (safety)
            max_time:       cap on time-step horizon per agent

        Returns:
            {robot_id: list of (x, y) waypoints} for every robot that
            successfully got a conflict-free plan. Robots that fail
            to plan are absent from the dict.
        """
        # Build per-robot start/goal NODES (not positions) for CBS
        cbs_input: Dict[int, Tuple[str, str]] = {}
        location_to_node = LOCATION_TO_NODE
        for rid, (pos, goal_loc) in robot_targets.items():
            # Map current position → start node
            #   Tier 1: if robot is AT a known location, use its mapping
            #   Tier 2: nearest waypoint to (pos)
            start_node = None
            for loc_name, loc_pos in ALL_LOCATIONS.items():
                if (abs(pos[0] - loc_pos[0]) < 0.4 and
                        abs(pos[1] - loc_pos[1]) < 0.4 and
                        loc_name in location_to_node):
                    start_node = location_to_node[loc_name]
                    break
            if start_node is None:
                start_node = self.graph.get_nearest_node(pos)

            # Map goal → node
            if goal_loc in location_to_node:
                goal_node = location_to_node[goal_loc]
            else:
                goal_pos = ALL_LOCATIONS.get(
                    goal_loc, CHARGING_STATIONS.get(goal_loc, (0.0, 0.0)))
                goal_node = self.graph.get_nearest_node(goal_pos)

            cbs_input[rid] = (start_node, goal_node)

        if not cbs_input:
            return {}

        # Run CBS
        cbs_result = self.cbs.plan(cbs_input,
                                    max_iterations=max_iterations,
                                    max_time=max_time)

        # Update statistics from CBS
        cbs_stats = self.cbs.stats
        self.conflicts_resolved += (cbs_stats.get('vertex_conflicts', 0) +
                                     cbs_stats.get('edge_conflicts', 0))
        self.total_replans += cbs_stats.get('low_level_calls', 0)

        if cbs_result is None:
            return {}

        # Translate CBS result (node-paths) back to:
        #  1) self.robot_paths (PathStep objects)
        #  2) waypoint coordinate lists for the caller
        all_waypoints: Dict[int, List[Tuple[float, float]]] = {}
        for rid, node_path in cbs_result.items():
            # Build PathStep list (collapse consecutive duplicates so
            # a `wait` shows up as time_step++ on the same node).
            path_steps: List[PathStep] = []
            for t, node in enumerate(node_path):
                pos = self.graph.get_node_position(node)
                path_steps.append(PathStep(node=node, position=pos, time_step=t))
            self.robot_paths[rid] = path_steps

            # Coordinate waypoints for the executor:
            # We emit the position of every step. Consecutive identical
            # positions (waits) are collapsed to one — the executor
            # interprets a single waypoint as "drive there and stop";
            # the wait time is naturally enforced because subsequent
            # robots won't conflict with this one in the CBS plan.
            current_pos = robot_targets[rid][0]
            waypoints: List[Tuple[float, float]] = []
            last = None
            for step in path_steps:
                pos = step.position if hasattr(step, 'position') else (step[0], step[1]) if isinstance(step, (list, tuple)) else None
                if pos is None:
                    continue
                if last is None or pos != last:
                    waypoints.append(pos)
                    last = pos
            # Drop redundant first waypoint == current_position
            if waypoints and (
                    abs(waypoints[0][0] - current_pos[0]) < 0.05 and
                    abs(waypoints[0][1] - current_pos[1]) < 0.05):
                waypoints.pop(0)
            # Append the actual goal coordinate (off-graph stop point)
            goal_loc = robot_targets[rid][1]
            goal_pos = ALL_LOCATIONS.get(
                goal_loc, CHARGING_STATIONS.get(goal_loc))
            if goal_pos and (not waypoints or
                              waypoints[-1] != goal_pos):
                waypoints.append(goal_pos)
            all_waypoints[rid] = waypoints

        return all_waypoints

    def get_cbs_statistics(self) -> dict:
        """Return the most recent CBS run statistics."""
        return dict(self.cbs.stats) if hasattr(self.cbs, 'stats') else {}

    # ==================================================================
    #  LIFELONG (Cooperative A*) API — for online task arrival streams
    # ==================================================================

    def plan_lifelong(self,
                      robot_id: int,
                      current_position: Tuple[float, float],
                      goal_location: str
                      ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path for a SINGLE robot using the lifelong planner.
        The robot's path will respect all currently committed
        reservations of other robots and write its own reservations
        into the global table.

        Use this in place of plan_path_for_robot() when running
        a continuous task stream — it guarantees no spatio-temporal
        conflicts WITHOUT rewinding any robot already in motion.

        Args:
            robot_id:         robot identifier
            current_position: (x, y) on the floor
            goal_location:    a name from ALL_LOCATIONS or
                              CHARGING_STATIONS

        Returns:
            list of (x, y) waypoints to follow, or None if no
            collision-free path exists within the planner horizon.
        """
        # Resolve START node — three-tier strategy identical to
        # plan_path_for_robot, ensuring consistency.
        start_node = None
        for _loc_name, _loc_pos in ALL_LOCATIONS.items():
            if (abs(current_position[0] - _loc_pos[0]) < 0.4 and
                    abs(current_position[1] - _loc_pos[1]) < 0.4 and
                    _loc_name in LOCATION_TO_NODE):
                start_node = LOCATION_TO_NODE[_loc_name]
                break
        if start_node is None:
            start_node = self.graph.get_nearest_node(current_position)

        # Resolve GOAL node
        if goal_location in LOCATION_TO_NODE:
            goal_node = LOCATION_TO_NODE[goal_location]
        else:
            goal_pos = ALL_LOCATIONS.get(
                goal_location, CHARGING_STATIONS.get(goal_location, (0, 0))
            )
            goal_node = self.graph.get_nearest_node(goal_pos)

        # Plan via lifelong (commits reservations on success)
        node_path = self.lifelong.plan(robot_id, start_node, goal_node)
        if node_path is None:
            return None

        # Mirror the result into self.robot_paths for stats /
        # legacy code that reads it.
        path_steps: List[PathStep] = []
        for t, n in enumerate(node_path):
            pos = self.graph.get_node_position(n)
            path_steps.append(PathStep(node=n, position=pos, time_step=t))
        self.robot_paths[robot_id] = path_steps

        # Build coordinate waypoints (collapse consecutive duplicates)
        waypoints: List[Tuple[float, float]] = []
        last = None
        for step in path_steps:
            if last is None or step.position != last:
                waypoints.append(step.position)
                last = step.position

        # Drop redundant first waypoint == current_position
        if waypoints and (
                abs(waypoints[0][0] - current_position[0]) < 0.05 and
                abs(waypoints[0][1] - current_position[1]) < 0.05):
            waypoints.pop(0)

        # Append the actual goal coordinate (off-graph stop)
        goal_pos = ALL_LOCATIONS.get(
            goal_location, CHARGING_STATIONS.get(goal_location))
        if goal_pos and (not waypoints or waypoints[-1] != goal_pos):
            waypoints.append(goal_pos)

        return waypoints

    def release_lifelong(self, robot_id: int) -> None:
        """
        Release a robot's reservations in the lifelong planner.
        Call when:
          • the robot completes its task (delivery_done)
          • the robot is removed from the active fleet
          • before re-planning the same robot for a new task
            (plan_lifelong will auto-release first, but you can
             also call this explicitly between phases)
        """
        self.lifelong.release(robot_id)
        self.resource_reservations.release(robot_id)
        # Mirror cleanup
        if robot_id in self.robot_paths:
            del self.robot_paths[robot_id]

    def lifelong_tick(self, n: int = 1) -> None:
        """
        Advance the lifelong planner's global clock by `n` ticks.
        Should be called once per simulation step from the main loop.
        Old reservations are automatically pruned.
        """
        self.lifelong.tick(n)

    def lifelong_reset(self) -> None:
        """Wipe ALL lifelong state (for a fresh experiment run)."""
        self.lifelong.reset()
        self.robot_paths.clear()
        self.reservations.clear()

    def find_nearest_rest_node(self,
                                position: Tuple[float, float],
                                exclude_robot_id: Optional[int] = None
                                ) -> Optional[Tuple[str, Tuple[float, float]]]:
        """
        Find the nearest REST_NODE (graph node where a robot can park
        between tasks) that is:
          • collision-free reachable from `position` (straight line check)
          • not already statically reserved by another robot
        
        Returns (node_name, (x, y)) or None if all rest nodes are taken.
        
        Used by lazy relocation: after completing a delivery, the robot
        relocates to the closest available rest node — not its
        assigned home spot. This minimises empty travel.
        
        Args:
            position:           current (x, y) of the robot
            exclude_robot_id:   ignore this robot's own static reservation
                                (so a robot can pick its own current
                                 rest spot without conflict)
        """
        # Get all rest node positions
        rest_positions = [(name, WAYPOINTS[name]) for name in REST_NODES
                          if name in WAYPOINTS]
        
        # Sort by distance
        rest_positions.sort(key=lambda np_: math.hypot(
            position[0] - np_[1][0], position[1] - np_[1][1]))
        
        # Get already-reserved rest nodes (by other robots)
        reserved_by_others = set()
        for rid, node in self.lifelong._static_reservations.items():
            if rid != exclude_robot_id:
                reserved_by_others.add(node)
        
        # Find first collision-free, unreserved rest node
        for name, pos in rest_positions:
            if name in reserved_by_others:
                continue
            if line_intersects_obstacles(position, pos):
                continue
            return (name, pos)
        
        # Fallback: take the closest even if reserved (caller decides)
        return rest_positions[0] if rest_positions else None

    def turning_zones(self, turn_radius: float = 0.45,
                      buffer: float = 0.10) -> Dict[str, Tuple[float, float]]:
        """Return wide graph junctions whose full turn circle is obstacle-free."""
        required = turn_radius + buffer
        zones = {}
        for name, pos in self.graph.nodes.items():
            # Intersections and REST nodes are the only intentional places
            # where an in-route heading reversal may be requested.
            if len(self.graph.edges.get(name, ())) < 3 and name not in REST_NODES:
                continue
            safe = True
            for box in OBSTACLE_BOXES:
                dx = max(abs(pos[0] - box["cx"]) - box["hx"], 0.0)
                dy = max(abs(pos[1] - box["cy"]) - box["hy"], 0.0)
                if math.hypot(dx, dy) < required:
                    safe = False
                    break
            if safe:
                zones[name] = pos
        return zones

    def reserve_turning_zone(self, robot_id: int, node: str,
                             start: float, end: float) -> bool:
        """Atomically reserve a validated turning zone."""
        if node not in self.turning_zones():
            return False
        return self.resource_reservations.reserve_batch(
            robot_id, [(("zone", node), start, end)])
    
    def reserve_home(self, robot_id: int, home_xy: Tuple[float, float]) -> None:
        """
        Mark a robot as permanently occupying its home spot. Updates BOTH
        the lifelong CBS (corridor graph) AND the grid reservation table.
        While reserved, no other robot will be routed through this spot.
        """
        # Lifelong CBS (for corridor-graph planning paths)
        home_node = self.graph.get_nearest_node(home_xy)
        self.lifelong.reserve_static(robot_id, home_node)
        
        # Grid reservation (for plan_grid_lifelong) — mark a 5×5 ring
        # around the home cell (~1m radius). This ensures other robots'
        # paths keep a safe distance from the parked robot's footprint
        # AND inflated safety zone.
        col, row = self.grid.world_to_grid(*home_xy)
        home_cells = set()
        for dc in (-2, -1, 0, 1, 2):
            for dr in (-2, -1, 0, 1, 2):
                home_cells.add((col + dc, row + dr))
        # Use the existing reservation table — this means peers will
        # treat the parked robot's home cells as blocked. The robot
        # itself bypasses this via the start_proximity exemption in
        # plan_grid_lifelong (4-cell ring around its own start).
        self._grid_path_reservations[robot_id] = home_cells

    def release_home(self, robot_id: int) -> None:
        """Drop the home-spot reservation (call when robot leaves home)."""
        self.lifelong.release_static(robot_id)
        # Also clear the grid reservation so peers can route through
        self.release_robot_grid(robot_id)

    def get_home_node(self, robot_id: int) -> Optional[str]:
        """Return the node the robot is currently pinned to, or None."""
        return self.lifelong.get_static_reservation(robot_id)

    def get_lifelong_statistics(self) -> dict:
        """Return current lifelong planner statistics + reservation size."""
        s = dict(self.lifelong.stats)
        s["reservation_size"] = self.lifelong.reservation_size()
        s["global_t"] = self.lifelong.global_t
        return s

    def clear_robot_path(self, robot_id: int):
        """Remove a robot's planned path (e.g., when task is completed)."""
        if robot_id in self.robot_paths:
            # Remove reservations
            to_remove = [key for key, rid in self.reservations.items() 
                        if rid == robot_id]
            for key in to_remove:
                del self.reservations[key]
            del self.robot_paths[robot_id]

    def get_next_waypoint(self, robot_id: int) -> Optional[Tuple[float, float]]:
        """Get the next waypoint for a robot to navigate to."""
        if robot_id in self.robot_paths and self.robot_paths[robot_id]:
            step = self.robot_paths[robot_id][0]
            if hasattr(step, 'position'):
                return step.position
            elif isinstance(step, (list, tuple)) and len(step) >= 2:
                return (step[0], step[1])
        return None

    def advance_robot(self, robot_id: int):
        """Mark the current waypoint as reached, advance to next."""
        if robot_id in self.robot_paths and self.robot_paths[robot_id]:
            self.robot_paths[robot_id].pop(0)

    # ────────────────────────────────────────────────────────
    # Level 2 — Deadlock detection + priority-inheritance break
    # ────────────────────────────────────────────────────────
    def init_deadlock_monitor(self) -> None:
        """Initialize per-robot stuck-time tracking (call once at sim start)."""
        # robot_id → (last_position, stuck_ticks)
        self._stuck_state: Dict[int, Tuple[Tuple[float, float], int]] = {}
        self._deadlock_break_count = 0   # statistic
        self._stuck_threshold = 10       # ticks before declaring stuck
        self._stuck_position_eps = 0.10  # m, position must change > 10cm

    def update_deadlock_monitor(self,
                                robot_states: Dict[int, dict]) -> List[int]:
        """
        Call this each tick with current robot positions. Detects robots
        that haven't moved >10cm for >10 ticks, returns list of stuck rids.
        Priority inheritance: when 2+ stuck robots block each other,
        the lower-priority one is forced to back off.
        
        Args:
            robot_states: {rid: {"position": (x,y), "state": RobotState, ...}}
        Returns:
            List of robot_ids that need priority-inheritance break
        """
        if not hasattr(self, "_stuck_state"):
            self.init_deadlock_monitor()
        
        stuck_now: List[int] = []
        for rid, rs in robot_states.items():
            pos = rs.get("position")
            state = rs.get("state")
            if pos is None or state is None:
                continue
            # Only monitor robots that should be moving
            from config import RobotState as _RS
            if state not in (_RS.EN_ROUTE_PICKUP, _RS.EN_ROUTE_DELIVERY,
                             _RS.RETURNING_TO_CHARGE, _RS.RETURNING_HOME):
                # Reset for non-moving states
                if rid in self._stuck_state:
                    del self._stuck_state[rid]
                continue
            # Check movement
            prev = self._stuck_state.get(rid)
            if prev is None:
                self._stuck_state[rid] = (pos, 0)
            else:
                last_pos, ticks = prev
                d = math.hypot(pos[0] - last_pos[0], pos[1] - last_pos[1])
                if d > self._stuck_position_eps:
                    # Robot moved — reset stuck counter
                    self._stuck_state[rid] = (pos, 0)
                else:
                    new_ticks = ticks + 1
                    self._stuck_state[rid] = (last_pos, new_ticks)
                    if new_ticks >= self._stuck_threshold:
                        stuck_now.append(rid)
        return stuck_now

    def break_deadlock(self,
                        stuck_rids: List[int],
                        robot_states: Dict[int, dict],
                        ) -> List[int]:
        """
        Apply priority inheritance: when ≥2 stuck robots are spatially
        adjacent (< 1.5m apart), the lower-priority one releases its
        reservations and re-plans (which forces a detour).
        
        Returns the list of robots whose reservations were cleared
        (the caller is responsible for re-planning these).
        """
        if len(stuck_rids) < 1:
            return []
        if not hasattr(self, "_priority_order"):
            self._priority_order = list(range(1, 9))
        
        # Build one connected local conflict set, then choose exactly one
        # victim. Multiple simultaneous recovery owners caused oscillation.
        candidates = set()
        for i, rid_a in enumerate(stuck_rids):
            for rid_b in stuck_rids[i+1:]:
                pa = robot_states[rid_a].get("position")
                pb = robot_states[rid_b].get("position")
                if pa is None or pb is None:
                    continue
                d = math.hypot(pa[0]-pb[0], pa[1]-pb[1])
                if d < 1.5:
                    candidates.update((rid_a, rid_b))
        if not candidates:
            return []
        def victim_key(rid):
            state = robot_states.get(rid, {})
            # max() selects the least entitled robot.
            return priority_key(
                bool(state.get("entered_zone", False)),
                int(state.get("task_priority", 0)),
                float(state.get("wait_age", 0.0)),
                float(state.get("conflict_distance", 0.0)),
                rid)
        yielder = max(candidates, key=victim_key)
        self._deadlock_break_count += 1
        self.release_lifelong(yielder)
        if hasattr(self, "_stuck_state") and yielder in self._stuck_state:
            pos = robot_states[yielder].get("position")
            self._stuck_state[yielder] = (pos, 0)
        return [yielder]

    # ────────────────────────────────────────────────────────
    # Level 3 — RHCR: Rolling-Horizon Cooperative Replanning
    # ────────────────────────────────────────────────────────
    def rhcr_replan(self,
                     robot_states: Dict[int, dict],
                     robot_goals: Dict[int, str]) -> Dict[int, List]:
        """
        Periodic global replan to find a better space-time schedule.
        Only accepts the new plan if its total path length is <= the
        current cumulative length (i.e. no regression).
        
        Args:
            robot_states: {rid: {"position": (x,y), ...}} for ALL active robots
            robot_goals:  {rid: goal_location_name} for robots WITH a task
        Returns:
            Dict of new paths per robot (only those whose paths changed),
            empty dict if no improvement found.
        """
        if not robot_goals:
            return {}
        # Snapshot old paths to compute cost
        old_total = 0
        for rid in robot_goals:
            p = self.lifelong._robot_paths.get(rid, [])
            old_total += len(p)
        
        # Collect (start_node, goal_node) for every active robot
        agents: Dict[int, Tuple[str, str]] = {}
        for rid, goal in robot_goals.items():
            pos = robot_states.get(rid, {}).get("position")
            if pos is None:
                continue
            start_node = self.graph.get_nearest_node(pos)
            from config import LOCATION_TO_NODE as _LTN
            goal_node = _LTN.get(goal, goal)
            if start_node and goal_node:
                agents[rid] = (start_node, goal_node)
        
        if len(agents) < 2:
            return {}
        
        # Compute candidate new plan via classical CBS (one-shot, all robots)
        try:
            candidate = self.cbs.plan(agents)
        except Exception:
            return {}
        
        if candidate is None:
            return {}
        
        # Check candidate cost
        new_total = sum(len(p) for p in candidate.values())
        if new_total >= old_total:
            return {}   # No improvement
        
        # Candidate generation is deliberately read-only. Runtime adoption
        # requires a two-phase dispatch/ACK transaction; mutating reservations
        # here would make physical robots follow old plans against new state.
        if self.lifelong.verbose:
            print(f"  [RHCR] read-only candidate: cost {old_total} → {new_total}")
        return candidate

    def get_coordination_stats(self) -> dict:
        """Return Level 1/2/3 coordination statistics for reporting."""
        stats = self.lifelong.get_resolution_stats()
        stats["deadlock_breaks"] = getattr(self, "_deadlock_break_count", 0)
        stats["rhcr_replans"] = getattr(self, "_rhcr_count", 0)
        return stats

    def set_verbose(self, on: bool) -> None:
        """Enable Level 1/2/3 verbose logging across the coordinator."""
        self.lifelong.set_verbose(on)

    def _reserve_ordered_grid_cells(self, robot_id, ordered_cells,
                                    start_delay=0.0):
        """Atomically reserve a grid route at its expected traversal times."""
        seconds_per_cell = 0.25 / 0.22
        timed = []
        for index, cell in enumerate(ordered_cells):
            start_t = (self.lifelong.global_t + start_delay +
                       index * seconds_per_cell)
            timed.append((('grid', cell), start_t,
                          start_t + seconds_per_cell))
        for index, (a, b) in enumerate(zip(ordered_cells,
                                           ordered_cells[1:])):
            start_t = (self.lifelong.global_t + start_delay +
                       index * seconds_per_cell)
            timed.append((('grid_edge', a, b), start_t,
                          start_t + seconds_per_cell))
        return self.resource_reservations.reserve_batch(robot_id, timed)

    def _plan_space_time_detour(self, robot_id, start_cell, goal_cell,
                                baseline_cells):
        """Find a modest grid detour around existing timed reservations.

        The returned route contains no intermediate wait actions because the
        robot controller cannot execute timed waypoint waits. A bounded
        initial delay is considered separately and can be represented by the
        supervisor's existing hold command.
        """
        seconds_per_cell = 0.25 / 0.22
        deadline = time.perf_counter() + SPACE_TIME_DETOUR_BUDGET_SECONDS
        expansions = 0
        max_steps = max(8, int(math.ceil(max(1, baseline_cells) * 1.8)))
        start_delays = [step * 0.5 for step in range(0, 41)]
        moves = (
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),            (1, 0),
            (-1, 1),  (0, 1),  (1, 1),
        )

        def heuristic(cell):
            dx = abs(cell[0] - goal_cell[0])
            dy = abs(cell[1] - goal_cell[1])
            return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

        def node_has_clearance(cell, start_t, end_t):
            # GRID_RES is 0.25m. Exclude every peer cell whose centre is
            # strictly closer than the 0.50m hard robot-conflict distance.
            for dc in range(-2, 3):
                for dr in range(-2, 3):
                    if dc * dc + dr * dr >= 4:
                        continue
                    nearby = (cell[0] + dc, cell[1] + dr)
                    if self.resource_reservations.owner_at(
                            ('grid', nearby), start_t, end_t,
                            exclude_owner=robot_id) is not None:
                        return False
            return True

        best = None
        for initial_delay in start_delays:
            if time.perf_counter() >= deadline:
                break
            queue = [(heuristic(start_cell), 0.0, 0, start_cell)]
            came_from = {}
            best_cost = {(start_cell, 0): 0.0}
            sequence = 0
            found_state = None

            while queue:
                if (expansions >= SPACE_TIME_DETOUR_MAX_EXPANSIONS or
                        time.perf_counter() >= deadline):
                    queue.clear()
                    break
                _, cost, steps, cell = heapq.heappop(queue)
                expansions += 1
                state = (cell, steps)
                if cost > best_cost.get(state, math.inf) + 1e-9:
                    continue
                if cell == goal_cell:
                    found_state = state
                    break
                if steps >= max_steps:
                    continue

                for dc, dr in moves:
                    nxt = (cell[0] + dc, cell[1] + dr)
                    if not self.grid.in_bounds(*nxt) or not self.grid.is_free(*nxt):
                        continue
                    next_steps = steps + 1
                    start_t = (self.lifelong.global_t + initial_delay +
                               steps * seconds_per_cell)
                    end_t = start_t + seconds_per_cell
                    if self.resource_reservations.owner_at(
                            ('grid_edge', cell, nxt), start_t, end_t,
                            exclude_owner=robot_id) is not None:
                        continue
                    if not node_has_clearance(nxt, start_t, end_t):
                        continue

                    step_cost = math.sqrt(2.0) if dc and dr else 1.0
                    new_cost = cost + step_cost
                    next_state = (nxt, next_steps)
                    if new_cost >= best_cost.get(next_state, math.inf):
                        continue
                    best_cost[next_state] = new_cost
                    came_from[next_state] = state
                    sequence += 1
                    heapq.heappush(queue, (
                        new_cost + heuristic(nxt), new_cost,
                        next_steps, nxt))

            if found_state is None:
                continue
            cells = []
            cursor = found_state
            while True:
                cells.append(cursor[0])
                if cursor == (start_cell, 0):
                    break
                cursor = came_from[cursor]
            cells.reverse()
            score = best_cost[found_state] + 0.6 * initial_delay
            if best is None or score < best[0]:
                best = (score, cells, initial_delay)

        if best is None:
            return None
        _, cells, initial_delay = best
        if not self._reserve_ordered_grid_cells(
                robot_id, cells, initial_delay):
            return None
        return cells, initial_delay

    def _cells_to_turning_waypoints(self, cells):
        """Compress a cell route only along collinear runs, preserving detours."""
        if not cells:
            return []
        kept = [cells[0]]
        previous_direction = None
        for previous, current in zip(cells, cells[1:]):
            direction = (current[0] - previous[0], current[1] - previous[1])
            if previous_direction is not None and direction != previous_direction:
                kept.append(previous)
            previous_direction = direction
        if kept[-1] != cells[-1]:
            kept.append(cells[-1])
        return [self.grid.grid_to_world(*cell) for cell in kept]

    # ────────────────────────────────────────────────────────
    # Grid-based path planning — replaces corridor graph for path
    # length optimization. Still uses LifelongPlanner reservation
    # table for multi-robot coordination, but treats waypoints as
    # virtual nodes for the time-step constraint solver.
    # ────────────────────────────────────────────────────────
    def plan_grid_lifelong(self,
                            robot_id: int,
                            current_position: Tuple[float, float],
                            goal_location
                            ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path using the FREE-SPACE grid with multi-robot
        Cooperative A* (Silver 2005).
        
        Algorithm:
          1. Release this robot's previous reservation (re-planning).
          2. Temporarily mark OTHER robots' active path cells (+1-cell
             inflation) as TEMPORARY OBSTACLES in the OccupancyGrid.
          3. Run GridAStar — A* naturally routes around peers' paths.
          4. Restore the temporary obstacles.
          5. Record THIS robot's new path cells in the reservation table.
          6. Subsample + drop-first-wp.
        
        Args:
            robot_id:         robot identifier
            current_position: (x, y) start position
            goal_location:    name from ALL_LOCATIONS/CHARGING_STATIONS,
                              a graph node name, or an explicit (x, y)
                              navigation target such as a rest position.
        
        Returns:
            List of (x, y) waypoints, or None if no path found.
        """
        # Resolve goal world coordinates
        if (isinstance(goal_location, (tuple, list)) and
                len(goal_location) >= 2):
            goal_xy = (float(goal_location[0]), float(goal_location[1]))
        elif goal_location in ALL_LOCATIONS:
            goal_xy = ALL_LOCATIONS[goal_location]
        elif goal_location in CHARGING_STATIONS:
            goal_xy = CHARGING_STATIONS[goal_location]
        elif goal_location in WAYPOINTS:
            goal_xy = WAYPOINTS[goal_location]
        else:
            return None
        
        # ── Step 1: release this robot's old reservation ──────
        self.release_robot_grid(robot_id)
        
        # ── Step 2: gather all OTHER robots' reserved cells ───
        other_cells = set()
        for rid, cells in self._grid_path_reservations.items():
            if rid == robot_id:
                continue
            other_cells |= cells
        
        # ── Step 3: temporarily occlude other robots' cells ────
        # We avoid blocking cells too close to THIS robot's start
        # (so robot can always step off its own square) and avoid
        # blocking the goal cell (so robot can always reach it).
        from grid_planner import (
            CELL_AVOID, CELL_OBSTACLE, CELL_FREE, CELL_INFLATED)
        start_col, start_row = self.grid.world_to_grid(*current_position)
        goal_col, goal_row = self.grid.world_to_grid(*goal_xy)
        
        # Inflate peer reservations by 2 cells (0.5m) to guarantee
        # minimum 0.75m separation between robots' planned paths.
        # This ensures robots use DIFFERENT corridors when one is taken.
        # 2-cell inflation blocks entire corridor width (corridors are ~1m),
        # forcing the second robot to find an alternative route.
        inflated_peer_cells = set()
        for (c, r) in other_cells:
            for dc in range(-2, 3):
                for dr in range(-2, 3):
                    inflated_peer_cells.add((c + dc, r + dr))
        
        # Start proximity = 2 cells (forces robot onto different corridor)
        # Goal proximity = 4 cells (robot MUST reach its destination dock,
        # even if near another robot's path — temporal delay handles timing)
        start_proximity = set()
        goal_proximity = set()
        for dc in range(-2, 3):
            for dr in range(-2, 3):
                start_proximity.add((start_col + dc, start_row + dr))
        for dc in range(-4, 5):
            for dr in range(-4, 5):
                goal_proximity.add((goal_col + dc, goal_row + dr))
        
        # Apply temporary OBSTACLE marks
        modified = []  # for restoration: (col, row, original_cell)
        for (c, r) in inflated_peer_cells:
            if (c, r) in start_proximity or (c, r) in goal_proximity:
                continue
            if not self.grid.in_bounds(c, r):
                continue
            if self.grid.cells[r][c] == CELL_FREE:
                modified.append((c, r, CELL_FREE))
                self.grid.cells[r][c] = CELL_OBSTACLE
        
        # ── Step 4: run A* on modified grid (peers blocked) ───
        # First attempt: route around active peers' paths.
        try:
            raw_path = self.grid_planner.plan(current_position, goal_xy,
                                                smooth=True)
        finally:
            # Restore — CRITICAL: always undo modifications
            for (c, r, original) in modified:
                self.grid.cells[r][c] = original
        
        # A hard block can close an entire narrow corridor. Before falling
        # back to the ordinary shortest path, retry with peer paths as a high
        # traversal cost. This permits a longer, separated corridor.
        peer_cost_detour = False
        if raw_path is None or len(raw_path) == 0:
            penalized = []
            for (c, r) in inflated_peer_cells:
                if ((c, r) in start_proximity or
                        (c, r) in goal_proximity or
                        not self.grid.in_bounds(c, r)):
                    continue
                if self.grid.cells[r][c] == CELL_FREE:
                    penalized.append((c, r, CELL_FREE))
                    self.grid.cells[r][c] = CELL_AVOID
            try:
                # Keep every grid turn. Smoothing only checks obstacles and
                # could otherwise cut back through the high-cost peer band.
                raw_path = self.grid_planner.plan(
                    current_position, goal_xy, smooth=False)
                if raw_path:
                    route_cells = []
                    for point in raw_path:
                        cell = self.grid.world_to_grid(*point)
                        if not route_cells or route_cells[-1] != cell:
                            route_cells.append(cell)
                    raw_path = self._cells_to_turning_waypoints(route_cells)
                    if raw_path:
                        raw_path[0] = current_position
                        raw_path[-1] = goal_xy
                        peer_cost_detour = True
            finally:
                for (c, r, original) in penalized:
                    self.grid.cells[r][c] = original

        # If no separated route exists, use the unrestricted shortest route;
        # temporal dispatch delay and controller holds then serialize access.
        if raw_path is None or len(raw_path) == 0:
            raw_path = self.grid_planner.plan(current_position, goal_xy,
                                              smooth=True)
            if raw_path is None or len(raw_path) == 0:
                return None
        
        # ── Step 5: record THIS robot's new reservation ────────
        my_cells = set()
        ordered_cells = []
        for i in range(len(raw_path) - 1):
            p1 = raw_path[i]
            p2 = raw_path[i + 1]
            dx, dy = p2[0]-p1[0], p2[1]-p1[1]
            dist = math.hypot(dx, dy)
            n_steps = max(2, int(dist / 0.25) + 1)
            for k in range(n_steps + 1):
                t = k / n_steps
                x = p1[0] + t * dx
                y = p1[1] + t * dy
                col, row = self.grid.world_to_grid(x, y)
                my_cells.add((col, row))
                if not ordered_cells or ordered_cells[-1] != (col, row):
                    ordered_cells.append((col, row))

        # Reserve continuous traversal windows. A grid cell is 0.25m and
        # nominal speed is 0.22m/s; search a bounded start delay when an
        # active node or reverse-edge window conflicts.
        accepted_delay = None
        for delay_step in range(41):  # bounded 0..20s cooperative horizon
            delay = delay_step * 0.5
            if self._reserve_ordered_grid_cells(robot_id, ordered_cells,
                                                delay):
                accepted_delay = delay
                break
        space_time_detour = False
        if accepted_delay is None:
            detour = self._plan_space_time_detour(
                robot_id, (start_col, start_row), (goal_col, goal_row),
                len(ordered_cells))
            if detour is None:
                return None
            ordered_cells, accepted_delay = detour
            my_cells = set(ordered_cells)
            raw_path = self._cells_to_turning_waypoints(ordered_cells)
            space_time_detour = True
        if accepted_delay > 0:
            self._dispatch_delays[robot_id] = max(
                accepted_delay, self._dispatch_delays.get(robot_id, 0.0))
        self._grid_path_reservations[robot_id] = my_cells
        # Store waypoints for temporal conflict detection by future planners
        # (coordinate_paths stored after lane separation — see below)
        # 日志: 规划结果
        
        # ── Step 6: subsample + drop-first ─────────────────────
        # A space-time detour must retain its turns; generic line-of-sight
        # subsampling could cut back across the reservation it was avoiding.
        preserve_detour = space_time_detour or peer_cost_detour
        path = (list(raw_path) if preserve_detour else
                subsample_path(raw_path, step_m=1.5))
        if (len(path) > 1 and
                abs(path[0][0] - current_position[0]) < 0.05 and
                abs(path[0][1] - current_position[1]) < 0.05):
            path = path[1:]
        
        # ── Step 6b: Lane separation — prevent head-on corridor conflicts ──
        # When two robots use the same corridor in opposite directions,
        # shift one to an adjacent lane (0.5m offset). This is cheaper
        # than serializing with dispatch delays.
        # Rules:
        #   West corridor (x ≈ -4.25): southbound stays, northbound → x=-4.75
        #   East corridor (x ≈ +4.25): southbound stays, northbound → x=+4.75
        if not preserve_detour:
            path = self._apply_lane_separation(path)
        
        # Store final path (WITH lane separation) for temporal conflict detection
        self.coordinate_paths[robot_id] = [current_position] + list(path)
        
        # ── Step 7: Temporal conflict detection & delay injection ──
        # Simulate this robot and all active peers forward in time.
        # If conflict (distance < 0.5m at same time), delay this robot.
        path = self._inject_delay_if_temporal_conflict(
            robot_id, current_position, path)
        
        return path
    
    def _apply_lane_separation(self, path):
        """Enforce corridor lane discipline for the ENTIRE path — all 6 corridors.
        
        ═══ Vertical (South-North) corridors ═══
          West corridor:  southbound → x=-4.25,  northbound → x=-4.75
          East corridor:  southbound → x=+4.25,  northbound → x=+4.75
        
        ═══ Horizontal (East-West) corridors ═══
          North outer   (y≈3.25):  eastbound → y=3.25,  westbound → y=3.75
          South outer   (y≈-3.25): eastbound → y=-3.25, westbound → y=-3.75
          North inner   (y≈2.0):   eastbound → y=2.0,   westbound → y=2.5
          South inner   (y≈-2.0):  eastbound → y=-2.0,  westbound → y=-2.5
        
        Strategy: Determine overall direction (dx for horizontal, dy for vertical),
        then shift ALL waypoints in that corridor to the correct lane.
        """
        if not path or len(path) < 2:
            return path
        
        # ── Constants ─────────────────────────────────────────────
        LANE_OFFSET = 0.5
        CORRIDOR_TOL = 0.6  # how close to corridor center counts as "in corridor"
        
        # Vertical corridors (south-north)
        WEST_CORRIDOR_X = -4.25
        EAST_CORRIDOR_X = 4.25
        
        # Horizontal corridors (east-west)
        NORTH_OUTER_Y = 3.25
        SOUTH_OUTER_Y = -3.25
        NORTH_INNER_Y = 2.0
        SOUTH_INNER_Y = -2.0
        
        # ── Overall direction from start → end ────────────────────
        overall_dx = path[-1][0] - path[0][0]  # >0 eastbound, <0 westbound
        overall_dy = path[-1][1] - path[0][1]  # >0 northbound, <0 southbound
        
        new_path = list(path)
        for i in range(len(new_path)):
            # ★ 不修改首尾waypoint: 起点=实际位置, 终点=精确目标
            if i == len(new_path) - 1:
                continue  # 保留精确目标位置
            x, y = new_path[i]
            
            # ══════════════════════════════════════════════════════
            # VERTICAL corridors: shift x based on north/south
            # Only applies to the "trunk" section of corridors
            # (|y| < 2.8 — excludes the outer horizontal zone)
            # ══════════════════════════════════════════════════════
            
            in_vertical_trunk = abs(y) < 2.8  # not in outer horizontal zones
            
            # West corridor zone: x ∈ [-3.65, -5.35]
            if in_vertical_trunk and (abs(x - WEST_CORRIDOR_X) < CORRIDOR_TOL or abs(x - (WEST_CORRIDOR_X - LANE_OFFSET)) < CORRIDOR_TOL):
                if overall_dy > 0.3:
                    new_path[i] = (WEST_CORRIDOR_X - LANE_OFFSET, y)  # northbound → x=-4.75
                else:
                    new_path[i] = (WEST_CORRIDOR_X, y)                # southbound → x=-4.25
            
            # East corridor zone: x ∈ [3.65, 5.35]
            elif in_vertical_trunk and (abs(x - EAST_CORRIDOR_X) < CORRIDOR_TOL or abs(x - (EAST_CORRIDOR_X + LANE_OFFSET)) < CORRIDOR_TOL):
                if overall_dy > 0.3:
                    new_path[i] = (EAST_CORRIDOR_X + LANE_OFFSET, y)  # northbound → x=+4.75
                else:
                    new_path[i] = (EAST_CORRIDOR_X, y)                # southbound → x=+4.25
            
            # ══════════════════════════════════════════════════════
            # HORIZONTAL corridors: shift y based on east/west
            # Applies to ALL x positions (including corridor ends)
            # ══════════════════════════════════════════════════════
            else:
                # North outer corridor: y ∈ [2.65, 4.35]
                if abs(y - NORTH_OUTER_Y) < CORRIDOR_TOL or abs(y - (NORTH_OUTER_Y + LANE_OFFSET)) < CORRIDOR_TOL:
                    if overall_dx < -0.3:
                        new_path[i] = (x, NORTH_OUTER_Y + LANE_OFFSET)  # westbound → y=3.75
                    else:
                        new_path[i] = (x, NORTH_OUTER_Y)                # eastbound → y=3.25
                
                # South outer corridor: y ∈ [-4.35, -2.65]
                elif abs(y - SOUTH_OUTER_Y) < CORRIDOR_TOL or abs(y - (SOUTH_OUTER_Y - LANE_OFFSET)) < CORRIDOR_TOL:
                    if overall_dx < -0.3:
                        new_path[i] = (x, SOUTH_OUTER_Y - LANE_OFFSET)  # westbound → y=-3.75
                    else:
                        new_path[i] = (x, SOUTH_OUTER_Y)                # eastbound → y=-3.25
                
                # North inner corridor: y ∈ [1.4, 2.6]
                elif abs(y - NORTH_INNER_Y) < CORRIDOR_TOL or abs(y - (NORTH_INNER_Y + LANE_OFFSET)) < CORRIDOR_TOL:
                    if overall_dx < -0.3:
                        new_path[i] = (x, NORTH_INNER_Y + LANE_OFFSET)  # westbound → y=2.5
                    else:
                        new_path[i] = (x, NORTH_INNER_Y)                # eastbound → y=2.0
                
                # South inner corridor: y ∈ [-2.6, -1.4]
                elif abs(y - SOUTH_INNER_Y) < CORRIDOR_TOL or abs(y - (SOUTH_INNER_Y - LANE_OFFSET)) < CORRIDOR_TOL:
                    if overall_dx < -0.3:
                        new_path[i] = (x, SOUTH_INNER_Y - LANE_OFFSET)  # westbound → y=-2.5
                    else:
                        new_path[i] = (x, SOUTH_INNER_Y)                # eastbound → y=-2.0
        
        for a, b in zip(new_path, new_path[1:]):
            if not self._segment_clear(a, b):
                return path
        return new_path

    def _inject_delay_if_temporal_conflict(self, robot_id, start_pos, path):
        """
        Check if this robot's path has temporal conflicts with active peers.
        If conflict exists, store a dispatch_delay for this robot.
        The supervisor reads this delay and holds the robot before sending it.
        
        Also returns modified path — no hold waypoints needed since supervisor
        handles the timing directly.
        """
        if not path:
            return path
        
        CONFLICT_RADIUS = 0.5  # m — minimum safe distance between robots
        ROBOT_SPEED = 0.22     # m/s
        DT = 0.5              # s — simulation timestep
        MAX_T = 80.0          # s
        
        # Get active peer paths
        active_peers = {}
        for rid, rdata in self.coordinate_paths.items():
            if rid == robot_id:
                continue
            if rdata and len(rdata) > 0:
                active_peers[rid] = rdata
        
        if not active_peers:
            return path
        
        def simulate_positions(start, waypoints, speed, dt, total_time):
            positions = []
            if hasattr(start, 'position'):
                x, y = start.position
            elif isinstance(start, (list, tuple)) and len(start) >= 2:
                x, y = start[0], start[1]
            else:
                return positions
            wp_idx = 0
            for _ in range(int(total_time / dt)):
                if wp_idx < len(waypoints):
                    wp = waypoints[wp_idx]
                    tx, ty = (wp.position if hasattr(wp, 'position') else 
                              (wp[0], wp[1]) if isinstance(wp, (list, tuple)) else (0,0))
                    dist = math.hypot(tx - x, ty - y)
                    if dist < 0.35:
                        wp_idx += 1
                        if wp_idx >= len(waypoints):
                            break
                        wp = waypoints[wp_idx]
                        tx, ty = (wp.position if hasattr(wp, 'position') else 
                                  (wp[0], wp[1]) if isinstance(wp, (list, tuple)) else (0,0))
                        dist = math.hypot(tx - x, ty - y)
                    if dist > 0.01:
                        step = min(speed * dt, dist)
                        x += (tx - x) / dist * step
                        y += (ty - y) / dist * step
                positions.append((x, y))
            return positions
        
        my_positions = simulate_positions(start_pos, path, ROBOT_SPEED, DT, MAX_T)
        
        # Check against each active peer
        max_delay_needed = 0.0
        for peer_id, peer_path in active_peers.items():
            if not peer_path:
                continue
            peer_start = peer_path[0]
            peer_wps = peer_path[1:] if len(peer_path) > 1 else peer_path
            peer_positions = simulate_positions(peer_start, peer_wps, ROBOT_SPEED, DT, MAX_T)
            
            # Find ALL time steps where paths are within conflict radius.
            # The required delay = time until NO MORE conflicts exist even
            # if we shift our start time later and later.
            n = min(len(my_positions), len(peer_positions))
            
            # Strategy: find the time when the peer has EXITED the area
            # that our path passes through. This means the peer's position
            # must be > CONFLICT_RADIUS from ALL points on our path.
            # Simplification: find when peer is far from our FIRST waypoint
            # (the entrance to the shared corridor).
            if len(my_positions) > 0:
                # Our entry point to potential conflict zone
                my_entry = my_positions[0]  # our first step
                
                # Find the LAST time the peer is near our entry region
                last_peer_near_our_entry = -1
                for t in range(n):
                    # Check if peer is near ANY of our early positions
                    for my_t in range(min(t+1, len(my_positions))):
                        d = math.hypot(my_positions[my_t][0] - peer_positions[t][0],
                                      my_positions[my_t][1] - peer_positions[t][1])
                        if d < CONFLICT_RADIUS:
                            last_peer_near_our_entry = t
                            break
                
                if last_peer_near_our_entry >= 0:
                    # Must wait until peer clears + safety margin
                    delay = (last_peer_near_our_entry + 8) * DT
                    max_delay_needed = max(max_delay_needed, delay)
        
        # Store dispatch delay for supervisor to use.
        # Cap at 10s — with proactive braking + progress monitoring,
        # we don't need large delays. If conflict persists, the progress
        # monitor will trigger dynamic replanning.
        MAX_DISPATCH_DELAY = 10.0
        if not hasattr(self, '_dispatch_delays'):
            self._dispatch_delays = {}
        if max_delay_needed > 0:
            self._dispatch_delays[robot_id] = min(max_delay_needed, MAX_DISPATCH_DELAY)
        
        return path
    
    def get_dispatch_delay(self, robot_id):
        """Get the recommended dispatch delay for a robot (seconds).
        The supervisor should wait this long before sending waypoints."""
        if not hasattr(self, '_dispatch_delays'):
            return 0.0
        return self._dispatch_delays.pop(robot_id, 0.0)
    
    def release_robot_grid(self, robot_id: int) -> None:
        """Clear this robot's grid path reservation. Call when robot
        finishes a leg (pickup arrival, delivery arrival, etc.) so
        other robots can plan through cells it no longer occupies."""
        self._grid_path_reservations.pop(robot_id, None)
        self.coordinate_paths.pop(robot_id, None)
        self.resource_reservations.release(robot_id)
    
    def get_statistics(self) -> dict:
        """Return coordination statistics."""
        return {
            "conflicts_detected": len(self.conflicts_detected),
            "conflicts_resolved": self.conflicts_resolved,
            "deadlocks_detected": self.deadlocks_detected,
            "total_replans": self.total_replans,
        }

    def get_path_length(self, robot_id: int) -> float:
        """Calculate remaining path length for a robot."""
        if robot_id not in self.robot_paths:
            return 0.0
        path = self.robot_paths[robot_id]
        total = 0.0
        for i in range(len(path) - 1):
            s1, s2 = path[i], path[i + 1]
            p1 = s1.position if hasattr(s1, 'position') else (s1[0], s1[1])
            p2 = s2.position if hasattr(s2, 'position') else (s2[0], s2[1])
            total += math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return total

    def get_congestion_map(self, grid_resolution: float = 1.0,
                           grid_width: int = 20, grid_height: int = 16
                           ) -> List[List[float]]:
        """
        Generate a congestion heatmap based on current robot paths.
        Used as input to the RL scheduler.
        
        Returns:
            2D grid where each cell contains the number of robot path
            segments passing through it.
        """
        # Initialize grid (centered on factory)
        grid = [[0.0] * grid_width for _ in range(grid_height)]
        
        half_w = grid_width * grid_resolution / 2
        half_h = grid_height * grid_resolution / 2
        
        for robot_id, path in self.robot_paths.items():
            for step in path:
                if hasattr(step, 'position'):
                    x, z = step.position
                elif isinstance(step, (list, tuple)) and len(step) >= 2:
                    x, z = step[0], step[1]
                else:
                    continue
                # Convert to grid coordinates
                gx = int((x + half_w) / grid_resolution)
                gz = int((z + half_h) / grid_resolution)
                if 0 <= gx < grid_width and 0 <= gz < grid_height:
                    grid[gz][gx] += 1.0
        
        return grid
