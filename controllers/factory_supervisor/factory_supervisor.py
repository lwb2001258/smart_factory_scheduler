"""
Factory Supervisor Controller for Webots.

This is the central controller that runs as a Webots supervisor node.
It manages:
1. Task generation (Poisson process)
2. Task scheduling (RL or baseline)
3. Multi-robot motion coordination
4. Metrics collection
5. Robot state monitoring via supervisor API

The supervisor communicates with individual robot controllers via
Webots Emitter/Receiver mechanism.
"""

import sys
import os
import json
import math
import random
import struct
import inspect
import itertools
import time as real_time
from typing import Dict, List, Optional, Tuple

# Webots controller API
try:
    from controller import Supervisor
except ImportError:
    # Fallback for development/testing outside Webots
    print("[WARNING] Webots controller module not found. Running in standalone mode.")
    class Supervisor:
        def __init__(self):
            self.timestep = 16
        def getBasicTimeStep(self):
            return self.timestep
        def step(self, timestep):
            return -1

# Local imports
from config import (
    TIMESTEP, SIM_DURATION, AUTO_STOP_SIMULATION, ENABLE_RUNTIME_RHCR,
    ENABLE_JOINT_RUNTIME,
    ENABLE_PROACTIVE_JOINT_SPEED,
    ENABLE_LEGACY_INTERLOCK_RECOVERY, SCENARIOS, MAX_ROBOTS, STARTUP_CONFIG,
    RobotState, TaskStatus, WORKSTATIONS, STORAGE_AREAS,
    CHARGING_STATIONS, ALL_LOCATIONS, GOAL_TOLERANCE, PARKING_SPOTS,
    REST_NODES, WAYPOINTS,
    LOW_BATTERY_THRESHOLD, TASK_ABORT_BATTERY_THRESHOLD,
    MIN_TASK_BATTERY, FULL_BATTERY_THRESHOLD,
    ASSIGNMENT_FAILURE_TTL,
    STALL_RELOCATION_TIMEOUT, STALL_PROGRESS_DISTANCE,
    STALL_GROUP_DISTANCE, RELOCATION_PEER_CLEARANCE,
    PROACTIVE_SCAN_BUDGET_SECONDS, PROACTIVE_SCAN_MAX_REPLANS,
    ENABLE_NONPHYSICAL_RECOVERY,
    BATTERY_CAPACITY, BATTERY_DRAIN_RATE, BATTERY_CHARGE_RATE,
    INITIAL_BATTERY_MIN, INITIAL_BATTERY_MAX,
    LOG_INTERVAL, LOCATION_TO_NODE
)
from task_generator import TaskGenerator, TransportTask
from motion_coordinator import MotionCoordinator
from schedulers import (
    create_scheduler, BaseScheduler, GreedyScheduler, HungarianScheduler,
    NearestNeighbourScheduler,
    Assignment, SchedulingContext, SchedulerResult,
)
from startup_gate import StartupGate
from metrics_collector import MetricsCollector, SafetyEvent
from joint_plan_transaction import JointPlanTransaction


class RobotInfo:
    """Tracks the state of a single robot in the factory."""
    
    def __init__(self, robot_id: int, initial_position: Tuple[float, float],
                 initial_battery: float = INITIAL_BATTERY_MAX):
        self.robot_id = robot_id
        self.position = initial_position
        self.heading = 0.0
        self.state = RobotState.IDLE
        self.battery = float(initial_battery)
        self.current_task: Optional[TransportTask] = None
        self.goal_location = None
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_idx = 0
        self.total_distance = 0.0
        self.tasks_completed = 0
        self.idle_time = 0.0
        self.active_time = 0.0
        self.last_position = initial_position
        self.velocity = (0.0, 0.0)
        self.sample_time = 0.0
        self.state_seq = 0
        self.path_version = 0
        self.controller_path_version = 0
        self.controller_waypoint_index = 0
        self.controller_active_plan_epoch = 0
        self.controller_paused_until = 0.0
        self.controller_joint_wait_until = 0.0
        self.controller_joint_wait_reason = None
        self.active_joint_started_at = 0.0
        self.active_joint_wait_deadline = 0.0
        self.pending_waypoints: Optional[List[Tuple[float, float]]] = None
        self.pending_plan_source: Optional[str] = None
        self.pending_plan_epoch: Optional[int] = None
        self.active_plan_epoch: Optional[int] = None
        self.active_plan_source: Optional[str] = None
        self.pending_previous_waypoints: Optional[List[Tuple[float, float]]] = None
        self.pending_previous_index = 0
        self.dispatch_not_before = 0.0
        self.hold_until = 0.0
        self.wait_started = 0.0
        self.yield_count = 0
        self.emergency_braking = False
        self.emergency_braking_since = None
        self.recovery_active = False
        self.recovery_resume_goal = None
        self.recovery_session_role = None
        self.recovery_session_until = 0.0
        self.recovery_started_position = None
        self.speed_scale = 1.0
        self.joint_speed_until = 0.0
        self.joint_speed_started = 0.0
        self.emergency_recovery_until = 0.0
        self.route_write_owner = None
        self.route_write_until = 0.0
        self.route_write_generation = 0
        self.pending_route_generation = None
        self.pending_is_runtime_replan = False
        
    def to_dict(self) -> dict:
        """Convert robot state to dictionary for scheduler input."""
        return {
            'position': self.position,
            'heading': self.heading,
            'state': self.state,
            'battery': self.battery,
            'current_task': self.current_task,
            'has_task': self.current_task is not None,
            'goal_location': self._get_current_goal(),
            'wait_age': max(0.0, self.sample_time - self.wait_started)
                        if self.wait_started else 0.0,
            'task_priority': getattr(self.current_task, 'priority', 0)
                             if self.current_task else 0,
            'path_version': self.path_version,
            'plan_epoch': self.active_plan_epoch,
            'plan_source': self.active_plan_source,
            'task_id': getattr(self.current_task, 'task_id', None),
            'planned_wait': bool(
                (self.pending_waypoints is not None and
                 self.sample_time <= self.dispatch_not_before + 0.5) or
                self.sample_time < max(
                    self.hold_until, self.controller_paused_until,
                    self.controller_joint_wait_until)),
            'wait_reason': (
                'dispatch_delay' if self.pending_waypoints is not None and
                self.sample_time <= self.dispatch_not_before + 0.5 else
                'controller_hold' if self.sample_time < max(
                    self.hold_until, self.controller_paused_until) else
                self.controller_joint_wait_reason
                if self.sample_time < self.controller_joint_wait_until
                else None),
            'wait_deadline': max(
                self.dispatch_not_before, self.hold_until,
                self.controller_paused_until,
                self.controller_joint_wait_until),
        }
    
    def _get_current_goal(self) -> Optional[str]:
        """Get current navigation goal location name."""
        if self.current_task is None:
            return None
        if self.state == RobotState.EN_ROUTE_PICKUP:
            return self.current_task.pickup_location
        elif self.state in (RobotState.CARRYING, RobotState.EN_ROUTE_DELIVERY):
            return self.current_task.delivery_location
        elif self.state == RobotState.CHARGING:
            # Find nearest charging station
            return "CS1"  # simplified
        return None


class FactorySupervisor:
    """
    Main factory supervisor that coordinates all robots.
    Runs as a Webots supervisor node.
    """

    def _log_replan(self, *args, **kwargs):
        """Print replan diagnostics with the current simulation timestamp."""
        timestamp = float(getattr(self, 'sim_time', 0.0))
        message = " ".join(str(item) for item in args)
        kwargs.setdefault('flush', True)
        print(f"[T={timestamp:.1f}s] {message}", **kwargs)

    @staticmethod
    def _navigation_goal(robot):
        """Return a plannable name or coordinate for the current journey."""
        named_goal = getattr(robot, 'goal_location', None)
        if named_goal:
            return named_goal
        # Legacy fallback for old snapshots only. New home/rest journeys set
        # goal_location explicitly so rolling prefixes cannot replace it.
        if (robot.state == RobotState.RETURNING_HOME and robot.waypoints):
            target = robot.waypoints[-1]
            return (float(target[0]), float(target[1]))
        return None

    def _has_active_navigation(self, robot) -> bool:
        """Return whether a robot owns an unfinished navigation leg."""
        moving_state = robot.state in (
            RobotState.EN_ROUTE_PICKUP,
            RobotState.CARRYING,
            RobotState.EN_ROUTE_DELIVERY,
            RobotState.RETURNING_HOME,
            RobotState.RETURNING_TO_CHARGE,
        )
        if moving_state and robot.active_plan_source == 'joint_grid_transaction':
            return self._navigation_goal(robot) is not None
        return bool(moving_state and robot.waypoints and
                    robot.current_waypoint_idx < len(robot.waypoints))

    @staticmethod
    def _goal_coordinates(goal):
        if isinstance(goal, (tuple, list)) and len(goal) >= 2:
            return (float(goal[0]), float(goal[1]))
        if goal in ALL_LOCATIONS:
            return ALL_LOCATIONS[goal]
        if goal in CHARGING_STATIONS:
            return CHARGING_STATIONS[goal]
        if goal in WAYPOINTS:
            return WAYPOINTS[goal]
        return None

    def _robot_completed_joint_txn(self, robot, active_txn) -> bool:
        """Return True once the robot has consumed the committed prefix."""
        points = active_txn.plans.get(robot.robot_id)
        if not points:
            return True
        if robot.controller_active_plan_epoch != active_txn.epoch:
            return False
        if robot.controller_waypoint_index >= len(points):
            return True
        last = points[-1]
        return math.hypot(
            robot.position[0] - last[0],
            robot.position[1] - last[1]) <= 0.30

    def _joint_staging_goal(self, goal_xy, robot_position, peer_positions,
                            reserved_positions=None):
        """Find a unique nearby free holding cell for a duplicate-goal follower.

        ``peer_positions`` contains all measured robot positions.  When several
        followers share one business goal, the caller also passes the staging
        cells already assigned to earlier followers so two robots never park on
        the same auxiliary cell and make the joint planner unsolvable.
        """
        grid = self.motion_coordinator.grid
        reserved = list(reserved_positions or ())
        candidates = []
        for radius in (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 3.5):
            for step in range(32):
                angle = 2.0 * math.pi * step / 32.0
                target = (
                    goal_xy[0] + radius * math.cos(angle),
                    goal_xy[1] + radius * math.sin(angle),
                )
                col, row = grid.world_to_grid(*target)
                if not grid.in_bounds(col, row) or not grid.is_free(col, row):
                    continue
                nearby = [
                    math.hypot(target[0] - peer[0], target[1] - peer[1])
                    for peer in list(peer_positions) + reserved
                ]
                clearance = min(nearby, default=float('inf'))
                if clearance < 0.80:
                    continue
                distance = math.hypot(
                    target[0] - robot_position[0],
                    target[1] - robot_position[1])
                candidates.append((clearance + 0.4 * radius - 0.05 * distance,
                                   target))
        if not candidates:
            return goal_xy
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def _check_joint_business_arrivals(self) -> None:
        """Promote physical goal arrivals that a rolling partial plan holds.

        A partial joint plan deliberately parks at its last reserved cell
        without sending ``reached_goal``. If that cell is already inside the
        business-goal tolerance, the supervisor must still perform the normal
        pickup/delivery transition; otherwise the fleet can stand still one
        cell away from a valid task target until an unrelated window timeout.
        """
        if not ENABLE_JOINT_RUNTIME:
            return
        for rid, robot in self.robots.items():
            if robot.active_plan_source != 'joint_grid_transaction':
                continue
            if not robot.current_task:
                continue
            if robot.state not in (
                    RobotState.EN_ROUTE_PICKUP, RobotState.EN_ROUTE_DELIVERY):
                continue
            goal = self._navigation_goal(robot)
            if goal is None:
                continue
            goal_xy = self._goal_coordinates(goal)
            if goal_xy is None:
                continue
            distance = math.hypot(
                robot.position[0] - goal_xy[0],
                robot.position[1] - goal_xy[1])
            if distance <= GOAL_TOLERANCE * 2.0:
                self._handle_goal_reached(rid)

    def _build_joint_fallback_plans(self, planning_agents):
        """Build per-robot free-space routes when space-time search fails.

        This is a liveness fallback, not a replacement for the coordinated
        planner.  Each path is generated by Cooperative A* against the
        current peer reservations and then committed as an atomic joint
        transaction.  The predictive shield remains active on top of it, so
        a temporary soft route can never turn into a fleet-wide park.
        """
        plans = {}
        offsets = {}
        partial = {}
        fallback_log = getattr(self, '_fallback_debug_path', None)
        if fallback_log is None:
            fallback_log = r'D:\code\smart_factory_scheduler\results\fallback_debug.log'
            self._fallback_debug_path = fallback_log
            with open(fallback_log, 'w', encoding='utf-8') as handle:
                handle.write('call,rid,event\n')
        with open(fallback_log, 'a', encoding='utf-8') as handle:
            handle.write(f'{self.sim_time:.3f},all,start\n')
        planner = getattr(self.motion_coordinator.grid_planner, 'plan', None)
        if planner is None:
            return plans, offsets, partial
        for rid in sorted(planning_agents):
            start, goal_xy = planning_agents[rid]
            with open(fallback_log, 'a', encoding='utf-8') as handle:
                handle.write(f'{self.sim_time:.3f},{rid},plan_start '
                             f'start={start} goal={goal_xy}\n')
            path = planner(start, goal_xy)
            with open(fallback_log, 'a', encoding='utf-8') as handle:
                handle.write(f'{self.sim_time:.3f},{rid},plan_end '
                             f'len={len(path) if path else 0}\n')
            if not path:
                continue
            points = [tuple(point) for point in path]
            if not points:
                continue
            if math.hypot(points[0][0] - start[0],
                          points[0][1] - start[1]) > 0.18:
                points.insert(0, tuple(start))
            true_goal = self._goal_coordinates(
                self._navigation_goal(self.robots[rid]))
            is_full_goal = (
                true_goal is not None and points and
                math.hypot(points[-1][0] - true_goal[0],
                           points[-1][1] - true_goal[1]) <=
                GOAL_TOLERANCE * 2.0)
            plans[rid] = points
            offsets[rid] = [0.0] * len(points)
            partial[rid] = not is_full_goal
        return plans, offsets, partial

    def _retire_joint_transaction(self, txn, reason: str = 'rolling_replan'):
        """Release an activated transaction without stopping its old route.

        The robot controllers keep executing their current committed paths
        until the next atomic transaction activates. Clearing the writer
        lease here lets a fresh all-active plan be prepared from measured
        positions without freezing the fleet behind one slow member.
        """
        if txn is None:
            return
        for rid in txn.members:
            robot = self.robots.get(rid)
            if robot is None:
                continue
            if robot.route_write_owner == 'joint_grid_transaction':
                robot.route_write_owner = None
                robot.route_write_until = 0.0
        if getattr(self, '_joint_plan_transaction', None) is txn:
            self._joint_plan_transaction = None
        print(f"[JOINT_RETIRE] epoch={txn.epoch} reason={reason} "
              f"members={sorted(txn.members)}")

    def _refresh_joint_grid_candidate(self):
        """Build and transactionally dispatch one all-active rolling plan."""
        active_txn = getattr(self, '_joint_plan_transaction', None)
        if active_txn is not None and active_txn.state == 'activated':
            all_complete = all(
                self._robot_completed_joint_txn(self.robots[rid],
                                                active_txn)
                for rid in active_txn.members)
            # Refresh immediately when every member already consumed its
            # prefix; otherwise use a short fixed rolling cadence. The old
            # plan keeps running while the next plan is prepared, so no
            # member ever needs to park waiting for the whole group.
            if not all_complete:
                min_interval = getattr(
                    self, '_joint_replan_min_interval', 2.0)
                if self.sim_time < active_txn.activate_at + min_interval:
                    return
            self._retire_joint_transaction(
                active_txn, reason='rolling_replan')
        agents = {}
        for rid, robot in self.robots.items():
            if not self._has_active_navigation(robot):
                moving_state = robot.state in (
                    RobotState.EN_ROUTE_PICKUP,
                    RobotState.CARRYING,
                    RobotState.EN_ROUTE_DELIVERY,
                    RobotState.RETURNING_HOME,
                    RobotState.RETURNING_TO_CHARGE,
                )
                recovery_leg = (
                    getattr(robot, 'recovery_active', False) or
                    robot.route_write_owner in (
                        '_command_reverse', '_resume_after_escape',
                        '_teleport_stalled_group', '_joint_group_recovery',
                    )
                )
                if moving_state and recovery_leg and self._navigation_goal(robot):
                    # A completed or abandoned recovery leg must not drop the
                    # robot out of the all-active planner.  Re-admit it with
                    # no stale waypoints so the next rolling window routes it
                    # from its measured position.
                    robot.active_plan_source = 'joint_grid_transaction'
                    robot.waypoints = []
                    robot.current_waypoint_idx = 0
                else:
                    continue
            goal_xy = self._goal_coordinates(self._navigation_goal(robot))
            if goal_xy is None:
                self._joint_grid_candidate = None
                self._joint_candidate_failures = int(getattr(
                    self, '_joint_candidate_failures', 0)) + 1
                # Hold only the unresolvable member; other robots may keep
                # following their currently safe joint plan.
                self._hold_robot(rid, 0.5)
                print(f"[JointGrid] T={self.sim_time:.1f}s rejected: "
                      f"active Robot {rid} has no resolvable business goal")
                continue
            agents[rid] = (robot.position, goal_xy)
        if not agents:
            self._joint_grid_candidate = None
            return
        # Multiple robots commonly share one workstation or storage dock.
        # Let the closest robot reserve that goal in this rolling window and
        # give every follower a collision-free staging point nearby. Without
        # this, the strict space-time planner sees two robots trying to park
        # in the same terminal cell and returns no_solution.
        grouped_goals = {}
        for rid, (_start, goal_xy) in agents.items():
            grouped_goals.setdefault(goal_xy, []).append(rid)
        planning_agents = {}
        for goal_xy, robot_ids in grouped_goals.items():
            primary = min(
                robot_ids,
                key=lambda rid: math.hypot(
                    self.robots[rid].position[0] - goal_xy[0],
                    self.robots[rid].position[1] - goal_xy[1]))
            followers = sorted(
                (rid for rid in robot_ids if rid != primary),
                key=lambda rid: math.hypot(
                    self.robots[rid].position[0] - goal_xy[0],
                    self.robots[rid].position[1] - goal_xy[1]))
            assigned_staging = []
            for rid in [primary] + followers:
                start = agents[rid][0]
                if rid == primary:
                    planning_agents[rid] = (start, goal_xy)
                    continue
                peers = [self.robots[peer_id].position
                         for peer_id in agents if peer_id != rid]
                staging = self._joint_staging_goal(
                    goal_xy, start, peers, assigned_staging)
                if staging != goal_xy:
                    assigned_staging.append(staging)
                planning_agents[rid] = (start, staging)
        failures = int(getattr(self, '_joint_candidate_failures', 0))
        budget = min(1.00, 0.25 + failures * 0.05)
        candidate = self.motion_coordinator.plan_joint_grid_candidate(
            planning_agents, max_seconds=budget)
        self._joint_grid_candidate = candidate
        if candidate is not None:
            self._joint_candidate_failures = 0
            omitted = getattr(candidate, 'omitted_robots', set())
            for rid in omitted:
                if rid in self.robots:
                    self._hold_robot(rid, 0.60)
            if omitted:
                self._next_joint_grid_tick = min(
                    getattr(self, '_next_joint_grid_tick', self.sim_time),
                    self.sim_time + 0.50)
            grid = self.motion_coordinator.grid
            plans = {}
            offsets = {}
            partial = {}
            for rid, timed_path in candidate.paths.items():
                # Slot zero is the measured start pose. Every later point
                # carries its earliest slot so repeated wait cells retain
                # their real time semantics in the controller.
                raw_points = [grid.grid_to_world(*step.cell)
                              for step in timed_path[1:]]
                # Slot 0 is the measured pose. The first future cell becomes
                # reachable immediately after atomic activation; otherwise
                # frequent rolling replans reset waypoint 0 before it can be
                # consumed and the fleet makes almost no forward progress.
                raw_offsets = [
                    max(0.0, (step.time_slot - 1) *
                         candidate.time_slot_seconds)
                    for step in timed_path[1:]
                ]
                points = []
                point_offsets = []
                if raw_points:
                    first = raw_points[0]
                    dx = first[0] - self.robots[rid].position[0]
                    dy = first[1] - self.robots[rid].position[1]
                    distance = math.hypot(dx, dy)
                    if distance > 0.18:
                        step_ratio = min(0.10, distance * 0.45) / distance
                        points.append((
                            self.robots[rid].position[0] + dx * step_ratio,
                            self.robots[rid].position[1] + dy * step_ratio,
                        ))
                        point_offsets.append(0.0)
                points.extend(raw_points)
                point_offsets.extend(raw_offsets)
                offsets[rid] = point_offsets
                goal = agents[rid][1]
                plans[rid] = points
                partial[rid] = bool(not points or math.hypot(
                    points[-1][0] - goal[0],
                    points[-1][1] - goal[1]) > GOAL_TOLERANCE * 2.0)
            txn = getattr(self, '_joint_plan_transaction', None)
            if (len(plans) >= 1 and
                    (txn is None or txn.state in ('activated', 'aborted'))):
                self._begin_joint_plan_transaction(
                    plans, waypoint_offsets=offsets,
                    partial_plans=partial)
        else:
            self._joint_candidate_failures = failures + 1
            # Preserve the last safe rolling prefix until the progress
            # watchdog confirms an actual liveness failure.  This avoids
            # replacing a still-safe route with an emergency per-robot route
            # on every transient space-time search miss.
            if getattr(self, '_joint_liveness_needed', False):
                fallback_plans, fallback_offsets, fallback_partial = (
                    self._build_joint_fallback_plans(planning_agents))
                txn = getattr(self, '_joint_plan_transaction', None)
                if (len(fallback_plans) >= 1 and
                        (txn is None or txn.state in ('activated', 'aborted'))):
                    self._begin_joint_plan_transaction(
                        fallback_plans, waypoint_offsets=fallback_offsets,
                        partial_plans=fallback_partial)
                    self._joint_liveness_needed = False
                    if getattr(self, '_next_joint_grid_log', 0.0) <= self.sim_time:
                        print(f"[JointGrid] T={self.sim_time:.1f}s fallback "
                              f"liveness plan dispatched for "
                              f"{len(fallback_plans)}/{len(agents)} robots")
        if self.sim_time >= getattr(self, '_next_joint_grid_log', 0.0):
            if candidate is None:
                print(f"[JointGrid] T={self.sim_time:.1f}s no complete "
                      f"candidate for {len(agents)} active robots within "
                      f"{budget * 1000:.0f}ms")
            else:
                print(f"[JointGrid] T={self.sim_time:.1f}s validated "
                      f"{len(candidate.paths)}-robot candidate in "
                      f"{candidate.planning_seconds * 1000:.1f}ms; "
                      f"expanded={candidate.expanded_nodes}")
            self._next_joint_grid_log = self.sim_time + 10.0

    def _navigation_is_activated(self, robot) -> bool:
        """Return whether an active leg is dispatched and not legally held."""
        if not self._has_active_navigation(robot):
            return False
        if robot.pending_waypoints is not None:
            return False
        return self.sim_time >= max(
            robot.dispatch_not_before,
            robot.hold_until,
            robot.controller_paused_until,
        )

    def _robot_should_be_moving(self, robot) -> bool:
        """True only while an activated plan should make physical progress."""
        return (self._navigation_is_activated(robot) and
                not robot.emergency_braking)

    def _robot_requires_recovery_monitoring(self, robot) -> bool:
        """Monitor activated legs even while a safety brake prevents motion."""
        if (robot.recovery_session_role is not None and
                self.sim_time < robot.recovery_session_until):
            return False
        return self._navigation_is_activated(robot)

    def _coordinate_emergency_pair(self) -> bool:
        """Assign one deterministic yielder for a close emergency pair."""
        candidates = []
        ids = sorted(self.robots)
        for index, rid_a in enumerate(ids):
            first = self.robots[rid_a]
            if not first.emergency_braking or not self._has_active_navigation(first):
                continue
            for rid_b in ids[index + 1:]:
                second = self.robots[rid_b]
                if (not second.emergency_braking or
                        not self._has_active_navigation(second)):
                    continue
                distance = math.hypot(
                    first.position[0] - second.position[0],
                    first.position[1] - second.position[1])
                if distance < 0.80:
                    candidates.append((distance, rid_a, rid_b))
        if not candidates:
            return False
        _, winner, yielder = min(candidates)
        first = self.robots[winner]
        second = self.robots[yielder]
        if (self.sim_time < first.recovery_session_until or
                self.sim_time < second.recovery_session_until):
            first._replan_requested = False
            second._replan_requested = False
            return True

        first.recovery_session_role = 'winner'
        second.recovery_session_role = 'yielder'
        first.recovery_session_until = self.sim_time + 8.0
        second.recovery_session_until = self.sim_time + 8.0
        first._replan_requested = False
        second._replan_requested = False
        second.recovery_started_position = tuple(second.position)
        if self._command_reverse(yielder, 0.3):
            print(f"[RecoverySession] Robot {winner} keeps right-of-way; "
                  f"Robot {yielder} executes validated escape")
            return True

        # The yielder may use the normal planner when no local escape exists;
        # the winner remains suppressed so both sides never replan together.
        second.recovery_session_role = None
        second.recovery_session_until = 0.0
        second._replan_requested = True
        return False
    
    def _joint_request_fresh_plan(self, robot_ids, reason: str) -> bool:
        """Force a new all-active rolling window from measured positions."""
        now = self.sim_time
        if getattr(self, '_joint_replan_cooldown_until', 0.0) > now:
            return False
        self._joint_replan_cooldown_until = now + 1.5
        txn = getattr(self, '_joint_plan_transaction', None)
        if txn is not None and txn.state not in ('activated', 'aborted'):
            txn.abort(reason)
            self._advance_joint_plan_transaction()
        if txn is not None and txn.state == 'activated':
            for rid in txn.members:
                robot = self.robots[rid]
                if robot.route_write_owner == 'joint_grid_transaction':
                    robot.route_write_owner = None
                    robot.route_write_until = 0.0
            self._joint_plan_transaction = None
        self._next_joint_grid_tick = min(
            getattr(self, '_next_joint_grid_tick', now), now)
        self._joint_candidate_failures = 0
        self._log_replan(
            f"[JointWatchdog] fresh joint plan requested reason={reason} "
            f"robots={sorted(robot_ids)}")
        return True

    def _joint_hold_group(self, robot_ids, duration: float) -> None:
        """Coordinated short stop for all members of a conflict component."""
        for rid in robot_ids:
            self._hold_robot(rid, duration)

    def _find_joint_safety_profile(self, component, base_trajectories,
                                   minimum_distance=0.65):
        """Find a non-stopping speed profile that preserves joint clearance."""
        component = tuple(sorted(component))
        levels = (1.0, 0.85, 0.70, 0.55, 0.40)
        search_started = real_time.perf_counter()
        cache = {}

        def trajectory(rid, scale):
            key = (rid, scale)
            if key not in cache:
                cache[key] = self._trajectory_for_scale(rid, scale, 4.0, 0.25)
            return cache[key]

        current = {rid: self.robots[rid].speed_scale for rid in component}
        profiles = [current]
        if len(component) <= 4:
            profiles.extend(
                dict(zip(component, scales))
                for scales in itertools.product(levels, repeat=len(component))
                if dict(zip(component, scales)) != current
            )
        else:
            profiles.extend({rid: level for rid in component}
                            for level in levels[1:])
            for offset, rid in enumerate(component):
                profiles.append({
                    peer: (levels[(index + offset) % len(levels)]
                           if peer == rid else 1.0)
                    for index, peer in enumerate(component)
                })

        safe = []
        for profile in profiles:
            if real_time.perf_counter() - search_started > 0.08:
                break
            candidate = dict(base_trajectories)
            for rid, scale in profile.items():
                candidate[rid] = trajectory(rid, scale)
            conflicts = self._trajectory_conflicts(
                candidate, minimum_distance, only_robot_ids=component)
            if conflicts:
                continue
            efficiency = sum(profile.values())
            minimum_speed = min(profile.values())
            changes = sum(
                abs(profile[rid] - self.robots[rid].speed_scale)
                for rid in component)
            safe.append((efficiency, minimum_speed, -changes, profile))
        if not safe:
            return None
        safe.sort(key=lambda item: item[:3], reverse=True)
        return safe[0][3]

    def _joint_predictive_speed_shield(self, active_robots) -> bool:
        """Coordinate dense traffic with low-intervention joint shaping.

        Normal predicted conflicts are handled only by speed profiles. A
        single-robot moving escape is reserved for an already-critical
        physical separation (<0.70 m), where a route-writer-free slowdown may
        be too late.
        """
        now = self.sim_time
        if getattr(self, '_shield_debug_enabled', None) is None:
            self._shield_debug_enabled = os.environ.get(
                'SMART_FACTORY_DEBUG_SHIELD', '0') == '1'
            self._shield_debug_path = os.environ.get(
                'SMART_FACTORY_DEBUG_SHIELD_PATH',
                r'D:\code\smart_factory_scheduler\results\joint_shield_debug.log')
            if self._shield_debug_enabled:
                with open(self._shield_debug_path, 'w', encoding='utf-8') as handle:
                    handle.write('sim_time,trajectories,conflicts,components,actions\n')
        horizon = 6.0
        sample_dt = 0.25
        minimum_distance = 0.72
        trajectories = {}
        for rid, robot in active_robots.items():
            if not robot.waypoints:
                continue
            if robot.current_waypoint_idx >= len(robot.waypoints):
                continue
            traj = self._trajectory_for_scale(
                rid, robot.speed_scale, horizon, sample_dt)
            if traj:
                trajectories[rid] = traj
        if len(trajectories) < 2:
            return False

        actual_conflicts = []
        active_ids = sorted(trajectories)
        for index, rid_a in enumerate(active_ids):
            first = self.robots[rid_a]
            for rid_b in active_ids[index + 1:]:
                second = self.robots[rid_b]
                distance = math.hypot(
                    first.position[0] - second.position[0],
                    first.position[1] - second.position[1])
                if distance < 0.85:
                    actual_conflicts.append(
                        (rid_a, rid_b, 0.0, distance))
        if actual_conflicts:
            actual_components = self._conflict_components(actual_conflicts)
            if self._shield_debug_enabled:
                with open(self._shield_debug_path, 'a', encoding='utf-8') as handle:
                    handle.write(
                        f'{now:.3f},actual,{actual_conflicts},'
                        f'{actual_components},layered_low_only\n')
            for component in actual_components:
                component = tuple(sorted(component))
                component_distances = [
                    math.hypot(self.robots[a].position[0] -
                               self.robots[b].position[0],
                               self.robots[a].position[1] -
                               self.robots[b].position[1])
                    for index, a in enumerate(component)
                    for b in component[index + 1:]
                ]
                closest = min(component_distances) if component_distances else 0.0
                yielder = component[-1]
                if closest < 0.70:
                    if self._joint_try_escape_component(component):
                        continue
                for rid in component:
                    scale = 0.45 if rid == yielder else 0.75
                    if self._set_robot_speed_scale(rid, scale):
                        self.robots[rid].joint_shield_until = now + 2.0
            return True

        conflicts = self._trajectory_conflicts(
            trajectories, minimum_distance)
        if self._shield_debug_enabled:
            with open(self._shield_debug_path, 'a', encoding='utf-8') as handle:
                handle.write(
                    f'{now:.3f},{sorted(trajectories)},{conflicts},,,\n')
        if not conflicts:
            if getattr(self, '_next_shield_log', 0.0) <= now:
                print(f"[JointShield] T={now:.1f}s no predicted conflicts "
                      f"active={sorted(trajectories)}", file=sys.stderr)
                self._next_shield_log = now + 5.0
            for rid, robot in active_robots.items():
                if (rid in trajectories and robot.speed_scale < 0.99 and
                        getattr(robot, 'joint_shield_until', 0.0) <= now):
                    self._set_robot_speed_scale(rid, 1.0)
                    robot.joint_shield_until = 0.0
            return False

        components = self._conflict_components(conflicts)
        acted = False
        if getattr(self, '_next_shield_log', 0.0) <= now:
            print(f"[JointShield] T={now:.1f}s conflicts={len(conflicts)} "
                  f"components={components}", file=sys.stderr)
            self._next_shield_log = now + 2.0
        for component in components:
            component = tuple(sorted(component))
            earliest = min(
                (conflict[2] for conflict in conflicts
                 if conflict[0] in component and conflict[1] in component),
                default=10.0)
            profile = self._find_joint_safety_profile(
                component, trajectories, minimum_distance)
            print(f"[JointShield] T={now:.1f}s component={component} "
                  f"earliest={earliest:.2f}s profile={profile}", file=sys.stderr)
            if profile is None:
                ordered = sorted(component)
                profile = {rid: 1.0 for rid in component}
                profile[ordered[-1]] = 0.55
                if len(component) > 2:
                    profile[ordered[-2]] = 0.70
            if not self._apply_joint_speed_profile(profile):
                continue
            for rid, scale in profile.items():
                trajectories[rid] = self._trajectory_for_scale(
                    rid, scale, horizon, sample_dt)
                self.robots[rid].joint_shield_until = now + 1.0
            acted = True
        return acted

    def _joint_collision_scan(self, active_robots) -> bool:
        """Predict rolling trajectories and intervene before a collision.

        Early (<1.2 s) conflicts use deterministic speed-shaping first. A full
        joint replan is reserved only for a genuinely imminent collision
        (<0.30 s) and is rate-limited so a dense scenario cannot starve
        forward progress by repeatedly aborting otherwise safe joint windows.
        """
        return self._joint_predictive_speed_shield(active_robots)
        now = self.sim_time
        pairs = []
        ids = sorted(self.robots)
        for index, rid_a in enumerate(ids):
            first = self.robots[rid_a]
            for rid_b in ids[index + 1:]:
                second = self.robots[rid_b]
                distance = math.hypot(
                    first.position[0] - second.position[0],
                    first.position[1] - second.position[1])
                if distance < 0.90:
                    pairs.append((distance, rid_a, rid_b))
        if not pairs:
            return False
        pairs.sort(key=lambda item: item[0])
        distance, rid_a, rid_b = pairs[0]
        first = self.robots[rid_a]
        second = self.robots[rid_b]
        if distance < 0.50:
            yielder = rid_b if rid_a < rid_b else rid_a
            yielder_robot = self.robots[yielder]
            winner = rid_a if yielder == rid_b else rid_b
            self._set_robot_speed_scale(winner, 0.60)
            self._set_robot_speed_scale(yielder, 0.40)
            if getattr(yielder_robot, '_joint_critical_escape_until', 0.0) <= now:
                if self._command_reverse(yielder, 0.75):
                    yielder_robot._joint_critical_escape_until = now + 2.0
                    self._joint_request_fresh_plan(
                        [rid_a, rid_b], 'joint_pair_too_close')
                    return True
            self._hold_robot(yielder, 0.35)
            self._joint_request_fresh_plan(
                [rid_a, rid_b], 'joint_pair_too_close')
            return True
        if distance < 0.80:
            yielder = rid_b if rid_a < rid_b else rid_a
            if self.robots[yielder].hold_until > now + 0.5:
                yielder = rid_a if yielder == rid_b else rid_b
            self._set_robot_speed_scale(yielder, 0.55)
            if distance < 0.65:
                self._hold_robot(yielder, 0.25)
            return True
        return False
        now = self.sim_time
        trajectories = {}
        for rid, robot in active_robots.items():
            if not robot.waypoints:
                continue
            if robot.current_waypoint_idx >= len(robot.waypoints):
                continue
            start_delay = max(
                0.0,
                getattr(robot, 'dispatch_not_before', 0.0) - now,
                getattr(robot, 'hold_until', 0.0) - now,
                getattr(robot, 'controller_paused_until', 0.0) - now,
                getattr(robot, 'controller_joint_wait_until', 0.0) - now,
            )
            if robot.emergency_braking:
                start_delay = 10.0
            traj = self._predict_trajectory(
                robot.position,
                robot.waypoints[robot.current_waypoint_idx:],
                0.22 * robot.speed_scale,
                10.0, 0.5, start_delay=start_delay)
            if traj:
                trajectories[rid] = traj
        if len(trajectories) < 2:
            return False
        conflicts = self._trajectory_conflicts(trajectories, 0.45)
        if not conflicts:
            return False

        components = self._conflict_components(conflicts)
        acted = False
        for component in components:
            component = list(component)
            times = [conflict[2] for conflict in conflicts
                     if conflict[0] in component and conflict[1] in component]
            earliest = min(times) if times else 10.0
            if earliest <= 0.30:
                # Do not abort the shared rolling transaction here. A full
                # replan is requested by the long-stall watchdog if this
                # imminent component fails to make progress, avoiding the
                # replan/abort loop that used to starve the fleet.
                yielder = max(component)
                for rid in component:
                    self._set_robot_speed_scale(
                        rid, 0.5 if rid == yielder else 0.8)
                self._joint_hold_group(component, 0.30)
                acted = True
                continue
            if earliest <= 1.2:
                # Keep the joint route; shape speed so one robot yields early.
                # The next watchdog pass may escalate if the projection is
                # still unsafe.
                yielder = max(component)
                for rid in component:
                    self._set_robot_speed_scale(
                        rid, 0.6 if rid == yielder else 1.0)
                self._joint_hold_group([yielder], 0.15)
                acted = True
        return acted

    def _joint_runtime_watchdog(self) -> None:
        """Joint-mode progress/collision guarantee.

        The legacy per-robot replan stack is intentionally disabled in joint
        mode. This watchdog keeps its benefits without allowing independent
        route writes: it detects stalls, stale endpoint waits and emergency
        requests, briefly holds the affected component, and forces a fresh
        all-active joint plan.
        """
        if not getattr(self, 'system_ready', True):
            return
        now = self.sim_time
        active = {
            rid: robot for rid, robot in self.robots.items()
            if self._has_active_navigation(robot)
        }
        if not active:
            return

        self._joint_collision_scan(active)
        emergency = []
        stale_wait = []
        stalled = []
        for rid, robot in active.items():
            legal_wait = bool(
                robot.pending_waypoints is not None or
                now < max(
                    robot.hold_until,
                    robot.controller_paused_until,
                    robot.controller_joint_wait_until,
                    robot.dispatch_not_before,
                ))
            if getattr(robot, '_replan_requested', False) or robot.emergency_braking:
                emergency.append(rid)
                robot._joint_watch_pos = robot.position
                robot._joint_watch_since = now
                continue
            if legal_wait:
                if robot.controller_joint_wait_reason in (
                        'joint_window_endpoint', 'joint_epoch_barrier'):
                    txn = getattr(self, '_joint_plan_transaction', None)
                    wait_started = getattr(robot, '_joint_wait_started', None)
                    if wait_started is None:
                        robot._joint_wait_started = now
                    elif (txn is None or txn.state == 'aborted' or
                            now - wait_started > 2.0):
                        stale_wait.append(rid)
                else:
                    robot._joint_wait_started = None
                robot._joint_watch_pos = robot.position
                robot._joint_watch_since = now
                continue

            last_pos = getattr(robot, '_joint_watch_pos', robot.position)
            moved = math.hypot(
                robot.position[0] - last_pos[0],
                robot.position[1] - last_pos[1])
            if moved >= STALL_PROGRESS_DISTANCE:
                robot._joint_watch_pos = robot.position
                robot._joint_watch_since = now
                robot._joint_stall_count = 0
                continue
            if getattr(robot, '_joint_watch_since', None) is None:
                robot._joint_watch_since = now
                robot._joint_watch_pos = robot.position
                continue
            if now - robot._joint_watch_since >= 3.0:
                stalled.append(rid)

        if emergency or stale_wait or stalled:
            ids = set(emergency + stale_wait + stalled)
            # Mark liveness fallback for the next rolling-planner miss.  The
            # coordinated space-time planner remains preferred; this flag only
            # permits emergency Cooperative-A* routes when the fleet is
            # actually at risk of parking forever.
            self._joint_liveness_needed = True
            # Do not pause the old joint route while the replacement is
            # prepared. The atomic transaction swaps the plan only after all
            # controllers have acknowledged the next safe window, so holding
            # here only converts a temporary slowdown into a full stop.
            self._joint_request_fresh_plan(ids, 'joint_runtime_watchdog')
            for rid in ids:
                robot = self.robots[rid]
                robot._joint_watch_pos = robot.position
                robot._joint_watch_since = now
                robot._joint_wait_started = None
            return

    def __init__(self, scenario: str = "C", scheduler_type: str = "FCFS",
                 seed: int = 42, model_path: Optional[str] = None):
        """
        Initialize the factory supervisor.
        
        Args:
            scenario: Experiment scenario ("A", "B", or "C").
            scheduler_type: Scheduler to use ("FCFS", "NearestNeighbour", 
                          "RoundRobin", "PPO_RL").
            seed: Random seed for reproducibility.
            model_path: Path to trained RL model (for PPO_RL scheduler).
        """
        # Initialize Webots supervisor
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        # Freeze the route-writer policy for this run. Tests and explicitly
        # configured legacy scenarios can exercise the old stack separately.
        self.joint_runtime_enforced = ENABLE_JOINT_RUNTIME
        
        # Scenario configuration
        self.scenario_config = SCENARIOS[scenario]
        self.num_robots = self.scenario_config['num_robots']
        self.scenario_name = scenario
        self._battery_rng = random.Random(seed)
        self.expected_robot_ids = set(range(1, self.num_robots + 1))
        self.startup_gate = StartupGate(set(self.expected_robot_ids))
        self.ready_robot_ids = set()
        self.duplicate_ready_robot_ids = set()
        self.robot_initialization_errors = {}
        self.system_ready = False
        self.initial_dispatch_triggered = False
        self.initial_dispatch_attempted = False
        self.dispatch_in_progress = False
        self.dispatch_pending = False
        self.startup_failed = False
        self.startup_timeout_logged = False
        self.startup_timestamp = 0.0
        self.last_robot_ready_time = None
        self.first_dispatch_time = None
        
        # Initialize components
        self.task_generator = TaskGenerator(
            mean_interval=self.scenario_config['task_interval'],
            seed=seed,
            initial_task_immediately=self.scenario_config.get(
                'initial_task_immediately', False)
        )
        self.motion_coordinator = MotionCoordinator(num_active_robots=self.num_robots)
        self.scheduler = create_scheduler(
            scheduler_type, model_path, seed=seed, allow_safe_fallback=True)
        self.safe_schedulers = [
            HungarianScheduler(), GreedyScheduler(),
            NearestNeighbourScheduler()]
        self._failed_assignment_pairs: Dict[Tuple[int, int], float] = {}
        self._next_plan_epoch = 1
        self._joint_plan_transaction = None
        self._joint_collision_danger = {}
        self._joint_preempt_component_until: Dict[Tuple[int, ...], float] = {}
        self.metrics = MetricsCollector(
            scenario_name=scenario,
            scheduler_name=self.scheduler.name,
            num_robots=self.num_robots
        )
        
        # Robot tracking
        self.robots: Dict[int, RobotInfo] = {}
        # The shared world template contains the maximum fleet (8) so one
        # world can serve A/B/C.  Remove surplus physical Robot nodes before
        # caching references; this is a real scene-tree deletion, not a
        # controller/visibility workaround.
        self._apply_scenario_robot_count()
        self._init_robots()
        
        # Set up communication
        self._init_communication()
        
        # Simulation state
        self.sim_time = 0.0
        self.step_count = 0
        self.running = True
        self._debug_state_path = os.environ.get(
            'SMART_FACTORY_DEBUG_STATES',
            r'D:\code\smart_factory_scheduler\results\debug_states.log')
        self._debug_state_enabled = (
            os.environ.get('SMART_FACTORY_DEBUG_STATES_ENABLED', '0') == '1')
        if self._debug_state_enabled:
            with open(self._debug_state_path, 'w', encoding='utf-8') as handle:
                handle.write('sim_time,robot_id,state,pos_x,pos_y,goal_x,goal_y,'
                             'active_plan_source,active_plan_epoch,'
                             'controller_epoch,controller_waypoint_idx,'
                             'waypoints,current_task,hold_until,speed_scale\n')
        
        print(f"[Supervisor] Initialized: Scenario {scenario} "
              f"({self.scenario_config['description']})")
        print(f"[Supervisor] Scheduler: {scheduler_type}")
        print(f"[Supervisor] Robots: {self.num_robots}")
        scene_count = self._count_scene_physical_robots()
        if scene_count != self.num_robots:
            raise RuntimeError(
                f"Scene robot count mismatch: expected {self.num_robots}, "
                f"found {scene_count}")
        print(f"[Supervisor] Scene physical robots: {scene_count}")

    def _count_scene_physical_robots(self) -> int:
        """Count physical ROBOT_n nodes currently present in the scene tree."""
        root = self.supervisor.getRoot()
        children = root.getField("children")
        if children is None:
            raise RuntimeError("world root has no children field")
        count = 0
        for index in range(children.getCount()):
            child = children.getMFNode(index)
            if child is None:
                continue
            name_field = child.getField("name")
            name = name_field.getSFString() if name_field else ""
            if name.startswith("robot_"):
                count += 1
        return count

    def _apply_scenario_robot_count(self):
        """Delete surplus static Robot nodes for the selected scenario."""
        target = int(self.num_robots)
        if target >= MAX_ROBOTS:
            return
        try:
            removed = []
            # Use the native node removal API. Keeping an MFNode index or a
            # handle returned before removal can leave an invalid Webots node
            # pointer and produce get_field errors.
            for rid in range(MAX_ROBOTS, target, -1):
                node = self.supervisor.getFromDef(f"ROBOT_{rid}")
                if node is None:
                    continue
                node.remove()
                removed.append(f"robot_{rid}")
            if removed:
                print(f"[Supervisor] Scenario {self.scenario_name}: removed "
                      f"surplus scene robots {', '.join(removed)}")
        except Exception as exc:
            # Fail closed: a scenario that cannot enforce its count must not
            # silently run with an incorrect fleet.
            raise RuntimeError(
                f"Unable to enforce {self.scenario_name} robot count "
                f"({target}): {exc}") from exc

    def _init_robots(self):
        """Initialize robot state tracking and cache Webots node references."""
        # Initial positions matching the .wbt file
        # Single source of truth shared with the Webots scene. Keeping a
        # second literal table here previously made startup reservations use
        # incorrect positions until the first GPS update.
        initial_positions = dict(PARKING_SPOTS)
        
        # Cache Webots Node references so we look them up only once.
        # DEF names in the .wbt are ROBOT_1 �?ROBOT_8.
        self.robot_nodes: Dict[int, object] = {}
        
        for rid in range(1, self.num_robots + 1):
            pos = initial_positions.get(rid, (0.0, 0.0))
            initial_battery = self._battery_rng.uniform(
                INITIAL_BATTERY_MIN, INITIAL_BATTERY_MAX)
            self.robots[rid] = RobotInfo(rid, pos, initial_battery)
            
            # Resolve the Webots scene node via its DEF name
            def_name = f"ROBOT_{rid}"
            try:
                node = self.supervisor.getFromDef(def_name)
                if node is not None:
                    self.robot_nodes[rid] = node
                else:
                    print(f"[Supervisor] WARNING: DEF {def_name} not found in scene")
            except Exception as e:
                print(f"[Supervisor] WARNING: Could not get node {def_name}: {e}")
        
        # Set motion coordinator priorities
        self.motion_coordinator.set_priorities(list(self.robots.keys()))

    def _init_communication(self):
        """Initialize Webots Emitter/Receiver for robot communication."""
        try:
            self.emitter = self.supervisor.getDevice("supervisor_emitter")
            self.receiver = self.supervisor.getDevice("supervisor_receiver")
            if self.receiver:
                self.receiver.enable(self.timestep)
        except Exception as e:
            print(f"[Supervisor] Communication init warning: {e}")
            self.emitter = None
            self.receiver = None

    def _get_robot_positions_from_webots(self):
        """Read robot positions using cached Webots supervisor node references.
        
        Uses getFromDef() nodes that were resolved once in _init_robots().
        Never calls getFromDevice() �?that API is only valid for devices
        (sensors / actuators) attached to the *calling* robot node, not for
        looking up other robots in the scene.
        """
        for rid in range(1, self.num_robots + 1):
            robot_node = self.robot_nodes.get(rid)
            if robot_node is None:
                continue  # Node was not found at init; skip
            
            try:
                pos_field = robot_node.getField("translation")
                if pos_field:
                    pos = pos_field.getSFVec3f()
                    # ENU coordinate system (as used in the .wbt file):
                    #   x = east, y = north, z = up
                    # We use (x, y) as the 2D ground-plane coordinates.
                    self.robots[rid].position = (pos[0], pos[1])
                
                rot_field = robot_node.getField("rotation")
                if rot_field:
                    rot = rot_field.getSFRotation()
                    # ENU: robots rotate around the z-axis (0, 0, 1, angle)
                    if abs(rot[2]) > 0.5:          # rotation axis �?z
                        self.robots[rid].heading = rot[3]
                    else:
                        # Fallback: compute heading from the rotation axis
                        self.robots[rid].heading = rot[3] if rot[2] >= 0 else -rot[3]
            except Exception:
                pass  # Keep last-known position on read failure

    def _send_command_to_robot(self, robot_id: int, command: dict):
        """Send a navigation command to a specific robot via Emitter."""
        if not self.emitter:
            print(f"[Supervisor] Send skipped for robot {robot_id}: emitter unavailable")
            return False
        try:
            msg = json.dumps({
                'target_robot': robot_id,
                'command': command
            })
            self.emitter.send(msg.encode('utf-8'))
            return True
        except Exception as e:
            print(f"[Supervisor] Send error to robot {robot_id}: {e}")
            return False

    def _begin_joint_plan_transaction(
            self, plans, timeout=0.5, waypoint_offsets=None,
            partial_plans=None) -> bool:
        """Prepare every member while all controllers keep their old paths."""
        active = self._joint_plan_transaction
        if active is not None and active.state not in ("activated", "aborted"):
            return False
        epoch = self._next_plan_epoch
        self._next_plan_epoch += 1
        normalized = {
            rid: [tuple(point) for point in path]
            for rid, path in plans.items() if path
        }
        if set(normalized) != set(plans) or not normalized:
            return False
        if any(not self._route_write_allowed(
                self.robots[rid], 'joint_grid_transaction')
                for rid in normalized):
            return False
        for rid in normalized:
            robot = self.robots[rid]
            robot.route_write_generation += 1
            robot.route_write_owner = 'joint_grid_transaction'
            robot.route_write_until = self.sim_time + timeout + 3.0
        versions = {
            rid: max(self.robots[rid].path_version,
                     self.robots[rid].controller_path_version) + 1
            for rid in normalized
        }
        txn = JointPlanTransaction(
            epoch=epoch, plans=normalized, versions=versions,
            created_at=self.sim_time, deadline=self.sim_time + timeout)
        self._joint_plan_transaction = txn
        txn.waypoint_offsets = waypoint_offsets or {}
        txn.partial_plans = partial_plans or {}
        txn.writer_generations = {
            rid: self.robots[rid].route_write_generation for rid in normalized}
        self.motion_coordinator.joint_transactions_attempted += 1
        for rid in sorted(normalized):
            sent = self._send_command_to_robot(rid, {
                'type': 'prepare_plan', 'plan_epoch': epoch,
                'path_version': versions[rid],
                'all_waypoints': [list(point) for point in normalized[rid]],
                'waypoint_not_before_offsets': list(
                    txn.waypoint_offsets.get(rid, ())),
                'partial_plan': bool(txn.partial_plans.get(rid, False)),
            })
            if not sent:
                txn.acknowledge(rid, False)
        self._advance_joint_plan_transaction()
        return txn.state == "preparing"

    def _advance_joint_plan_transaction(self):
        txn = getattr(self, '_joint_plan_transaction', None)
        if txn is None:
            return
        decision = txn.decision(self.sim_time)
        if decision == "ready":
            activate_at = self.sim_time + 0.5
            txn.begin_arming(activate_at)
            send_results = []
            for rid in sorted(txn.members):
                send_results.append(self._send_command_to_robot(rid, {
                    'type': 'arm_plan', 'plan_epoch': txn.epoch,
                }))
            if not all(send_results):
                txn.abort("arm_send_failed")
        if txn.state == "arming" and self.sim_time >= txn.activate_at - 0.1:
            txn.abort("arm_ack_timeout")
        if txn.state == "armed":
            activate_at = self.sim_time + 0.5
            txn.begin_commit(activate_at)
            send_results = []
            for rid in sorted(txn.members):
                send_results.append(self._send_command_to_robot(rid, {
                    'type': 'commit_plan', 'plan_epoch': txn.epoch,
                    'activate_at': activate_at,
                }))
            if not all(send_results):
                txn.abort("commit_send_failed")
        if (txn.state == "committing" and
                self.sim_time >= txn.activate_at - 0.1):
            txn.abort("commit_ack_timeout")
        if (txn.state == "committed" and
                self.sim_time >= txn.activate_at + 0.5):
            txn.abort("activation_ack_timeout")
        if txn.state == "activation_confirmed":
            for rid in txn.members:
                robot = self.robots[rid]
                robot.path_version = txn.versions[rid]
                robot.waypoints = list(txn.plans[rid])
                robot.current_waypoint_idx = 0
                robot.dispatch_not_before = txn.activate_at
                robot.active_plan_epoch = txn.epoch
                robot.active_plan_source = 'joint_grid_transaction'
                robot.active_joint_started_at = txn.activate_at
                txn_offsets = getattr(txn, 'waypoint_offsets', {}).get(
                    rid, ())
                last_offset = txn_offsets[-1] if txn_offsets else 0.0
                robot.active_joint_wait_deadline = (
                    txn.activate_at + last_offset + 10.0)
                if getattr(self, 'metrics', None) is not None:
                    self.metrics.record_route_dispatch(
                        rid, robot.path_version, txn.epoch,
                        'joint_grid_transaction', len(txn.plans[rid]),
                        self.sim_time)
            txn.mark_activated()
            max_last_offset = max(
                (values[-1] if values else 0.0 for values in
                 getattr(txn, 'waypoint_offsets', {}).values()),
                default=0.0)
            for rid in txn.members:
                # Cover the full rolling-prefix timeout used by the refresh
                # gate, plus a small handoff margin.
                self.robots[rid].route_write_until = (
                    txn.activate_at + max_last_offset + 11.0)
            self.motion_coordinator.joint_transactions_activated += 1
            print(f"[JointTxn] epoch={txn.epoch} armed for "
                  f"T={txn.activate_at:.3f}, members={sorted(txn.members)}")
            return
        if txn.state == "aborted":
            next_retry = getattr(txn, '_next_abort_retry', 0.0)
            if (txn.aborted_members != txn.members and
                    self.sim_time >= next_retry):
                for rid in txn.members:
                    if rid not in txn.aborted_members:
                        self._send_command_to_robot(rid, {
                            'type': 'abort_plan', 'plan_epoch': txn.epoch})
                txn._next_abort_retry = self.sim_time + 0.1
                if not getattr(txn, '_abort_logged', False):
                    txn._abort_logged = True
                    self.motion_coordinator.joint_transactions_aborted += 1
                    print(f"[JointTxn] epoch={txn.epoch} aborted: "
                          f"{txn.failure_reason}")
            if txn.aborted_members == txn.members:
                for rid, generation in txn.writer_generations.items():
                    robot = self.robots[rid]
                    if (robot.route_write_owner == 'joint_grid_transaction' and
                            robot.route_write_generation == generation):
                        robot.route_write_owner = None
                        robot.route_write_until = 0.0

    def _advance_joint_execution_barrier(self):
        """No fan-out release is needed for an atomically committed prefix."""
        return
    def _set_robot_speed_scale(self, robot_id: int, scale: float) -> bool:
        """Adjust speed without resetting the active path or its version."""
        robot = self.robots[robot_id]
        scale = max(0.4, min(1.0, float(scale)))
        if abs(robot.speed_scale - scale) < 0.01:
            return True
        if self._send_command_to_robot(
                robot_id, {'type': 'set_speed_scale', 'scale': scale}):
            robot.speed_scale = scale
            return True
        return False

    @staticmethod
    def _route_writer_priority(source: str) -> int:
        if source == '_command_reverse':
            return 100
        if source in ('_joint_group_recovery', '_resume_after_escape'):
            return 95
        if source == 'joint_grid_transaction':
            return 90
        if source in ('_assign_tasks_impl', '_handle_goal_reached',
                      '_return_robot_home', '_relocate_idle_robots'):
            return 80
        if source == '_proactive_path_conflict_scan':
            return 50
        return 60

    def _route_write_allowed(self, robot, source: str) -> bool:
        if (robot.route_write_owner is None or
                robot.route_write_owner == source or
                self.sim_time >= robot.route_write_until):
            return True
        return (self._route_writer_priority(source) >
                self._route_writer_priority(robot.route_write_owner))

    def _dispatch_plan(self, robot_id: int, waypoints,
                       delay: Optional[float] = None,
                       source: Optional[str] = None,
                       is_runtime_replan: bool = False):
        """Single versioned entry point for navigation plan dispatch."""
        self._last_command_send_ok = False
        self._last_plan_dispatch_performed = False
        if not waypoints:
            return False
        if source is None:
            frame = inspect.currentframe()
            source = (frame.f_back.f_code.co_name
                      if frame is not None and frame.f_back is not None
                      else 'unknown')
        recovery_sources = {
            '_command_reverse', '_joint_group_recovery',
            '_resume_after_escape', '_teleport_stalled_group',
        }
        txn = getattr(self, '_joint_plan_transaction', None)
        if (txn is not None and robot_id in txn.members and
                txn.state == 'activated' and
                source != 'joint_grid_transaction'):
            # A completed business leg may immediately install its pickup /
            # delivery successor. Retire the old rolling epoch so its
            # waypoint-index gate cannot constrain that new route. Other
            # members keep executing their last safe route until the next
            # all-active transaction replaces it.
            for member in txn.members:
                peer = self.robots[member]
                if peer.route_write_owner == 'joint_grid_transaction':
                    peer.route_write_owner = None
                    peer.route_write_until = 0.0
            self._joint_plan_transaction = None
            txn = None
        if (txn is not None and robot_id in txn.members and
                txn.state not in ("activated", "aborted")):
            txn.abort("superseded_by_legacy_dispatch")
            self._advance_joint_plan_transaction()
            if source not in recovery_sources:
                return False
        joint_intent_sources = {
            '_assign_tasks_impl', '_handle_goal_reached',
            '_send_to_charging', '_send_to_home',
            '_relocate_idle_robots',
        }
        if (ENABLE_JOINT_RUNTIME and not is_runtime_replan and
                source in joint_intent_sources):
            # The caller has already installed the authoritative business
            # goal/waypoints in RobotInfo. Do not let a normal leg start as a
            # single-robot navigate; the next loop turns the intent into an
            # all-active atomic joint transaction.
            self._last_command_send_ok = True
            self._next_joint_grid_tick = min(
                getattr(self, '_next_joint_grid_tick', self.sim_time),
                self.sim_time)
            return True
        robot = self.robots[robot_id]
        if (getattr(self, 'joint_runtime_enforced', False) and
                source != 'joint_grid_transaction' and
                source not in recovery_sources):
            # Production joint mode has exactly one physical route writer
            # except for validated recovery actions. Recovery still triggers
            # a fresh all-active joint window immediately afterwards.
            if hasattr(getattr(self, 'metrics', None),
                       'record_unauthorized_route_write'):
                self.metrics.record_unauthorized_route_write(
                    self.sim_time, robot_id, source, robot.path_version,
                    robot.active_plan_epoch)
            print(f"[UNAUTHORIZED_ROUTE_WRITE] robot={robot_id} "
                  f"source={source} decision=rejected")
            self._next_joint_grid_tick = min(
                getattr(self, '_next_joint_grid_tick', self.sim_time),
                self.sim_time)
            if hasattr(self.motion_coordinator, 'rollback_robot_plan'):
                self.motion_coordinator.rollback_robot_plan(robot_id)
            return False
        if delay is None:
            delay = self.motion_coordinator.get_dispatch_delay(robot_id)
        activating_pending = bool(
            delay <= 0 and robot.pending_waypoints is not None and
            robot.pending_plan_source == source)
        if activating_pending:
            if robot.pending_route_generation != robot.route_write_generation:
                robot.pending_waypoints = None
                robot.pending_plan_source = None
                robot.pending_plan_epoch = None
                robot.pending_route_generation = None
                robot.pending_is_runtime_replan = False
                robot.pending_previous_waypoints = None
                robot.pending_previous_index = 0
                robot.dispatch_not_before = 0.0
                return False
        if not self._route_write_allowed(robot, source):
            return False
        else:
            robot.route_write_generation += 1
            robot.route_write_owner = source
        plan_epoch = (robot.pending_plan_epoch
                      if delay <= 0 and robot.pending_plan_epoch is not None
                      else self._next_plan_epoch)
        if plan_epoch == self._next_plan_epoch:
            self._next_plan_epoch += 1
        plan = [tuple(wp) for wp in waypoints]
        robot.path_version += 1
        if delay > 0:
            robot.pending_waypoints = plan
            robot.pending_plan_source = source
            robot.pending_plan_epoch = plan_epoch
            robot.pending_route_generation = robot.route_write_generation
            robot.pending_is_runtime_replan = is_runtime_replan
            robot.dispatch_not_before = self.sim_time + delay
            self._last_command_send_ok = self._send_command_to_robot(robot_id, {
                'type': 'hold', 'until': robot.dispatch_not_before,
                'path_version': robot.path_version,
            })
            if not self._last_command_send_ok:
                robot.pending_waypoints = None
                robot.pending_plan_source = None
                robot.pending_plan_epoch = None
                robot.pending_route_generation = None
                robot.pending_is_runtime_replan = False
                robot.pending_previous_waypoints = None
                robot.pending_previous_index = 0
                robot.dispatch_not_before = 0.0
                self.motion_coordinator.rollback_robot_plan(robot_id)
            else:
                robot.route_write_owner = source
                robot.route_write_until = robot.dispatch_not_before + 1.0
            # A hold only prepares a delayed switch; the controller still
            # owns its previous active route. Keep the rollback snapshot
            # until _dispatch_delayed_robots sends the actual navigate.
            return self._last_command_send_ok
        was_pending = robot.pending_waypoints is not None
        previous_waypoints = robot.pending_previous_waypoints
        previous_index = robot.pending_previous_index
        robot.dispatch_not_before = 0.0
        self._last_command_send_ok = self._send_command_to_robot(robot_id, {
            'type': 'navigate',
            'target': list(plan[0]),
            'all_waypoints': [list(wp) for wp in plan],
            'path_version': robot.path_version,
        })
        if self._last_command_send_ok:
            self._last_plan_dispatch_performed = True
            robot.active_plan_epoch = plan_epoch
            robot.active_plan_source = source
            robot.route_write_owner = source
            robot.route_write_until = max(
                self.sim_time + (3.0 if source == '_command_reverse' else 1.0),
                robot.dispatch_not_before + 1.0)
            robot.pending_waypoints = None
            robot.pending_plan_source = None
            robot.pending_plan_epoch = None
            robot.pending_route_generation = None
            robot.pending_is_runtime_replan = False
            robot.pending_previous_waypoints = None
            robot.pending_previous_index = 0
            if getattr(self, 'metrics', None) is not None:
                self.metrics.record_route_dispatch(
                    robot_id, robot.path_version, plan_epoch, source, len(plan),
                    self.sim_time)
            self.motion_coordinator.commit_robot_plan(robot_id)
            if is_runtime_replan:
                self.motion_coordinator.record_runtime_replan()
                if hasattr(getattr(self, 'metrics', None), 'record_replan'):
                    self.metrics.record_replan(
                        self.sim_time, robot_id, source,
                        robot.active_plan_epoch, 'succeeded', True)
                print(f"[REPLAN_DISPATCH] robot={robot_id} source={source} "
                      f"epoch={robot.active_plan_epoch} "
                      f"path_version={robot.path_version}")
        else:
            if was_pending and previous_waypoints is not None:
                robot.waypoints = list(previous_waypoints)
                robot.current_waypoint_idx = previous_index
            robot.pending_waypoints = None
            robot.pending_plan_source = None
            robot.pending_plan_epoch = None
            robot.pending_route_generation = None
            robot.pending_is_runtime_replan = False
            robot.pending_previous_waypoints = None
            robot.pending_previous_index = 0
            self.motion_coordinator.rollback_robot_plan(robot_id)
        return self._last_command_send_ok

    def _hold_robot(self, robot_id: int, duration: float):
        """Pause without replacing the controller's active path."""
        robot = self.robots[robot_id]
        robot.hold_until = max(robot.hold_until, self.sim_time + duration)
        if not robot.wait_started:
            robot.wait_started = self.sim_time
        robot.yield_count += 1
        self._send_command_to_robot(robot_id, {
            'type': 'hold', 'until': robot.hold_until,
            'path_version': robot.path_version,
        })

    def _install_runtime_plan(self, robot_id: int, waypoints,
                              delay: Optional[float] = None,
                              source: Optional[str] = None) -> bool:
        """Version, dispatch and transactionally install a runtime route.

        Business-leg transitions and planned relocations are route installs,
        not replans.  Every other replacement of an active route remains part
        of the strict zero-replan audit.
        """
        self._last_plan_dispatch_performed = False
        if not waypoints:
            self.motion_coordinator.rollback_robot_plan(robot_id)
            return False
        robot = self.robots[robot_id]
        if source is None:
            frame = inspect.currentframe()
            source = (frame.f_back.f_code.co_name
                      if frame is not None and frame.f_back is not None
                      else 'unknown')
        normal_route_sources = {
            '_handle_goal_reached', '_send_to_home',
            '_relocate_idle_robots', '_send_to_charging',
            'joint_planner',
        }
        is_runtime_replan = source not in normal_route_sources
        if not self._route_write_allowed(robot, source):
            self.motion_coordinator.rollback_robot_plan(robot_id)
            return False
        candidate = [tuple(wp) for wp in waypoints]
        remaining = [tuple(wp) for wp in
                     robot.waypoints[robot.current_waypoint_idx:]]
        if (robot.route_write_owner == source and
                self.sim_time < robot.route_write_until and
                candidate == remaining):
            self.motion_coordinator.rollback_robot_plan(robot_id)
            return True
        previous_waypoints = list(robot.waypoints)
        previous_index = robot.current_waypoint_idx
        # Keep a rollback snapshot even when delay=None: _dispatch_plan may
        # resolve an implicit positive delay from the coordinator.
        robot.pending_previous_waypoints = list(previous_waypoints)
        robot.pending_previous_index = previous_index
        robot.waypoints = candidate
        robot.current_waypoint_idx = 0
        self._dispatch_plan(
            robot_id, candidate, delay=delay, source=source,
            is_runtime_replan=is_runtime_replan)
        accepted = bool(self._last_command_send_ok)
        if not accepted:
            robot.waypoints = previous_waypoints
            robot.current_waypoint_idx = previous_index
        return accepted

    def _broadcast_peer_positions(self):
        """Send all robot positions to each robot for peer conflict avoidance."""
        if not self.emitter:
            return
        # Build versioned samples while retaining a legacy "positions" field.
        all_positions = {}
        all_samples = {}
        for rid, robot in self.robots.items():
            if robot.position and robot.state != RobotState.CHARGING:
                all_positions[rid] = list(robot.position[:2])
                all_samples[rid] = {
                    'position': list(robot.position[:2]),
                    'velocity': list(robot.velocity),
                    'sample_time': robot.sample_time,
                    'seq': robot.state_seq,
                    'path_version': robot.path_version,
                }
        
        # Send to each robot (excluding itself from peer list)
        for rid in self.robots:
            peer_positions = {
                str(other_id): pos for other_id, pos in all_positions.items()
                if other_id != rid and other_id in all_positions
            }
            if peer_positions:
                try:
                    msg = json.dumps({
                        'target_robot': rid,
                        'command': {
                            'type': 'peer_positions',
                            'positions': peer_positions,
                            'samples': {
                                str(other_id): sample
                                for other_id, sample in all_samples.items()
                                if other_id != rid
                            },
                            'sample_time': self.sim_time,
                        }
                    }, separators=(',', ':'))
                    self.emitter.send(msg.encode('utf-8'))
                except Exception:
                    pass

    def _receive_messages(self):
        """Process incoming messages from robots."""
        if not self.receiver:
            return
        
        while self.receiver.getQueueLength() > 0:
            try:
                data = self.receiver.getString()
                msg = json.loads(data)
                robot_id = msg.get('robot_id')
                if msg.get('type') == 'ROBOT_READY':
                    self._on_robot_ready(msg)
                elif msg.get('type') == 'PLAN_PREPARED':
                    txn = self._joint_plan_transaction
                    if (txn is not None and
                            int(msg.get('plan_epoch', -1)) == txn.epoch):
                        txn.acknowledge(
                            int(msg.get('robot_id', -1)),
                            msg.get('accepted') is True)
                        self._advance_joint_plan_transaction()
                elif msg.get('type') == 'PLAN_ARMED':
                    txn = self._joint_plan_transaction
                    if (txn is not None and
                            int(msg.get('plan_epoch', -1)) == txn.epoch and
                            msg.get('armed') is True):
                        txn.acknowledge_armed(
                            int(msg.get('robot_id', -1)))
                        self._advance_joint_plan_transaction()
                elif msg.get('type') == 'PLAN_ACTIVATED':
                    txn = self._joint_plan_transaction
                    if (txn is not None and
                            int(msg.get('plan_epoch', -1)) == txn.epoch):
                        txn.acknowledge_activated(
                            int(msg.get('robot_id', -1)))
                        self._advance_joint_plan_transaction()
                elif msg.get('type') == 'PLAN_COMMITTED':
                    txn = self._joint_plan_transaction
                    if (txn is not None and
                            int(msg.get('plan_epoch', -1)) == txn.epoch):
                        txn.acknowledge_committed(
                            int(msg.get('robot_id', -1)))
                        self._advance_joint_plan_transaction()
                elif msg.get('type') == 'PLAN_ABORTED':
                    txn = self._joint_plan_transaction
                    if (txn is not None and
                            int(msg.get('plan_epoch', -1)) == txn.epoch):
                        txn.acknowledge_aborted(
                            int(msg.get('robot_id', -1)))
                        rid = int(msg.get('robot_id', -1))
                        if rid in self.robots and 'path_version' in msg:
                            version = int(msg['path_version'])
                            self.robots[rid].path_version = max(
                                self.robots[rid].path_version, version)
                            self.robots[rid].controller_path_version = max(
                                self.robots[rid].controller_path_version,
                                version)
                
                if robot_id and robot_id in self.robots:
                    # Update robot state from its report.
                    # Position / heading / battery: keep "in msg" because
                    # any incoming value is a valid update.
                    if 'position' in msg:
                        self.robots[robot_id].position = tuple(msg['position'])
                    if 'heading' in msg:
                        self.robots[robot_id].heading = msg['heading']
                    if 'battery' in msg:
                        # Supervisor owns the charging model.  A controller's
                        # telemetry is authoritative while driving, but must
                        # not overwrite the supervisor's increasing charge
                        # value while the robot is docked at a station.
                        robot = self.robots[robot_id]
                        if robot.state not in (RobotState.IDLE,
                                               RobotState.WAITING,
                                               RobotState.RETURNING_TO_CHARGE,
                                               RobotState.CHARGING):
                            robot.battery = float(msg['battery'])
                    if 'emergency_braking' in msg:
                        robot = self.robots[robot_id]
                        emergency = msg.get('emergency_braking') is True
                        if (emergency and self.sim_time >=
                                robot.emergency_recovery_until):
                            robot.emergency_braking = True
                            if robot.emergency_braking_since is None:
                                robot.emergency_braking_since = self.sim_time
                        elif not emergency:
                            robot.emergency_braking = False
                            robot.emergency_braking_since = None
                    if 'active_path_version' in msg:
                        robot = self.robots[robot_id]
                        robot.controller_path_version = int(
                            msg['active_path_version'])
                        robot.controller_waypoint_index = int(
                            msg.get('active_waypoint_index', 0))
                        robot.controller_paused_until = float(
                            msg.get('paused_until', 0.0))
                        wait_version_matches = int(msg.get(
                            'active_path_version', -1)) == robot.path_version
                        wait_epoch_matches = (
                            robot.active_plan_source ==
                            'joint_grid_transaction' and
                            int(msg.get('active_plan_epoch', -1)) ==
                            robot.active_plan_epoch)
                        if wait_version_matches and wait_epoch_matches:
                            (robot.controller_joint_wait_until,
                             robot.controller_joint_wait_reason) = (
                                self._validated_joint_wait(robot, msg))
                        else:
                            robot.controller_joint_wait_until = 0.0
                            robot.controller_joint_wait_reason = None
                        robot.controller_active_plan_epoch = int(
                            msg.get('active_plan_epoch', 0))
                        # In full-prefix joint mode, trigger the next rolling
                        # window when this robot is actually waiting at its
                        # committed endpoint, not merely after the first cell.
                        if (ENABLE_JOINT_RUNTIME and
                                robot.controller_joint_wait_reason in (
                                    'joint_window_endpoint',
                                    'joint_epoch_barrier')):
                            self._next_joint_grid_tick = min(
                                getattr(self, '_next_joint_grid_tick',
                                        self.sim_time),
                                self.sim_time + 0.1)
                    if 'speed_scale' in msg:
                        reported_scale = max(
                            0.4, min(1.0, float(msg['speed_scale'])))
                        if getattr(robot, 'joint_shield_until', 0.0) <= self.sim_time:
                            robot.speed_scale = reported_scale
                    
                    # ⚠️ FLAG fields: must check the VALUE is True, not
                    # just that the key exists. Robot status reports
                    # ALWAYS include 'reached_goal' (with value False
                    # most of the time), so 'reached_goal' in msg is
                    # always True. Using msg.get(...) ensures we only
                    # fire the handler when the robot actually arrived.
                    if msg.get('reached_waypoint') is True:
                        self._handle_waypoint_reached(robot_id)
                    if msg.get('reached_goal') is True:
                        # Physical proximity is the authoritative arrival
                        # signal. A fresh rolling joint transaction can
                        # advance path_version/epoch between the controller
                        # reaching a business goal and this status packet
                        # being consumed, so version equality must not drop a
                        # real arrival. _handle_goal_reached independently
                        # rejects far-away stale reports.
                        self._handle_goal_reached(robot_id)
                    if msg.get('replan_requested') is True:
                        self.robots[robot_id]._replan_requested = True
                        if getattr(self, 'metrics', None) is not None:
                            robot = self.robots[robot_id]
                            self.metrics.record_replan_request(
                                self.sim_time, robot_id, robot.path_version,
                                robot.active_plan_epoch)
                        print(f"[Supervisor] Replan request received from Robot {robot_id}")
                
                self.receiver.nextPacket()
            except Exception as e:
                print(f"[Supervisor] Receive error: {e}")
                try:
                    self.receiver.nextPacket()
                except:
                    break

    def _validated_joint_wait(self, robot: RobotInfo, msg: dict):
        """Accept only a bounded wait belonging to the active transaction."""
        raw_reason = msg.get('planned_wait_reason')
        raw_until = msg.get('planned_wait_until', 0.0)
        try:
            wait_until = float(raw_until)
        except (TypeError, ValueError):
            wait_until = float('nan')
        if raw_reason is None and math.isfinite(wait_until) and (
                wait_until <= self.sim_time):
            return 0.0, None

        allowed_reasons = {
            'joint_slot_deadline', 'joint_epoch_barrier',
            'joint_window_endpoint',
        }
        authoritative_deadline = robot.active_joint_wait_deadline
        authoritative_start = robot.active_joint_started_at
        protocol_valid = (
            robot.active_plan_source == 'joint_grid_transaction' and
            math.isfinite(authoritative_start) and
            authoritative_start > 0.0 and
            raw_reason in allowed_reasons and
            math.isfinite(authoritative_deadline) and
            authoritative_deadline > 0.0 and
            math.isfinite(wait_until) and
            authoritative_start <= wait_until <=
            authoritative_deadline + 1e-6)
        if protocol_valid and wait_until <= self.sim_time:
            # Status packets can cross their deadline in transit. They grant
            # no current exemption and are not a protocol violation.
            return 0.0, None
        if protocol_valid:
            return min(wait_until, authoritative_deadline), raw_reason

        print(f"[INVALID_PLANNED_WAIT] robot={robot.robot_id} "
              f"epoch={robot.active_plan_epoch} reason={raw_reason} "
              f"until={raw_until} authoritative_deadline="
              f"{authoritative_deadline} authoritative_start="
              f"{authoritative_start}")
        if getattr(self, 'metrics', None) is not None:
            self.metrics.record_safety_event(SafetyEvent(
                event_id=(f"invalid_wait-{robot.robot_id}-"
                          f"{self.sim_time:.3f}"),
                sim_time=self.sim_time,
                event_type='invalid_planned_wait',
                robot_id=robot.robot_id,
                path_version=robot.path_version,
                decision='rejected'))
        return 0.0, None

    def _on_robot_ready(self, payload):
        """Validate a robot handshake against the current scenario set."""
        robot_id = payload.get("robot_id")
        gate_result = self.startup_gate.mark_ready(
            robot_id, payload.get("robot_name", ""),
            payload.get("ready") is True,
            payload.get("optional_error", ""))
        if gate_result == "unknown":
            print(f"[STARTUP] Ignoring READY from unknown robot {robot_id}")
            return
        if gate_result == "duplicate":
            self.duplicate_ready_robot_ids.add(robot_id)
            return
        if robot_id not in self.expected_robot_ids:
            print(f"[STARTUP] Ignoring READY from unknown robot {robot_id}")
            return
        if payload.get("ready") is not True:
            self.robot_initialization_errors[robot_id] = payload.get(
                "optional_error", "robot_reported_error")
            print(f"[STARTUP_ERROR] robot=robot_{robot_id} "
                  f"error={self.robot_initialization_errors[robot_id]}")
            return
        if robot_id in self.ready_robot_ids:
            self.duplicate_ready_robot_ids.add(robot_id)
            return
        expected_name = f"robot_{robot_id}"
        if payload.get("robot_name") != expected_name:
            self.robot_initialization_errors[robot_id] = "robot_name_mismatch"
            print(f"[STARTUP_ERROR] robot={robot_id} "
                  f"expected_name={expected_name} "
                  f"actual_name={payload.get('robot_name')}")
            return
        self.ready_robot_ids.add(robot_id)
        self.last_robot_ready_time = self.sim_time
        print(f"[ROBOT_READY] robot={expected_name} "
              f"ready={len(self.ready_robot_ids)}/{len(self.expected_robot_ids)}")
        if self.ready_robot_ids == self.expected_robot_ids:
            self._on_all_robots_ready()

    def _on_all_robots_ready(self):
        """Idempotent transition into SYSTEM_READY."""
        if self.system_ready:
            return
        if self.robot_initialization_errors:
            return
        self.system_ready = True
        print(f"[SYSTEM_READY] all robots initialized "
              f"scenario={self.scenario_name} count={len(self.ready_robot_ids)}")

    def _check_startup_timeout(self):
        if self.system_ready or self.startup_timeout_logged:
            return
        timeout = float(STARTUP_CONFIG["robot_ready_timeout_seconds"])
        if self.sim_time < timeout:
            return
        self.startup_timeout_logged = True
        missing = sorted(self.expected_robot_ids - self.ready_robot_ids)
        print(f"[STARTUP_TIMEOUT] scenario={self.scenario_name} "
              f"expected={sorted(self.expected_robot_ids)} "
              f"ready={sorted(self.ready_robot_ids)} missing={missing} "
              f"errors={self.robot_initialization_errors} "
              "dispatch_started=false")
        self.startup_failed = True

    @staticmethod
    def _trajectory_conflicts(trajectories, minimum_distance=0.75,
                              only_robot_ids=None):
        """Return the first predicted conflict for every relevant pair."""
        ids = sorted(trajectories)
        selected = set(only_robot_ids) if only_robot_ids is not None else None
        conflicts = []
        for index, rid_a in enumerate(ids):
            for rid_b in ids[index + 1:]:
                if selected is not None and not ({rid_a, rid_b} & selected):
                    continue
                for sample_a, sample_b in zip(
                        trajectories[rid_a], trajectories[rid_b]):
                    distance = math.hypot(
                        sample_a[0] - sample_b[0],
                        sample_a[1] - sample_b[1])
                    if distance < minimum_distance:
                        conflicts.append(
                            (rid_a, rid_b, sample_a[2], distance))
                        break
        return conflicts

    @staticmethod
    def _conflict_components(conflicts):
        """Build connected robot groups from pairwise conflicts."""
        adjacency = {}
        for rid_a, rid_b, *_ in conflicts:
            adjacency.setdefault(rid_a, set()).add(rid_b)
            adjacency.setdefault(rid_b, set()).add(rid_a)
        components = []
        unseen = set(adjacency)
        while unseen:
            stack = [min(unseen)]
            component = set()
            while stack:
                rid = stack.pop()
                if rid in component:
                    continue
                component.add(rid)
                stack.extend(adjacency.get(rid, ()))
            unseen.difference_update(component)
            components.append(tuple(sorted(component)))
        return components

    def _trajectory_for_scale(self, robot_id, scale, horizon=10.0, dt=0.5):
        """Predict one robot from measured state using a candidate speed."""
        robot = self.robots[robot_id]
        start_delay = max(
            0.0,
            robot.dispatch_not_before - self.sim_time,
            robot.hold_until - self.sim_time,
            robot.controller_paused_until - self.sim_time,
            getattr(robot, 'controller_joint_wait_until', 0.0) - self.sim_time,
        )
        if robot.emergency_braking:
            start_delay = horizon
        return self._predict_trajectory(
            robot.position,
            robot.waypoints[robot.current_waypoint_idx:],
            0.22 * scale, horizon, dt, start_delay=start_delay)

    def _find_joint_speed_profile(self, component, base_trajectories,
                                  minimum_distance=0.75):
        """Find a safe, efficient group profile and recheck external peers."""
        component = tuple(sorted(component))
        # Preventive shaping must not approximate a stop. More severe
        # conflicts are handed to route replanning instead of degrading a
        # whole conflict group below 85% nominal speed.
        levels = (1.0, 0.85)
        cache = {}

        def trajectory(rid, scale):
            key = (rid, scale)
            if key not in cache:
                cache[key] = self._trajectory_for_scale(rid, scale)
            return cache[key]

        profiles = [{
            rid: self.robots[rid].speed_scale for rid in component
        }]
        for winner in component:
            for slow_level in levels[1:]:
                profiles.append({
                    rid: (1.0 if rid == winner else slow_level)
                    for rid in component
                })
        for offset in range(len(component)):
            profiles.append({
                rid: levels[min((rank + offset) % len(component),
                                len(levels) - 1)]
                for rank, rid in enumerate(component)
            })

        safe = []
        for profile in profiles:
            candidate = dict(base_trajectories)
            for rid, scale in profile.items():
                candidate[rid] = trajectory(rid, scale)
            conflicts = self._trajectory_conflicts(
                candidate, minimum_distance, only_robot_ids=component)
            if not conflicts:
                efficiency = sum(profile.values())
                changes = sum(
                    abs(profile[rid] - self.robots[rid].speed_scale)
                    for rid in component)
                safe.append((efficiency, -changes, profile))
        if not safe:
            return None
        safe.sort(key=lambda item: (item[0], item[1]), reverse=True)
        return safe[0][2]

    def _apply_joint_speed_profile(self, profile) -> bool:
        """Send a speed group as one rollback-capable transaction."""
        originals = {rid: self.robots[rid].speed_scale for rid in profile}
        changed = []
        for rid in sorted(profile):
            target = profile[rid]
            if abs(originals[rid] - target) < 0.01:
                continue
            if not self._set_robot_speed_scale(rid, target):
                for changed_rid in reversed(changed):
                    self._set_robot_speed_scale(
                        changed_rid, originals[changed_rid])
                return False
            changed.append(rid)
        return True

    def _proactive_path_conflict_scan(self):
        """
        轨迹预测 + 时空冲突检测�?
        
        �?0�?~1s)执行一次：
          1. 对每个活跃机器人，沿其waypoints预测未来10秒轨�?
          2. 对每对机器人，比对同一时刻的预测位�?
          3. 若距�?< 碰撞半径 �?低优先级机器�?ID�?重规�?
        
        这自然覆盖所有碰撞方向（正面、侧面、垂直交叉）�?
        """
        PREDICTION_HORIZON = 10.0   # �?�?预测未来10�?
        SAMPLE_DT = 0.5             # �?�?�?.5s采样一个位�?
        ROBOT_SPEED = 0.22          # m/s
        # Predict the controller's first slowdown boundary, not physical
        # overlap. This leaves planning time before the 0.65 m hard stop.
        COLLISION_RADIUS = 0.75
        REPLAN_COOLDOWN = 4.0
        DUPLICATE_HOLD = 3.0
        scan_deadline = real_time.perf_counter() + PROACTIVE_SCAN_BUDGET_SECONDS
        
        # ── Step 1: 预测所有活跃机器人的未来轨�?──
        trajectories = {}  # {robot_id: [(x, y, t), ...]}
        
        for rid, robot in self.robots.items():
            if robot.state not in (
                    RobotState.EN_ROUTE_PICKUP,
                    RobotState.CARRYING,
                    RobotState.EN_ROUTE_DELIVERY,
                    RobotState.RETURNING_HOME,
                    RobotState.RETURNING_TO_CHARGE):
                continue
            if not robot.waypoints:
                continue
            if robot.current_waypoint_idx >= len(robot.waypoints):
                continue
            start_delay = max(
                0.0,
                getattr(robot, 'dispatch_not_before', 0.0) - self.sim_time,
                getattr(robot, 'hold_until', 0.0) - self.sim_time,
                getattr(robot, 'controller_paused_until', 0.0) - self.sim_time,
            )
            if getattr(robot, 'emergency_braking', False):
                start_delay = PREDICTION_HORIZON
            traj = self._predict_trajectory(
                robot.position,
                robot.waypoints[robot.current_waypoint_idx:],
                ROBOT_SPEED * robot.speed_scale,
                PREDICTION_HORIZON, SAMPLE_DT,
                start_delay=start_delay)
            if traj:
                trajectories[rid] = traj
        
        if len(trajectories) < 2:
            if getattr(self, 'metrics', None) is not None:
                self.metrics.record_conflict_scan([], self.sim_time)
            for rid, robot in self.robots.items():
                if robot.speed_scale < 0.99:
                    self._set_robot_speed_scale(rid, 1.0)
            return  # 不到2个机器人在移动，无冲突可�?
        
        # 日志: 显示当前活跃轨迹�?
        if self.sim_time >= getattr(self, '_next_scan_log', 0.0):
            print(f"[Conflict] T={self.sim_time:.0f}s trajectory scan: "
                  f"{len(trajectories)} active robots, "
                  f"IDs={list(trajectories.keys())}")
            self._next_scan_log = self.sim_time + 30.0
        
        # ── Step 2: 两两比对，检测时空冲�?──
        robot_ids = list(trajectories.keys())
        raw_conflicts = self._trajectory_conflicts(
            trajectories, COLLISION_RADIUS)
        if getattr(self, 'metrics', None) is not None:
            self.metrics.record_conflict_scan(raw_conflicts, self.sim_time)
        conflicts = [
            (max(rid_a, rid_b), min(rid_a, rid_b), conflict_time)
            for rid_a, rid_b, conflict_time, _ in raw_conflicts
        ]

        # Jointly shape every connected conflict group. A candidate is
        # accepted only after rebuilding the changed trajectories and checking
        # them against both group members and all external active robots.
        jointly_resolved = set()
        speed_managed = set()
        for component in (
                self._conflict_components(raw_conflicts)
                if ENABLE_PROACTIVE_JOINT_SPEED else ()):
            # Pair conflicts have a deterministic lightweight yielder below.
            # Joint search is reserved for connected chains where independent
            # pair decisions can assign contradictory roles.
            if len(component) < 3:
                continue
            if not hasattr(self, '_joint_component_next_attempt'):
                self._joint_component_next_attempt = {}
            component_key = tuple(component)
            if self.sim_time < self._joint_component_next_attempt.get(
                    component_key, 0.0):
                continue
            self._joint_component_next_attempt[component_key] = (
                self.sim_time + 0.75)
            self.motion_coordinator.joint_plans_attempted += 1
            profile = self._find_joint_speed_profile(
                component, trajectories, COLLISION_RADIUS)
            if profile is None:
                continue
            originals = {
                rid: self.robots[rid].speed_scale for rid in component
            }
            if not self._apply_joint_speed_profile(profile):
                self.motion_coordinator.joint_plan_rollbacks += 1
                continue
            for rid, scale in profile.items():
                trajectories[rid] = self._trajectory_for_scale(
                    rid, scale, PREDICTION_HORIZON, SAMPLE_DT)
            post_conflicts = self._trajectory_conflicts(
                trajectories, COLLISION_RADIUS,
                only_robot_ids=component)
            if post_conflicts:
                self._apply_joint_speed_profile(originals)
                self.motion_coordinator.joint_plan_rollbacks += 1
                for rid, scale in originals.items():
                    trajectories[rid] = self._trajectory_for_scale(
                        rid, scale, PREDICTION_HORIZON, SAMPLE_DT)
                continue
            jointly_resolved.update(component)
            self.motion_coordinator.joint_plans_accepted += 1
            speed_managed.update(component)
            for rid in component:
                if self.robots[rid].joint_speed_started <= 0.0:
                    self.robots[rid].joint_speed_started = self.sim_time
                self.robots[rid].joint_speed_until = max(
                    self.robots[rid].joint_speed_until,
                    self.sim_time + 3.0)
            print(f"[JointPlan] robots={list(component)} "
                  f"speed_profile={profile} revalidated=true")
        
        # ── Step 3: 重规划冲突机器人 ──
        replanned = set()
        for rid_yield, rid_stay, t_conflict in conflicts:
            # This code runs synchronously with Webots. Limit work per scan so
            # a dense conflict set cannot prevent the next simulation step.
            if (len(replanned) >= PROACTIVE_SCAN_MAX_REPLANS or
                    real_time.perf_counter() >= scan_deadline):
                break
            if rid_yield in replanned:
                continue  # 已经重规划过�?
            if rid_yield in jointly_resolved and rid_stay in jointly_resolved:
                continue
            # Beyond six seconds, keep observing. Inside six seconds first
            # try a genuinely different global route; this is early enough
            # to use another aisle without emergency manoeuvres.
            if t_conflict > 6.0:
                continue
            robot = self.robots[rid_yield]
            if robot.pending_waypoints is not None:
                # A delayed candidate owns the only rollback snapshot. Do not
                # stack another proposal before it activates or rolls back.
                continue
            if self.sim_time < getattr(
                    robot, '_proactive_replan_cooldown_until', 0.0):
                continue
            goal = self._navigation_goal(robot)
            if not goal:
                continue

            old_remaining = list(
                robot.waypoints[robot.current_waypoint_idx:])
            new_path = self.motion_coordinator.plan_grid_lifelong(
                rid_yield, robot.position, goal)
            if new_path and len(new_path) > 0:
                same_path = self._paths_equivalent(
                    old_remaining, new_path, tolerance=0.15)
                robot._proactive_replan_cooldown_until = (
                    self.sim_time + REPLAN_COOLDOWN)
                if same_path:
                    # Re-sending an identical route every scan resets the
                    # local navigator without resolving the collision. Hold
                    # briefly, retain the authoritative route, and let the
                    # independent 10-second watchdog relocate/replan if no
                    # physical progress follows.
                    self.motion_coordinator.rollback_robot_plan(rid_yield)
                    duplicate_count = getattr(
                        robot, '_duplicate_conflict_replans', 0) + 1
                    robot._duplicate_conflict_replans = duplicate_count
                    if t_conflict > 2.0:
                        # No geometric alternative is needed yet: negotiate
                        # arrival time while both robots still have room.
                        self._set_robot_speed_scale(rid_stay, 1.0)
                        self._set_robot_speed_scale(rid_yield, 0.6)
                        speed_managed.update((rid_yield, rid_stay))
                        replanned.add(rid_yield)
                        continue
                    escaped = self._command_reverse(rid_yield, 0.3)
                    if escaped:
                        self._log_replan(
                            f"[Replan] Robot {rid_yield} predicted collision "
                            f"with Robot {rid_stay} in {t_conflict:.1f}s; "
                            "same route rejected, validated moving escape "
                            f"installed (repeat={duplicate_count})")
                    else:
                        # Never create a stationary obstacle in an arbitrary
                        # corridor position. Keep the robot moving slowly and
                        # rescan; local safety remains the final guard.
                        self._set_robot_speed_scale(rid_yield, 0.4)
                        speed_managed.add(rid_yield)
                        self._log_replan(
                            f"[Replan] Robot {rid_yield} same route and no "
                            "escape candidate; continuous low-speed motion "
                            f"selected (repeat={duplicate_count})")
                    replanned.add(rid_yield)
                    continue

                robot._duplicate_conflict_replans = 0
                self._install_runtime_plan(rid_yield, new_path)
                replanned.add(rid_yield)
                self._log_replan(f"[Replan] Robot {rid_yield} predicted collision with Robot {rid_stay} "
                      f"in {t_conflict:.1f}s; active replan: "
                      f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f}) "
                      f"�?goal={goal}, new path={len(new_path)} steps")
            else:
                robot._proactive_replan_cooldown_until = (
                    self.sim_time + REPLAN_COOLDOWN)
                if t_conflict > 2.0:
                    self._set_robot_speed_scale(rid_stay, 1.0)
                    self._set_robot_speed_scale(rid_yield, 0.6)
                    speed_managed.update((rid_yield, rid_stay))
                    replanned.add(rid_yield)
                    continue
                escaped = self._command_reverse(rid_yield, 0.3)
                if escaped:
                    self._log_replan(
                        f"[Replan] Robot {rid_yield} predicted collision with "
                        f"Robot {rid_stay}; no route candidate, validated "
                        "moving escape installed")
                else:
                    self._set_robot_speed_scale(rid_yield, 0.4)
                    speed_managed.add(rid_yield)
                    self._log_replan(
                        f"[Replan] Robot {rid_yield} predicted collision with "
                        f"Robot {rid_stay}; no route or escape candidate, "
                        "continuous low-speed motion selected")
                replanned.add(rid_yield)

        # Restore nominal speed once a robot is no longer part of a predicted
        # conflict. This makes speed shaping local to the rolling horizon.
        for rid, robot in self.robots.items():
            if rid not in speed_managed and robot.speed_scale < 0.99:
                if robot.joint_speed_until <= 0.0:
                    self._set_robot_speed_scale(rid, 1.0)
                    continue
                if (robot.joint_speed_started > 0.0 and
                        self.sim_time - robot.joint_speed_started >= 8.0):
                    if self._set_robot_speed_scale(rid, 1.0):
                        robot.joint_speed_until = 0.0
                        robot.joint_speed_started = 0.0
                    continue
                if self.sim_time < robot.joint_speed_until:
                    continue
                nominal = dict(trajectories)
                nominal[rid] = self._trajectory_for_scale(
                    rid, 1.0, PREDICTION_HORIZON, SAMPLE_DT)
                if self._trajectory_conflicts(
                        nominal, COLLISION_RADIUS, only_robot_ids={rid}):
                    robot.joint_speed_until = self.sim_time + 1.0
                    continue
                if self._set_robot_speed_scale(rid, 1.0):
                    trajectories[rid] = nominal[rid]
                    robot.joint_speed_until = 0.0
                    robot.joint_speed_started = 0.0

    @staticmethod
    def _paths_equivalent(path_a, path_b, tolerance=0.15):
        """Return True when two waypoint sequences are materially identical."""
        if len(path_a) != len(path_b):
            return False
        return all(math.hypot(float(a[0]) - float(b[0]),
                              float(a[1]) - float(b[1])) <= tolerance
                   for a, b in zip(path_a, path_b))
    
    def _predict_trajectory(self, start_pos, waypoints, speed,
                            horizon, dt, start_delay=0.0):
        """
        沿waypoints匀速模拟，产出未来轨迹采样点�?
        
        Args:
            start_pos: (x, y) 当前位置
            waypoints: 剩余waypoint列表 [(x,y), ...]
            speed: 匀�?m/s
            horizon: 预测时长(�?
            dt: 采样间隔(�?
        
        Returns:
            [(x, y, t), ...] 未来轨迹�?
        """
        trajectory = []
        cx, cy = start_pos[0], start_pos[1]
        wp_idx = 0
        start_delay = max(0.0, min(float(start_delay), float(horizon)))
        t = start_delay
        next_sample = dt

        # A pending/held/braking robot remains at its measured position until
        # it is actually allowed to move. Keep global sample indices aligned
        # across robots so pairwise comparisons refer to the same time.
        while next_sample <= start_delay and next_sample <= horizon:
            trajectory.append((cx, cy, next_sample))
            next_sample += dt
        
        while t < horizon and wp_idx < len(waypoints):
            wx, wy = waypoints[wp_idx][0], waypoints[wp_idx][1]
            dx, dy = wx - cx, wy - cy
            dist_to_wp = math.sqrt(dx*dx + dy*dy)
            
            if dist_to_wp < 0.01:
                # 已到达此waypoint
                wp_idx += 1
                continue
            
            # 到达此waypoint需要的时间
            time_to_wp = dist_to_wp / speed
            
            # 在前进到此waypoint的过程中采样
            while next_sample <= t + time_to_wp and next_sample <= horizon:
                # 在t=next_sample时的位置
                frac = (next_sample - t) / time_to_wp
                px = cx + frac * dx
                py = cy + frac * dy
                trajectory.append((px, py, next_sample))
                next_sample += dt
            
            # 前进到waypoint
            t += time_to_wp
            cx, cy = wx, wy
            wp_idx += 1
        
        # 如果waypoints走完了但时间没到，机器人会停在最后位�?
        while next_sample <= horizon:
            trajectory.append((cx, cy, next_sample))
            next_sample += dt
        
        return trajectory
    
    def _monitor_progress_and_replan(self):
        """Proactive Progress Guarantee + Emergency Replan Response.
        
        Two triggers for replanning:
        1. EMERGENCY: Robot's controller set _replan_requested = True
           (Layer 0 emergency stop triggered, robot backed up and needs new path)
        2. STALL: Robot hasn't moved for 3+ seconds
           (path blocked, needs alternative route)
        
        Both cases: replan path around current peer positions.
        """
        STALL_THRESHOLD = 3.0  # seconds of no progress �?trigger replan
        MIN_PROGRESS_DIST = 0.15  # must move at least this much per check

        self._coordinate_emergency_pair()
        
        for rid, robot in self.robots.items():
            if robot.state not in (
                    RobotState.EN_ROUTE_PICKUP,
                    RobotState.CARRYING,
                    RobotState.EN_ROUTE_DELIVERY,
                    RobotState.RETURNING_HOME,
                    RobotState.RETURNING_TO_CHARGE):
                continue
            if not robot.waypoints:
                continue
            if robot.current_waypoint_idx >= len(robot.waypoints):
                continue
            if (robot.recovery_session_role == 'winner' and
                    self.sim_time < robot.recovery_session_until):
                robot._replan_requested = False
                continue
            # ── Priority 1: Emergency replan request from robot controller ──
            if getattr(robot, '_replan_requested', False):
                # Keep the request pending while it is in cooldown. An
                # emergency-braked robot is not expected to move, but this
                # request is precisely what must release it.
                # �?冷却�? 同一robot 5秒内不重复重规划
                REPLAN_COOLDOWN = 3.0
                last_replan_t = getattr(robot, '_last_replan_time', 0.0)
                if self.sim_time - last_replan_t < REPLAN_COOLDOWN:
                    continue  # 冷却�?跳过
                robot._replan_requested = False
                
                goal = self._navigation_goal(robot)
                if not goal:
                    self._log_replan(f"[Replan] Robot {rid} requested replanning but goal_location is empty; "
                          f"state={robot.state}, task={robot.current_task}")
                    continue
                
                # �?检查连续重规划次数 �?如果3次无进展,用让路策�?
                replan_count = getattr(robot, '_replan_fail_count', 0)
                replan_pos = getattr(robot, '_replan_start_pos', None)
                if replan_pos:
                    moved = math.hypot(robot.position[0] - replan_pos[0],
                                       robot.position[1] - replan_pos[1])
                    if moved < 0.3:
                        replan_count += 1
                    else:
                        replan_count = 0  # 有进�?重置计数
                else:
                    replan_count = 0
                robot._replan_fail_count = replan_count
                robot._replan_start_pos = robot.position
                
                if replan_count >= 3:
                    # �?让路策略: 找到阻挡此robot的peer,让低优先级的让路
                    self._log_replan(f"[Replan] Robot {rid} made no progress after {replan_count} replans; "
                          f"starting yield coordination")
                    robot._replan_fail_count = 0
                    
                    # 找最近的peer(可能是阻挡�?
                    closest_peer = None
                    closest_dist = float('inf')
                    for pr, prob in self.robots.items():
                        if pr == rid:
                            continue
                        if not hasattr(prob, 'position'):
                            continue
                        d = math.hypot(robot.position[0] - prob.position[0],
                                       robot.position[1] - prob.position[1])
                        if d < closest_dist:
                            closest_dist = d
                            closest_peer = pr
                    
                    if closest_peer is not None and closest_dist < 1.5:
                        if rid < closest_peer:
                            # 我优先级�?ID�? �?对方让路,我重规划(把对方当静态障�?
                            self._log_replan(f"[Replan] Robot {rid} (high priority) replanning; "
                                  f"Robot {closest_peer} (low priority) yields")
                            peer_robot = self.robots[closest_peer]
                            peer_robot._last_replan_time = self.sim_time + 3.0
                            if not self._command_reverse(closest_peer, 0.3):
                                self._set_robot_speed_scale(
                                    closest_peer, 0.4)
                            # 我自己重规划
                            new_path = self.motion_coordinator.plan_grid_lifelong(
                                rid, robot.position, goal)
                            if new_path and len(new_path) > 0:
                                self._install_runtime_plan(rid, new_path)
                                self._log_replan(f"[Replan] Robot {rid} yield-aware replan succeeded: {len(new_path)} steps")
                        else:
                            # 我优先级�?ID�? �?我让�?�?
                            self._log_replan(f"[Replan] Robot {rid} (low priority) yields for 3 s; "
                                  f"Robot {closest_peer} proceeds first")
                            robot._last_replan_time = self.sim_time + 3.0
                            if not self._command_reverse(rid, 0.3):
                                self._set_robot_speed_scale(rid, 0.4)
                    else:
                        # 没有近距离peer,可能是死�?�?�?秒再�?
                        robot._last_replan_time = self.sim_time + 3.0
                    continue
                
                # 正常重规�?
                new_path = self.motion_coordinator.plan_grid_lifelong(
                    rid, robot.position, goal)
                if new_path and len(new_path) > 0:
                    # �?重复路径检�? 如果首个waypoint和上次一�?说明没有替代�?
                    last_first_wp = getattr(robot, '_last_replan_first_wp', None)
                    same_path = False
                    if last_first_wp and len(new_path) > 0:
                        dist_to_last = math.hypot(
                            new_path[0][0] - last_first_wp[0],
                            new_path[0][1] - last_first_wp[1])
                        if dist_to_last < 0.5:
                            same_path = True
                    robot._last_replan_first_wp = new_path[0]
                    
                    if same_path:
                        # 路径没变 �?增加失败计数(由让路策略处�?
                        robot._replan_fail_count = getattr(robot, '_replan_fail_count', 0) + 1
                    
                    self._install_runtime_plan(rid, new_path)
                    self._log_replan(f"[Replan] Robot {rid} path-conflict replan: "
                          f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f}) "
                          f"�?goal={goal}, new path={len(new_path)} steps, "
                          f"avoiding peers={[r for r in self.robots.keys() if r != rid]}"
                          f"{' (duplicate path)' if same_path else ''}")
                else:
                    self._log_replan(f"[Replan] Robot {rid} replan failed (no feasible path), "
                          f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f})")
                
                robot._last_replan_time = self.sim_time
                # �?不重�?_last_progress_pos/_time! 
                # 让死锁检测器能基�?是否真的移动�?来判�?
                # 只有当robot确实移动>0.15m时才由下面的progress检测重�?
                continue

            if not self._robot_should_be_moving(robot):
                robot._last_progress_pos = robot.position
                robot._last_progress_time = self.sim_time
                continue

            # ── Priority 2: Stall detection (no progress for 3s) ──
            # Idle/waiting and charging robots are intentionally stationary;
            # never treat them as blocked or relocate them.
            if robot.state not in (
                    RobotState.EN_ROUTE_PICKUP,
                    RobotState.CARRYING,
                    RobotState.EN_ROUTE_DELIVERY,
                    RobotState.RETURNING_HOME,
                    RobotState.RETURNING_TO_CHARGE):
                robot._last_progress_pos = robot.position
                robot._last_progress_time = self.sim_time
                continue
            # Track progress: has robot moved significantly since last check?
            if not hasattr(robot, '_last_progress_pos'):
                robot._last_progress_pos = robot.position
                robot._last_progress_time = self.sim_time
                continue
            
            dist_moved = math.hypot(
                robot.position[0] - robot._last_progress_pos[0],
                robot.position[1] - robot._last_progress_pos[1])
            
            progress_threshold = MIN_PROGRESS_DIST * max(
                0.5, robot.speed_scale)
            if dist_moved > progress_threshold:
                # Making progress �?reset timer
                robot._last_progress_pos = robot.position
                robot._last_progress_time = self.sim_time
            else:
                # Not making progress �?check how long
                stall_time = self.sim_time - robot._last_progress_time
                
                if stall_time > STALL_THRESHOLD:
                    # Robot has been stalled too long �?dynamic replan
                    print(f"[Supervisor] Robot {rid} stalled for {stall_time:.1f} s; attempting replan...")
                    goal = self._navigation_goal(robot)
                    if goal:
                        new_path = self.motion_coordinator.plan_grid_lifelong(
                            rid, robot.position, goal)
                        if new_path and len(new_path) > 0:
                            remaining = robot.waypoints[
                                robot.current_waypoint_idx:]
                            if self._paths_equivalent(
                                    remaining, new_path, tolerance=0.15):
                                self.motion_coordinator.rollback_robot_plan(rid)
                                self._command_reverse(rid, 0.75)
                            else:
                                self._install_runtime_plan(rid, new_path)
                            self._log_replan(f"[Replan] Robot {rid} stalled for {stall_time:.1f} s; replanning: "
                                  f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f}) "
                                  f"�?goal={goal}, new path={len(new_path)} steps")
                    # Reset timer regardless (avoid spam replanning)
                    robot._last_progress_pos = robot.position
                    robot._last_progress_time = self.sim_time

    def _recover_long_stalled_robots(self):
        """Relocate/replan active robots with no physical progress for 5 s.

        This watchdog is deliberately independent of the optional legacy
        interlock resolver. Short replans and one-second conflict holds must
        not postpone the physical-progress deadline.
        """
        long_stalled = []
        for rid, robot in self.robots.items():
            if (robot.recovery_session_role is not None and
                    self.sim_time < robot.recovery_session_until):
                continue
            active = self._robot_requires_recovery_monitoring(robot)
            if not active:
                robot._deadlock_stuck_since = None
                robot._deadlock_watch_pos = robot.position
                continue

            watch_pos = getattr(robot, '_deadlock_watch_pos', robot.position)
            moved = math.hypot(robot.position[0] - watch_pos[0],
                               robot.position[1] - watch_pos[1])
            progress_threshold = STALL_PROGRESS_DISTANCE * max(
                0.5, robot.speed_scale)
            if moved >= progress_threshold:
                robot._deadlock_watch_pos = robot.position
                robot._deadlock_stuck_since = None
                continue

            if getattr(robot, '_deadlock_stuck_since', None) is None:
                robot._deadlock_stuck_since = self.sim_time
            if (self.sim_time - robot._deadlock_stuck_since >=
                    STALL_RELOCATION_TIMEOUT):
                long_stalled.append(rid)

        if not long_stalled:
            return False

        # Recover nearby overdue robots as one group so their landing points
        # are validated together.
        group = {long_stalled[0]}
        changed = True
        while changed:
            changed = False
            for rid in long_stalled:
                if rid in group:
                    continue
                if any(math.hypot(
                        self.robots[rid].position[0] - self.robots[other].position[0],
                        self.robots[rid].position[1] - self.robots[other].position[1]) <
                       STALL_GROUP_DISTANCE
                       for other in group):
                    group.add(rid)
                    changed = True

        self._record_long_stall_event(group)

        if not ENABLE_NONPHYSICAL_RECOVERY:
            yielder = max(group)
            recovered = self._command_reverse(yielder, 0.3)
            if recovered:
                rb = self.robots[yielder]
                rb._deadlock_watch_pos = rb.position
                rb._deadlock_stuck_since = None
                print(f"[Deadlock] Physical retreat recovery dispatched for "
                      f"Robot {yielder}; teleport recovery is disabled")
            return recovered

        if not self._teleport_stalled_group(sorted(group)):
            print(f"[Deadlock] Robots {sorted(group)} exceeded the "
                  f"{STALL_RELOCATION_TIMEOUT:g} s "
                  "no-progress deadline, but no safe relocation was found; "
                  "will retry")
            return False

        self._deadlock_state = 'IDLE'
        self._deadlock_queue = []
        self._deadlock_cooldown_until = self.sim_time + 3.0
        return True

    def _record_long_stall_event(self, robot_ids) -> bool:
        """Record one auditable deadlock event per stable group and window."""
        group = tuple(sorted(int(rid) for rid in robot_ids))
        if not group:
            return False
        key = (group, int(self.sim_time // 5.0))
        if getattr(self, '_last_long_stall_event_key', None) == key:
            return False
        self._last_long_stall_event_key = key
        self.motion_coordinator.deadlocks_detected += 1
        if getattr(self, 'metrics', None) is not None:
            self.metrics.record_deadlock(list(group), self.sim_time)
        return True

    def _resolve_multi_robot_deadlock(self):
        """
        多机互锁恢复：当>=2个机器人同时停滞时，按优先级顺序逐个恢复�?
        
        逻辑�?
          1. 检�? 停滞>1.5s的机器人>=2�?�?互锁
          2. 排序: 按ID升序(ID�?优先级高)
          3. 最高优先级: 以其他停滞peer为静态障碍重规划
          4. 若失�? 命令其后退0.3m,再重规划
          5. 等最高优先级移动�?处理下一�?
          6. 冷却: 恢复�?秒内不再触发
        """
        DEADLOCK_STALL_THRESHOLD = 1.5  # s �?停滞超过此时间算"停住"
        DEADLOCK_COOLDOWN = 2.0          # s �?恢复后冷却期
        REVERSE_DIST = 0.3               # m �?后退距离
        MOVE_CONFIRM_DIST = 0.3          # m �?确认移动的距�?

        # Independent 10-second watchdog. Ordinary 3-second replan timers are
        # reset after each attempt, so they cannot be used for this deadline.
        long_stalled = []
        for rid, robot in self.robots.items():
            active = (robot.state in (
                          RobotState.EN_ROUTE_PICKUP,
                          RobotState.CARRYING,
                          RobotState.EN_ROUTE_DELIVERY,
                          RobotState.RETURNING_HOME,
                          RobotState.RETURNING_TO_CHARGE) and
                      robot.waypoints and
                      robot.current_waypoint_idx < len(robot.waypoints))
            if not active:
                robot._deadlock_stuck_since = None
                robot._deadlock_watch_pos = robot.position
                continue
            watch_pos = getattr(robot, '_deadlock_watch_pos', robot.position)
            moved = math.hypot(robot.position[0] - watch_pos[0],
                               robot.position[1] - watch_pos[1])
            if moved >= 0.15:
                robot._deadlock_watch_pos = robot.position
                robot._deadlock_stuck_since = None
            else:
                if getattr(robot, '_deadlock_stuck_since', None) is None:
                    robot._deadlock_stuck_since = self.sim_time
                if self.sim_time - robot._deadlock_stuck_since >= 10.0:
                    long_stalled.append(rid)
        if long_stalled:
            # Build the connected blockage containing the first overdue robot.
            # An isolated robot forms a valid one-member group.
            group = {long_stalled[0]}
            changed = True
            while changed:
                changed = False
                for rid in long_stalled:
                    if rid in group:
                        continue
                    if any(math.hypot(
                            self.robots[rid].position[0] - self.robots[other].position[0],
                            self.robots[rid].position[1] - self.robots[other].position[1]) < 1.8
                           for other in group):
                        group.add(rid)
                        changed = True
            if (ENABLE_NONPHYSICAL_RECOVERY and
                    self._teleport_stalled_group(sorted(group))):
                self._deadlock_state = 'IDLE'
                self._deadlock_queue = []
                self._deadlock_cooldown_until = self.sim_time + 3.0
                return
        
        # 冷却期检�?
        if not hasattr(self, '_deadlock_cooldown_until'):
            self._deadlock_cooldown_until = 0.0
        if self.sim_time < self._deadlock_cooldown_until:
            return
        
        # 恢复流程状态机
        if not hasattr(self, '_deadlock_state'):
            self._deadlock_state = 'IDLE'  # IDLE / RESOLVING / WAIT_MOVE
            self._deadlock_queue = []       # 待恢复队�?[rid, ...]
            self._deadlock_current = None   # 当前正在恢复的机器人
            self._deadlock_start_pos = None # 恢复开始时的位�?
        
        # ── 状�? WAIT_MOVE �?等待当前机器人移动确�?──
        if self._deadlock_state == 'WAIT_MOVE':
            robot = self.robots.get(self._deadlock_current)
            if robot and self._deadlock_start_pos:
                moved = math.hypot(
                    robot.position[0] - self._deadlock_start_pos[0],
                    robot.position[1] - self._deadlock_start_pos[1])
                if moved > MOVE_CONFIRM_DIST:
                    print(f"[Deadlock] Robot {self._deadlock_current} "
                          f"moved {moved:.2f} m; recovery confirmed")
                    # 处理队列中下一�?
                    if self._deadlock_queue:
                        self._resolve_next_in_queue()
                    else:
                        self._deadlock_state = 'IDLE'
                        self._deadlock_cooldown_until = self.sim_time + DEADLOCK_COOLDOWN
                        print(f"[Deadlock] All interlocked robots recovered")
                else:
                    # 等待超时(5s)�?尝试后退
                    if not hasattr(self, '_deadlock_wait_start'):
                        self._deadlock_wait_start = self.sim_time
                    if self.sim_time - self._deadlock_wait_start > 5.0:
                        # 重规划失�?无法移动 �?命令后退
                        self._command_reverse(self._deadlock_current, REVERSE_DIST)
                        self._deadlock_wait_start = self.sim_time
            return
        
        # ── 状�? RESOLVING �?正在处理队列 ──
        if self._deadlock_state == 'RESOLVING':
            return  # �?WAIT_MOVE 完成
        
        # ── 状�? IDLE �?检测是否有多机互锁 ──
        stalled_robots = []
        for rid, robot in self.robots.items():
            if not robot.current_task or not robot.waypoints:
                continue
            if robot.current_waypoint_idx >= len(robot.waypoints):
                continue
            if not hasattr(robot, '_last_progress_pos'):
                continue
            dist_moved = math.hypot(
                robot.position[0] - robot._last_progress_pos[0],
                robot.position[1] - robot._last_progress_pos[1])
            if dist_moved < 0.10:
                stall_time = self.sim_time - getattr(robot, '_last_progress_time', self.sim_time)
                if stall_time > DEADLOCK_STALL_THRESHOLD:
                    stalled_robots.append((rid, stall_time))
        
        if len(stalled_robots) < 2:
            return  # 不构成互�?
        
        # 检测到互锁�?
        stalled_robots.sort(key=lambda x: x[0])  # 按ID升序
        stall_info = ", ".join(f"Robot {r} (stalled {t:.1f} s)" for r, t in stalled_robots)
        print(f"[Deadlock] Interlock detected among {len(stalled_robots)} robots: {stall_info}")
        
        # �?优先检测迎面对�?同走廊对�? �?低优先级后退让路
        head_on = self._detect_head_on_pair(stalled_robots)
        if head_on:
            high_rid, low_rid = head_on
            print(f"[Deadlock] Head-on conflict! Robot {high_rid} (high priority) continues; "
                  f"Robot {low_rid} (low priority) reverses to yield")
            
            low_robot = self.robots[low_rid]
            high_robot = self.robots[high_rid]
            
            # 低优先级robot: 后退1.0m(远离对方)
            goal_low = getattr(low_robot, 'goal_location', None)
            if goal_low:
                dx_between = abs(low_robot.position[0] - high_robot.position[0])
                dy_between = abs(low_robot.position[1] - high_robot.position[1])
                
                if dx_between < dy_between:
                    # 垂直走廊: 沿y方向后退
                    dy = low_robot.position[1] - high_robot.position[1]
                    retreat_dir = 1.0 if dy > 0 else -1.0
                    retreat_y = low_robot.position[1] + retreat_dir * 1.0
                    retreat_y = max(-4.5, min(4.5, retreat_y))
                    retreat_pos = (low_robot.position[0], retreat_y)
                else:
                    # 水平走廊: 沿x方向后退
                    dx = low_robot.position[0] - high_robot.position[0]
                    retreat_dir = 1.0 if dx > 0 else -1.0
                    retreat_x = low_robot.position[0] + retreat_dir * 1.0
                    retreat_x = max(-8.0, min(8.0, retreat_x))
                    retreat_pos = (retreat_x, low_robot.position[1])
                
                retreat_path = self.motion_coordinator.plan_grid_lifelong(
                    low_rid, low_robot.position, retreat_pos)
                if retreat_path:
                    self._install_runtime_plan(
                        low_rid, retreat_path, delay=0.0)
                else:
                    self._set_robot_speed_scale(low_rid, 0.4)
                print(f"[Deadlock] Robot {low_rid} reversed to "
                      f"({retreat_pos[0]:.2f},{retreat_pos[1]:.2f})")
                low_robot._last_replan_time = self.sim_time + 3.0
            
            # 高优先级robot: 立即重规�?
            goal_high = getattr(high_robot, 'goal_location', None)
            if goal_high:
                new_path = self.motion_coordinator.plan_grid_lifelong(
                    high_rid, high_robot.position, goal_high)
                if new_path:
                    self._install_runtime_plan(high_rid, new_path)
                    print(f"[Deadlock] Robot {high_rid} replan succeeded: {len(new_path)} steps")
            
            self._deadlock_state = 'IDLE'
            self._deadlock_cooldown_until = self.sim_time + 2.0
            return
        
        # �?非迎�? 按优先级顺序逐个恢复
        self._deadlock_queue = [r for r, _ in stalled_robots[1:]]
        self._deadlock_state = 'RESOLVING'
        first_rid = stalled_robots[0][0]
        self._resolve_robot_deadlock(first_rid, [r for r, _ in stalled_robots[1:]])

    def _teleport_stalled_group(self, robot_ids) -> bool:
        """Relocate stalled robots nearby, then replan from there.

        A landing point is accepted only when it is a free grid cell, keeps a
        safe distance from every peer/other landing, and admits a fresh path
        to the robot's current business goal.  All robots in the recovery
        group are validated before any Webots node is moved.
        """
        selected = {}
        other_positions = [
            robot.position for rid, robot in self.robots.items()
            if rid not in robot_ids
        ]
        for rid in sorted(robot_ids):
            robot = self.robots[rid]
            goal = self._navigation_goal(robot) or robot._get_current_goal()
            if not goal:
                return False
            if (isinstance(goal, (tuple, list)) and len(goal) >= 2):
                goal_xy = (float(goal[0]), float(goal[1]))
            else:
                goal_xy = (ALL_LOCATIONS.get(goal) or
                           CHARGING_STATIONS.get(goal) or
                           WAYPOINTS.get(goal))
            if goal_xy is None:
                return False
            remaining = list(robot.waypoints[robot.current_waypoint_idx:])
            origin = robot.position

            # Search near the current position.  Start in the direction of
            # the old route, then fan around the robot in 22.5-degree steps.
            if remaining:
                heading = math.atan2(remaining[0][1] - origin[1],
                                     remaining[0][0] - origin[0])
            else:
                heading = 0.0
            # Drop the obsolete reservation before asking the lifelong
            # planner to reserve a route from a different start position.
            self.motion_coordinator.release_robot_grid(rid)
            candidates = []
            seen_cells = set()
            # A robot can stop inside the grid's obstacle-inflation band
            # (for example while turning away from a dock).  That band is
            # wider than 1.5 m around some shelf corners, so the old search
            # never considered a valid cell and retried forever.  Search
            # progressively farther, but always relocate to the centre of a
            # free cell rather than to an arbitrary point which merely rounds
            # to that cell.
            for radius in (0.6, 0.8, 1.0, 1.2, 1.5, 1.8, 2.2, 2.6, 3.0):
                for offset_index in range(16):
                    # 0,+1,-1,+2,-2,... tries points closest to the desired
                    # travel direction before increasingly lateral points.
                    signed = ((offset_index + 1) // 2) * (
                        1 if offset_index % 2 else -1)
                    if offset_index == 0:
                        signed = 0
                    angle = heading + signed * (math.pi / 8.0)
                    sample = (
                        origin[0] + radius * math.cos(angle),
                        origin[1] + radius * math.sin(angle))
                    cell = self.motion_coordinator.grid.world_to_grid(*sample)
                    if cell in seen_cells:
                        continue
                    seen_cells.add(cell)
                    if self.motion_coordinator.grid.is_free(*cell):
                        candidates.append(
                            self.motion_coordinator.grid.grid_to_world(*cell))

            landing = None
            for point in candidates:
                occupied = other_positions + [item[0] for item in selected.values()]
                if occupied and min(math.hypot(point[0] - p[0], point[1] - p[1])
                                    for p in occupied) < RELOCATION_PEER_CLEARANCE:
                    continue

                # Establish physical reachability first.  The cooperative
                # planner can reject every candidate solely because stale
                # space-time reservations have filled its bounded horizon.
                # That is a coordination failure, not an unsafe landing.
                static_path = self.motion_coordinator.grid_planner.plan(
                    point, goal_xy, smooth=False)
                if not static_path:
                    continue
                new_path = self.motion_coordinator.plan_grid_lifelong(
                    rid, point, goal)
                used_emergency_path = False
                if not isinstance(new_path, (list, tuple)) or not new_path:
                    # Deadlock recovery must not depend on the reservations
                    # that caused the deadlock.  Preserve every grid turn so
                    # the emergency route cannot cut across a shelf corner.
                    cells = []
                    for waypoint in static_path:
                        cell = self.motion_coordinator.grid.world_to_grid(
                            *waypoint)
                        if not cells or cells[-1] != cell:
                            cells.append(cell)
                    new_path = self.motion_coordinator._cells_to_turning_waypoints(
                        cells)
                    if (new_path and
                            math.hypot(new_path[0][0] - point[0],
                                       new_path[0][1] - point[1]) < 0.05):
                        new_path = new_path[1:]
                    if not new_path:
                        continue
                    # Replace, rather than retain, the stale reservation.
                    self.motion_coordinator.release_robot_grid(rid)
                    self.motion_coordinator._grid_path_reservations[rid] = set(cells)
                    self.motion_coordinator.coordinate_paths[rid] = (
                        [point] + list(new_path))
                    used_emergency_path = True
                landing = (point, list(new_path), used_emergency_path)
                break
            if landing is None:
                return False
            selected[rid] = landing

        # Validate every landing before mutating any Webots node.
        for rid, (point, new_path, used_emergency_path) in selected.items():
            node = self.robot_nodes.get(rid)
            if node is None:
                return False
            field = node.getField("translation")
            if field is None:
                return False
        for rid, (point, new_path, used_emergency_path) in selected.items():
            robot = self.robots[rid]
            node = self.robot_nodes[rid]
            field = node.getField("translation")
            old = field.getSFVec3f()
            field.setSFVec3f([point[0], point[1], old[2]])
            if hasattr(node, "resetPhysics"):
                node.resetPhysics()
            robot.position = point
            robot._deadlock_watch_pos = point
            robot._deadlock_stuck_since = None
            robot._last_progress_pos = point
            robot._last_progress_time = self.sim_time
            robot.waypoints = list(new_path)
            robot.current_waypoint_idx = 0
            dispatched = self._dispatch_plan(
                rid, new_path, delay=0.0,
                source='_teleport_stalled_group', is_runtime_replan=True)
            if not dispatched:
                # The node has already been moved.  The old controller route
                # no longer has a valid physical origin, so fail closed
                # instead of claiming recovery or retaining split state.
                robot.waypoints = []
                robot.current_waypoint_idx = 0
                robot.pending_waypoints = None
                robot.route_write_owner = None
                self._send_command_to_robot(rid, {'type': 'stop'})
                print(f"[Deadlock] Robot {rid} relocation route dispatch "
                      "failed; robot stopped for a fresh joint plan")
                return False
            print(f"[Deadlock] Robot {rid} stuck for over "
                  f"{STALL_RELOCATION_TIMEOUT:g} s; safely relocating to "
                  f"nearby ({point[0]:.2f},{point[1]:.2f}) and replanning "
                  f"{len(new_path)} waypoints to "
                  f"{self._navigation_goal(robot) or robot._get_current_goal()}"
                  f"{' using reservation-independent emergency path' if used_emergency_path else ''}")
        return True

    def _recover_emergency_braking(self):
        """Resolve controller-reported emergency braking after ten seconds."""
        timeout = 10.0
        target_tolerance = GOAL_TOLERANCE * 2.5
        for rid, robot in self.robots.items():
            if (robot.recovery_session_role is not None and
                    self.sim_time < robot.recovery_session_until):
                continue
            started = robot.emergency_braking_since
            if (not robot.emergency_braking or started is None or
                    self.sim_time - started < timeout):
                continue
            if self.sim_time < robot.emergency_recovery_until:
                continue

            final_target = None
            if robot.waypoints:
                final_target = tuple(robot.waypoints[-1])
            elif robot.goal_location in ALL_LOCATIONS:
                final_target = ALL_LOCATIONS[robot.goal_location]
            elif robot.goal_location in CHARGING_STATIONS:
                final_target = CHARGING_STATIONS[robot.goal_location]
            at_target = (final_target is not None and math.hypot(
                robot.position[0] - final_target[0],
                robot.position[1] - final_target[1]) <= target_tolerance)

            robot.emergency_braking = False
            robot.emergency_braking_since = None
            robot.emergency_recovery_until = self.sim_time + 2.0
            if at_target:
                self._send_command_to_robot(rid, {'type': 'stop'})
                print(f"[EmergencyRecovery] Robot {rid} blocked at target "
                      "for 10 s; accepting target arrival")
                self._handle_goal_reached(rid, force=True)
                continue

            if (ENABLE_NONPHYSICAL_RECOVERY and
                    self._teleport_stalled_group([rid])):
                print(f"[EmergencyRecovery] Robot {rid} blocked for 10 s; "
                      "relocated to a safe point on its remaining path")
                continue

            robot._replan_requested = True
            robot.emergency_braking = True
            robot.emergency_braking_since = self.sim_time
    
    def _resolve_next_in_queue(self):
        """处理死锁队列中的下一个机器人"""
        if not self._deadlock_queue:
            self._deadlock_state = 'IDLE'
            self._deadlock_cooldown_until = self.sim_time + 3.0
            return
        
        next_rid = self._deadlock_queue.pop(0)
        # 剩余未恢复的作为障碍
        remaining = list(self._deadlock_queue)
        self._resolve_robot_deadlock(next_rid, remaining)
    
    def _detect_head_on_pair(self, stalled_robots):
        """检测是否有两个robot在同一走廊迎面对撞�?
        
        支持两种情况:
          1. 垂直走廊: |x1-x2|<0.8m, 方向y相反 (一北一�?
          2. 水平走廊: |y1-y2|<0.8m, 方向x相反 (一东一�?
        
        返回: (higher_priority_rid, lower_priority_rid) �?None
        """
        from config import ALL_LOCATIONS
        
        def _resolve_goal(g):
            """将goal_location转为坐标tuple (可能是str或tuple)"""
            if g is None:
                return None
            if isinstance(g, (list, tuple)) and len(g) >= 2:
                return (float(g[0]), float(g[1]))
            if isinstance(g, str):
                pos = ALL_LOCATIONS.get(g)
                return pos if pos else None
            return None
        
        for i in range(len(stalled_robots)):
            rid_a = stalled_robots[i][0]
            ra = self.robots[rid_a]
            goal_a = _resolve_goal(getattr(ra, 'goal_location', None))
            if not goal_a:
                continue
            for j in range(i+1, len(stalled_robots)):
                rid_b = stalled_robots[j][0]
                rb = self.robots[rid_b]
                goal_b = _resolve_goal(getattr(rb, 'goal_location', None))
                if not goal_b:
                    continue
                
                dx_pos = abs(ra.position[0] - rb.position[0])
                dy_pos = abs(ra.position[1] - rb.position[1])
                
                head_on_detected = False
                
                # 情况1: 垂直走廊(x接近, y方向相反)
                if dx_pos < 0.8:
                    dir_a_y = goal_a[1] - ra.position[1]
                    dir_b_y = goal_b[1] - rb.position[1]
                    if dir_a_y * dir_b_y < 0:
                        head_on_detected = True
                
                # 情况2: 水平走廊(y接近, x方向相反)
                if not head_on_detected and dy_pos < 0.8:
                    dir_a_x = goal_a[0] - ra.position[0]
                    dir_b_x = goal_b[0] - rb.position[0]
                    if dir_a_x * dir_b_x < 0:
                        head_on_detected = True
                
                if head_on_detected:
                    if rid_a < rid_b:
                        return (rid_a, rid_b)
                    else:
                        return (rid_b, rid_a)
        return None

    def _resolve_robot_deadlock(self, rid, obstacle_rids):
        """
        为指定机器人解锁：以其他停滞peer为静态障碍重规划�?
        """
        robot = self.robots.get(rid)
        if not robot:
            self._resolve_next_in_queue()
            return
        
        goal = self._navigation_goal(robot)
        if not goal:
            self._resolve_next_in_queue()
            return
        
        # 重规�?plan_grid_lifelong已经把其他peer的reservation当障�?
        new_path = self.motion_coordinator.plan_grid_lifelong(
            rid, robot.position, goal)
        
        if new_path and len(new_path) > 0:
            self._install_runtime_plan(rid, new_path)
            print(f"[Deadlock] Priority recovery: Robot {rid} replanning "
                  f"(avoiding {obstacle_rids}); new path={len(new_path)} steps")
            # 进入等待移动确认
            self._deadlock_current = rid
            self._deadlock_start_pos = tuple(robot.position)
            self._deadlock_state = 'WAIT_MOVE'
            self._deadlock_wait_start = self.sim_time
        else:
            # 重规划失�?�?后退再试
            print(f"[Deadlock] Robot {rid} replan failed; commanding reverse by {0.3} m")
            self._command_reverse(rid, 0.3)
            self._deadlock_current = rid
            self._deadlock_start_pos = tuple(robot.position)
            self._deadlock_state = 'WAIT_MOVE'
            self._deadlock_wait_start = self.sim_time
    
    def _joint_escape_robot(self, rid, min_peer_clearance=0.85):
        """Move one robot to a validated nearby standoff point.

        Unlike the legacy reverse command, this searches the full 360-degree
        ring around the robot and prefers a point that simultaneously clears
        every current peer position and makes positive progress toward the
        original business goal.  It installs a moving escape plan, never a
        hold, so it cannot create a stationary blocking pair.
        """
        robot = self.robots.get(rid)
        if not robot:
            return False
        original_goal = self._navigation_goal(robot)
        goal_xy = (ALL_LOCATIONS.get(original_goal)
                   if isinstance(original_goal, str) else original_goal)
        peers = [peer.position for peer_id, peer in self.robots.items()
                 if peer_id != rid]
        candidates = []
        distances = (0.45, 0.60, 0.80, 1.00, 1.25, 1.50, 1.75)
        for distance in distances:
            for step in range(32):
                angle = 2.0 * math.pi * step / 32.0
                target = (robot.position[0] + distance * math.cos(angle),
                          robot.position[1] + distance * math.sin(angle))
                clearance = min((math.hypot(target[0] - px, target[1] - py)
                                 for px, py in peers), default=math.inf)
                if clearance < min_peer_clearance:
                    continue
                if not self.motion_coordinator._segment_clear(
                        robot.position, target):
                    continue
                goal_progress = 0.0
                if goal_xy is not None:
                    goal_progress = (
                        math.hypot(robot.position[0] - goal_xy[0],
                                   robot.position[1] - goal_xy[1]) -
                        math.hypot(target[0] - goal_xy[0],
                                   target[1] - goal_xy[1]))
                score = (min(clearance, 1.6) + 0.55 * goal_progress -
                         0.04 * distance)
                candidates.append((score, clearance, target))
        if not candidates:
            return False
        candidates.sort(key=lambda item: item[0], reverse=True)
        for _score, clearance, target in candidates:
            path = self.motion_coordinator.plan_grid_lifelong(
                rid, robot.position, target)
            if not path:
                continue
            if not self._install_runtime_plan(
                    rid, path, delay=0.0, source='_command_reverse'):
                continue
            robot.recovery_active = True
            robot.recovery_resume_goal = original_goal
            if getattr(self, 'metrics', None) is not None:
                self.metrics.record_escape(
                    self.sim_time, rid, '_joint_escape_robot', target)
            print(f"[JointEscape] T={self.sim_time:.1f}s robot={rid} "\
                  f"standoff=({target[0]:.2f},{target[1]:.2f}) "\
                  f"clearance={clearance:.2f}m")
            return True
        return False

    def _joint_try_escape_component(self, component) -> bool:
        """Deterministically separate one robot from a close conflict group."""
        now = self.sim_time
        component = tuple(sorted(component))

        def yield_score(rid):
            robot = self.robots[rid]
            goal = self._goal_coordinates(self._navigation_goal(robot))
            goal_distance = 0.0
            if goal is not None:
                goal_distance = math.hypot(
                    robot.position[0] - goal[0], robot.position[1] - goal[1])
            peer_distance = min(
                (math.hypot(robot.position[0] - self.robots[peer].position[0],
                            robot.position[1] - self.robots[peer].position[1])
                 for peer in component if peer != rid),
                default=0.0)
            # Prefer escaping the robot with the longest remaining trip and
            # the least local clearance.  This mirrors normal right-of-way
            # reasoning: a robot already near its dock should stay on task.
            return (goal_distance, -peer_distance, -rid)

        ordered = sorted(component, key=yield_score, reverse=True)
        for rid in ordered:
            robot = self.robots[rid]
            if getattr(robot, '_joint_escape_until', 0.0) > now:
                continue
            if getattr(robot, 'recovery_active', False):
                continue
            if self._joint_escape_robot(rid, min_peer_clearance=0.80):
                robot._joint_escape_until = now + 3.0
                self._joint_liveness_needed = True
                self._next_joint_grid_tick = min(
                    getattr(self, '_next_joint_grid_tick', now + 2.0),
                    now + 2.0)
                for peer_id in component:
                    if peer_id == rid:
                        continue
                    self._set_robot_speed_scale(peer_id, 0.60)
                return True

        # If the cluster is already inside the minimum safe envelope and no
        # validated moving standoff exists, relocate the whole component to
        # nearby free-grid cells before the situation becomes a fleet-wide
        # park.  This is a last-resort liveness guarantee, not a routing
        # strategy; it is rate-limited and immediately followed by a fresh
        # all-active joint plan.
        component_distances = [
            math.hypot(self.robots[a].position[0] - self.robots[b].position[0],
                       self.robots[a].position[1] - self.robots[b].position[1])
            for index, a in enumerate(component)
            for b in component[index + 1:]
        ]
        if not component_distances or min(component_distances) >= 0.62:
            return False
        if getattr(self, '_joint_teleport_cooldown_until', 0.0) > now:
            return False
        if self._teleport_stalled_group(list(component)):
            self._joint_teleport_cooldown_until = now + 2.0
            self._joint_liveness_needed = True
            self._next_joint_grid_tick = min(
                getattr(self, '_next_joint_grid_tick', now + 0.5),
                now + 0.5)
            print(f"[JointEscape] T={now:.1f}s teleport-separated "
                  f"component={component}")
            return True
        self._joint_teleport_cooldown_until = now + 1.0
        return False

    def _command_reverse(self, rid, dist):
        """Plan and validate several physical escape directions."""
        robot = self.robots.get(rid)
        if not robot:
            return False
        heading = getattr(robot, 'heading', 0.0)
        original_goal = self._navigation_goal(robot)
        goal_xy = (ALL_LOCATIONS.get(original_goal)
                   if isinstance(original_goal, str) else original_goal)
        peers = [peer.position for peer_id, peer in self.robots.items()
                 if peer_id != rid]
        candidates = []
        # Search the full surrounding free space. Useful aisle clearance is
        # often lateral and farther away than the old 0.5 m candidate ring.
        offsets = tuple(math.pi + i * math.pi / 8 for i in range(16))
        distances = sorted({max(0.3, dist), 0.5, 0.75, 1.0, 1.5})
        for distance in distances:
            for offset in offsets:
                angle = heading + offset
                target = (robot.position[0] + distance * math.cos(angle),
                          robot.position[1] + distance * math.sin(angle))
                clearance = min((math.hypot(target[0] - px,
                                            target[1] - py)
                                 for px, py in peers), default=math.inf)
                goal_progress = 0.0
                if goal_xy is not None:
                    goal_progress = (
                        math.hypot(robot.position[0] - goal_xy[0],
                                   robot.position[1] - goal_xy[1]) -
                        math.hypot(target[0] - goal_xy[0],
                                   target[1] - goal_xy[1]))
                # Once safely clear, prefer motion that also advances the
                # business route instead of always retreating farthest away.
                score = (min(clearance, 1.5) + 0.6 * goal_progress -
                         0.05 * distance)
                candidates.append((score, clearance, target))
        candidates.sort(key=lambda item: item[0], reverse=True)

        for _score, clearance, target in candidates:
            if clearance < 0.80:
                continue
            if not self.motion_coordinator._segment_clear(
                    robot.position, target):
                continue
            retreat_path = self.motion_coordinator.plan_grid_lifelong(
                rid, robot.position, target)
            if not retreat_path:
                continue
            if (self._install_runtime_plan(
                    rid, retreat_path, delay=0.0) and
                    self._last_plan_dispatch_performed):
                robot.recovery_active = True
                robot.recovery_resume_goal = original_goal
                if getattr(self, 'metrics', None) is not None:
                    self.metrics.record_escape(
                        self.sim_time, rid, '_command_reverse', target)
                print(f"[Deadlock] Robot {rid} escape path dispatched to "
                      f"({target[0]:.2f}, {target[1]:.2f}); "
                      f"peer clearance={clearance:.2f} m")
                return True
        return False
    

    def _dispatch_delayed_robots(self):
        """Check and dispatch robots that were waiting due to temporal conflicts."""
        if not getattr(self, "system_ready", True):
            return
        for rid, robot in self.robots.items():
            if (robot.pending_waypoints is not None and
                    self.sim_time >= robot.dispatch_not_before):
                pending = robot.pending_waypoints
                source = robot.pending_plan_source or 'unknown'
                is_runtime = robot.pending_is_runtime_replan
                self._dispatch_plan(
                    rid, pending, delay=0.0, source=source,
                    is_runtime_replan=is_runtime)
    
    def _update_robot_states(self, dt: float):
        """Update robot states including battery and distance tracking."""
        self._recover_emergency_braking()
        for rid, robot in self.robots.items():
            # Update distance traveled
            if robot.last_position:
                dx = robot.position[0] - robot.last_position[0]
                dz = robot.position[1] - robot.last_position[1]
                dist = math.sqrt(dx*dx + dz*dz)
                robot.total_distance += dist
            robot.last_position = robot.position
            if dt > 0:
                robot.velocity = (dx / dt, dz / dt)
            robot.sample_time = self.sim_time
            robot.state_seq += 1
            if robot.hold_until and self.sim_time >= robot.hold_until:
                robot.hold_until = 0.0
                robot.wait_started = 0.0

            # Webots can miss the final reached-goal packet.  Confirm arrival
            # geometrically so a robot physically inside a station always
            # starts the five-second swap timer.
            if (robot.state == RobotState.RETURNING_TO_CHARGE and
                    robot.goal_location in CHARGING_STATIONS):
                station_xy = CHARGING_STATIONS[robot.goal_location]
                if math.hypot(robot.position[0] - station_xy[0],
                              robot.position[1] - station_xy[1]) <= 0.5:
                    robot.state = RobotState.CHARGING
                    robot.waypoints = []
                    robot.current_waypoint_idx = 0
                    robot.charging_started_at = self.sim_time
                    robot.battery_swap_ready_at = self.sim_time + 5.0
                    robot.next_charge_log = self.sim_time
                    print(f"[T={self.sim_time:.1f}] Robot {rid} station arrival "
                          "confirmed by position; battery swap started")

            # Process the battery swap before normal state updates.  This is
            # intentionally keyed by the arrival timer as well as CHARGING,
            # so a delayed controller status message cannot prevent a swap.
            swap_at = getattr(robot, "battery_swap_ready_at", None)
            if (swap_at is not None and self.sim_time >= swap_at and
                    getattr(robot, "charging_started_at", None) is not None):
                robot.battery = self._battery_rng.uniform(95.0, 100.0)
                self._send_command_to_robot(robot_id=rid, command={
                    'type': 'battery_swap',
                    'battery': robot.battery,
                })
                robot.battery_swap_ready_at = None
                robot.charging_started_at = None
                if robot.current_task is not None:
                    robot.current_task.status = TaskStatus.PENDING
                    robot.current_task.assigned_robot = None
                    robot.current_task.assignment_time = None
                    robot.current_task = None
                robot.state = RobotState.IDLE
                print(f"[T={self.sim_time:.1f}] Robot {rid} BATTERY SWAP "
                      f"completed: battery={robot.battery:.1f}%, returning to service")
                continue
            
            # Update battery: task/home movement drains; idle, waiting and
            # charging-related states do not drain. Charging gains only when
            # the robot has arrived at the station.
            if robot.state in (RobotState.EN_ROUTE_PICKUP,
                               RobotState.CARRYING,
                               RobotState.EN_ROUTE_DELIVERY,
                                RobotState.RETURNING_HOME):
                robot.battery = max(0, robot.battery - BATTERY_DRAIN_RATE * dt)
                robot.active_time += dt
            elif robot.state == RobotState.CHARGING:
                # Be defensive against a restored/repeated arrival message:
                # every CHARGING state must have exactly one swap start time.
                if getattr(robot, "charging_started_at", None) is None:
                    robot.charging_started_at = self.sim_time
                    robot.battery_swap_ready_at = self.sim_time + 5.0
                # Keep charging telemetry visible without logging every
                # Webots timestep.
                next_charge_log = getattr(robot, "next_charge_log", 0.0)
                if self.sim_time >= next_charge_log:
                    station = robot.goal_location or "unknown"
                    print(f"[T={self.sim_time:.1f}] Robot {rid} CHARGING at "
                          f"{station}: battery={robot.battery:.1f}%")
                    robot.next_charge_log = self.sim_time + 5.0
            elif robot.state == RobotState.IDLE:
                robot.idle_time += dt
            
            # Check low battery �?only trigger return-to-charge when IDLE and
            # battery is below the threshold. RETURNING_TO_CHARGE / CHARGING
            # robots are already handled by their respective state code.
            if (robot.battery < LOW_BATTERY_THRESHOLD and
                    robot.state not in (RobotState.RETURNING_TO_CHARGE,
                                        RobotState.CHARGING)):
                if robot.battery < TASK_ABORT_BATTERY_THRESHOLD:
                    if robot.current_task is not None:
                        self._requeue_task_for_low_battery(rid, cancel=True)
                    self._send_to_charging(rid)
                elif robot.current_task is None:
                    # Idle low-battery robots charge before receiving work;
                    # active robots in the 15-25% band finish their task.
                    self._send_to_charging(rid)
            
            # Simulate robot movement toward waypoints (if not using physics)
            self._simulate_movement(rid, dt)

    def _simulate_movement(self, robot_id: int, dt: float):
        """
        Track waypoint progression using the robot's REAL position
        (read from Webots in _get_robot_positions_from_webots).

        We do NOT advance the waypoint based on speculative motion �?
        we wait until the robot actually drives close enough.

        The robot controller is the source of truth for movement; this
        method only reflects what's already happened on the physics
        side and triggers state transitions when goals are reached.
        """
        robot = self.robots[robot_id]

        # Joint execution progress is controller-authored and epoch-versioned.
        # Applying the legacy 0.40 m inference to a 0.25 m joint grid consumes
        # cells without motion and can remove robots from the next all-active
        # candidate. The joint refresh gate already checks controller epoch,
        # controller index and measured 0.18 m proximity.
        if robot.active_plan_source == 'joint_grid_transaction':
            return

        if not robot.waypoints or robot.current_waypoint_idx >= len(robot.waypoints):
            return

        target = robot.waypoints[robot.current_waypoint_idx]
        rx, ry = robot.position
        dist = math.hypot(target[0] - rx, target[1] - ry)

        # Use a slightly larger tolerance than the controller-side
        # GOAL_THRESHOLD (0.35m) so we don't miss a transition in
        # the rare case the supervisor's snapshot is between waypoints.
        SUPERVISOR_TOLERANCE = 0.40

        if dist < SUPERVISOR_TOLERANCE:
            # Reached this waypoint �?advance.
            robot.current_waypoint_idx += 1

            if robot.current_waypoint_idx >= len(robot.waypoints):
                # All waypoints consumed �?robot has reached the final goal.
                # _handle_goal_reached fires the EN_ROUTE_PICKUP �?
                # EN_ROUTE_DELIVERY �?IDLE transitions.
                self._handle_goal_reached(robot_id)

    def _handle_waypoint_reached(self, robot_id: int):
        """Handle when a robot reaches an intermediate waypoint."""
        robot = self.robots[robot_id]
        self.motion_coordinator.advance_robot(robot_id)

    def _handle_goal_reached(self, robot_id: int, force: bool = False):
        """Handle when a robot reaches its final goal.
        
        DEFENSE: Verify the robot is physically close to its task goal
        before firing state transitions. This guards against spurious
        triggers (e.g. message routing bugs, stale flags) that would
        otherwise cause "instant 0.3s task completion" artefacts.
        """
        robot = self.robots[robot_id]

        # An escape waypoint is a temporary motion leg, never a business
        # pickup/delivery/home arrival. Resume the authoritative goal through
        # the same planner and post-validation gate before state transitions.
        if robot.recovery_active:
            resume_goal = robot.recovery_resume_goal
            robot.recovery_active = False
            robot.recovery_resume_goal = None
            # The escape leg has completed, so its emergency writer lease
            # must not block the validated successor route.  Limit this
            # hand-off to the active recovery owner; never clear an unrelated
            # higher-priority writer.
            if robot.route_write_owner == '_command_reverse':
                robot.route_write_owner = None
                robot.route_write_until = 0.0
            if ENABLE_JOINT_RUNTIME and getattr(self, 'joint_runtime_enforced', False):
                # In joint mode a single-robot resume can immediately steer
                # the robot back into the same dense cluster.  Hand the
                # original goal back to the all-active joint planner instead;
                # the next rolling window will dispatch a fleet-coordinated
                # successor route from this safer standoff position.
                robot.goal_location = resume_goal
                robot._replan_requested = False
                # The escape leg has been consumed.  Re-admit this robot to
                # the next all-active joint planning window even though its
                # temporary route is exhausted; otherwise _has_active_navigation
                # excludes it forever and it becomes a stationary blocker.
                robot.active_plan_source = 'joint_grid_transaction'
                robot.waypoints = []
                robot.current_waypoint_idx = 0
                self._joint_liveness_needed = True
                self._next_joint_grid_tick = min(
                    getattr(self, '_next_joint_grid_tick', self.sim_time),
                    self.sim_time)
                return
            resumed = (self.motion_coordinator.plan_grid_lifelong(
                robot_id, robot.position, resume_goal) if resume_goal else None)
            if resumed and self._install_runtime_plan(
                    robot_id, resumed, delay=0.0,
                    source='_resume_after_escape'):
                robot.recovery_session_until = self.sim_time
                print(f"[Deadlock] Robot {robot_id} escape leg complete; "
                      "validated original route resumes without corridor hold")
                return
            # Keep an unfinished waypoint so the emergency request is visible
            # to the normal monitor rather than being filtered as completed.
            robot.current_waypoint_idx = max(0, len(robot.waypoints) - 1)
            robot._replan_requested = True
            return
        
        # Sanity-check: actual robot position must be near the expected
        # goal for this state. If not, this is a spurious trigger and
        # we silently ignore it (the robot is still en-route).
        if robot.current_task and not force:
            if robot.state == RobotState.EN_ROUTE_PICKUP:
                expected = robot.current_task.pickup_position
            elif robot.state == RobotState.EN_ROUTE_DELIVERY:
                expected = robot.current_task.delivery_position
            else:
                expected = None
            if expected is not None:
                self._get_robot_positions_from_webots()
                dx = robot.position[0] - expected[0]
                dy = robot.position[1] - expected[1]
                d = math.sqrt(dx*dx + dy*dy)
                # Allow some slack vs GOAL_TOLERANCE because this is
                # a sanity check, not the primary trigger.
                if d > GOAL_TOLERANCE * 2.5:  # ~0.75 m
                    # Spurious �?robot is not near goal. Ignore.
                    return
        # The committed leg is complete; its writer no longer owns the
        # successor business transition.
        robot.route_write_owner = None
        robot.route_write_until = 0.0
        
        if robot.state == RobotState.EN_ROUTE_PICKUP and robot.current_task:
            # Arrived at pickup location
            # Read fresh robot position before planning the delivery leg.
            self._get_robot_positions_from_webots()

            # Plan delivery path via lifelong (avoids other robots'
            # reservations); fall back to A*.
            delivery_path = self.motion_coordinator.plan_grid_lifelong(
                robot_id, robot.position,
                robot.current_task.delivery_location)
            if delivery_path is None:
                # The pickup leg is physically complete. Drop its retained
                # grid reservation before using the legacy fallback planner;
                # otherwise peers would avoid a path the robot has left.
                self.motion_coordinator.release_robot_grid(robot_id)
                delivery_path = self.motion_coordinator.plan_path_for_robot(
                    robot_id, robot.position,
                    robot.current_task.delivery_location)
            if delivery_path:
                if not self._dispatch_plan(
                        robot_id, delivery_path,
                        source='_handle_goal_reached'):
                    self.motion_coordinator.rollback_robot_plan(robot_id)
                    robot._replan_requested = True
                    robot._last_replan_time = self.sim_time - 3.0
                    return
                robot.state = RobotState.CARRYING
                robot.current_task.status = TaskStatus.IN_PROGRESS
                robot.current_task.pickup_time = self.sim_time
                robot.goal_location = robot.current_task.delivery_location
                robot.waypoints = delivery_path
                robot.current_waypoint_idx = 0
                robot.state = RobotState.EN_ROUTE_DELIVERY
            else:
                robot._replan_requested = True
                robot._last_replan_time = self.sim_time - 3.0
                return
            
            print(f"[T={self.sim_time:.1f}] Robot {robot_id} picked up task "
                  f"{robot.current_task.task_id} at {robot.current_task.pickup_location}")
        
        elif robot.state == RobotState.EN_ROUTE_DELIVERY and robot.current_task:
            # Arrived at delivery location - task complete!
            robot.current_task.status = TaskStatus.COMPLETED
            robot.current_task.completion_time = self.sim_time
            robot.tasks_completed += 1
            
            print(f"[T={self.sim_time:.1f}] Robot {robot_id} COMPLETED task "
                  f"{robot.current_task.task_id}: "
                  f"{robot.current_task.pickup_location} -> "
                  f"{robot.current_task.delivery_location} "
                  f"(duration: {robot.current_task.completion_duration:.1f}s)")
            
            # Record metrics
            self.metrics.record_task_completion(
                robot.current_task, robot_id, self.sim_time
            )
            
            # Release this robot's task-time reservations so others can
            # route through where it has been (lifelong CBS bookkeeping).
            self.motion_coordinator.release_lifelong(robot_id)
            self.motion_coordinator.release_robot_grid(robot_id)

            robot.goal_location = None
            # Task complete �?robot is now IDLE in place.
            # Lazy relocation: the main loop's _relocate_idle_robots()
            # will walk this robot to the nearest free REST_NODE on
            # the next tick, BUT only if no new task arrives first.
            # This way, "task chaining" is automatic: a new task posted
            # to scheduler can claim the IDLE robot without waiting
            # for it to walk back to a parking spot.
            robot.current_task = None
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            self.motion_coordinator.clear_robot_path(robot_id)
            # A task may finish exactly as the battery crosses the low
            # threshold.  Route to charging immediately before idle/home
            # relocation or a new dispatch can claim this robot.
            if robot.battery < LOW_BATTERY_THRESHOLD:
                self._send_to_charging(robot_id)
                return
            # Reserve the current position's nearest node so peers
            # don't route through us while we wait.
            cur_node = self.motion_coordinator.graph.get_nearest_node(
                robot.position)
            if cur_node:
                self.motion_coordinator.lifelong.reserve_static(
                    robot_id, cur_node)
        
        elif robot.state == RobotState.RETURNING_HOME:
            home_goal = self._goal_coordinates(robot.goal_location)
            if (home_goal is None or math.hypot(
                    robot.position[0] - home_goal[0],
                    robot.position[1] - home_goal[1]) > GOAL_TOLERANCE * 2.0):
                return
            # Robot reached a rest node (or its initial home spot).
            # Go IDLE; reserve the *actual current node* �?not the
            # original home �?so the static reservation matches reality.
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            robot.goal_location = None
            self.motion_coordinator.clear_robot_path(robot_id)
            self.motion_coordinator.release_lifelong(robot_id)
            self.motion_coordinator.release_robot_grid(robot_id)
            cur_node = self.motion_coordinator.graph.get_nearest_node(
                robot.position)
            if cur_node:
                self.motion_coordinator.lifelong.reserve_static(
                    robot_id, cur_node)
        
        elif robot.state == RobotState.RETURNING_TO_CHARGE:
            station_goal = self._goal_coordinates(robot.goal_location)
            if (station_goal is None or math.hypot(
                    robot.position[0] - station_goal[0],
                    robot.position[1] - station_goal[1]) >
                    GOAL_TOLERANCE * 2.0):
                return
            # Arrived at charging station �?now actually start charging.
            robot.state = RobotState.CHARGING
            robot.next_charge_log = self.sim_time
            robot.charging_started_at = self.sim_time
            robot.battery_swap_ready_at = self.sim_time + 5.0
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            self.motion_coordinator.clear_robot_path(robot_id)
            print(f"[T={self.sim_time:.1f}] Robot {robot_id} ARRIVED at charging "
                  f"station, battery={robot.battery:.1f}% (will refill to "
                  f"95-100% by T={robot.battery_swap_ready_at:.1f}s)")

        elif robot.state == RobotState.CHARGING:
            # Already charging �?nothing to do; battery handled elsewhere
            pass

    def _send_to_home(self, robot_id: int):
        """
        Send a robot back to its designated home parking spot.
        Robots park here when IDLE so they don't block workstations
        or storage lanes from peers needing those locations next.

        Skips the trip if the robot is already at home (or PARKING_SPOTS
        has no entry for this id). Plans via lifelong CBS so we don't
        overlap any in-flight robot's reservations.
        """
        home_xy = PARKING_SPOTS.get(robot_id)
        if home_xy is None:
            # No home defined �?fall back to direct IDLE
            robot = self.robots[robot_id]
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            robot.goal_location = None
            self.motion_coordinator.clear_robot_path(robot_id)
            return

        robot = self.robots[robot_id]
        # If already at home (within tolerance), skip the trip
        dist = math.hypot(robot.position[0] - home_xy[0],
                          robot.position[1] - home_xy[1])
        if dist < GOAL_TOLERANCE:
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            robot.goal_location = None
            self.motion_coordinator.clear_robot_path(robot_id)
            self.motion_coordinator.reserve_home(robot_id, home_xy)
            return

        # Refresh real position before planning
        self._get_robot_positions_from_webots()

        # Lifelong plan to home XY �?encode as a temporary location key
        # by inserting it into ALL_LOCATIONS-equivalent lookup. Easier:
        # call plan_lifelong with a synthetic goal �?but our coordinator
        # API needs a name. We reuse plan_path_for_robot's "any (x,y)"
        # variant by treating home as a node lookup.
        from config import WAYPOINTS
        # Find the nearest graph node to home
        home_node = self.motion_coordinator.graph.get_nearest_node(home_xy)
        # Now plan via the LifelongPlanner directly
        node_path = self.motion_coordinator.lifelong.plan(
            robot_id, self.motion_coordinator.graph.get_nearest_node(
                robot.position), home_node)
        if node_path is None:
            # Fall back: go straight to IDLE if no path available
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            robot.goal_location = None
            self.motion_coordinator.clear_robot_path(robot_id)
            return

        waypoints = [WAYPOINTS[n] for n in node_path]
        # Append the actual home XY as a final off-graph stop
        if waypoints[-1] != home_xy:
            waypoints.append(home_xy)
        # Drop redundant first waypoint == current_position
        if waypoints and (
                abs(waypoints[0][0] - robot.position[0]) < 0.05 and
                abs(waypoints[0][1] - robot.position[1]) < 0.05):
            waypoints.pop(0)
        if not waypoints:
            # We're already there
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            robot.goal_location = None
            self.motion_coordinator.reserve_home(robot_id, home_xy)
            return
        
        robot.state = RobotState.RETURNING_HOME
        robot.goal_location = tuple(home_xy)
        self._install_runtime_plan(robot_id, waypoints, delay=0.0)
        print(f"[T={self.sim_time:.1f}] Robot {robot_id} returning to home "
              f"spot {home_xy}")

    def _relocate_idle_robots(self):
        """
        Lazy relocation �?for each IDLE robot not already at a REST_NODE,
        find the nearest free rest node and dispatch the robot there.
        
        Called once per main loop tick. It is interruptible: if a new
        task arrives during the walk, the scheduler simply picks up
        this IDLE-with-waypoints robot like any other.
        """
        if not getattr(self, "system_ready", True):
            return
        for rid, robot in self.robots.items():
            if robot.state != RobotState.IDLE:
                continue
            if robot.waypoints:
                continue   # already relocating
            # Skip if already at a rest node
            cur_node = self.motion_coordinator.graph.get_nearest_node(
                robot.position)
            if cur_node in REST_NODES:
                continue
            # Find nearest free rest node
            result = self.motion_coordinator.find_nearest_rest_node(
                robot.position, exclude_robot_id=rid)
            if result is None:
                continue
            target_name, target_xy = result
            d2 = math.hypot(robot.position[0] - target_xy[0],
                             robot.position[1] - target_xy[1])
            if d2 < 0.4:
                continue
            self.motion_coordinator.release_home(rid)
            # Plan via lifelong (node-name based, guaranteed on-graph)
            node_path = self.motion_coordinator.lifelong.plan(
                rid, cur_node, target_name)
            if not node_path:
                continue
            wpts = [WAYPOINTS[n] for n in node_path]
            # Drop redundant first waypoint
            if wpts and (
                    abs(wpts[0][0] - robot.position[0]) < 0.05 and
                    abs(wpts[0][1] - robot.position[1]) < 0.05):
                wpts.pop(0)
            if not wpts:
                continue
            robot.state = RobotState.RETURNING_HOME
            robot.goal_location = tuple(target_xy)
            self._install_runtime_plan(rid, wpts, delay=0.0)

    def _send_to_charging(self, robot_id: int):
        """Route to the nearest reachable station, then shortest queue."""
        robot = self.robots[robot_id]
        if self.sim_time < getattr(robot, "charging_retry_at", 0.0):
            return

        # Free old reservations, then plan fresh path to CS via lifelong.
        self.motion_coordinator.release_lifelong(robot_id)
        self.motion_coordinator.release_robot_grid(robot_id)
        self._get_robot_positions_from_webots()

        def distance(name):
            pos = CHARGING_STATIONS[name]
            return math.hypot(robot.position[0] - pos[0],
                              robot.position[1] - pos[1])

        nearest_first = sorted(CHARGING_STATIONS, key=distance)
        queue_lengths = {name: 0 for name in CHARGING_STATIONS}
        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue
            if other.state in (RobotState.RETURNING_TO_CHARGE,
                               RobotState.CHARGING):
                if other.goal_location in queue_lengths:
                    queue_lengths[other.goal_location] += 1

        # Prefer the nearest station. If it is not reachable, select among
        # alternatives by queue length and then distance.
        candidates = nearest_first[:1]
        path = None
        nearest_cs = None
        for station in candidates:
            path = self.motion_coordinator.plan_grid_lifelong(
                robot_id, robot.position, station)
            if path is None:
                path = self.motion_coordinator.plan_path_for_robot(
                    robot_id, robot.position, station)
            if path:
                nearest_cs = station
                break
        if path is None or not nearest_cs:
            candidates = sorted((name for name in CHARGING_STATIONS
                                 if name != nearest_first[0]),
                                key=lambda name: (queue_lengths[name],
                                                  distance(name)))
            for station in candidates:
                path = self.motion_coordinator.plan_grid_lifelong(
                    robot_id, robot.position, station)
                if path is None:
                    path = self.motion_coordinator.plan_path_for_robot(
                        robot_id, robot.position, station)
                if path:
                    nearest_cs = station
                    print(f"[T={self.sim_time:.1f}] Robot {robot_id} "
                          f"selected {station} by queue length="
                          f"{queue_lengths[station]}")
                    break
        if path:
            robot.charging_retry_at = 0.0
            robot.waypoints = path
            robot.current_waypoint_idx = 0
            # Critical: robot is ONLY 'returning' to charge. It will become
            # CHARGING only when _handle_goal_reached confirms arrival at
            # the station. This prevents the bug where battery starts
            # refilling mid-trip and the robot gets stuck.
            robot.state = RobotState.RETURNING_TO_CHARGE
            robot.goal_location = nearest_cs
            # Use the same versioned/possibly delayed dispatch path as task
            # navigation.  Sending a bare command here used to leave the
            # supervisor and controller with different path versions.
            dispatched_now = self._dispatch_plan(robot_id, path)
            command_accepted = dispatched_now or (
                getattr(self, "_last_command_send_ok", False) and
                robot.pending_waypoints is not None)
            if not command_accepted:
                # Do not leave a low-battery robot in RETURNING_TO_CHARGE
                # without a controller command; retry from the next tick.
                robot.state = RobotState.WAITING
                robot.charging_retry_at = self.sim_time + 1.0
                robot.goal_location = None
                robot.waypoints = []
                robot.current_waypoint_idx = 0
                print(f"[T={self.sim_time:.1f}] Robot {robot_id} charging "
                      "command failed; retrying station route")
                return
            print(f"[T={self.sim_time:.1f}] Robot {robot_id} RETURNING TO "
                  f"CHARGE at {nearest_cs} (battery={robot.battery:.1f}%)")
        elif nearest_first and distance(nearest_first[0]) <= 0.5:
            # An empty path means "already there" only when the physical
            # position confirms station arrival.  Previously every planning
            # failure entered CHARGING at the delivery dock, making the robot
            # appear permanently stuck instead of driving to a station.
            nearest_cs = nearest_first[0]
            robot.charging_retry_at = 0.0
            robot.state = RobotState.CHARGING
            robot.goal_location = nearest_cs
            robot.charging_started_at = self.sim_time
            robot.battery_swap_ready_at = self.sim_time + 5.0
            robot.next_charge_log = self.sim_time
        else:
            # Both stations are temporarily unreachable (usually because of
            # active reservations). Keep the robot in a non-dispatchable
            # retry state. _update_robot_states calls this method again for
            # low-battery WAITING robots on subsequent ticks.
            robot.state = RobotState.WAITING
            robot.charging_retry_at = self.sim_time + 1.0
            robot.goal_location = None
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.pending_waypoints = None
            robot.dispatch_not_before = 0.0
            print(f"[T={self.sim_time:.1f}] Robot {robot_id} has no charging "
                  "route yet; waiting for reservations and retrying")

    def _requeue_task_for_low_battery(self, robot_id: int, cancel: bool = False):
        """Cancel or requeue an interrupted task before charging."""
        robot = self.robots[robot_id]
        task = robot.current_task
        if task is None:
            return
        # This project currently defines PENDING/ASSIGNED/IN_PROGRESS/
        # COMPLETED/FAILED (there is no CANCELLED enum member).  Only an
        # active task can be interrupted and returned to the pending queue.
        if cancel:
            task.status = TaskStatus.FAILED
            task.assigned_robot = None
            task.assignment_time = None
        elif task.status in (TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS,
                             TaskStatus.PENDING):
            task.status = TaskStatus.PENDING
            task.assigned_robot = None
            task.assignment_time = None
        robot.current_task = None
        robot.goal_location = None
        robot.waypoints = []
        robot.current_waypoint_idx = 0
        robot.pending_waypoints = None
        self.motion_coordinator.release_lifelong(robot_id)
        self.motion_coordinator.release_robot_grid(robot_id)
        robot.state = RobotState.IDLE
        action = "cancelled" if cancel else "requeued"
        print(f"[T={self.sim_time:.1f}] Robot {robot_id} battery="
              f"{robot.battery:.1f}%: task {action}; charging required")

    def _assign_tasks(self):
        """Serialize dispatch requests and coalesce re-entrant events."""
        if getattr(self, "dispatch_in_progress", False):
            self.dispatch_pending = True
            return
        self.dispatch_in_progress = True
        try:
            self._assign_tasks_impl()
        finally:
            self.dispatch_in_progress = False
        if getattr(self, "dispatch_pending", False):
            self.dispatch_pending = False
            self._assign_tasks_impl()

    def _runtime_path_cost_provider(self, robot_states):
        """Return the same static Grid-A* cost oracle used during search."""
        from training_scenarios import FactoryAStarCostOracle
        oracle = getattr(self, "_scheduler_path_cost_oracle", None)
        if oracle is None:
            oracle = FactoryAStarCostOracle(robot_states, self.num_robots)
            self._scheduler_path_cost_oracle = oracle
        else:
            oracle.bind_robot_states(robot_states)
        return oracle

    def _assign_tasks_impl(self):
        """Run the scheduler to assign pending tasks to idle robots."""
        if not getattr(self, "system_ready", True) or getattr(
                self, "startup_failed", False):
            return
        pending = self.task_generator.get_pending_tasks()
        if not pending:
            return
        if not self.initial_dispatch_attempted:
            self.initial_dispatch_attempted = True
            self.first_dispatch_time = self.sim_time
            print(f"[INITIAL_DISPATCH_ATTEMPT] pending_tasks={len(pending)} "
                  f"idle_robots={sum(r.state == RobotState.IDLE for r in self.robots.values())}")
        
        # Build robot states dict
        robot_states = {rid: robot.to_dict() for rid, robot in self.robots.items()}
        # Final dispatch-side battery guard.  It protects against stale
        # controller snapshots and keeps every scheduler consistent.
        for rid, robot in self.robots.items():
            if robot.battery < TASK_ABORT_BATTERY_THRESHOLD and robot.current_task is not None:
                self._requeue_task_for_low_battery(rid, cancel=True)
                self._send_to_charging(rid)
            elif (robot.current_task is None and
                  robot.state == RobotState.IDLE and
                  robot.battery < LOW_BATTERY_THRESHOLD):
                self._send_to_charging(rid)
        robot_states = {rid: robot.to_dict() for rid, robot in self.robots.items()}
        for state in robot_states.values():
            if (state.get('state') == RobotState.IDLE and
                    state.get('battery', 0.0) < LOW_BATTERY_THRESHOLD):
                state['state'] = RobotState.WAITING
        
        # Get congestion map
        congestion_map = self.motion_coordinator.get_congestion_map()
        
        # Keep assigning until no more assignments possible
        max_assignments = min(len(pending), len([r for r in self.robots.values() 
                                                  if r.state == RobotState.IDLE]))
        
        for _ in range(max_assignments):
            pending = self.task_generator.get_pending_tasks()
            if not pending:
                break
            
            robot_states = {rid: robot.to_dict() for rid, robot in self.robots.items()}
            
            self._failed_assignment_pairs = {
                pair: expiry
                for pair, expiry in self._failed_assignment_pairs.items()
                if expiry > self.sim_time
            }
            context = SchedulingContext(
                current_time=self.sim_time,
                congestion_map=congestion_map,
                path_cost_provider=self._runtime_path_cost_provider(
                    robot_states),
                failed_pairs=frozenset(self._failed_assignment_pairs),
                configuration={"runtime_geometry": "factory-grid-astar-v3"},
            )
            try:
                decision = self.scheduler.assign(pending, robot_states, context)
            except Exception as exc:
                decision = SchedulerResult(
                    algorithm_name=getattr(self.scheduler, "name", "unknown"),
                    diagnostics={"reason": "scheduler_exception",
                                 "exception": type(exc).__name__})
                print(f"[Supervisor] Scheduler {decision.algorithm_name} "
                      f"exception: {exc}; trying safe fallback chain")
            scheduling_seconds = decision.computation_time
            active_scheduler = self.scheduler
            self.metrics.record_rl_diagnostics(decision.diagnostics)
            # A safety-wrapped RL scheduler can return a feasible Hungarian
            # assignment internally.  Count and attribute that decision as a
            # fallback instead of silently reporting it as native RL work.
            if decision.diagnostics.get("fallback", False):
                self.metrics.record_scheduler_fallback(invalid_output=False)
            if not decision.is_feasible or not decision.assignments:
                reason = decision.diagnostics.get("reason", "")
                self.metrics.record_scheduler_fallback(
                    invalid_output=reason not in {
                        "no_candidates", "no_feasible_pair",
                        "empty_assignment", "empty_assignments"})
                print(f"[T={self.sim_time:.1f}] Scheduler {self.scheduler.name} "
                      f"rejected: {decision.diagnostics.get('reason')}; "
                      "trying safe fallback chain")
                for fallback in self.safe_schedulers:
                    try:
                        decision = fallback.assign(
                            pending, robot_states, context)
                    except Exception as exc:
                        decision = SchedulerResult(
                            algorithm_name=getattr(fallback, "name", "unknown"),
                            diagnostics={"reason": "fallback_exception",
                                         "exception": type(exc).__name__})
                        print(f"[Supervisor] Fallback {decision.algorithm_name} "
                              f"exception: {exc}")
                    scheduling_seconds += decision.computation_time
                    active_scheduler = fallback
                    if decision.is_feasible and decision.assignments:
                        break
            self.metrics.record_scheduling_latency(scheduling_seconds)
            if not decision.is_feasible or not decision.assignments:
                break

            assignment = decision.assignments[0]
            robot_id, task = assignment.robot_id, assignment.task
            
            robot = self.robots[robot_id]

            # CRITICAL: read the robot's real position from Webots BEFORE
            # planning �?robot.position may be stale if this assignment
            # fires in the same tick the robot just finished a previous
            # task and its state was reset.
            self._get_robot_positions_from_webots()

            # Plan path via lifelong CBS (avoids other robots' reservations);
            # fall back to per-robot A* if no conflict-free plan exists.
            pickup_path = self.motion_coordinator.plan_grid_lifelong(
                robot_id, robot.position, task.pickup_location)
            if pickup_path is None:
                pickup_path = self.motion_coordinator.plan_path_for_robot(
                    robot_id, robot.position, task.pickup_location)
            
            if pickup_path:
                # Commit task and robot state only after a usable plan exists.
                task.status = TaskStatus.ASSIGNED
                task.assigned_robot = robot_id
                task.assignment_time = self.sim_time
                robot.current_task = task
                robot.state = RobotState.EN_ROUTE_PICKUP
                robot.goal_location = task.pickup_location
                robot._last_progress_pos = robot.position
                robot._last_progress_time = self.sim_time
                self.motion_coordinator.release_home(robot_id)
                robot.waypoints = pickup_path
                robot.current_waypoint_idx = 0
                self._dispatch_plan(robot_id, pickup_path)
                if not getattr(self, "_last_command_send_ok", False):
                    # A usable path is not enough: the controller must receive
                    # the command before the assignment becomes authoritative.
                    task.status = TaskStatus.PENDING
                    task.assigned_robot = None
                    task.assignment_time = None
                    robot.current_task = None
                    robot.state = RobotState.IDLE
                    robot.goal_location = None
                    robot.waypoints = []
                    robot.current_waypoint_idx = 0
                    robot.pending_waypoints = None
                    robot.dispatch_not_before = 0.0
                    active_scheduler.on_assignment_rejected(
                        assignment, "command_send_failed")
                    self._failed_assignment_pairs[
                        (robot_id, task.task_id)
                    ] = self.sim_time + ASSIGNMENT_FAILURE_TTL
                    print(f"[T={self.sim_time:.1f}] Rolled back task "
                          f"{task.task_id}/robot {robot_id}: command send failed")
                    continue
                active_scheduler.on_assignment_committed(assignment)
                self.metrics.record_scheduler_commit(
                    decision.algorithm_name or active_scheduler.name,
                    native=(active_scheduler is self.scheduler and
                            not decision.diagnostics.get("fallback", False)))
                self.initial_dispatch_triggered = True
                self._failed_assignment_pairs.pop(
                    (robot_id, task.task_id), None)
                print(f"[T={self.sim_time:.1f}] Assigned task {task.task_id} to robot "
                      f"{robot_id}: {task.pickup_location} -> {task.delivery_location}")
            else:
                active_scheduler.on_assignment_rejected(
                    assignment, "pickup_path_unreachable")
                self._failed_assignment_pairs[
                    (robot_id, task.task_id)
                ] = self.sim_time + ASSIGNMENT_FAILURE_TTL
                print(f"[T={self.sim_time:.1f}] Rejected task {task.task_id}/robot "
                      f"{robot_id}: pickup path unreachable")

    def _check_deadlocks(self):
        """Periodically check for and resolve deadlocks."""
        robot_positions = {}
        robot_goals = {}
        
        for rid, robot in self.robots.items():
            nearest_node = self.motion_coordinator.graph.get_nearest_node(robot.position)
            robot_positions[rid] = nearest_node
            
            goal = robot._get_current_goal()
            if goal:
                if goal in LOCATION_TO_NODE:
                    robot_goals[rid] = LOCATION_TO_NODE[goal]
                else:
                    robot_goals[rid] = self.motion_coordinator.graph.get_nearest_node(
                        ALL_LOCATIONS.get(goal, CHARGING_STATIONS.get(goal, (0, 0)))
                    )
        
        deadlocks = self.motion_coordinator.detect_deadlock(robot_positions, robot_goals)
        
        for cycle in deadlocks:
            print(f"[T={self.sim_time:.1f}] DEADLOCK detected: robots {cycle}")
            self.motion_coordinator.resolve_deadlock(cycle)
            self.metrics.record_deadlock(cycle, self.sim_time)

    def run(self):
        """Main simulation loop."""
        # Reset lifelong planner state for a fresh run.
        self.motion_coordinator.lifelong_reset()
        # Register each robot's initial position as its home spot in CBS
        # so peers don't try to route through these nodes from t=0.
        for rid in self.robots:
            home_xy = PARKING_SPOTS.get(rid)
            if home_xy is not None:
                self.motion_coordinator.reserve_home(rid, home_xy)
        print(f"\n{'='*60}")
        print(f"Starting simulation: Scenario {self.scenario_name}")
        duration_label = (f"{SIM_DURATION}s (automatic batch stop)"
                          if AUTO_STOP_SIMULATION else
                          "interactive/unlimited (stop from Webots GUI)")
        print(f"Duration: {duration_label} | Robots: {self.num_robots}")
        print(f"Collision avoidance: 10 s trajectory prediction + path-segment braking + deadlock recovery")
        print(f"Scan interval: 1 s | Prediction: 10 s sampled every 0.5 s | Collision radius: 0.5 m")
        print(f"{'='*60}")
        print(f"Scheduler: {self.scheduler.name}")
        print("Runtime RHCR/CBS: " +
              ("enabled (experimental)" if ENABLE_RUNTIME_RHCR else
               "disabled (read-only candidate; avoids synchronous stalls)"))
        print("Legacy 1.5s interlock recovery: " +
              ("enabled (experimental)" if ENABLE_LEGACY_INTERLOCK_RECOVERY
               else ("disabled (3s replan + "
                     f"{STALL_RELOCATION_TIMEOUT:g}s safe relocation remain active)")))
        print("Non-physical teleport recovery: " +
              ("enabled (diagnostic runs only)"
               if ENABLE_NONPHYSICAL_RECOVERY else
               "disabled (physical retreat/replan only)"))
        print(f"{'='*60}\n")
        
        dt = self.timestep / 1000.0  # Convert ms to seconds
        
        completed_normally = False
        while self.running:
            step_status = self.supervisor.step(self.timestep)
            if step_status == -1:
                break
            self.sim_time += dt
            self.step_count += 1
            self.motion_coordinator.set_sim_time(self.sim_time)
            
            # Check simulation end
            if AUTO_STOP_SIMULATION and self.sim_time >= SIM_DURATION:
                self.running = False
                completed_normally = True
                break
            
            # 1. Get robot positions from Webots
            self._get_robot_positions_from_webots()
            
            # 1.5 Broadcast all robot positions for peer conflict avoidance
            # EVERY STEP �?peer avoidance is safety-critical, no delays allowed.
            # At 32ms timestep, peer positions must be as fresh as possible
            # to give robots maximum reaction time before collision.
            self._broadcast_peer_positions()
            
            # 2. Receive messages from robots
            self._receive_messages()
            self._check_startup_timeout()
            
            # 2.5 Dispatch delayed robots (temporal conflict resolution)
            self._dispatch_delayed_robots()
            
            # 3. Update robot states (incl. battery, movement tracking)
            self._update_robot_states(dt)
            self._check_joint_business_arrivals()

            if self._debug_state_enabled and self.step_count % 16 == 0:
                try:
                    with open(self._debug_state_path, 'a',
                              encoding='utf-8') as handle:
                        for rid in sorted(self.robots):
                            robot = self.robots[rid]
                            goal = self._navigation_goal(robot)
                            goal_xy = self._goal_coordinates(goal)
                            handle.write(
                                f'{self.sim_time:.3f},{rid},{robot.state.name},'
                                f'{robot.position[0]:.3f},{robot.position[1]:.3f},'
                                f'{goal_xy[0] if goal_xy else 0:.3f},'
                                f'{goal_xy[1] if goal_xy else 0:.3f},'
                                f'{robot.active_plan_source},'
                                f'{robot.active_plan_epoch},'
                                f'{robot.controller_active_plan_epoch},'
                                f'{robot.controller_waypoint_index},'
                                f'{len(robot.waypoints)},'
                                f'{robot.current_task.task_id if robot.current_task else None},'
                                f'{robot.hold_until:.3f},{robot.speed_scale:.2f}\n')
                except Exception as debug_error:
                    print(f'[DEBUG_STATES] write failed: {debug_error}')

            # 3.4 Joint-runtime safety net: coordinated progress and
            # predicted-collision intervention. Legacy per-robot replan is
            # disabled in joint mode, so this is the only source of recovery.
            if not hasattr(self, '_next_joint_watchdog_tick'):
                self._next_joint_watchdog_tick = self.sim_time
            if (ENABLE_JOINT_RUNTIME and
                    self.sim_time >= self._next_joint_watchdog_tick):
                self._joint_runtime_watchdog()
                self._next_joint_watchdog_tick += 0.5

            # 3.5 Advance lifelong planner clock once per ~1.5 s of
            # sim-time. This corresponds roughly to one graph edge
            # traversal at 0.22 m/s and 2 m edge length.
            if not hasattr(self, '_next_ll_tick'):
                self._next_ll_tick = 1.5
            if self.sim_time >= self._next_ll_tick:
                self.motion_coordinator.lifelong_tick(1)
                self._next_ll_tick += 1.5

            # Re-anchor timed grid reservations to measured progress. This
            # prevents nominal-speed windows expiring while DWA or a legal
            # hold has slowed the physical robot.
            if not hasattr(self, '_next_reservation_refresh'):
                self._next_reservation_refresh = self.sim_time
            if self.sim_time >= self._next_reservation_refresh:
                for rid, robot in self.robots.items():
                    if (robot.waypoints and
                            robot.current_waypoint_idx < len(robot.waypoints)):
                        not_before = max(
                            robot.dispatch_not_before,
                            robot.hold_until,
                            robot.controller_paused_until)
                        self.motion_coordinator.refresh_active_plan_timing(
                            rid, robot.position,
                            robot.waypoints[robot.current_waypoint_idx:],
                            not_before=not_before)
                self._next_reservation_refresh += 0.5
            
            # 3.6 Lazy relocation �?IDLE robots not at a rest node
            # walk to nearest one. Runs every ~0.5s to avoid spamming
            # plan() calls.
            if not hasattr(self, '_next_reloc_tick'):
                self._next_reloc_tick = 1.0
            if self.sim_time >= self._next_reloc_tick:
                self._relocate_idle_robots()
                self._next_reloc_tick += 0.5
            
            # 3.6b Progress monitor �?detect stalled robots and replan
            if not hasattr(self, '_next_conflict_scan'):
                self._next_conflict_scan = self.sim_time
            if (not ENABLE_JOINT_RUNTIME and
                    self.sim_time >= self._next_conflict_scan):
                self._proactive_path_conflict_scan()
                self._next_conflict_scan += 0.25
            if not hasattr(self, '_next_joint_grid_tick'):
                self._next_joint_grid_tick = 2.0
            if self.sim_time >= self._next_joint_grid_tick:
                if ENABLE_JOINT_RUNTIME:
                    self._refresh_joint_grid_candidate()
                self._next_joint_grid_tick += 2.0
            if (os.environ.get('SMART_FACTORY_TXN_SMOKE', '0') == '1' and
                    not getattr(self, '_joint_txn_smoke_attempted', False) and
                    self.sim_time >= 5.0):
                smoke_plans = {
                    rid: list(robot.waypoints[robot.current_waypoint_idx:])
                    for rid, robot in self.robots.items()
                    if self._has_active_navigation(robot)
                }
                if len(smoke_plans) >= 2:
                    self._joint_txn_smoke_attempted = True
                    self._begin_joint_plan_transaction(smoke_plans)
            self._advance_joint_plan_transaction()
            self._advance_joint_execution_barrier()
            if not hasattr(self, '_next_progress_tick'):
                self._next_progress_tick = 2.0
            if (not ENABLE_JOINT_RUNTIME and
                    self.sim_time >= self._next_progress_tick):
                self._monitor_progress_and_replan()
                self._recover_long_stalled_robots()
                if ENABLE_LEGACY_INTERLOCK_RECOVERY:
                    self._resolve_multi_robot_deadlock()
                self._next_progress_tick += 1.0  # check every 1s
            
            # 3.7 Level 2 �?Deadlock detection + priority inheritance.
            # Every ~0.5s scan for stuck robots; force lower-priority
            # ones to yield when 2+ are blocking each other.
            if not hasattr(self, '_next_deadlock_tick'):
                self.motion_coordinator.init_deadlock_monitor()
                self._next_deadlock_tick = 1.0
            if (not ENABLE_JOINT_RUNTIME and
                    self.sim_time >= self._next_deadlock_tick):
                rs_dict = {rid: {
                    "position": r.position,
                    "state": (r.state if self._robot_should_be_moving(r)
                              else RobotState.WAITING),
                    "goal_location": getattr(r, 'goal_location', None),
                    "speed_scale": r.speed_scale,
                } for rid, r in self.robots.items()}
                stuck = self.motion_coordinator.update_deadlock_monitor(rs_dict)
                if stuck:
                    broken = self.motion_coordinator.break_deadlock(
                        stuck, rs_dict)
                    for rid in broken:
                        rb = self.robots[rid]
                        if rb.current_task and getattr(rb, 'goal_location', None):
                            new_p = self.motion_coordinator.plan_grid_lifelong(
                                rid, rb.position, rb.goal_location)
                            if new_p:
                                self._install_runtime_plan(rid, new_p)
                self._next_deadlock_tick += 0.5
            
            # 3.8 Level 3 �?Periodic RHCR (every 5 sim-seconds).
            # Global CBS replan accepts only if total cost decreases.
            if not hasattr(self, '_next_rhcr_tick'):
                self._next_rhcr_tick = 5.0
            if ENABLE_RUNTIME_RHCR and self.sim_time >= self._next_rhcr_tick:
                active_goals = {}
                rs_dict_rhcr = {}
                for rid, r in self.robots.items():
                    if r.current_task and getattr(r, 'goal_location', None):
                        active_goals[rid] = r.goal_location
                        rs_dict_rhcr[rid] = {"position": r.position}
                if len(active_goals) >= 2:
                    self.motion_coordinator.rhcr_replan(
                        rs_dict_rhcr, active_goals)
                self._next_rhcr_tick += 5.0
            elif not ENABLE_RUNTIME_RHCR:
                # Keep the deadline ahead of simulation time so enabling the
                # feature in a future fresh run does not require catch-up.
                self._next_rhcr_tick = self.sim_time + 5.0
            
            # 4. Generate new tasks
            new_task = self.task_generator.update(self.sim_time)
            if new_task:
                print(f"[T={self.sim_time:.1f}] New task {new_task.task_id}: "
                      f"{new_task.pickup_location} -> {new_task.delivery_location}")
                self.metrics.record_task_arrival(new_task, self.sim_time)
            
            # 5. Assign tasks to robots
            self._assign_tasks()
            
            # 6. Check for deadlocks (every 5 seconds)
            if (not ENABLE_JOINT_RUNTIME and
                    self.step_count % int(5.0 / dt) == 0):
                self._check_deadlocks()
            
            # 7. Record metrics
            if self.step_count % LOG_INTERVAL == 0:
                self._log_status()
                self.metrics.record_step(
                    self.sim_time,
                    {rid: r.to_dict() for rid, r in self.robots.items()},
                    self.task_generator.get_statistics(),
                    self.motion_coordinator.get_statistics()
                )
        
        if not completed_normally:
            # Webots returned -1 before the requested horizon (for example a
            # batch-process timeout, GUI stop/reset, or external termination).
            # Do not publish a short partial run as a valid experiment result.
            run_mode = "batch" if AUTO_STOP_SIMULATION else "interactive"
            print(f"[Supervisor] {run_mode} run ended at "
                  f"{self.sim_time:.3f}s. Results not saved.")
            return

        # Simulation complete
        self._finalize()
        # In batch mode the other robot controllers keep Webots alive after
        # the supervisor returns. Explicitly terminate the simulation once
        # results are flushed so automated validation has a reliable exit.
        if AUTO_STOP_SIMULATION and hasattr(self.supervisor, "simulationQuit"):
            self.supervisor.simulationQuit(0)

    def _log_status(self):
        """Print periodic status update."""
        task_stats = self.task_generator.get_statistics()
        coord_stats = self.motion_coordinator.get_statistics()
        
        idle_count = sum(1 for r in self.robots.values() if r.state == RobotState.IDLE)
        active_count = sum(1 for r in self.robots.values() if r.state != RobotState.IDLE)
        
        print(f"[T={self.sim_time:.1f}s] "
              f"Tasks: {task_stats['completed']}/{task_stats['total_generated']} completed | "
              f"Pending: {task_stats['pending']} | "
              f"Robots idle/active: {idle_count}/{active_count} | "
              f"Conflicts: {coord_stats['conflicts_resolved']}")
        for rid, robot in self.robots.items():
            if robot.state in (RobotState.RETURNING_TO_CHARGE,
                               RobotState.CHARGING,
                               RobotState.WAITING):
                print(f"  [BatteryState] Robot {rid}: state={robot.state} "
                      f"goal={robot.goal_location or '-'} "
                      f"battery={robot.battery:.1f}% "
                      f"waypoint={robot.current_waypoint_idx}/"
                      f"{len(robot.waypoints)}")

    def _finalize(self):
        """Finalize simulation and save results."""
        print(f"\n{'='*60}")
        print(f"Simulation Complete: Scenario {self.scenario_name}")
        print(f"{'='*60}")
        
        # Final statistics
        task_stats = self.task_generator.get_statistics()
        coord_stats = self.motion_coordinator.get_statistics()
        
        print(f"\n--- Task Statistics ---")
        print(f"  Total generated: {task_stats['total_generated']}")
        print(f"  Completed:       {task_stats['completed']}")
        print(f"  Pending:         {task_stats['pending']}")
        print(f"  Throughput:      {task_stats['throughput']} tasks")
        print(f"  Avg completion:  {task_stats['avg_completion_time']:.2f}s")
        print(f"  Avg wait time:   {task_stats['avg_waiting_time']:.2f}s")
        
        print(f"\n--- Coordination Statistics ---")
        print(f"  Conflicts detected:  {coord_stats['conflicts_detected']}")
        print(f"  Conflicts resolved:  {coord_stats['conflicts_resolved']}")
        print(f"  Deadlocks detected:  {coord_stats['deadlocks_detected']}")
        print(f"  Total re-plans:      {coord_stats['total_replans']}")
        
        print(f"\n--- Robot Statistics ---")
        for rid, robot in self.robots.items():
            idle_pct = (robot.idle_time / max(self.sim_time, 1)) * 100
            print(f"  Robot {rid}: completed={robot.tasks_completed}, "
                  f"distance={robot.total_distance:.1f}m, "
                  f"idle={idle_pct:.1f}%, battery={robot.battery:.1f}%")
        
        # Save detailed results
        self.metrics.save_results(
            self.robots, task_stats, coord_stats, self.sim_time
        )
        
        print(f"\nResults saved to: {self.metrics.output_path}")


# ================================================================
# MAIN ENTRY POINT
# ================================================================

def main():
    """Main entry point for the factory supervisor controller."""
    # Parse arguments from Webots controllerArgs or environment
    scenario = os.environ.get("SCENARIO", "C")
    scheduler_type = os.environ.get("SCHEDULER", "FCFS")
    seed = int(os.environ.get("SEED", "42"))
    model_path = os.environ.get("MODEL_PATH", None)
    
    # Override from command line args if available
    if len(sys.argv) > 1:
        scenario = sys.argv[1]
    if len(sys.argv) > 2:
        scheduler_type = sys.argv[2]
    if len(sys.argv) > 3:
        seed = int(sys.argv[3])
    if len(sys.argv) > 4:
        model_path = sys.argv[4]

    # The world carries controllerArgs ["C"] as its interactive default.
    # Batch experiments set SCENARIO explicitly and must be able to override
    # that world default so A/B/C comparisons actually run distinct fleets.
    scenario = os.environ.get("SCENARIO", scenario)
    
    # Create and run supervisor
    supervisor = FactorySupervisor(
        scenario=scenario,
        scheduler_type=scheduler_type,
        seed=seed,
        model_path=model_path
    )
    supervisor.run()


if __name__ == "__main__":
    main()
