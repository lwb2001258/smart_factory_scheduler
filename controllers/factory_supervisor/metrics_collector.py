"""
Metrics Collector for Smart Factory Simulation.

Collects, aggregates, and saves performance metrics for
experimental evaluation and comparison between scheduling methods.

Key Performance Indicators (KPIs):
1. System throughput (tasks completed per unit time)
2. Average task completion time
3. Robot idle time percentage
4. Number of coordination conflicts
5. Total distance travelled
"""

import json
import math
import os
import statistics
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from task_generator import TransportTask


@dataclass
class StepRecord:
    """Record of metrics at a single simulation timestep."""
    sim_time: float
    tasks_completed: int
    tasks_pending: int
    tasks_active: int
    robots_idle: int
    robots_active: int
    conflicts_total: int
    avg_battery: float
    min_pair_distance: Optional[float]
    robot_positions: Optional[dict] = None
    robot_task_states: Optional[dict] = None


@dataclass
class SafetyEvent:
    event_id: str
    sim_time: float
    event_type: str
    robot_id: int
    peer_id: Optional[int] = None
    path_version: int = 0
    decision: str = ""
    minimum_distance: Optional[float] = None
    ttc: Optional[float] = None


class MetricsCollector:
    """
    Collects and manages all performance metrics during simulation.
    """
    
    def __init__(self, scenario_name: str, scheduler_name: str, num_robots: int,
                 provenance: Optional[dict] = None):
        self.scenario_name = scenario_name
        self.scheduler_name = scheduler_name
        self.num_robots = num_robots
        self.provenance = dict(provenance or {})
        
        # Time-series data
        self.step_records: List[StepRecord] = []
        
        # Task-level data
        self.task_arrivals: List[dict] = []
        self.task_completions: List[dict] = []
        
        # Conflict/deadlock events
        self.conflict_events: List[dict] = []
        self.route_dispatch_events: List[dict] = []
        self.route_override_events: List[dict] = []
        self._active_conflicts: Dict[tuple, dict] = {}
        self._next_conflict_id = 1
        self._last_route_dispatch: Dict[int, dict] = {}
        self.conflict_episode_count = 0
        self.route_dispatch_count = 0
        self.route_override_count = 0
        self.unauthorized_route_write_count = 0
        self.unauthorized_route_write_events: List[dict] = []
        self.replan_events: List[dict] = []
        self.escape_events: List[dict] = []
        self.unplanned_stop_events: List[dict] = []
        self.replan_count = 0
        self.replan_request_count = 0
        self.replan_request_events: List[dict] = []
        self.joint_plan_requests_by_reason: Dict[str, int] = {}
        self.joint_plan_requests_by_class: Dict[str, int] = {}
        self.escape_count = 0
        self.unplanned_stop_count = 0
        self._motion_watch: Dict[int, dict] = {}
        self.planned_wait_events: List[dict] = []
        self.planned_wait_count = 0
        self.planned_wait_total_seconds = 0.0
        self._active_planned_waits: Dict[int, dict] = {}
        self.yield_resume_events: List[dict] = []
        self.yield_start_count = 0
        self.yield_standoff_selected_count = 0
        self.yield_wait_start_count = 0
        self.yield_wait_cleared_count = 0
        self.yield_wait_timeout_count = 0
        self.yield_resume_proposed_count = 0
        self.yield_resume_rejected_count = 0
        self.yield_resume_committed_count = 0
        self.yield_resume_rollback_count = 0
        self.nonphysical_recoveries = 0
        self.nonphysical_recovery_events: List[dict] = []
        self.safety_event_count = 0
        self.minimum_pair_distance_seen = float('inf')
        self.pair_distance_violation_samples = 0
        self.deadlock_events: List[dict] = []
        self.safety_events: List[dict] = []
        self._last_safety_event: Dict[tuple, float] = {}
        self.scheduling_latencies_ms: List[float] = []
        self.supervisor_step_wall_ms: List[float] = []
        self.supervisor_webots_step_wall_ms: List[float] = []
        self.phase_wall_ms: Dict[str, List[float]] = {}
        self.joint_planning_wall_ms: List[float] = []
        self.joint_planning_tier_wall_ms: Dict[str, List[float]] = {}
        self.robot_dwa_wall_ms: List[float] = []
        self.joint_equivalent_refreshes_suppressed = 0
        self.communication_wall_ms: Dict[str, List[float]] = {}
        self.communication_messages: Dict[str, int] = {}
        self.communication_bytes: Dict[str, int] = {}
        self.invalid_scheduler_outputs = 0
        self.scheduler_fallbacks = 0
        self.native_scheduler_commits = 0
        self.fallback_scheduler_commits = 0
        self.commits_by_algorithm: Dict[str, int] = {}
        self.rl_inference_latencies_ms: List[float] = []
        self.rl_timeout_count = 0
        self.rl_policy_decisions = 0
        self.rl_fallback_decisions = 0
        self.rl_event_ledger: List[dict] = []
        self.rl_decisions: List[dict] = []
        self.joint_cell_traversal_seconds: List[float] = []
        self.joint_window_gap_events: List[dict] = []
        self.reservation_expired_movement_events: List[dict] = []
        self.motion_continuity_by_robot: Dict[int, dict] = {}
        self.motion_continuity_events: List[dict] = []
        self.progress_lease_events: List[dict] = []
        self.terminal_handoff_events: List[dict] = []

        # Output path
        self.output_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "results"
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.output_path = os.path.join(
            self.output_dir,
            f"experiment_{scenario_name}_{scheduler_name}_{timestamp}.json"
        )

    def record_rl_event(self, event: dict):
        """Record a canonical, JSON-safe RL event emitted by runtime code."""
        if isinstance(event, dict):
            self.rl_event_ledger.append(dict(event))
            if len(self.rl_event_ledger) > 10000:
                del self.rl_event_ledger[:1000]

    def record_rl_decision(self, decision: dict):
        """Record a committed policy decision for offline Webots training."""
        if isinstance(decision, dict):
            self.rl_decisions.append(dict(decision))

    def record_terminal_handoff(self, event_type: str, sim_time: float,
                                robot_id: int, terminal: str, epoch: int,
                                owner_id=None, reason=None,
                                attempt=None) -> None:
        """Record transition-only terminal ownership/handoff evidence."""
        self.terminal_handoff_events.append({
            'event_type': str(event_type), 'sim_time': float(sim_time),
            'robot_id': int(robot_id), 'terminal': str(terminal),
            'terminal_epoch': int(epoch), 'owner_id': owner_id,
            'reason': reason,
            'attempt': (int(attempt) if attempt is not None else None),
        })
        self._cap_events(self.terminal_handoff_events)

    def record_scheduling_latency(self, seconds: float):
        if math.isfinite(seconds) and seconds >= 0:
            self.scheduling_latencies_ms.append(seconds * 1000.0)

    def record_supervisor_step_wall(self, seconds: float) -> None:
        """Record one complete synchronous Webots/Supervisor iteration."""
        value = float(seconds) * 1000.0
        if math.isfinite(value) and value >= 0.0:
            self.supervisor_step_wall_ms.append(value)

    def record_supervisor_webots_step_wall(self, seconds: float) -> None:
        """Record only the blocking Webots `Supervisor.step()` call."""
        value = float(seconds) * 1000.0
        if math.isfinite(value) and value >= 0.0:
            self.supervisor_webots_step_wall_ms.append(value)

    def record_phase_wall(self, phase: str, seconds: float) -> None:
        """Record one named Supervisor Python phase for hotspot evidence."""
        value = float(seconds) * 1000.0
        if math.isfinite(value) and value >= 0.0:
            self.phase_wall_ms.setdefault(str(phase), []).append(value)

    def record_joint_planning_wall(self, seconds: float) -> None:
        """Record wall time spent building one joint candidate."""
        value = float(seconds) * 1000.0
        if math.isfinite(value) and value >= 0.0:
            self.joint_planning_wall_ms.append(value)

    def record_joint_planning_tier_wall(self, tier: str,
                                        seconds: float) -> None:
        """Record one bounded joint-grid search tier for hotspot evidence."""
        value = float(seconds) * 1000.0
        if math.isfinite(value) and value >= 0.0:
            self.joint_planning_tier_wall_ms.setdefault(
                str(tier), []).append(value)

    def record_robot_dwa_wall(self, seconds: float) -> None:
        """Record a reported DWA wall time sample from robot telemetry."""
        value = float(seconds) * 1000.0
        if math.isfinite(value) and value >= 0.0:
            self.robot_dwa_wall_ms.append(value)

    def record_joint_refresh_suppressed(self) -> None:
        """Count equivalent candidates discarded without route churn."""
        self.joint_equivalent_refreshes_suppressed += 1

    def record_joint_plan_request(self, reason: str,
                                  request_class: str) -> None:
        """Count accepted edge-triggered requests without hot-path I/O."""
        reason_key = str(reason or 'unknown')
        class_key = str(request_class or 'unknown')
        self.joint_plan_requests_by_reason[reason_key] = (
            self.joint_plan_requests_by_reason.get(reason_key, 0) + 1)
        self.joint_plan_requests_by_class[class_key] = (
            self.joint_plan_requests_by_class.get(class_key, 0) + 1)

    def record_communication(self, channel: str, seconds: float,
                             messages: int = 0, byte_count: int = 0) -> None:
        """Accumulate bounded in-memory communication cost evidence."""
        value = float(seconds) * 1000.0
        if not math.isfinite(value) or value < 0.0:
            return
        key = str(channel)
        self.communication_wall_ms.setdefault(key, []).append(value)
        self.communication_messages[key] = (
            self.communication_messages.get(key, 0) + int(messages))
        self.communication_bytes[key] = (
            self.communication_bytes.get(key, 0) + int(byte_count))

    def record_scheduler_fallback(self, invalid_output: bool = True):
        self.scheduler_fallbacks += 1
        if invalid_output:
            self.invalid_scheduler_outputs += 1

    def record_rl_diagnostics(self, diagnostics: Optional[dict]):
        """Capture safety-wrapper diagnostics for auditable RL evaluation."""
        diagnostics = diagnostics or {}
        inference_ms = diagnostics.get("inference_ms")
        if isinstance(inference_ms, (int, float)) and math.isfinite(inference_ms):
            self.rl_inference_latencies_ms.append(float(inference_ms))
        self.rl_timeout_count = max(
            self.rl_timeout_count, int(diagnostics.get("timeout_count", 0)))
        self.rl_policy_decisions = max(
            self.rl_policy_decisions,
            int(diagnostics.get("policy_decisions", 0)))
        self.rl_fallback_decisions = max(
            self.rl_fallback_decisions,
            int(diagnostics.get("fallback_decisions", 0)))

    def record_scheduler_commit(self, algorithm_name: str,
                                native: bool = True):
        """Attribute committed work to the algorithm that actually chose it."""
        if native:
            self.native_scheduler_commits += 1
        else:
            self.fallback_scheduler_commits += 1
        name = str(algorithm_name or "unknown")
        self.commits_by_algorithm[name] = (
            self.commits_by_algorithm.get(name, 0) + 1)

    def record_motion_telemetry(self, sim_time: float, robot_id: int,
                                status: dict):
        """Integrate ordered controller samples in O(1) per status packet."""
        rid = int(robot_id)
        try:
            seq = int(status.get('status_seq', -1))
            sample_time = float(status.get('status_sample_time', float('nan')))
            linear = abs(float(status.get('commanded_linear_speed', 0.0)))
            angular = abs(float(status.get('commanded_angular_speed', 0.0)))
        except (TypeError, ValueError):
            return
        if seq < 1 or not all(math.isfinite(value) for value in (
                sample_time, linear, angular)):
            return
        state = self.motion_continuity_by_robot.setdefault(rid, {
            'session': str(status.get('status_session', 'legacy')),
            'last_seq': 0, 'last_sample_time': sample_time,
            'status_samples': 0, 'status_drops': 0,
            'eligible_seconds': 0.0, 'moving_seconds': 0.0,
            'turning_seconds': 0.0, 'valid_wait_seconds': 0.0,
            'unblocked_zero_speed_seconds': 0.0,
            'unblocked_stop_episodes': 0,
            'unblocked_stop_started_at': None,
            'unblocked_stop_reason': None,
        })
        session = str(status.get('status_session', 'legacy'))
        if session != state['session']:
            state['session'] = session
            state['last_seq'] = 0
            state['last_sample_time'] = sample_time
            state['unblocked_stop_started_at'] = None
            state['unblocked_stop_reason'] = None

        if seq <= state['last_seq'] or sample_time < state['last_sample_time']:
            return
        contiguous = state['last_seq'] == 0 or seq == state['last_seq'] + 1
        if state['last_seq']:
            state['status_drops'] += max(0, seq - state['last_seq'] - 1)
        dt = sample_time - state['last_sample_time']
        state['last_seq'] = seq
        state['last_sample_time'] = sample_time
        state['status_samples'] += 1

        if status.get('dwa_control_active') is True:
            try:
                dwa_ms = float(status.get('dwa_control_wall_ms', 0.0))
            except (TypeError, ValueError):
                dwa_ms = 0.0
            if math.isfinite(dwa_ms) and dwa_ms >= 0.0:
                self.record_robot_dwa_wall(dwa_ms / 1000.0)

        # Missing intervals are intentionally not classified as clear/moving.
        if not contiguous or dt <= 0.0 or dt > 0.5:
            # Never bridge an episode across an unobserved interval.
            state['unblocked_stop_started_at'] = None
            return
        eligible = status.get('navigating') is True
        emergency = status.get('emergency_braking') is True
        moving = linear >= 0.03
        turning = not moving and angular >= 0.10
        # Until Step 5A-2 supplies validator evidence, a fixed-slot wait is
        # observable but cannot receive a valid-conflict exemption.
        wait_evidence = status.get('wait_validator_evidence')
        try:
            evidence_until = float(
                wait_evidence.get('valid_until', 0.0))
        except (AttributeError, TypeError, ValueError):
            evidence_until = float('-inf')
        valid_wait = (
            isinstance(wait_evidence, dict) and
            wait_evidence.get('validated') is True and
            math.isfinite(evidence_until) and
            evidence_until >= sample_time)
        unblocked_zero = eligible and not emergency and not moving and (
            not turning) and not valid_wait
        if eligible:
            state['eligible_seconds'] += dt
            if moving:
                state['moving_seconds'] += dt
            elif turning:
                state['turning_seconds'] += dt
            elif valid_wait:
                state['valid_wait_seconds'] += dt
            elif unblocked_zero:
                state['unblocked_zero_speed_seconds'] += dt

        started = state['unblocked_stop_started_at']
        if unblocked_zero and started is None:
            changed_at = status.get('command_motion_changed_at', sample_time)
            try:
                changed_at = float(changed_at)
            except (TypeError, ValueError):
                changed_at = sample_time
            if not math.isfinite(changed_at) or not 0.0 <= changed_at <= sample_time:
                changed_at = sample_time
            # The actuator may have remained at zero through an intervening
            # validated wait. That excluded interval must cut the unblocked
            # episode even though the motor transition timestamp is older.
            state['unblocked_stop_started_at'] = max(
                changed_at, sample_time - dt)
            state['unblocked_stop_reason'] = status.get(
                'control_stop_reason') or status.get(
                    'planned_wait_reason') or 'unclassified_zero_command'
        elif not unblocked_zero and started is not None:
            changed_at = status.get('command_motion_changed_at', sample_time)
            try:
                ended_at = float(changed_at)
            except (TypeError, ValueError):
                ended_at = sample_time
            if not math.isfinite(ended_at) or not started <= ended_at <= sample_time:
                ended_at = sample_time
            duration = max(0.0, ended_at - started)
            if duration > 0.5:
                state['unblocked_stop_episodes'] += 1
                self.motion_continuity_events.append({
                    'robot_id': rid, 'started_at': started,
                    'ended_at': ended_at, 'duration': duration,
                    'reason': state['unblocked_stop_reason'],
                })
                self._cap_events(self.motion_continuity_events)
            state['unblocked_stop_started_at'] = None
            state['unblocked_stop_reason'] = None

    def record_progress_lease_event(self, sim_time: float, robot_id: int,
                                    generation: int, stage: str,
                                    elapsed: float) -> None:
        """Record one transition of the cross-epoch liveness lease."""
        self.progress_lease_events.append({
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'lease_generation': int(generation), 'stage': str(stage),
            'no_progress_seconds': float(elapsed),
        })
        self._cap_events(self.progress_lease_events)

    def record_unauthorized_route_write(self, sim_time: float, robot_id: int,
                                        source: str, path_version: int,
                                        plan_epoch=None):
        self.unauthorized_route_write_count += 1
        self.unauthorized_route_write_events.append({
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'source': str(source), 'path_version': int(path_version),
            'plan_epoch': plan_epoch, 'decision': 'rejected',
        })
        self._cap_events(self.unauthorized_route_write_events)

    @staticmethod
    def _percentile(values: List[float], percentile: float) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        index = min(len(ordered) - 1,
                    max(0, math.ceil(percentile * len(ordered)) - 1))
        return ordered[index]
    
    def record_step(self, sim_time: float, robot_states: Dict[int, dict],
                    task_stats: dict, coord_stats: dict):
        """Record metrics at a simulation step."""
        from config import RobotState

        task_motion_states = {
            RobotState.EN_ROUTE_PICKUP, RobotState.CARRYING,
            RobotState.EN_ROUTE_DELIVERY,
        }
        for rid, state in robot_states.items():
            position = state.get('position')
            active = state.get('state') in task_motion_states
            planned_wait = state.get('planned_wait') is True
            wait_episode = self._active_planned_waits.get(rid)
            wait_ended = False
            if planned_wait:
                wait_identity = (
                    state.get('task_id'), state.get('plan_epoch'),
                    state.get('plan_source'),
                    int(state.get('path_version', 0)),
                    state.get('wait_reason'))
                if wait_episode is not None:
                    episode_identity = (
                        wait_episode.get('task_id'),
                        wait_episode.get('plan_epoch'),
                        wait_episode.get('plan_source'),
                        wait_episode.get('path_version'),
                        wait_episode.get('reason'))
                    if episode_identity != wait_identity:
                        wait_episode['ended_at'] = float(sim_time)
                        wait_episode['duration'] = max(
                            0.0, float(sim_time) -
                            wait_episode['started_at'])
                        wait_episode['outcome'] = 'superseded'
                        self.planned_wait_total_seconds += (
                            wait_episode['duration'])
                        self.planned_wait_events.append(wait_episode)
                        self._cap_events(self.planned_wait_events)
                        self._active_planned_waits.pop(rid, None)
                        wait_episode = None
                deadline = state.get('wait_deadline')
                if not isinstance(deadline, (int, float)) or not math.isfinite(
                        deadline):
                    deadline = None
                if wait_episode is None:
                    wait_episode = {
                        'robot_id': rid,
                        'started_at': float(sim_time),
                        'last_seen_at': float(sim_time),
                        'reason': state.get('wait_reason'),
                        'deadline': deadline,
                        'max_deadline': deadline,
                        'task_id': state.get('task_id'),
                        'plan_epoch': state.get('plan_epoch'),
                        'plan_source': state.get('plan_source'),
                        'path_version': int(state.get('path_version', 0)),
                        'goal_location': state.get('goal_location'),
                        'ended_at': None,
                        'duration': None,
                        'outcome': 'active',
                    }
                    self._active_planned_waits[rid] = wait_episode
                    self.planned_wait_count += 1
                else:
                    wait_episode['last_seen_at'] = float(sim_time)
                    if deadline is not None:
                        previous = wait_episode.get('max_deadline')
                        wait_episode['max_deadline'] = (
                            deadline if previous is None else
                            max(float(previous), deadline))
            elif wait_episode is not None:
                wait_episode = self._active_planned_waits.pop(rid)
                wait_episode['ended_at'] = float(sim_time)
                wait_episode['duration'] = max(
                    0.0, float(sim_time) - wait_episode['started_at'])
                max_deadline = wait_episode.get('max_deadline')
                wait_episode['outcome'] = (
                    'deadline_expired'
                    if max_deadline is not None and sim_time >= max_deadline
                    else 'resumed')
                self.planned_wait_total_seconds += wait_episode['duration']
                self.planned_wait_events.append(wait_episode)
                self._cap_events(self.planned_wait_events)
                wait_ended = True
            watch = self._motion_watch.get(rid)
            if not active or planned_wait or position is None:
                self._motion_watch[rid] = {
                    'position': position, 'progress_at': sim_time,
                    'reported': False}
                continue
            if wait_ended:
                self._motion_watch[rid] = {
                    'position': tuple(position), 'progress_at': sim_time,
                    'reported': False}
                continue
            if watch is None or watch.get('position') is None:
                self._motion_watch[rid] = {
                    'position': tuple(position), 'progress_at': sim_time,
                    'reported': False}
                continue
            distance = math.hypot(
                position[0] - watch['position'][0],
                position[1] - watch['position'][1])
            if distance >= 0.05:
                watch.update(position=tuple(position), progress_at=sim_time,
                             reported=False)
            elif (not watch['reported'] and
                  sim_time - watch['progress_at'] >= 10.0):
                event = {
                    'sim_time': sim_time, 'robot_id': rid,
                    'state': str(state.get('state')),
                    'stopped_seconds': sim_time - watch['progress_at'],
                    'position': list(position),
                    'path_version': int(state.get('path_version', 0)),
                    'plan_epoch': state.get('plan_epoch'),
                    'plan_source': state.get('plan_source'),
                    'task_id': state.get('task_id'),
                    'goal_location': state.get('goal_location'),
                    'planned_wait': False,
                }
                self.unplanned_stop_events.append(event)
                self.unplanned_stop_count += 1
                self._cap_events(self.unplanned_stop_events)
                watch['reported'] = True
        
        idle_count = sum(1 for rs in robot_states.values() 
                        if rs['state'] == RobotState.IDLE)
        active_count = len(robot_states) - idle_count
        avg_battery = sum(rs.get('battery', 100) for rs in robot_states.values()) / max(len(robot_states), 1)
        pair_distances = []
        robot_ids = list(robot_states)
        for index, rid_a in enumerate(robot_ids):
            pos_a = robot_states[rid_a].get('position')
            if pos_a is None:
                continue
            for rid_b in robot_ids[index + 1:]:
                pos_b = robot_states[rid_b].get('position')
                if pos_b is not None:
                    distance = ((pos_a[0] - pos_b[0]) ** 2 +
                                (pos_a[1] - pos_b[1]) ** 2) ** 0.5
                    pair_distances.append((distance, rid_a, rid_b))

        closest = min(pair_distances, default=(float('inf'), None, None))
        self.minimum_pair_distance_seen = min(
            self.minimum_pair_distance_seen, closest[0])
        if closest[0] < 0.50:
            self.pair_distance_violation_samples += 1
        for distance, rid_a, rid_b in pair_distances:
            if distance >= 0.50:
                continue
            self.record_safety_event(SafetyEvent(
                event_id=f"distance-{rid_a}-{rid_b}-{sim_time:.3f}",
                sim_time=sim_time,
                event_type="pair_distance_violation",
                robot_id=rid_a,
                peer_id=rid_b,
                decision="observed",
                minimum_distance=distance,
            ))
        
        record = StepRecord(
            sim_time=sim_time,
            tasks_completed=task_stats.get('completed', 0),
            tasks_pending=task_stats.get('pending', 0),
            tasks_active=task_stats.get('active', 0),
            robots_idle=idle_count,
            robots_active=active_count,
            conflicts_total=coord_stats.get('conflicts_resolved', 0),
            avg_battery=avg_battery,
            min_pair_distance=(closest[0]
                               if math.isfinite(closest[0]) else None),
            robot_positions={
                str(rid): [float(rs['position'][0]), float(rs['position'][1])]
                for rid, rs in robot_states.items()
                if rs.get('position') is not None
            },
            robot_task_states={
                str(rid): {
                    'state': str(rs.get('state')),
                    'task_id': rs.get('task_id'),
                    'goal_location': rs.get('goal_location'),
                    'plan_source': rs.get('plan_source'),
                    'plan_epoch': rs.get('plan_epoch'),
                    'path_version': int(rs.get('path_version', 0)),
                    'controller_waypoint_index': int(rs.get(
                        'controller_waypoint_index', 0)),
                    'planned_wait': bool(rs.get('planned_wait', False)),
                    'wait_reason': rs.get('wait_reason'),
                    'emergency_braking': bool(rs.get(
                        'emergency_braking', False)),
                    'speed_scale': float(rs.get('speed_scale', 1.0)),
                    'controller_motion_state': str(rs.get(
                        'controller_motion_state', 'unknown')),
                    'controller_linear_speed': float(rs.get(
                        'controller_linear_speed', 0.0)),
                    'controller_angular_speed': float(rs.get(
                        'controller_angular_speed', 0.0)),
                    'controller_measured_left_wheel_speed': (
                        float(rs['controller_measured_left_wheel_speed'])
                        if rs.get('controller_measured_left_wheel_speed')
                        is not None else None),
                    'controller_measured_right_wheel_speed': (
                        float(rs['controller_measured_right_wheel_speed'])
                        if rs.get('controller_measured_right_wheel_speed')
                        is not None else None),
                    'controller_stop_reason': str(rs.get(
                        'controller_stop_reason', 'unknown')),
                    'controller_local_risk_level': str(rs.get(
                        'controller_local_risk_level', 'unknown')),
                    'controller_status_sample_time': float(rs.get(
                        'controller_status_sample_time', 0.0)),
                    'heading': float(rs.get('heading', 0.0)),
                    'controller_target': (
                        [float(value) for value in rs['controller_target'][:2]]
                        if rs.get('controller_target') is not None else None),
                    'controller_target_distance': (
                        float(rs['controller_target_distance'])
                        if rs.get('controller_target_distance') is not None
                        else None),
                    'controller_reported_target': (
                        [float(value) for value in
                         rs['controller_reported_target'][:2]]
                        if rs.get('controller_reported_target') is not None
                        else None),
                    'controller_reported_target_distance': (
                        float(rs['controller_reported_target_distance'])
                        if rs.get('controller_reported_target_distance')
                        is not None else None),
                    'controller_reported_waypoint_count': int(rs.get(
                        'controller_reported_waypoint_count', 0)),
                    'controller_target_mismatch': bool(rs.get(
                        'controller_target_mismatch', False)),
                    'controller_paused_until': float(
                        rs.get('controller_paused_until', 0.0)),
                    'controller_joint_wait_until': float(
                        rs.get('controller_joint_wait_until', 0.0)),
                    'controller_joint_wait_reason': rs.get(
                        'controller_joint_wait_reason'),
                    'dispatch_not_before': float(
                        rs.get('dispatch_not_before', 0.0)),
                    'hold_until': float(rs.get('hold_until', 0.0)),
                }
                for rid, rs in robot_states.items()
            },
        )
        self.step_records.append(record)

    @staticmethod
    def _cap_events(events: List[dict]):
        if len(events) > 10000:
            del events[:1000]

    def record_replan(self, sim_time: float, robot_id: int, source: str,
                      plan_epoch, result: str, dispatched: bool):
        if dispatched and result == 'succeeded':
            self.replan_count += 1
        self.replan_events.append({
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'source': str(source), 'plan_epoch': plan_epoch,
            'result': str(result), 'dispatched': bool(dispatched),
        })
        self._cap_events(self.replan_events)

    def record_yield_event(self, event_type: str, sim_time: float,
                           robot_id: int, winner=None, target=None):
        """Record one deterministic yield/resume state-machine transition."""
        counter_attr = {
            'yield_start': 'yield_start_count',
            'yield_standoff_selected': 'yield_standoff_selected_count',
            'yield_wait_start': 'yield_wait_start_count',
            'yield_wait_cleared': 'yield_wait_cleared_count',
            'yield_wait_timeout': 'yield_wait_timeout_count',
            'yield_resume_proposed': 'yield_resume_proposed_count',
            'yield_resume_rejected': 'yield_resume_rejected_count',
            'yield_resume_committed': 'yield_resume_committed_count',
            'yield_resume_rollback': 'yield_resume_rollback_count',
        }.get(event_type)
        if counter_attr is not None:
            setattr(self, counter_attr, getattr(self, counter_attr, 0) + 1)
        self.yield_resume_events.append({
            'sim_time': float(sim_time), 'event_type': str(event_type),
            'robot_id': int(robot_id), 'winner': winner,
            'target': list(target) if target is not None else None,
        })
        self._cap_events(self.yield_resume_events)

    def record_nonphysical_recovery(self, sim_time: float, robot_id: int,
                                    target=None):
        """Audit teleport-style recoveries separately from physical escapes."""
        self.nonphysical_recoveries += 1
        self.nonphysical_recovery_events.append({
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'target': list(target) if target is not None else None,
        })
        self._cap_events(self.nonphysical_recovery_events)

    def record_replan_request(self, sim_time: float, robot_id: int,
                              path_version: int, plan_epoch):
        self.replan_request_count += 1
        self.replan_request_events.append({
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'path_version': int(path_version), 'plan_epoch': plan_epoch,
        })
        self._cap_events(self.replan_request_events)

    def record_escape(self, sim_time: float, robot_id: int,
                      source: str, target=None):
        self.escape_count += 1
        self.escape_events.append({
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'source': str(source),
            'target': list(target) if target is not None else None,
        })
        self._cap_events(self.escape_events)
        if len(self.step_records) > 10000:
            del self.step_records[:1000]
    
    def record_task_arrival(self, task: TransportTask, sim_time: float):
        """Record a new task arrival."""
        self.task_arrivals.append({
            'task_id': task.task_id,
            'arrival_time': sim_time,
            'pickup': task.pickup_location,
            'delivery': task.delivery_location,
            'priority': task.priority,
            'priority_rank': task.priority_rank,
            'target_completion_time': task.target_completion_time,
            'deadline': task.deadline,
            'deadline_type': task.deadline_type,
            'deadline_source': task.deadline_source,
            'late_penalty_per_second': task.late_penalty_per_second,
        })
    
    def record_task_completion(self, task: TransportTask, robot_id: int, sim_time: float):
        """Record a task completion."""
        self.task_completions.append({
            'task_id': task.task_id,
            'robot_id': robot_id,
            'arrival_time': task.arrival_time,
            'assignment_time': task.assignment_time,
            'pickup_time': task.pickup_time,
            'completion_time': sim_time,
            'waiting_time': task.waiting_time,
            'completion_duration': task.completion_duration,
            'execution_time': task.execution_time,
            'pickup': task.pickup_location,
            'delivery': task.delivery_location,
            'priority_rank': task.priority_rank,
            'deadline': task.deadline,
            'deadline_type': task.deadline_type,
            'tardiness': task.tardiness,
            'on_time': (None if task.deadline is None else
                        sim_time <= task.deadline + 1e-9),
            'business_weight': task.business_weight,
        })
    
    def record_conflict_scan(self, conflicts, sim_time: float) -> None:
        """Track predicted conflicts as episodes from detection to resolve."""
        observed = {}
        for rid_a, rid_b, time_to_conflict, distance in conflicts:
            key = tuple(sorted((int(rid_a), int(rid_b))))
            observed[key] = (float(time_to_conflict), float(distance))
            episode = self._active_conflicts.get(key)
            if episode is None:
                episode = {
                    'conflict_id': self._next_conflict_id,
                    'type': 'predicted_trajectory',
                    'robots_involved': list(key),
                    'detected_at': float(sim_time),
                    'last_seen_at': float(sim_time),
                    'status': 'active',
                    'minimum_predicted_distance': float(distance),
                    'minimum_time_to_conflict': float(time_to_conflict),
                }
                self._next_conflict_id += 1
                self.conflict_episode_count += 1
                self._active_conflicts[key] = episode
                self.conflict_events.append(episode)
            else:
                episode['last_seen_at'] = float(sim_time)
                episode['minimum_predicted_distance'] = min(
                    episode['minimum_predicted_distance'], float(distance))
                episode['minimum_time_to_conflict'] = min(
                    episode['minimum_time_to_conflict'],
                    float(time_to_conflict))
        for key in set(self._active_conflicts) - set(observed):
            episode = self._active_conflicts.pop(key)
            episode['status'] = 'resolved'
            episode['resolved_at'] = float(sim_time)
        while len(self.conflict_events) > 10000:
            resolved_index = next(
                (i for i, item in enumerate(self.conflict_events)
                 if item.get('status') == 'resolved'), None)
            if resolved_index is None:
                break
            del self.conflict_events[resolved_index]

    def record_route_dispatch(self, robot_id: int, path_version: int,
                              plan_epoch: int, source: str, waypoint_count: int,
                              sim_time: float, diagnostics: dict = None) -> bool:
        """Audit route ownership and rapid cross-source replacement."""
        event = {
            'sim_time': float(sim_time), 'robot_id': int(robot_id),
            'path_version': int(path_version), 'source': str(source),
            'plan_epoch': int(plan_epoch),
            'waypoint_count': int(waypoint_count),
        }
        if diagnostics:
            event['diagnostics'] = dict(diagnostics)
        previous = self._last_route_dispatch.get(int(robot_id))
        if (previous is not None and
                sim_time - previous['sim_time'] < 1.0 and
                previous['path_version'] != event['path_version']):
            self.route_override_events.append({
                'sim_time': float(sim_time), 'robot_id': int(robot_id),
                'previous_source': previous['source'],
                'new_source': event['source'],
                'previous_path_version': previous['path_version'],
                'new_path_version': int(path_version),
                'previous_plan_epoch': previous['plan_epoch'],
                'new_plan_epoch': int(plan_epoch),
            })
            self.route_override_count += 1
        self._last_route_dispatch[int(robot_id)] = event
        self.route_dispatch_events.append(event)
        self.route_dispatch_count += 1
        if len(self.route_dispatch_events) > 10000:
            del self.route_dispatch_events[:1000]
        if len(self.route_override_events) > 10000:
            del self.route_override_events[:1000]
        return True
    
    def record_deadlock(self, cycle: List[int], sim_time: float):
        """Record a deadlock detection event."""
        self.deadlock_events.append({
            'sim_time': sim_time,
            'robots_involved': cycle,
        })

    def record_safety_event(self, event: SafetyEvent,
                            throttle_seconds: float = 0.5) -> bool:
        """Record state changes while suppressing repeated per-frame noise."""
        key = (event.event_type, event.robot_id, event.peer_id, event.decision)
        previous = self._last_safety_event.get(key)
        if previous is not None and event.sim_time - previous < throttle_seconds:
            return False
        self._last_safety_event[key] = event.sim_time
        self.safety_events.append(asdict(event))
        self.safety_event_count += 1
        if len(self.safety_events) > 10000:
            del self.safety_events[:1000]
        return True
    
    def record_joint_cell_traversal(self, seconds: float) -> None:
        if math.isfinite(seconds) and seconds >= 0:
            self.joint_cell_traversal_seconds.append(float(seconds))

    def record_joint_window_gap(self, sim_time: float, robot_id: int,
                                epoch: int, deadline: float) -> None:
        key = (int(robot_id), int(epoch))
        if any((event["robot_id"], event["plan_epoch"]) == key
               for event in self.joint_window_gap_events):
            return
        self.joint_window_gap_events.append({
            "sim_time": float(sim_time), "robot_id": int(robot_id),
            "plan_epoch": int(epoch), "deadline": float(deadline)})

    def record_expired_reservation_movement(
            self, sim_time: float, robot_id: int, epoch: int,
            waypoint_index: int, deadline: float) -> None:
        self.reservation_expired_movement_events.append({
            "sim_time": float(sim_time), "robot_id": int(robot_id),
            "plan_epoch": int(epoch), "waypoint_index": int(waypoint_index),
            "deadline": float(deadline)})

    def compute_final_metrics(self, robots: dict, task_stats: dict,
                              coord_stats: dict, total_time: float) -> dict:
        """
        Compute final summary metrics for the experiment.
        
        Returns comprehensive metrics dictionary.
        """
        # Throughput
        throughput = task_stats.get('completed', 0) / max(total_time / 60.0, 1)  # per minute
        
        # Average task completion time
        completion_times = [tc['completion_duration'] for tc in self.task_completions 
                          if tc['completion_duration'] is not None]
        avg_completion = sum(completion_times) / max(len(completion_times), 1)
        
        # Average waiting time
        waiting_times = [tc['waiting_time'] for tc in self.task_completions 
                        if tc['waiting_time'] is not None]
        avg_waiting = sum(waiting_times) / max(len(waiting_times), 1)
        
        # Robot idle time
        robot_idle_pcts = []
        total_distance = 0.0
        tasks_per_robot = []
        
        for rid, robot in robots.items():
            idle_pct = (robot.idle_time / max(total_time, 1)) * 100
            robot_idle_pcts.append(idle_pct)
            total_distance += robot.total_distance
            tasks_per_robot.append(robot.tasks_completed)
        
        avg_idle_pct = sum(robot_idle_pcts) / max(len(robot_idle_pcts), 1)
        
        # Workload balance (coefficient of variation)
        if tasks_per_robot and sum(tasks_per_robot) > 0:
            mean_tasks = sum(tasks_per_robot) / len(tasks_per_robot)
            variance = sum((t - mean_tasks)**2 for t in tasks_per_robot) / len(tasks_per_robot)
            workload_cv = (variance ** 0.5) / max(mean_tasks, 1)
        else:
            workload_cv = 0.0
        
        latency = self.scheduling_latencies_ms
        supervisor_steps = self.supervisor_step_wall_ms
        webots_steps = self.supervisor_webots_step_wall_ms
        joint_planning = self.joint_planning_wall_ms
        route_dispatches_by_source = {}
        for event in self.route_dispatch_events:
            source = event.get('source', 'unknown')
            route_dispatches_by_source[source] = (
                route_dispatches_by_source.get(source, 0) + 1)
        route_overrides_by_transition = {}
        for event in self.route_override_events:
            transition = (f"{event.get('previous_source', 'unknown')}->"
                          f"{event.get('new_source', 'unknown')}")
            route_overrides_by_transition[transition] = (
                route_overrides_by_transition.get(transition, 0) + 1)
        communication = {}
        for channel, samples in self.communication_wall_ms.items():
            communication[channel] = {
                'samples': len(samples),
                'messages': self.communication_messages.get(channel, 0),
                'bytes': self.communication_bytes.get(channel, 0),
                'wall_p50_ms': self._percentile(samples, 0.50),
                'wall_p95_ms': self._percentile(samples, 0.95),
                'wall_p99_ms': self._percentile(samples, 0.99),
                'wall_max_ms': max(samples, default=0.0),
            }

        def wall_summary(samples):
            if not samples:
                return {
                    'samples': 0,
                    'mean_ms': 0.0,
                    'p50_ms': 0.0,
                    'p95_ms': 0.0,
                    'p99_ms': 0.0,
                    'max_ms': 0.0,
                }
            return {
                'samples': len(samples),
                'mean_ms': statistics.fmean(samples),
                'p50_ms': self._percentile(samples, 0.50),
                'p95_ms': self._percentile(samples, 0.95),
                'p99_ms': self._percentile(samples, 0.99),
                'max_ms': max(samples),
            }

        phase_metrics = {
            channel: wall_summary(samples)
            for channel, samples in self.phase_wall_ms.items()
        }
        joint_tier_metrics = {
            tier: wall_summary(samples)
            for tier, samples in self.joint_planning_tier_wall_ms.items()
        }
        robot_dwa_metrics = wall_summary(self.robot_dwa_wall_ms)
        completion_marks = sorted(
            float(item['completion_time']) for item in self.task_completions
            if item.get('completion_time') is not None)
        progress_marks = [0.0] + completion_marks + [float(total_time)]
        completion_gaps = [later - earlier for earlier, later in zip(
            progress_marks, progress_marks[1:])]
        active_wait_seconds = sum(
            max(0.0, float(total_time) - episode['started_at'])
            for episode in self._active_planned_waits.values())
        deadline_completions = [item for item in self.task_completions
                                if item.get('deadline') is not None]
        tardiness_values = [float(item.get('tardiness') or 0.0)
                            for item in deadline_completions]
        on_time_count = sum(bool(item.get('on_time'))
                            for item in deadline_completions)
        weighted_tardiness = sum(
            float(item.get('tardiness') or 0.0) *
            float(item.get('business_weight') or 1.0)
            for item in deadline_completions)
        arrived_deadline_tasks = [item for item in self.task_arrivals
                                  if item.get('deadline') is not None]
        completed_ids = {item['task_id'] for item in self.task_completions
                         if item.get('task_id') is not None}
        overdue_unfinished = sum(
            float(item.get('deadline')) < float(total_time) and
            item['task_id'] not in completed_ids
            for item in arrived_deadline_tasks)
        continuity = {}
        continuity_totals = {
            'eligible_seconds': 0.0, 'moving_seconds': 0.0,
            'turning_seconds': 0.0, 'valid_wait_seconds': 0.0,
            'unblocked_zero_speed_seconds': 0.0,
            'unblocked_stop_episodes': 0, 'status_samples': 0,
            'status_drops': 0,
        }
        for rid, raw in self.motion_continuity_by_robot.items():
            item = {key: raw[key] for key in continuity_totals}
            eligible = item['eligible_seconds']
            clear_eligible = max(
                0.0, eligible - item['valid_wait_seconds'])
            item['clear_eligible_seconds'] = clear_eligible
            item['clear_motion_duty_cycle'] = (
                (item['moving_seconds'] + item['turning_seconds']) /
                clear_eligible if clear_eligible > 0.0 else None)
            item['unblocked_zero_speed_ratio'] = (
                item['unblocked_zero_speed_seconds'] / eligible
                if eligible > 0.0 else None)
            continuity[str(rid)] = item
            for key in continuity_totals:
                continuity_totals[key] += item[key]
        total_eligible = continuity_totals['eligible_seconds']
        clear_eligible = max(
            0.0, total_eligible - continuity_totals['valid_wait_seconds'])
        continuity_totals['clear_eligible_seconds'] = clear_eligible
        continuity_totals['clear_motion_duty_cycle'] = (
            (continuity_totals['moving_seconds'] +
             continuity_totals['turning_seconds']) / clear_eligible
            if clear_eligible > 0.0 else None)
        continuity_totals['unblocked_zero_speed_ratio'] = (
            continuity_totals['unblocked_zero_speed_seconds'] /
            total_eligible if total_eligible > 0.0 else None)
        return {
            # Primary KPIs
            "throughput_per_minute": throughput,
            "total_tasks_completed": task_stats.get('completed', 0),
            "total_tasks_generated": task_stats.get('total_generated', 0),
            "avg_task_completion_time": avg_completion,
            "avg_waiting_time": avg_waiting,
            "deadline_tasks_arrived": len(arrived_deadline_tasks),
            "deadline_tasks_completed": len(deadline_completions),
            "on_time_completed": on_time_count,
            "on_time_rate_all_arrivals": (
                on_time_count / max(len(arrived_deadline_tasks), 1)),
            "hard_deadline_breaches": (
                sum(value > 0.0 for value in tardiness_values) +
                overdue_unfinished),
            "overdue_unfinished_tasks": overdue_unfinished,
            "total_tardiness_seconds": sum(tardiness_values),
            "weighted_tardiness": weighted_tardiness,
            "max_tardiness_seconds": max(tardiness_values, default=0.0),
            "avg_robot_idle_pct": avg_idle_pct,
            "total_distance_all_robots": total_distance,
            "total_conflicts_resolved": coord_stats.get('conflicts_resolved', 0),
            "predicted_conflicts": self.conflict_episode_count,
            "route_dispatches": self.route_dispatch_count,
            "rapid_route_overrides": self.route_override_count,
            "route_dispatches_by_source": route_dispatches_by_source,
            "route_overrides_by_transition": route_overrides_by_transition,
            "unauthorized_route_writes": self.unauthorized_route_write_count,
            "audited_replans": self.replan_count,
            "replan_requests": self.replan_request_count,
            "joint_plan_requests_by_reason": dict(sorted(
                self.joint_plan_requests_by_reason.items())),
            "joint_plan_requests_by_class": dict(sorted(
                self.joint_plan_requests_by_class.items())),
            "physical_escapes": self.escape_count,
            "nonphysical_recoveries": self.nonphysical_recoveries,
            "yield_start": self.yield_start_count,
            "yield_standoff_selected": self.yield_standoff_selected_count,
            "yield_wait_start": self.yield_wait_start_count,
            "yield_wait_cleared": self.yield_wait_cleared_count,
            "yield_wait_timeout": self.yield_wait_timeout_count,
            "yield_resume_proposed": self.yield_resume_proposed_count,
            "yield_resume_rejected": self.yield_resume_rejected_count,
            "yield_resume_committed": self.yield_resume_committed_count,
            "yield_resume_rollback": self.yield_resume_rollback_count,
            "safety_event_count": self.safety_event_count,
            "unplanned_task_stops": self.unplanned_stop_count,
            "planned_wait_episodes": self.planned_wait_count,
            "planned_wait_total_seconds": (
                self.planned_wait_total_seconds + active_wait_seconds),
            "total_deadlocks": coord_stats.get('deadlocks_detected', 0),
            "min_pair_distance": (
                self.minimum_pair_distance_seen
                if math.isfinite(self.minimum_pair_distance_seen) else None),
            "pair_distance_violations": (
                self.pair_distance_violation_samples),
            
            # Secondary metrics
            "max_completion_time": max(completion_times) if completion_times else 0,
            "min_completion_time": min(completion_times) if completion_times else 0,
            "workload_balance_cv": workload_cv,
            "tasks_per_robot": tasks_per_robot,
            "robot_idle_percentages": robot_idle_pcts,
            "total_replans": coord_stats.get('total_replans', 0),
            "joint_plans_attempted": coord_stats.get(
                'joint_plans_attempted', 0),
            "joint_plans_accepted": coord_stats.get(
                'joint_plans_accepted', 0),
            "joint_plan_rollbacks": coord_stats.get(
                'joint_plan_rollbacks', 0),
            "longest_completion_plateau_seconds": max(
                completion_gaps, default=float(total_time)),
            "final_completion_plateau_seconds": (
                float(total_time) - completion_marks[-1]
                if completion_marks else float(total_time)),
            "robots_without_completed_tasks": sum(
                1 for count in tasks_per_robot if count == 0),
            "scheduling_latency_mean_ms": (
                statistics.fmean(latency) if latency else 0.0),
            "scheduling_latency_p50_ms": self._percentile(latency, 0.50),
            "scheduling_latency_p95_ms": self._percentile(latency, 0.95),
            "scheduling_latency_p99_ms": self._percentile(latency, 0.99),
            "scheduling_latency_max_ms": max(latency, default=0.0),
            "supervisor_step_wall_p50_ms": self._percentile(
                supervisor_steps, 0.50),
            "supervisor_step_wall_p95_ms": self._percentile(
                supervisor_steps, 0.95),
            "supervisor_step_wall_p99_ms": self._percentile(
                supervisor_steps, 0.99),
            "supervisor_step_wall_max_ms": max(
                supervisor_steps, default=0.0),
            "supervisor_step_samples": len(supervisor_steps),
            "supervisor_webots_step_wall_p50_ms": self._percentile(
                webots_steps, 0.50),
            "supervisor_webots_step_wall_p95_ms": self._percentile(
                webots_steps, 0.95),
            "supervisor_webots_step_wall_p99_ms": self._percentile(
                webots_steps, 0.99),
            "supervisor_webots_step_wall_max_ms": max(
                webots_steps, default=0.0),
            "supervisor_webots_step_samples": len(webots_steps),
            "supervisor_phase_metrics": phase_metrics,
            "joint_planning_wall_p50_ms": self._percentile(
                joint_planning, 0.50),
            "joint_planning_wall_p95_ms": self._percentile(
                joint_planning, 0.95),
            "joint_planning_wall_p99_ms": self._percentile(
                joint_planning, 0.99),
            "joint_planning_wall_max_ms": max(
                joint_planning, default=0.0),
            "joint_equivalent_refreshes_suppressed": (
                self.joint_equivalent_refreshes_suppressed),
            "joint_planning_samples": len(joint_planning),
            "joint_planning_tier_metrics": joint_tier_metrics,
            "robot_dwa_metrics": robot_dwa_metrics,
            "communication_metrics": communication,
            "invalid_scheduler_outputs": self.invalid_scheduler_outputs,
            "scheduler_fallbacks": self.scheduler_fallbacks,
            "native_scheduler_commits": self.native_scheduler_commits,
            "fallback_scheduler_commits": self.fallback_scheduler_commits,
            "scheduler_commits_by_algorithm": dict(
                self.commits_by_algorithm),
            "rl_policy_decisions": self.rl_policy_decisions,
            "rl_fallback_decisions": self.rl_fallback_decisions,
            "rl_fallback_rate": (
                self.rl_fallback_decisions /
                max(1, self.rl_policy_decisions + self.rl_fallback_decisions)),
            "rl_timeout_count": self.rl_timeout_count,
            "rl_inference_mean_ms": (
                statistics.fmean(self.rl_inference_latencies_ms)
                if self.rl_inference_latencies_ms else 0.0),
            "rl_inference_p95_ms": self._percentile(
                self.rl_inference_latencies_ms, 0.95),
            "rl_inference_max_ms": max(
                self.rl_inference_latencies_ms, default=0.0),
            "joint_cell_traversal_p50_seconds": self._percentile(
                self.joint_cell_traversal_seconds, 0.50),
            "joint_cell_traversal_p95_seconds": self._percentile(
                self.joint_cell_traversal_seconds, 0.95),
            "joint_cell_traversal_p99_seconds": self._percentile(
                self.joint_cell_traversal_seconds, 0.99),
            "joint_cell_traversal_samples": len(
                self.joint_cell_traversal_seconds),
            "joint_window_gaps": len(self.joint_window_gap_events),
            "reservation_expired_movements": len(
                self.reservation_expired_movement_events),
            "motion_continuity": continuity_totals,
            "motion_continuity_by_robot": continuity,
            "unknown_stop_episodes": sum(
                event.get('reason') in (
                    'unknown', 'unclassified_zero_command')
                for event in self.motion_continuity_events),
            "progress_lease_soft_deadlines": sum(
                event.get('stage') == 'soft_replan'
                for event in self.progress_lease_events),
            "progress_lease_hard_deadlines": sum(
                event.get('stage') in ('hard_escape', 'hard_zero_escape')
                for event in self.progress_lease_events),
            "terminal_service_complete": sum(
                event.get('event_type') == 'service_complete'
                for event in self.terminal_handoff_events),
            "terminal_physically_clear": sum(
                event.get('event_type') == 'physically_clear'
                for event in self.terminal_handoff_events),
            "terminal_inbound_denied": sum(
                event.get('event_type') == 'inbound_denied'
                for event in self.terminal_handoff_events),
            "terminal_inbound_granted": sum(
                event.get('event_type') == 'inbound_granted'
                for event in self.terminal_handoff_events),
            "terminal_egress_retry_failed": sum(
                event.get('event_type') == 'egress_retry_failed'
                for event in self.terminal_handoff_events),
            "terminal_egress_retry_dispatched": sum(
                event.get('event_type') == 'egress_retry_dispatched'
                for event in self.terminal_handoff_events),
        }
    
    def save_results(self, robots: dict, task_stats: dict,
                     coord_stats: dict, total_time: float):
        """Save all results to JSON file."""
        final_metrics = self.compute_final_metrics(
            robots, task_stats, coord_stats, total_time
        )
        
        active_wait_events = []
        for episode in self._active_planned_waits.values():
            snapshot = dict(episode)
            snapshot['duration'] = max(
                0.0, float(total_time) - snapshot['started_at'])
            active_wait_events.append(snapshot)

        results = {
            "experiment_info": {
                "scenario": self.scenario_name,
                "scheduler": self.scheduler_name,
                "num_robots": self.num_robots,
                "sim_duration": total_time,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "provenance": self.provenance,
            },
            "summary_metrics": final_metrics,
            "task_stats": task_stats,
            "coordination_stats": coord_stats,
            "task_completions": self.task_completions,
            "rl_event_ledger": self.rl_event_ledger,
            "rl_decisions": self.rl_decisions,
            "conflict_events": self.conflict_events,
            "route_dispatch_events": self.route_dispatch_events,
            "route_override_events": self.route_override_events,
            "unauthorized_route_write_events": (
                self.unauthorized_route_write_events),
            "replan_events": self.replan_events,
            "replan_request_events": self.replan_request_events,
            "escape_events": self.escape_events,
            "yield_resume_events": self.yield_resume_events,
            "nonphysical_recovery_events": self.nonphysical_recovery_events,
            "unplanned_stop_events": self.unplanned_stop_events,
            "planned_wait_events": (
                self.planned_wait_events + active_wait_events),
            "deadlock_events": self.deadlock_events,
            "safety_events": self.safety_events,
            "joint_cell_traversal_seconds": self.joint_cell_traversal_seconds,
            "joint_window_gap_events": self.joint_window_gap_events,
            "reservation_expired_movement_events": (
                self.reservation_expired_movement_events),
            "motion_continuity_events": self.motion_continuity_events,
            "terminal_handoff_events": self.terminal_handoff_events,
            "progress_lease_events": self.progress_lease_events,
            "time_series": [asdict(sr) for sr in self.step_records[-100:]],  # last 100 steps
        }
        
        with open(self.output_path, 'w') as f:
            json.dump(results, f, indent=2, default=str, allow_nan=False)
        
        return self.output_path
    
    def print_summary(self, metrics: dict):
        """Print a formatted summary of metrics."""
        print("\n" + "="*50)
        print("EXPERIMENT RESULTS SUMMARY")
        print("="*50)
        print(f"Scenario: {self.scenario_name} | Scheduler: {self.scheduler_name}")
        print(f"Robots: {self.num_robots}")
        print("-"*50)
        print(f"Throughput:           {metrics['throughput_per_minute']:.2f} tasks/min")
        print(f"Tasks completed:      {metrics['total_tasks_completed']}/{metrics['total_tasks_generated']}")
        print(f"Avg completion time:  {metrics['avg_task_completion_time']:.2f}s")
        print(f"Avg waiting time:     {metrics['avg_waiting_time']:.2f}s")
        print(f"Avg robot idle:       {metrics['avg_robot_idle_pct']:.1f}%")
        print(f"Total distance:       {metrics['total_distance_all_robots']:.1f}m")
        print(f"Conflicts resolved:   {metrics['total_conflicts_resolved']}")
        print(f"Deadlocks:           {metrics['total_deadlocks']}")
        print(f"Workload balance CV: {metrics['workload_balance_cv']:.3f}")
        print("="*50 + "\n")
