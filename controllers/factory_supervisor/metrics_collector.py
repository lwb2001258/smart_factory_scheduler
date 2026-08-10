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
    min_pair_distance: float


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
    
    def __init__(self, scenario_name: str, scheduler_name: str, num_robots: int):
        self.scenario_name = scenario_name
        self.scheduler_name = scheduler_name
        self.num_robots = num_robots
        
        # Time-series data
        self.step_records: List[StepRecord] = []
        
        # Task-level data
        self.task_arrivals: List[dict] = []
        self.task_completions: List[dict] = []
        
        # Conflict/deadlock events
        self.conflict_events: List[dict] = []
        self.deadlock_events: List[dict] = []
        self.safety_events: List[dict] = []
        self._last_safety_event: Dict[tuple, float] = {}
        self.scheduling_latencies_ms: List[float] = []
        self.invalid_scheduler_outputs = 0
        self.scheduler_fallbacks = 0
        self.native_scheduler_commits = 0
        self.fallback_scheduler_commits = 0
        self.commits_by_algorithm: Dict[str, int] = {}
        self.rl_inference_latencies_ms: List[float] = []
        self.rl_timeout_count = 0
        self.rl_policy_decisions = 0
        self.rl_fallback_decisions = 0
        
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

    def record_scheduling_latency(self, seconds: float):
        if math.isfinite(seconds) and seconds >= 0:
            self.scheduling_latencies_ms.append(seconds * 1000.0)

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
                    pair_distances.append(((pos_a[0] - pos_b[0]) ** 2 +
                                           (pos_a[1] - pos_b[1]) ** 2) ** 0.5)
        
        record = StepRecord(
            sim_time=sim_time,
            tasks_completed=task_stats.get('completed', 0),
            tasks_pending=task_stats.get('pending', 0),
            tasks_active=task_stats.get('active', 0),
            robots_idle=idle_count,
            robots_active=active_count,
            conflicts_total=coord_stats.get('conflicts_resolved', 0),
            avg_battery=avg_battery,
            min_pair_distance=min(pair_distances) if pair_distances else float('inf'),
        )
        self.step_records.append(record)
    
    def record_task_arrival(self, task: TransportTask, sim_time: float):
        """Record a new task arrival."""
        self.task_arrivals.append({
            'task_id': task.task_id,
            'arrival_time': sim_time,
            'pickup': task.pickup_location,
            'delivery': task.delivery_location,
            'priority': task.priority,
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
        })
    
    def record_conflict(self, conflict_info: dict, sim_time: float):
        """Record a coordination conflict event."""
        self.conflict_events.append({
            'sim_time': sim_time,
            **conflict_info,
        })
    
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
        return True
    
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
        return {
            # Primary KPIs
            "throughput_per_minute": throughput,
            "total_tasks_completed": task_stats.get('completed', 0),
            "total_tasks_generated": task_stats.get('total_generated', 0),
            "avg_task_completion_time": avg_completion,
            "avg_waiting_time": avg_waiting,
            "avg_robot_idle_pct": avg_idle_pct,
            "total_distance_all_robots": total_distance,
            "total_conflicts_resolved": coord_stats.get('conflicts_resolved', 0),
            "total_deadlocks": coord_stats.get('deadlocks_detected', 0),
            "min_pair_distance": min(
                (record.min_pair_distance for record in self.step_records),
                default=float('inf')),
            "pair_distance_violations": sum(
                1 for record in self.step_records
                if record.min_pair_distance < 0.50),
            
            # Secondary metrics
            "max_completion_time": max(completion_times) if completion_times else 0,
            "min_completion_time": min(completion_times) if completion_times else 0,
            "workload_balance_cv": workload_cv,
            "tasks_per_robot": tasks_per_robot,
            "robot_idle_percentages": robot_idle_pcts,
            "total_replans": coord_stats.get('total_replans', 0),
            "scheduling_latency_mean_ms": (
                statistics.fmean(latency) if latency else 0.0),
            "scheduling_latency_p50_ms": self._percentile(latency, 0.50),
            "scheduling_latency_p95_ms": self._percentile(latency, 0.95),
            "scheduling_latency_p99_ms": self._percentile(latency, 0.99),
            "scheduling_latency_max_ms": max(latency, default=0.0),
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
        }
    
    def save_results(self, robots: dict, task_stats: dict,
                     coord_stats: dict, total_time: float):
        """Save all results to JSON file."""
        final_metrics = self.compute_final_metrics(
            robots, task_stats, coord_stats, total_time
        )
        
        results = {
            "experiment_info": {
                "scenario": self.scenario_name,
                "scheduler": self.scheduler_name,
                "num_robots": self.num_robots,
                "sim_duration": total_time,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            },
            "summary_metrics": final_metrics,
            "task_stats": task_stats,
            "coordination_stats": coord_stats,
            "task_completions": self.task_completions,
            "deadlock_events": self.deadlock_events,
            "safety_events": self.safety_events,
            "time_series": [asdict(sr) for sr in self.step_records[-100:]],  # last 100 steps
        }
        
        with open(self.output_path, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
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
