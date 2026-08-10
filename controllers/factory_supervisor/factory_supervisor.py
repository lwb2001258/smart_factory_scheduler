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
from metrics_collector import MetricsCollector


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
        self.pending_waypoints: Optional[List[Tuple[float, float]]] = None
        self.dispatch_not_before = 0.0
        self.hold_until = 0.0
        self.wait_started = 0.0
        self.yield_count = 0
        self.emergency_braking = False
        self.emergency_braking_since = None
        self.emergency_recovery_until = 0.0
        
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
        # Returning-home/rest journeys intentionally have no business
        # location name. Their authoritative destination is the last
        # waypoint, which the grid planner accepts as an explicit coordinate.
        if (robot.state == RobotState.RETURNING_HOME and robot.waypoints):
            target = robot.waypoints[-1]
            return (float(target[0]), float(target[1]))
        return None
    
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
        initial_positions = {
            1: (-7.0, 3.0),
            2: (-7.0, -3.0),
            3: (7.0, 3.0),
            4: (7.0, -3.0),
            5: (-4.0, 0.0),
            6: (4.0, 0.0),
            7: (-8.0, 5.0),
            8: (8.0, -5.0),
        }
        
        # Cache Webots Node references so we look them up only once.
        # DEF names in the .wbt are ROBOT_1 … ROBOT_8.
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
        Never calls getFromDevice() — that API is only valid for devices
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
                    if abs(rot[2]) > 0.5:          # rotation axis ≈ z
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

    def _dispatch_plan(self, robot_id: int, waypoints, delay: Optional[float] = None):
        """Single versioned entry point for navigation plan dispatch."""
        self._last_command_send_ok = False
        if not waypoints:
            return False
        robot = self.robots[robot_id]
        plan = [tuple(wp) for wp in waypoints]
        if delay is None:
            delay = self.motion_coordinator.get_dispatch_delay(robot_id)
        robot.path_version += 1
        if delay > 0:
            robot.pending_waypoints = plan
            robot.dispatch_not_before = self.sim_time + delay
            self._last_command_send_ok = self._send_command_to_robot(robot_id, {
                'type': 'hold', 'until': robot.dispatch_not_before,
                'path_version': robot.path_version,
            })
            if not self._last_command_send_ok:
                robot.pending_waypoints = None
                robot.dispatch_not_before = 0.0
            return False
        robot.pending_waypoints = None
        robot.dispatch_not_before = 0.0
        self._last_command_send_ok = self._send_command_to_robot(robot_id, {
            'type': 'navigate',
            'target': list(plan[0]),
            'all_waypoints': [list(wp) for wp in plan],
            'path_version': robot.path_version,
        })
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
                    })
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
                    
                    # ⚠️ FLAG fields: must check the VALUE is True, not
                    # just that the key exists. Robot status reports
                    # ALWAYS include 'reached_goal' (with value False
                    # most of the time), so 'reached_goal' in msg is
                    # always True. Using msg.get(...) ensures we only
                    # fire the handler when the robot actually arrived.
                    if msg.get('reached_waypoint') is True:
                        self._handle_waypoint_reached(robot_id)
                    if msg.get('reached_goal') is True:
                        self._handle_goal_reached(robot_id)
                    if msg.get('replan_requested') is True:
                        self.robots[robot_id]._replan_requested = True
                        print(f"[Supervisor] Replan request received from Robot {robot_id}")
                
                self.receiver.nextPacket()
            except Exception as e:
                print(f"[Supervisor] Receive error: {e}")
                try:
                    self.receiver.nextPacket()
                except:
                    break

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

    def _proactive_path_conflict_scan(self):
        """
        轨迹预测 + 时空冲突检测。
        
        每30帧(~1s)执行一次：
          1. 对每个活跃机器人，沿其waypoints预测未来10秒轨迹
          2. 对每对机器人，比对同一时刻的预测位置
          3. 若距离 < 碰撞半径 → 低优先级机器人(ID大)重规划
        
        这自然覆盖所有碰撞方向（正面、侧面、垂直交叉）。
        """
        PREDICTION_HORIZON = 10.0   # 秒 — 预测未来10秒
        SAMPLE_DT = 0.5             # 秒 — 每0.5s采样一个位置
        ROBOT_SPEED = 0.22          # m/s
        COLLISION_RADIUS = 0.50     # m — 中心距<此值视为碰撞风险
        REPLAN_COOLDOWN = 2.0       # s — suppress same-conflict replan storms
        DUPLICATE_HOLD = 1.0        # s — let the higher-priority peer clear
        scan_deadline = real_time.perf_counter() + PROACTIVE_SCAN_BUDGET_SECONDS
        
        # ── Step 1: 预测所有活跃机器人的未来轨迹 ──
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
            
            traj = self._predict_trajectory(
                robot.position,
                robot.waypoints[robot.current_waypoint_idx:],
                ROBOT_SPEED, PREDICTION_HORIZON, SAMPLE_DT)
            if traj:
                trajectories[rid] = traj
        
        if len(trajectories) < 2:
            return  # 不到2个机器人在移动，无冲突可能
        
        # 日志: 显示当前活跃轨迹数
        if self.sim_time >= getattr(self, '_next_scan_log', 0.0):
            print(f"[Conflict] T={self.sim_time:.0f}s trajectory scan: "
                  f"{len(trajectories)} active robots, "
                  f"IDs={list(trajectories.keys())}")
            self._next_scan_log = self.sim_time + 30.0
        
        # ── Step 2: 两两比对，检测时空冲突 ──
        robot_ids = list(trajectories.keys())
        conflicts = []  # [(rid_yield, rid_stay, t_conflict)]
        
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                rid_a, rid_b = robot_ids[i], robot_ids[j]
                traj_a, traj_b = trajectories[rid_a], trajectories[rid_b]
                
                # 比对同一时间索引的位置
                min_len = min(len(traj_a), len(traj_b))
                for k in range(min_len):
                    ax, ay, t = traj_a[k]
                    bx, by, _ = traj_b[k]
                    dist = math.sqrt((ax - bx)**2 + (ay - by)**2)
                    if dist < COLLISION_RADIUS:
                        # 冲突！优先级低的(ID大)让路
                        rid_yield = max(rid_a, rid_b)
                        rid_stay = min(rid_a, rid_b)
                        conflicts.append((rid_yield, rid_stay, t))
                        break  # 只记录第一个冲突时刻
        
        # ── Step 3: 重规划冲突机器人 ──
        replanned = set()
        for rid_yield, rid_stay, t_conflict in conflicts:
            # This code runs synchronously with Webots. Limit work per scan so
            # a dense conflict set cannot prevent the next simulation step.
            if (len(replanned) >= PROACTIVE_SCAN_MAX_REPLANS or
                    real_time.perf_counter() >= scan_deadline):
                break
            if rid_yield in replanned:
                continue  # 已经重规划过了
            
            robot = self.robots[rid_yield]
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
                    self._hold_robot(rid_yield, DUPLICATE_HOLD)
                    duplicate_count = getattr(
                        robot, '_duplicate_conflict_replans', 0) + 1
                    robot._duplicate_conflict_replans = duplicate_count
                    self._log_replan(
                        f"[Replan] Robot {rid_yield} predicted collision with "
                        f"Robot {rid_stay} in {t_conflict:.1f}s; planner returned "
                        f"the same {len(new_path)}-step path. Holding "
                        f"{DUPLICATE_HOLD:.1f}s instead of resending "
                        f"(repeat={duplicate_count})")
                    replanned.add(rid_yield)
                    continue

                robot._duplicate_conflict_replans = 0
                robot.waypoints = list(new_path)
                robot.current_waypoint_idx = 0
                self._send_command_to_robot(rid_yield, {
                    'type': 'navigate',
                    'target': list(new_path[0]),
                    'all_waypoints': [list(w) for w in new_path],
                })
                replanned.add(rid_yield)
                self._log_replan(f"[Replan] Robot {rid_yield} predicted collision with Robot {rid_stay} "
                      f"in {t_conflict:.1f}s; active replan: "
                      f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f}) "
                      f"→ goal={goal}, new path={len(new_path)} steps")
            else:
                robot._proactive_replan_cooldown_until = (
                    self.sim_time + REPLAN_COOLDOWN)
                self._hold_robot(rid_yield, DUPLICATE_HOLD)
                self._log_replan(
                    f"[Replan] Robot {rid_yield} predicted collision with "
                    f"Robot {rid_stay}, but no alternative path was found; "
                    f"holding {DUPLICATE_HOLD:.1f}s before retry")
                replanned.add(rid_yield)

    @staticmethod
    def _paths_equivalent(path_a, path_b, tolerance=0.15):
        """Return True when two waypoint sequences are materially identical."""
        if len(path_a) != len(path_b):
            return False
        return all(math.hypot(float(a[0]) - float(b[0]),
                              float(a[1]) - float(b[1])) <= tolerance
                   for a, b in zip(path_a, path_b))
    
    def _predict_trajectory(self, start_pos, waypoints, speed,
                            horizon, dt):
        """
        沿waypoints匀速模拟，产出未来轨迹采样点。
        
        Args:
            start_pos: (x, y) 当前位置
            waypoints: 剩余waypoint列表 [(x,y), ...]
            speed: 匀速 m/s
            horizon: 预测时长(秒)
            dt: 采样间隔(秒)
        
        Returns:
            [(x, y, t), ...] 未来轨迹点
        """
        trajectory = []
        cx, cy = start_pos[0], start_pos[1]
        wp_idx = 0
        t = 0.0
        next_sample = dt
        
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
        
        # 如果waypoints走完了但时间没到，机器人会停在最后位置
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
        STALL_THRESHOLD = 3.0  # seconds of no progress → trigger replan
        MIN_PROGRESS_DIST = 0.15  # must move at least this much per check
        
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
            
            # ── Priority 1: Emergency replan request from robot controller ──
            if getattr(robot, '_replan_requested', False):
                robot._replan_requested = False
                
                # ★ 冷却期: 同一robot 5秒内不重复重规划
                REPLAN_COOLDOWN = 3.0
                last_replan_t = getattr(robot, '_last_replan_time', 0.0)
                if self.sim_time - last_replan_t < REPLAN_COOLDOWN:
                    continue  # 冷却中,跳过
                
                goal = self._navigation_goal(robot)
                if not goal:
                    self._log_replan(f"[Replan] Robot {rid} requested replanning but goal_location is empty; "
                          f"state={robot.state}, task={robot.current_task}")
                    continue
                
                # ★ 检查连续重规划次数 → 如果3次无进展,用让路策略
                replan_count = getattr(robot, '_replan_fail_count', 0)
                replan_pos = getattr(robot, '_replan_start_pos', None)
                if replan_pos:
                    moved = math.hypot(robot.position[0] - replan_pos[0],
                                       robot.position[1] - replan_pos[1])
                    if moved < 0.3:
                        replan_count += 1
                    else:
                        replan_count = 0  # 有进展,重置计数
                else:
                    replan_count = 0
                robot._replan_fail_count = replan_count
                robot._replan_start_pos = robot.position
                
                if replan_count >= 3:
                    # ★ 让路策略: 找到阻挡此robot的peer,让低优先级的让路
                    self._log_replan(f"[Replan] Robot {rid} made no progress after {replan_count} replans; "
                          f"starting yield coordination")
                    robot._replan_fail_count = 0
                    
                    # 找最近的peer(可能是阻挡者)
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
                            # 我优先级高(ID小) → 对方让路,我重规划(把对方当静态障碍)
                            self._log_replan(f"[Replan] Robot {rid} (high priority) replanning; "
                                  f"Robot {closest_peer} (low priority) yields")
                            # 命令对方原地等待3秒
                            peer_robot = self.robots[closest_peer]
                            peer_robot._last_replan_time = self.sim_time + 3.0
                            self._hold_robot(closest_peer, 3.0)
                            # 我自己重规划
                            new_path = self.motion_coordinator.plan_grid_lifelong(
                                rid, robot.position, goal)
                            if new_path and len(new_path) > 0:
                                robot.waypoints = list(new_path)
                                robot.current_waypoint_idx = 0
                                self._send_command_to_robot(rid, {
                                    'type': 'navigate',
                                    'target': list(new_path[0]),
                                    'all_waypoints': [list(w) for w in new_path],
                                })
                                self._log_replan(f"[Replan] Robot {rid} yield-aware replan succeeded: {len(new_path)} steps")
                        else:
                            # 我优先级低(ID大) → 我让路3秒
                            self._log_replan(f"[Replan] Robot {rid} (low priority) yields for 3 s; "
                                  f"Robot {closest_peer} proceeds first")
                            robot._last_replan_time = self.sim_time + 3.0
                            self._hold_robot(rid, 3.0)
                    else:
                        # 没有近距离peer,可能是死路 → 等3秒再试
                        robot._last_replan_time = self.sim_time + 3.0
                    continue
                
                # 正常重规划
                new_path = self.motion_coordinator.plan_grid_lifelong(
                    rid, robot.position, goal)
                if new_path and len(new_path) > 0:
                    # ★ 重复路径检测: 如果首个waypoint和上次一样,说明没有替代路
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
                        # 路径没变 → 增加失败计数(由让路策略处理)
                        robot._replan_fail_count = getattr(robot, '_replan_fail_count', 0) + 1
                    
                    robot.waypoints = list(new_path)
                    robot.current_waypoint_idx = 0
                    self._send_command_to_robot(rid, {
                        'type': 'navigate',
                        'target': list(new_path[0]),
                        'all_waypoints': [list(w) for w in new_path],
                    })
                    self._log_replan(f"[Replan] Robot {rid} path-conflict replan: "
                          f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f}) "
                          f"→ goal={goal}, new path={len(new_path)} steps, "
                          f"avoiding peers={[r for r in self.robots.keys() if r != rid]}"
                          f"{' (duplicate path)' if same_path else ''}")
                else:
                    self._log_replan(f"[Replan] Robot {rid} replan failed (no feasible path), "
                          f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f})")
                
                robot._last_replan_time = self.sim_time
                # ★ 不重置 _last_progress_pos/_time! 
                # 让死锁检测器能基于"是否真的移动了"来判断
                # 只有当robot确实移动>0.15m时才由下面的progress检测重置
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
            
            if dist_moved > MIN_PROGRESS_DIST:
                # Making progress — reset timer
                robot._last_progress_pos = robot.position
                robot._last_progress_time = self.sim_time
            else:
                # Not making progress — check how long
                stall_time = self.sim_time - robot._last_progress_time
                
                if stall_time > STALL_THRESHOLD:
                    # Robot has been stalled too long → dynamic replan
                    print(f"[Supervisor] Robot {rid} stalled for {stall_time:.1f} s; attempting replan...")
                    goal = self._navigation_goal(robot)
                    if goal:
                        new_path = self.motion_coordinator.plan_grid_lifelong(
                            rid, robot.position, goal)
                        if new_path and len(new_path) > 0:
                            robot.waypoints = list(new_path)
                            robot.current_waypoint_idx = 0
                            self._send_command_to_robot(rid, {
                                'type': 'navigate',
                                'target': list(new_path[0]),
                                'all_waypoints': [list(w) for w in new_path],
                            })
                            self._log_replan(f"[Replan] Robot {rid} stalled for {stall_time:.1f} s; replanning: "
                                  f"pos=({robot.position[0]:.2f},{robot.position[1]:.2f}) "
                                  f"→ goal={goal}, new path={len(new_path)} steps")
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
            if moved >= STALL_PROGRESS_DISTANCE:
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

    def _resolve_multi_robot_deadlock(self):
        """
        多机互锁恢复：当>=2个机器人同时停滞时，按优先级顺序逐个恢复。
        
        逻辑：
          1. 检测: 停滞>1.5s的机器人>=2个 → 互锁
          2. 排序: 按ID升序(ID小=优先级高)
          3. 最高优先级: 以其他停滞peer为静态障碍重规划
          4. 若失败: 命令其后退0.3m,再重规划
          5. 等最高优先级移动后,处理下一个
          6. 冷却: 恢复后3秒内不再触发
        """
        DEADLOCK_STALL_THRESHOLD = 1.5  # s — 停滞超过此时间算"停住"
        DEADLOCK_COOLDOWN = 2.0          # s — 恢复后冷却期
        REVERSE_DIST = 0.3               # m — 后退距离
        MOVE_CONFIRM_DIST = 0.3          # m — 确认移动的距离

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
            if self._teleport_stalled_group(sorted(group)):
                self._deadlock_state = 'IDLE'
                self._deadlock_queue = []
                self._deadlock_cooldown_until = self.sim_time + 3.0
                return
        
        # 冷却期检查
        if not hasattr(self, '_deadlock_cooldown_until'):
            self._deadlock_cooldown_until = 0.0
        if self.sim_time < self._deadlock_cooldown_until:
            return
        
        # 恢复流程状态机
        if not hasattr(self, '_deadlock_state'):
            self._deadlock_state = 'IDLE'  # IDLE / RESOLVING / WAIT_MOVE
            self._deadlock_queue = []       # 待恢复队列 [rid, ...]
            self._deadlock_current = None   # 当前正在恢复的机器人
            self._deadlock_start_pos = None # 恢复开始时的位置
        
        # ── 状态: WAIT_MOVE — 等待当前机器人移动确认 ──
        if self._deadlock_state == 'WAIT_MOVE':
            robot = self.robots.get(self._deadlock_current)
            if robot and self._deadlock_start_pos:
                moved = math.hypot(
                    robot.position[0] - self._deadlock_start_pos[0],
                    robot.position[1] - self._deadlock_start_pos[1])
                if moved > MOVE_CONFIRM_DIST:
                    print(f"[Deadlock] Robot {self._deadlock_current} "
                          f"moved {moved:.2f} m; recovery confirmed")
                    # 处理队列中下一个
                    if self._deadlock_queue:
                        self._resolve_next_in_queue()
                    else:
                        self._deadlock_state = 'IDLE'
                        self._deadlock_cooldown_until = self.sim_time + DEADLOCK_COOLDOWN
                        print(f"[Deadlock] All interlocked robots recovered")
                else:
                    # 等待超时(5s)→ 尝试后退
                    if not hasattr(self, '_deadlock_wait_start'):
                        self._deadlock_wait_start = self.sim_time
                    if self.sim_time - self._deadlock_wait_start > 5.0:
                        # 重规划失败/无法移动 → 命令后退
                        self._command_reverse(self._deadlock_current, REVERSE_DIST)
                        self._deadlock_wait_start = self.sim_time
            return
        
        # ── 状态: RESOLVING — 正在处理队列 ──
        if self._deadlock_state == 'RESOLVING':
            return  # 等 WAIT_MOVE 完成
        
        # ── 状态: IDLE — 检测是否有多机互锁 ──
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
            return  # 不构成互锁
        
        # 检测到互锁！
        stalled_robots.sort(key=lambda x: x[0])  # 按ID升序
        stall_info = ", ".join(f"Robot {r} (stalled {t:.1f} s)" for r, t in stalled_robots)
        print(f"[Deadlock] Interlock detected among {len(stalled_robots)} robots: {stall_info}")
        
        # ★ 优先检测迎面对撞(同走廊对向) → 低优先级后退让路
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
                
                self._send_command_to_robot(low_rid, {
                    'type': 'navigate',
                    'target': list(retreat_pos),
                    'all_waypoints': [list(retreat_pos)],
                })
                print(f"[Deadlock] Robot {low_rid} reversed to "
                      f"({retreat_pos[0]:.2f},{retreat_pos[1]:.2f})")
                low_robot._last_replan_time = self.sim_time + 3.0
            
            # 高优先级robot: 立即重规划
            goal_high = getattr(high_robot, 'goal_location', None)
            if goal_high:
                new_path = self.motion_coordinator.plan_grid_lifelong(
                    high_rid, high_robot.position, goal_high)
                if new_path:
                    high_robot.waypoints = list(new_path)
                    high_robot.current_waypoint_idx = 0
                    self._send_command_to_robot(high_rid, {
                        'type': 'navigate',
                        'target': list(new_path[0]),
                        'all_waypoints': [list(w) for w in new_path],
                    })
                    print(f"[Deadlock] Robot {high_rid} replan succeeded: {len(new_path)} steps")
            
            self._deadlock_state = 'IDLE'
            self._deadlock_cooldown_until = self.sim_time + 2.0
            return
        
        # ★ 非迎面: 按优先级顺序逐个恢复
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
            self._dispatch_plan(rid, new_path, delay=0.0)
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

            if self._teleport_stalled_group([rid]):
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
        """检测是否有两个robot在同一走廊迎面对撞。
        
        支持两种情况:
          1. 垂直走廊: |x1-x2|<0.8m, 方向y相反 (一北一南)
          2. 水平走廊: |y1-y2|<0.8m, 方向x相反 (一东一西)
        
        返回: (higher_priority_rid, lower_priority_rid) 或 None
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
        为指定机器人解锁：以其他停滞peer为静态障碍重规划。
        """
        robot = self.robots.get(rid)
        if not robot:
            self._resolve_next_in_queue()
            return
        
        goal = self._navigation_goal(robot)
        if not goal:
            self._resolve_next_in_queue()
            return
        
        # 重规划(plan_grid_lifelong已经把其他peer的reservation当障碍)
        new_path = self.motion_coordinator.plan_grid_lifelong(
            rid, robot.position, goal)
        
        if new_path and len(new_path) > 0:
            robot.waypoints = list(new_path)
            robot.current_waypoint_idx = 0
            self._send_command_to_robot(rid, {
                'type': 'navigate',
                'target': list(new_path[0]),
                'all_waypoints': [list(w) for w in new_path],
            })
            print(f"[Deadlock] Priority recovery: Robot {rid} replanning "
                  f"(avoiding {obstacle_rids}); new path={len(new_path)} steps")
            # 进入等待移动确认
            self._deadlock_current = rid
            self._deadlock_start_pos = tuple(robot.position)
            self._deadlock_state = 'WAIT_MOVE'
            self._deadlock_wait_start = self.sim_time
        else:
            # 重规划失败 → 后退再试
            print(f"[Deadlock] Robot {rid} replan failed; commanding reverse by {0.3} m")
            self._command_reverse(rid, 0.3)
            self._deadlock_current = rid
            self._deadlock_start_pos = tuple(robot.position)
            self._deadlock_state = 'WAIT_MOVE'
            self._deadlock_wait_start = self.sim_time
    
    def _command_reverse(self, rid, dist):
        """Command a retreat only when the complete segment is safe."""
        robot = self.robots.get(rid)
        if not robot:
            return
        # 计算后退目标: 沿当前朝向反方向
        heading = getattr(robot, 'heading', 0.0)
        rx = robot.position[0] - dist * math.cos(heading)
        ry = robot.position[1] - dist * math.sin(heading)
        target = (rx, ry)
        if not self.motion_coordinator._segment_clear(robot.position, target):
            self._hold_robot(rid, 2.0)
            return False
        for peer_id, peer in self.robots.items():
            if peer_id != rid and math.hypot(rx - peer.position[0],
                                             ry - peer.position[1]) < 0.6:
                self._hold_robot(rid, 2.0)
                return False
        # 发送后退点作为导航目标
        self._send_command_to_robot(rid, {
            'type': 'navigate',
            'target': [rx, ry],
            'all_waypoints': [[rx, ry]],
        })
        print(f"[Deadlock] Robot {rid} reversed to ({rx:.2f}, {ry:.2f})")
        return True
    

    def _dispatch_delayed_robots(self):
        """Check and dispatch robots that were waiting due to temporal conflicts."""
        if not getattr(self, "system_ready", True):
            return
        for rid, robot in self.robots.items():
            if (robot.pending_waypoints is not None and
                    self.sim_time >= robot.dispatch_not_before):
                pending = robot.pending_waypoints
                self._dispatch_plan(rid, pending, delay=0.0)
    
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
            
            # Check low battery — only trigger return-to-charge when IDLE and
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

        We do NOT advance the waypoint based on speculative motion —
        we wait until the robot actually drives close enough.

        The robot controller is the source of truth for movement; this
        method only reflects what's already happened on the physics
        side and triggers state transitions when goals are reached.
        """
        robot = self.robots[robot_id]

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
            # Reached this waypoint — advance.
            robot.current_waypoint_idx += 1

            if robot.current_waypoint_idx >= len(robot.waypoints):
                # All waypoints consumed — robot has reached the final goal.
                # _handle_goal_reached fires the EN_ROUTE_PICKUP →
                # EN_ROUTE_DELIVERY → IDLE transitions.
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
                    # Spurious — robot is not near goal. Ignore.
                    return
        
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
                delivery_path = self.motion_coordinator.plan_path_for_robot(
                    robot_id, robot.position,
                    robot.current_task.delivery_location)
            if delivery_path:
                robot.state = RobotState.CARRYING
                robot.current_task.status = TaskStatus.IN_PROGRESS
                robot.current_task.pickup_time = self.sim_time
                robot.goal_location = robot.current_task.delivery_location
                robot.waypoints = delivery_path
                robot.current_waypoint_idx = 0
                robot.state = RobotState.EN_ROUTE_DELIVERY
                self._dispatch_plan(robot_id, delivery_path)
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
            # Task complete — robot is now IDLE in place.
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
            # Robot reached a rest node (or its initial home spot).
            # Go IDLE; reserve the *actual current node* — not the
            # original home — so the static reservation matches reality.
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
            self.motion_coordinator.clear_robot_path(robot_id)
            self.motion_coordinator.release_lifelong(robot_id)
            self.motion_coordinator.release_robot_grid(robot_id)
            cur_node = self.motion_coordinator.graph.get_nearest_node(
                robot.position)
            if cur_node:
                self.motion_coordinator.lifelong.reserve_static(
                    robot_id, cur_node)
        
        elif robot.state == RobotState.RETURNING_TO_CHARGE:
            # Arrived at charging station — now actually start charging.
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
            # Already charging — nothing to do; battery handled elsewhere
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
            # No home defined → fall back to direct IDLE
            robot = self.robots[robot_id]
            robot.waypoints = []
            robot.current_waypoint_idx = 0
            robot.state = RobotState.IDLE
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
            self.motion_coordinator.clear_robot_path(robot_id)
            self.motion_coordinator.reserve_home(robot_id, home_xy)
            return

        # Refresh real position before planning
        self._get_robot_positions_from_webots()

        # Lifelong plan to home XY — encode as a temporary location key
        # by inserting it into ALL_LOCATIONS-equivalent lookup. Easier:
        # call plan_lifelong with a synthetic goal — but our coordinator
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
            self.motion_coordinator.reserve_home(robot_id, home_xy)
            return
        
        robot.waypoints = waypoints
        robot.current_waypoint_idx = 0
        robot.state = RobotState.RETURNING_HOME
        self._send_command_to_robot(robot_id, {
            'type': 'navigate',
            'target': list(waypoints[0]),
            'all_waypoints': [list(wp) for wp in waypoints],
        })
        print(f"[T={self.sim_time:.1f}] Robot {robot_id} returning to home "
              f"spot {home_xy}")

    def _relocate_idle_robots(self):
        """
        Lazy relocation — for each IDLE robot not already at a REST_NODE,
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
            robot.waypoints = wpts
            robot.current_waypoint_idx = 0
            robot.state = RobotState.RETURNING_HOME
            # Send first waypoint to robot
            self._send_command_to_robot(rid, {
                'type': 'navigate',
                'target': list(wpts[0]),
                'all_waypoints': [list(wp) for wp in wpts],
            })

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
            # planning — robot.position may be stale if this assignment
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
        print(f"{'='*60}\n")
        
        dt = self.timestep / 1000.0  # Convert ms to seconds
        
        completed_normally = False
        while self.running:
            step_status = self.supervisor.step(self.timestep)
            if step_status == -1:
                break
            self.sim_time += dt
            self.step_count += 1
            
            # Check simulation end
            if AUTO_STOP_SIMULATION and self.sim_time >= SIM_DURATION:
                self.running = False
                completed_normally = True
                break
            
            # 1. Get robot positions from Webots
            self._get_robot_positions_from_webots()
            
            # 1.5 Broadcast all robot positions for peer conflict avoidance
            # EVERY STEP — peer avoidance is safety-critical, no delays allowed.
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
            
            # 3.5 Advance lifelong planner clock once per ~1.5 s of
            # sim-time. This corresponds roughly to one graph edge
            # traversal at 0.22 m/s and 2 m edge length.
            if not hasattr(self, '_next_ll_tick'):
                self._next_ll_tick = 1.5
            if self.sim_time >= self._next_ll_tick:
                self.motion_coordinator.lifelong_tick(1)
                self._next_ll_tick += 1.5
            
            # 3.6 Lazy relocation — IDLE robots not at a rest node
            # walk to nearest one. Runs every ~0.5s to avoid spamming
            # plan() calls.
            if not hasattr(self, '_next_reloc_tick'):
                self._next_reloc_tick = 1.0
            if self.sim_time >= self._next_reloc_tick:
                self._relocate_idle_robots()
                self._next_reloc_tick += 0.5
            
            # 3.6b Progress monitor — detect stalled robots and replan
            if not hasattr(self, '_next_conflict_scan'):
                self._next_conflict_scan = self.sim_time
            if self.sim_time >= self._next_conflict_scan:
                self._proactive_path_conflict_scan()
                self._next_conflict_scan += 0.25
            if not hasattr(self, '_next_progress_tick'):
                self._next_progress_tick = 2.0
            if self.sim_time >= self._next_progress_tick:
                self._monitor_progress_and_replan()
                self._recover_long_stalled_robots()
                if ENABLE_LEGACY_INTERLOCK_RECOVERY:
                    self._resolve_multi_robot_deadlock()
                self._next_progress_tick += 1.0  # check every 1s
            
            # 3.7 Level 2 — Deadlock detection + priority inheritance.
            # Every ~0.5s scan for stuck robots; force lower-priority
            # ones to yield when 2+ are blocking each other.
            if not hasattr(self, '_next_deadlock_tick'):
                self.motion_coordinator.init_deadlock_monitor()
                self._next_deadlock_tick = 1.0
            if self.sim_time >= self._next_deadlock_tick:
                rs_dict = {rid: {
                    "position": r.position,
                    "state": r.state,
                    "goal_location": getattr(r, 'goal_location', None),
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
                                rb.waypoints = list(new_p)
                                rb.current_waypoint_idx = 0
                                self._send_command_to_robot(rid, {
                                    'type': 'navigate',
                                    'target': list(new_p[0]),
                                    'all_waypoints': [list(w) for w in new_p],
                                })
                self._next_deadlock_tick += 0.5
            
            # 3.8 Level 3 — Periodic RHCR (every 5 sim-seconds).
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
            if self.step_count % int(5.0 / dt) == 0:
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
