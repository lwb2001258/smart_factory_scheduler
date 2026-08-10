"""
PPO Training Script for the Deep RL Task Scheduler.

Trains the PPO agent in a simulated environment (standalone, no Webots required).
Uses the factory simulation logic directly for fast training iterations.

Training approach:
- Curriculum learning: start with few robots, increase gradually
- Reward shaping: task completion + idle penalty + congestion penalty
- Experience replay with GAE (Generalized Advantage Estimation)

Usage:
    python train_ppo.py --episodes 5000 --save-dir ../results/models/
"""

import os
import sys
import math
import json
import argparse
import numpy as np
from typing import List, Dict, Tuple, Optional
import copy

# Add controller path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 
                                'controllers', 'factory_supervisor'))

from config import (
    SCENARIOS, RL_CONFIG, RL_ENVIRONMENT_VERSION, MAX_ROBOTS, TIMESTEP,
    RobotState, TaskStatus,
    REWARD_TASK_COMPLETE, REWARD_IDLE_PENALTY, REWARD_CONGESTION_PENALTY,
    REWARD_DISTANCE_PENALTY, REWARD_BALANCE_BONUS,
    REWARD_TASK_FAILURE,
    BATTERY_CAPACITY, BATTERY_DRAIN_RATE, CHARGING_STATIONS,
    FULL_BATTERY_THRESHOLD, INITIAL_BATTERY_MIN, INITIAL_BATTERY_MAX,
    LOW_BATTERY_THRESHOLD, REST_NODES, TASK_ABORT_BATTERY_THRESHOLD,
    WAYPOINTS
)
from task_generator import TaskGenerator
from motion_coordinator import MotionCoordinator
from schedulers import Assignment, PPONetwork, RLScheduler
from training_scenarios import factory_free_positions
from training_scenarios import factory_scenario
from rl_environment import (RLEnvironmentConfig, RewardConfig,
                            SchedulingEnvironment)


class PPOBuffer:
    """Experience buffer for PPO training with GAE."""
    
    def __init__(self, capacity: int):
        self.capacity = capacity
        self.states = []
        self.actions = []
        self.rewards = []
        self.values = []
        self.log_probs = []
        self.dones = []
        self.action_masks = []
        self.advantages = []
        self.returns = []
    
    def add(self, state, action, reward, value, log_prob, done,
            action_mask=None):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.values.append(value)
        self.log_probs.append(log_prob)
        self.dones.append(done)
        if action_mask is None:
            action_mask = np.ones(1, dtype=bool)
        self.action_masks.append(np.asarray(action_mask, dtype=bool).copy())
    
    def compute_gae(self, gamma: float = 0.99, lam: float = 0.95,
                    last_value: float = 0.0):
        """Compute Generalized Advantage Estimation."""
        n = len(self.rewards)
        self.advantages = [0.0] * n
        self.returns = [0.0] * n
        
        gae = 0.0
        for t in reversed(range(n)):
            if t == n - 1:
                next_value = last_value
            else:
                next_value = self.values[t + 1]
            
            delta = self.rewards[t] + gamma * next_value * (1 - self.dones[t]) - self.values[t]
            gae = delta + gamma * lam * (1 - self.dones[t]) * gae
            self.advantages[t] = gae
            self.returns[t] = gae + self.values[t]
        
        # Normalize advantages
        adv_array = np.array(self.advantages)
        if len(adv_array) > 1:
            self.advantages = list((adv_array - adv_array.mean()) / (adv_array.std() + 1e-8))
    
    def clear(self):
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.values.clear()
        self.log_probs.clear()
        self.dones.clear()
        self.action_masks.clear()
        self.advantages.clear()
        self.returns.clear()
    
    def __len__(self):
        return len(self.states)
    
    @property
    def is_full(self):
        return len(self.states) >= self.capacity


class PPOTrainer:
    """
    PPO Trainer for the scheduling policy.
    
    Uses a simplified environment that runs the scheduling logic
    without Webots physics for fast training.
    """
    
    def __init__(self, config: dict = None, seed: int = 42):
        self.config = config or RL_CONFIG
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        
        # State and action dimensions
        # PPO now shares the exact observation/action contract used by
        # DQN/SARSA: fixed robot and task slots, pairwise assignment actions,
        # an explicit NO_OP, and the same feasibility mask.
        self.env_config = RLEnvironmentConfig()
        self.reward_config = RewardConfig(
            task_completion=self.config.get('reward_completion', 12.0),
            valid_assignment=self.config.get('reward_assignment', 0.75),
            priority=self.config.get('reward_priority', 0.4),
            distance_weight=self.config.get('penalty_distance', -0.05),
            waiting_weight=self.config.get('penalty_waiting', -0.02),
            age_bonus=self.config.get('reward_age_bonus', 0.3),
        )
        probe = SchedulingEnvironment(
            self.env_config, self.reward_config,
            simulation_mode="abstract")
        self.state_dim = probe.observation_dim
        self.action_dim = probe.action_dim
        
        # Network
        self.network = PPONetwork(
            self.state_dim, self.action_dim,
            self.config['hidden_size']
        )
        
        # Training buffer
        self.buffer = PPOBuffer(self.config['buffer_size'])
        
        # Learning rate
        self.lr = self.config['learning_rate']
        
        # Training stats
        self.episode_rewards = []
        self.episode_lengths = []
        self.validation_history = []
        self.best_validation_score = -math.inf
        self.training_step = 0
        self.update_diagnostics = []
        
        print(f"PPO Trainer initialized:")
        print(f"  State dim: {self.state_dim}")
        print(f"  Action dim: {self.action_dim}")
        print(f"  Hidden size: {self.config['hidden_size']}")

    @staticmethod
    def _masked_distribution(probabilities: np.ndarray,
                             action_mask: np.ndarray) -> np.ndarray:
        """Return one numerically safe distribution over legal actions."""
        mask = np.asarray(action_mask, dtype=bool)
        if mask.shape != probabilities.shape:
            raise ValueError("PPO action mask shape mismatch")
        masked = np.where(mask, probabilities, 0.0)
        total = float(masked.sum())
        if not np.isfinite(total) or total <= 1e-12:
            legal = np.flatnonzero(mask)
            if legal.size == 0:
                raise ValueError("PPO state has no legal action")
            masked = np.zeros_like(probabilities, dtype=float)
            masked[legal] = 1.0 / legal.size
            return masked
        return masked / total

    def train_pairwise_episode(self, num_robots: int, max_tasks: int,
                               seed: int, training: bool = True) -> dict:
        """Train/evaluate PPO in the same environment as DQN and SARSA."""
        robots, tasks, context = factory_scenario(
            seed, min_robots=num_robots, max_robots=num_robots,
            max_tasks=self.env_config.max_tasks,
            max_generated_tasks=max_tasks)
        env = SchedulingEnvironment(
            self.env_config, self.reward_config,
            simulation_mode="abstract")
        state, _ = env.reset(robots, tasks, context, seed=seed)
        total_reward = 0.0
        steps = 0
        invalid_actions = 0
        terminated = truncated = False
        while not (terminated or truncated):
            mask = env.get_action_mask()
            probabilities, value = self.network.forward(state)
            masked = self._masked_distribution(probabilities, mask)
            if training:
                action = int(self.rng.choice(self.action_dim, p=masked))
            else:
                action = int(np.argmax(masked))
            log_prob = math.log(max(float(masked[action]), 1e-12))
            next_state, reward, terminated, truncated, info = env.step(action)
            invalid_actions += int(bool(info.get("invalid_action", False)))
            if training:
                self.buffer.add(
                    state, action, float(reward), value, log_prob,
                    float(terminated or truncated), mask)
            state = next_state
            total_reward += float(reward)
            steps += 1
        # PPO is an on-policy rollout method.  Keep complete episodes in the
        # rollout until the configured capacity is reached instead of doing a
        # noisy update from only ~10 samples after every episode.  Episode
        # boundaries are retained in ``dones`` so GAE cannot leak across them.
        if training:
            self.episode_rewards.append(total_reward)
            self.episode_lengths.append(steps)
        if training and self.buffer.is_full:
            self._update_network(last_value=0.0)
        completed_tasks = [task for task in env._tasks
                           if task.status == TaskStatus.COMPLETED]
        completed = len(completed_tasks)
        waiting_values = [float(task.waiting_time) for task in completed_tasks
                          if task.waiting_time is not None]
        completion_values = [float(task.completion_time - task.arrival_time)
                             for task in completed_tasks
                             if task.completion_time is not None]
        return {
            "reward": total_reward,
            "tasks_completed": int(completed),
            "tasks_generated": len(env._tasks),
            "tasks_failed": sum(task.status == TaskStatus.FAILED
                                for task in env._tasks),
            "invalid_actions": invalid_actions,
            "steps": steps,
            "mean_waiting_time": (float(np.mean(waiting_values))
                                  if waiting_values else 0.0),
            "mean_completion_time": (float(np.mean(completion_values))
                                     if completion_values else 0.0),
            "makespan": float(env._context.current_time),
            "total_distance": float(sum(
                state.get("total_distance", 0.0)
                for state in env._robots.values())),
        }
    
    def _legacy_train_episode(self, num_robots: int, task_interval: float,
                              episode_length: float = 300.0, seed: int = 42,
                              training: bool = True) -> dict:
        """
        Retained only to reproduce pre-pairwise PPO experiments.

        New training and validation use ``train_pairwise_episode`` so PPO,
        DQN and SARSA share one environment contract. Do not call this method
        for new checkpoints.
        
        Returns episode statistics.
        """
        # Initialize environment
        task_gen = TaskGenerator(mean_interval=task_interval, seed=seed,
                                 initial_task_immediately=True)
        coordinator = MotionCoordinator(num_active_robots=num_robots)
        coordinator.lifelong_reset()
        episode_rng = np.random.default_rng(seed)
        # RLScheduler's inference constructor intentionally requires a model
        # file for Webots. Validation instead evaluates the current in-memory
        # network, so keep its in-memory mode but select actions deterministically.
        scheduler = RLScheduler(training=True, seed=seed,
                                deterministic=not training)
        scheduler.network = self.network  # Share network
        
        # Robot states
        # Use only centres that are free in the same inflated occupancy grid
        # as runtime A*. Some legacy graph/rest nodes lie in policy keep-outs.
        initial_positions = factory_free_positions()
        
        robot_states = {}
        robot_tasks = {}
        robot_waypoints = {}
        charging_ready_at = {}
        idle_since = {}
        
        for rid in range(1, num_robots + 1):
            pos = initial_positions[rid - 1]
            robot_states[rid] = {
                'position': pos,
                'heading': 0.0,
                'state': RobotState.IDLE,
                'battery': float(episode_rng.uniform(INITIAL_BATTERY_MIN,
                                                     INITIAL_BATTERY_MAX)),
                'current_task': None,
                'has_task': False,
                'goal_location': None,
            }
            robot_tasks[rid] = None
            robot_waypoints[rid] = []
            charging_ready_at[rid] = None
            idle_since[rid] = 0.0
        
        coordinator.set_priorities(list(range(1, num_robots + 1)))
        for rid in range(1, num_robots + 1):
            coordinator.reserve_home(rid, robot_states[rid]['position'])
        
        # Episode loop
        dt = 0.1  # 100ms steps for training (faster than real simulation)
        sim_time = 0.0
        total_reward = 0.0
        steps = 0
        tasks_completed = 0
        tasks_failed = 0
        transition_index_by_task = {}
        next_lifelong_tick = 1.5

        def send_to_charging(rid: int) -> bool:
            """Use the shortest reachable factory A* route to a charger."""
            rs = robot_states[rid]
            coordinator.release_lifelong(rid)
            coordinator.release_robot_grid(rid)
            candidates = []
            for station, target in CHARGING_STATIONS.items():
                path = coordinator.grid_planner.plan(
                    rs['position'], target, smooth=True)
                if path:
                    points = [rs['position']] + list(path)
                    distance = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                   for a, b in zip(points, points[1:]))
                    candidates.append((distance, station))
                elif math.hypot(target[0] - rs['position'][0],
                                target[1] - rs['position'][1]) <= 0.5:
                    rs['state'] = RobotState.CHARGING
                    rs['goal_location'] = station
                    charging_ready_at[rid] = sim_time + 5.0
                    return True
            if not candidates:
                rs['state'] = RobotState.WAITING
                rs['goal_location'] = None
                robot_waypoints[rid] = []
                return False
            _, station = min(candidates, key=lambda item: item[0])
            path = coordinator.plan_grid_lifelong(
                rid, rs['position'], station)
            if path is None:
                path = coordinator.plan_path_for_robot(
                    rid, rs['position'], station)
            if not path:
                rs['state'] = RobotState.WAITING
                rs['goal_location'] = None
                robot_waypoints[rid] = []
                return False
            rs['state'] = RobotState.RETURNING_TO_CHARGE
            rs['goal_location'] = station
            robot_waypoints[rid] = list(path)
            return True
        
        while sim_time < episode_length:
            sim_time += dt
            steps += 1

            if sim_time >= next_lifelong_tick:
                coordinator.lifelong_tick(1)
                next_lifelong_tick += 1.5
            
            # Generate tasks
            new_task = task_gen.update(sim_time)
            
            # Simulate movement
            completed_this_step = 0
            for rid in range(1, num_robots + 1):
                rs = robot_states[rid]

                if rs['state'] in (RobotState.EN_ROUTE_PICKUP,
                                   RobotState.EN_ROUTE_DELIVERY,
                                   RobotState.RETURNING_HOME,
                                   RobotState.RETURNING_TO_CHARGE):
                    rs['battery'] = max(
                        0.0, rs['battery'] - BATTERY_DRAIN_RATE * dt)
                elif rs['state'] == RobotState.CHARGING:
                    if (charging_ready_at[rid] is not None and
                            sim_time >= charging_ready_at[rid]):
                        rs['battery'] = float(episode_rng.uniform(
                            FULL_BATTERY_THRESHOLD, BATTERY_CAPACITY))
                        rs['state'] = RobotState.IDLE
                        rs['goal_location'] = None
                        charging_ready_at[rid] = None
                        idle_since[rid] = sim_time
                    continue

                active_task = robot_tasks[rid]
                if (active_task is not None and
                        rs['battery'] < TASK_ABORT_BATTERY_THRESHOLD):
                    active_task.status = TaskStatus.FAILED
                    active_task.assigned_robot = None
                    tasks_failed += 1
                    total_reward += REWARD_TASK_FAILURE
                    transition_idx = transition_index_by_task.pop(
                        active_task.task_id, None)
                    if training and transition_idx is not None:
                        self.buffer.rewards[transition_idx] += REWARD_TASK_FAILURE
                    robot_tasks[rid] = None
                    rs['current_task'] = None
                    rs['has_task'] = False
                    robot_waypoints[rid] = []
                    send_to_charging(rid)
                    continue
                if (active_task is None and
                        rs['battery'] < LOW_BATTERY_THRESHOLD and
                        rs['state'] not in (
                            RobotState.RETURNING_TO_CHARGE,
                            RobotState.CHARGING)):
                    send_to_charging(rid)

                if rs['state'] in (RobotState.IDLE, RobotState.WAITING):
                    if (rs['state'] == RobotState.WAITING and
                            rs['battery'] < LOW_BATTERY_THRESHOLD):
                        send_to_charging(rid)
                    elif (rs['state'] == RobotState.IDLE and
                          sim_time - idle_since[rid] >= 1.0):
                        current_node = coordinator.graph.get_nearest_node(
                            rs['position'])
                        if current_node not in REST_NODES:
                            rest = coordinator.find_nearest_rest_node(
                                rs['position'], exclude_robot_id=rid)
                            if rest is not None:
                                target_node, _ = rest
                                coordinator.release_home(rid)
                                nodes = coordinator.lifelong.plan(
                                    rid, current_node, target_node)
                                if nodes:
                                    waypoints = [WAYPOINTS[node]
                                                 for node in nodes]
                                    if (waypoints and math.hypot(
                                            waypoints[0][0] - rs['position'][0],
                                            waypoints[0][1] - rs['position'][1])
                                            < 0.05):
                                        waypoints.pop(0)
                                    if waypoints:
                                        robot_waypoints[rid] = waypoints
                                        rs['state'] = RobotState.RETURNING_HOME
                    continue
                
                if robot_waypoints[rid]:
                    target = robot_waypoints[rid][0]
                    pos = rs['position']
                    dx = target[0] - pos[0]
                    dz = target[1] - pos[1]
                    dist = (dx*dx + dz*dz) ** 0.5
                    
                    if dist < 0.4:
                        robot_waypoints[rid].pop(0)
                        if not robot_waypoints[rid]:
                            task = robot_tasks[rid]
                            if task and rs['state'] == RobotState.EN_ROUTE_PICKUP:
                                task.status = TaskStatus.IN_PROGRESS
                                task.pickup_time = sim_time
                                rs['state'] = RobotState.EN_ROUTE_DELIVERY
                                path = coordinator.plan_grid_lifelong(
                                    rid, pos, task.delivery_location)
                                if path is None:
                                    path = coordinator.plan_path_for_robot(
                                        rid, pos, task.delivery_location)
                                if path:
                                    robot_waypoints[rid] = list(path)
                                else:
                                    # Do not leave an active robot forever in
                                    # EN_ROUTE_DELIVERY with no waypoints.
                                    task.status = TaskStatus.FAILED
                                    task.assigned_robot = None
                                    tasks_failed += 1
                                    total_reward += REWARD_TASK_FAILURE
                                    transition_idx = transition_index_by_task.pop(
                                        task.task_id, None)
                                    if training and transition_idx is not None:
                                        self.buffer.rewards[transition_idx] += (
                                            REWARD_TASK_FAILURE)
                                    robot_tasks[rid] = None
                                    rs['state'] = RobotState.IDLE
                                    rs['current_task'] = None
                                    rs['has_task'] = False
                                    rs['goal_location'] = None
                                    coordinator.release_lifelong(rid)
                                    coordinator.release_robot_grid(rid)
                                    idle_since[rid] = sim_time
                            elif task and rs['state'] == RobotState.EN_ROUTE_DELIVERY:
                                task.status = TaskStatus.COMPLETED
                                task.completion_time = sim_time
                                completed_this_step += 1
                                tasks_completed += 1
                                total_reward += REWARD_TASK_COMPLETE
                                transition_idx = transition_index_by_task.pop(
                                    task.task_id, None)
                                if training and transition_idx is not None:
                                    self.buffer.rewards[transition_idx] += (
                                        REWARD_TASK_COMPLETE)
                                robot_tasks[rid] = None
                                rs['state'] = RobotState.IDLE
                                rs['current_task'] = None
                                rs['has_task'] = False
                                rs['goal_location'] = None
                                coordinator.clear_robot_path(rid)
                                coordinator.release_lifelong(rid)
                                coordinator.release_robot_grid(rid)
                                idle_since[rid] = sim_time
                            elif rs['state'] == RobotState.RETURNING_TO_CHARGE:
                                rs['state'] = RobotState.CHARGING
                                robot_waypoints[rid] = []
                                charging_ready_at[rid] = sim_time + 5.0
                                coordinator.release_lifelong(rid)
                                coordinator.release_robot_grid(rid)
                            elif rs['state'] == RobotState.RETURNING_HOME:
                                rs['state'] = RobotState.IDLE
                                rs['goal_location'] = None
                                idle_since[rid] = sim_time
                                current_node = (
                                    coordinator.graph.get_nearest_node(
                                        rs['position']))
                                if current_node:
                                    coordinator.lifelong.reserve_static(
                                        rid, current_node)
                    else:
                        speed = min(0.22, dist * 0.5) * dt
                        rs['position'] = (
                            pos[0] + dx / dist * speed,
                            pos[1] + dz / dist * speed
                        )
                        active_task = robot_tasks[rid]
                        if (active_task is not None and
                                rs['state'] == RobotState.EN_ROUTE_PICKUP):
                            transition_idx = transition_index_by_task.get(
                                active_task.task_id)
                            distance_reward = speed * REWARD_DISTANCE_PENALTY
                            total_reward += distance_reward
                            if training and transition_idx is not None:
                                self.buffer.rewards[transition_idx] += distance_reward
            
            # Assign tasks using RL scheduler
            pending = task_gen.get_pending_tasks()
            if pending:
                congestion = coordinator.get_congestion_map()
                
                # Encode state before assignment
                state = scheduler.encode_state(robot_states, pending, congestion)
                
                result = scheduler.assign_task(pending, robot_states, congestion)
                
                if result:
                    rid, task = result
                    path = coordinator.plan_grid_lifelong(
                        rid, robot_states[rid]['position'], task.pickup_location)
                    if path is None:
                        path = coordinator.plan_path_for_robot(
                            rid, robot_states[rid]['position'], task.pickup_location)
                    if path:
                        task.status = TaskStatus.ASSIGNED
                        task.assigned_robot = rid
                        task.assignment_time = sim_time
                        robot_tasks[rid] = task
                        robot_states[rid]['state'] = RobotState.EN_ROUTE_PICKUP
                        robot_states[rid]['current_task'] = task
                        robot_states[rid]['has_task'] = True
                        robot_states[rid]['goal_location'] = task.pickup_location
                        coordinator.release_home(rid)
                        robot_waypoints[rid] = list(path)
                        idle_since[rid] = sim_time
                        scheduler.on_assignment_committed(
                            Assignment(rid, task))
                    else:
                        scheduler.on_assignment_rejected(
                            Assignment(rid, task), "pickup_path_unreachable")
                        continue
                    
                    # Compute reward
                    reward = scheduler.compute_reward(
                        robot_states, 0, 0, 0.0)
                    total_reward += reward
                    
                    # Store experience
                    action = rid - 1
                    if training:
                        experience = (scheduler.experience_buffer[-1]
                                      if scheduler.experience_buffer else {})
                        value = experience.get('value', 0.0)
                        log_prob = experience.get('log_prob', 0.0)
                        done = 1.0 if sim_time >= episode_length else 0.0
                        self.buffer.add(
                            state, action, reward, value, log_prob, done)
                        transition_index_by_task[task.task_id] = len(self.buffer) - 1
            
            # Periodic reward for system state
            if steps % 50 == 0:
                idle_count = sum(1 for rs in robot_states.values() 
                               if rs['state'] == RobotState.IDLE)
                step_reward = idle_count * REWARD_IDLE_PENALTY * 50
                total_reward += step_reward
                if training and len(self.buffer) > 0:
                    self.buffer.rewards[-1] += step_reward
        
        # Flush every episode.  The previous implementation discarded useful
        # short rollouts until the cross-episode buffer happened to fill.
        if training and len(self.buffer) > 0:
            final_pending = task_gen.get_pending_tasks()
            final_state = scheduler.encode_state(
                robot_states, final_pending, coordinator.get_congestion_map())
            _, last_value = self.network.forward(final_state)
            self._update_network(last_value=last_value)
        
        stats = {
            'reward': total_reward,
            'tasks_completed': tasks_completed,
            'tasks_generated': task_gen.task_counter,
            'tasks_failed': tasks_failed,
            'steps': steps,
        }
        
        if training:
            self.episode_rewards.append(total_reward)
            self.episode_lengths.append(steps)
        
        return stats

    def evaluate(self) -> dict:
        """Evaluate on the same five held-out seed domain as DQN/SARSA."""
        cases = []
        validation_seeds = range(200000 + self.seed * 100,
                                 200005 + self.seed * 100)
        for index, seed in enumerate(validation_seeds):
            robots, tasks = ((3, 5) if index % 3 == 0 else
                             ((5, 8) if index % 3 == 1 else (8, 12)))
            cases.append((robots, tasks, seed))
        results = [self.train_pairwise_episode(
            robots, tasks, seed, training=False)
            for robots, tasks, seed in cases]
        generated = sum(row['tasks_generated'] for row in results)
        completed = sum(row['tasks_completed'] for row in results)
        failed = sum(row['tasks_failed'] for row in results)
        completion_rate = completed / max(generated, 1)
        mean_reward = float(np.mean([row['reward'] for row in results]))
        invalid_actions = sum(row["invalid_actions"] for row in results)
        mean_completed = completed / max(len(results), 1)
        mean_waiting = float(np.mean(
            [row["mean_waiting_time"] for row in results]))
        mean_completion_time = float(np.mean(
            [row["mean_completion_time"] for row in results]))
        mean_makespan = float(np.mean([row["makespan"] for row in results]))
        mean_distance = float(np.mean(
            [row["total_distance"] for row in results]))
        # Completion remains the safety gate, while time and distance break
        # ties between the many finite-workload policies that all reach 100%.
        # Coefficients put the terms on comparable validation-scale ranges.
        score = (1000.0 * completion_rate
                 - 0.50 * mean_completion_time
                 - 0.25 * mean_waiting
                 - 0.10 * mean_makespan
                 - 0.05 * mean_distance
                 - 100.0 * invalid_actions
                 + 0.01 * mean_reward)
        return {
            'score': score, 'mean_reward': mean_reward,
            'completion_rate': completion_rate,
            'tasks_completed': completed, 'tasks_generated': generated,
            'tasks_failed': failed, 'invalid_actions': invalid_actions,
            'mean_completed': mean_completed,
            'mean_waiting_time': mean_waiting,
            'mean_completion_time': mean_completion_time,
            'mean_makespan': mean_makespan,
            'mean_distance': mean_distance,
        }
    
    def _update_network(self, last_value: float = 0.0):
        """Update the policy network using PPO."""
        self.buffer.compute_gae(
            gamma=self.config['gamma'],
            lam=self.config['gae_lambda'],
            last_value=last_value,
        )
        
        n = len(self.buffer)
        if n == 0:
            return
        
        states = np.array(self.buffer.states)
        actions = np.array(self.buffer.actions)
        action_masks = np.asarray(self.buffer.action_masks, dtype=bool)
        old_log_probs = np.array(self.buffer.log_probs)
        advantages = np.array(self.buffer.advantages)
        returns = np.array(self.buffer.returns)
        kl_samples = []
        entropy_samples = []
        value_error_samples = []
        clipped_samples = 0
        policy_samples = 0
        
        # PPO update epochs
        for epoch in range(self.config['num_epochs']):
            # Mini-batch updates
            indices = self.rng.permutation(n)
            batch_size = self.config['batch_size']
            
            for start in range(0, n, batch_size):
                end = min(start + batch_size, n)
                batch_idx = indices[start:end]
                
                batch_states = states[batch_idx]
                batch_actions = actions[batch_idx]
                batch_old_lp = old_log_probs[batch_idx]
                batch_masks = action_masks[batch_idx]
                batch_adv = advantages[batch_idx]
                batch_ret = returns[batch_idx]
                
                # Compute PPO gradients with the old log-probability fixed
                # for every epoch.
                for i in range(len(batch_idx)):
                    state = batch_states[i]
                    action = int(batch_actions[i])
                    
                    h1_pre = state @ self.network.W1 + self.network.b1
                    h1 = np.maximum(0, h1_pre)
                    h2_pre = h1 @ self.network.W2 + self.network.b2
                    h2 = np.maximum(0, h2_pre)
                    logits = h2 @ self.network.W_policy + self.network.b_policy
                    logits -= np.max(logits)
                    probs = np.exp(np.clip(logits, -30.0, 30.0))
                    probs /= max(np.sum(probs), 1e-8)
                    # The old log-probability was sampled from a masked and
                    # renormalised policy. Reconstruct that exact distribution
                    # for the PPO ratio, entropy and gradient. Mixing masked
                    # old probabilities with unmasked new probabilities makes
                    # the clipped objective mathematically invalid.
                    mask = batch_masks[i]
                    probs = self._masked_distribution(probs, mask)
                    value = float((h2 @ self.network.W_value +
                                   self.network.b_value)[0])
                    
                    new_log_prob = math.log(max(probs[action], 1e-8))
                    kl_samples.append(float(batch_old_lp[i] - new_log_prob))
                    
                    # PPO clipped objective
                    ratio = math.exp(new_log_prob - batch_old_lp[i])
                    clip_ratio = max(min(ratio, 
                                       1 + self.config['clip_epsilon']),
                                   1 - self.config['clip_epsilon'])
                    
                    adv = batch_adv[i]
                    
                    clipped = ((adv >= 0 and ratio > 1 + self.config['clip_epsilon'])
                               or (adv < 0 and ratio < 1 - self.config['clip_epsilon']))
                    clipped_samples += int(clipped)
                    policy_samples += 1
                    policy_scale = 0.0 if clipped else ratio * adv
                    one_hot = np.zeros(self.action_dim)
                    one_hot[action] = 1.0
                    d_logits = policy_scale * (one_hot - probs)
                    entropy = -float(np.sum(
                        probs * np.log(np.maximum(probs, 1e-8))))
                    entropy_samples.append(entropy)
                    d_logits += self.config['entropy_coeff'] * (
                        -probs * (np.log(np.maximum(probs, 1e-8)) + entropy))
                    d_logits *= mask

                    value_error = batch_ret[i] - value
                    value_error_samples.append(value_error * value_error)
                    d_value = self.config['value_coeff'] * value_error

                    gradients = {}
                    gradients['W_policy'] = np.outer(h2, d_logits)
                    gradients['b_policy'] = d_logits
                    gradients['W_value'] = h2.reshape(-1, 1) * d_value
                    gradients['b_value'] = np.array([d_value])

                    d_h2 = (self.network.W_policy @ d_logits +
                            self.network.W_value[:, 0] * d_value)
                    d_h2 *= (h2_pre > 0)
                    gradients['W2'] = np.outer(h1, d_h2)
                    gradients['b2'] = d_h2
                    d_h1 = (self.network.W2 @ d_h2) * (h1_pre > 0)
                    gradients['W1'] = np.outer(state, d_h1)
                    gradients['b1'] = d_h1

                    norm = math.sqrt(sum(
                        float(np.sum(gradient * gradient))
                        for gradient in gradients.values()))
                    scale = min(
                        1.0, self.config['max_grad_norm'] / max(norm, 1e-8))
                    for name, gradient in gradients.items():
                        setattr(self.network, name,
                                getattr(self.network, name) +
                                self.lr * scale * gradient)
        
        self.training_step += 1
        self.update_diagnostics.append({
            "update": self.training_step,
            "rollout_steps": n,
            "approx_kl": float(np.mean(kl_samples)) if kl_samples else 0.0,
            "clip_fraction": (clipped_samples / max(policy_samples, 1)),
            "policy_entropy": (float(np.mean(entropy_samples))
                               if entropy_samples else 0.0),
            "value_mse": (float(np.mean(value_error_samples))
                          if value_error_samples else 0.0),
        })
        self.buffer.clear()
    
    def train(self, num_episodes: int = 5000, save_dir: str = "models",
              curriculum: bool = True):
        """
        Full training loop with optional curriculum learning.
        
        Curriculum: starts with simple scenarios and gradually increases difficulty.
        """
        os.makedirs(save_dir, exist_ok=True)
        
        print(f"\n{'='*60}")
        print(f"PPO TRAINING - {num_episodes} episodes")
        print(f"Curriculum learning: {curriculum}")
        print(f"Save directory: {save_dir}")
        print(f"{'='*60}\n")

        # Evaluate and preserve the starting policy before the first update.
        # This is essential for resumed stage-2/champion training: a degraded
        # first update must not overwrite the stronger promoted checkpoint.
        baseline = self.evaluate()
        baseline['episode'] = 0
        self.validation_history.append(baseline)
        self.best_validation_score = baseline['score']
        self.network.save(os.path.join(
            save_dir, "ppo_model_best_validation"))
        print("Initial validation | "
              f"Completion: {baseline['completion_rate']:.1%} | "
              f"Score: {baseline['score']:.4f}")
        
        for episode in range(num_episodes):
            # Curriculum: gradually increase difficulty
            if curriculum:
                progress = episode / num_episodes
                if progress < 0.3:
                    num_robots = 3
                    task_interval = 30.0
                elif progress < 0.6:
                    num_robots = 5
                    task_interval = 15.0
                else:
                    num_robots = 8
                    task_interval = 8.0
            else:
                num_robots = 5
                task_interval = 15.0
            
            seed = self.seed + episode
            
            max_tasks = 5 if num_robots == 3 else (8 if num_robots == 5 else 12)
            stats = self.train_pairwise_episode(
                num_robots=num_robots, max_tasks=max_tasks,
                seed=seed, training=True)
            
            # Log progress
            should_evaluate = (
                (episode + 1) % self.config['eval_interval'] == 0 or
                episode + 1 == num_episodes)
            if should_evaluate:
                recent_rewards = self.episode_rewards[-100:]
                avg_reward = sum(recent_rewards) / max(len(recent_rewards), 1)
                validation = self.evaluate()
                validation['episode'] = episode + 1
                self.validation_history.append(validation)
                if validation['score'] > self.best_validation_score:
                    self.best_validation_score = validation['score']
                    best_path = os.path.join(
                        save_dir, "ppo_model_best_validation")
                    self.network.save(best_path)
                
                print(f"Episode {episode + 1}/{num_episodes} | "
                      f"Robots: {num_robots} | "
                      f"Avg reward: {avg_reward:.2f} | "
                      f"Tasks: {stats['tasks_completed']}/{stats['tasks_generated']} | "
                      f"Validation: {validation['completion_rate']:.1%} | "
                      f"Updates: {self.training_step}")
            
            # Save model periodically
            if (episode + 1) % self.config['save_interval'] == 0:
                save_path = os.path.join(save_dir, f"ppo_model_ep{episode+1}")
                self.network.save(save_path)
                print(f"  Model saved: {save_path}.npz")

        # Flush a partial final rollout, then evaluate the actually-final
        # policy.  This also makes very small Quick/smoke runs train at least
        # once even when they never fill a 2,048-step rollout.
        if len(self.buffer) > 0:
            self._update_network(last_value=0.0)
            validation = self.evaluate()
            validation['episode'] = num_episodes
            validation['post_rollout_flush'] = True
            self.validation_history.append(validation)
            if validation['score'] > self.best_validation_score:
                self.best_validation_score = validation['score']
                self.network.save(os.path.join(
                    save_dir, "ppo_model_best_validation"))
        
        # Save final model
        final_path = os.path.join(save_dir, "ppo_model_final")
        self.network.save(final_path)
        print(f"\nFinal model saved: {final_path}.npz")
        
        # Save training history
        history_path = os.path.join(save_dir, "training_history.json")
        with open(history_path, 'w') as f:
            json.dump({
                'episode_rewards': self.episode_rewards,
                'episode_lengths': self.episode_lengths,
                'validation_history': self.validation_history,
                'best_validation_score': self.best_validation_score,
                'selection_split': 'fixed_held_out_validation',
                'total_updates': self.training_step,
                'update_diagnostics': self.update_diagnostics,
                'environment_version': RL_ENVIRONMENT_VERSION,
                'observation_dim': self.state_dim,
                'action_dim': self.action_dim,
                'action_semantics': 'robot_slot_x_task_slot_plus_no_op',
                'reward_config': self.reward_config.__dict__,
            }, f)
        print(f"Training history saved: {history_path}")
        
        return self.network


# ================================================================
# MAIN
# ================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Train PPO scheduling agent for smart factory"
    )
    parser.add_argument("--episodes", "-e", type=int, 
                       default=RL_CONFIG['training_episodes'],
                       help=f"Number of training episodes (default: {RL_CONFIG['training_episodes']})")
    parser.add_argument("--save-dir", "-d", default=None,
                       help="Directory to save models")
    parser.add_argument("--no-curriculum", action="store_true",
                       help="Disable curriculum learning")
    parser.add_argument("--lr", type=float, default=RL_CONFIG['learning_rate'],
                       help="Learning rate")
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip", type=float, default=0.2)
    parser.add_argument("--entropy", type=float, default=0.02)
    parser.add_argument("--value-coeff", type=float, default=0.5)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--buffer-size", type=int, default=2048)
    parser.add_argument("--hidden-size", type=int, default=256)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", help="Existing .npz checkpoint to continue from")
    parser.add_argument("--reward-completion", type=float, default=12.0)
    parser.add_argument("--reward-assignment", type=float, default=0.75)
    parser.add_argument("--reward-priority", type=float, default=0.4)
    parser.add_argument("--reward-age-bonus", type=float, default=0.3)
    parser.add_argument("--penalty-distance", type=float, default=-0.05)
    parser.add_argument("--penalty-waiting", type=float, default=-0.02)
    
    args = parser.parse_args()
    
    if args.save_dir is None:
        args.save_dir = os.path.join(
            os.path.dirname(__file__), '..', 'results', 'models'
        )
    
    config = dict(RL_CONFIG)
    config['learning_rate'] = args.lr
    config['gamma'] = args.gamma
    config['gae_lambda'] = args.gae_lambda
    config['clip_epsilon'] = args.clip
    config['entropy_coeff'] = args.entropy
    config['value_coeff'] = args.value_coeff
    config['num_epochs'] = args.epochs
    config['batch_size'] = args.batch_size
    config['buffer_size'] = args.buffer_size
    config['hidden_size'] = args.hidden_size
    config['reward_completion'] = args.reward_completion
    config['reward_assignment'] = args.reward_assignment
    config['reward_priority'] = args.reward_priority
    config['reward_age_bonus'] = args.reward_age_bonus
    config['penalty_distance'] = args.penalty_distance
    config['penalty_waiting'] = args.penalty_waiting
    
    trainer = PPOTrainer(config, seed=args.seed)
    if args.resume:
        trainer.network.load(args.resume)
    trainer.train(
        num_episodes=args.episodes,
        save_dir=args.save_dir,
        curriculum=not args.no_curriculum
    )


if __name__ == "__main__":
    main()
