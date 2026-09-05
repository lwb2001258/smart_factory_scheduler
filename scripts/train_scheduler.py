"""Train SARSA or DQN in the abstract scheduling environment."""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "controllers" / "factory_supervisor"
sys.path.insert(0, str(SUPERVISOR))

from rl_agents import DQNAgent, DQNConfig, SarsaAgent, SarsaConfig
from rl_environment import (RLEnvironmentConfig, RewardConfig,
                            SchedulingEnvironment)
from training_scenarios import manifest_scenario


def scenario(seed: int, max_robots=8, max_tasks=20):
    scenario_name = {3: "A", 5: "B", 8: "C"}.get(max_robots)
    if scenario_name is None:
        raise ValueError("manifest curriculum supports 3, 5, or 8 robots")
    return manifest_scenario(seed, scenario_name, max_tasks=max_tasks)


def evaluate(agent, algorithm: str, seeds, env_config, reward_config=None):
    rewards, completed, invalid = [], [], 0
    for seed in seeds:
        env = SchedulingEnvironment(env_config, reward_config,
                                    simulation_mode="abstract")
        state, _ = env.reset(*scenario(seed), seed=seed)
        total = 0.0
        for _ in range(env_config.max_steps_per_episode):
            mask = env.get_action_mask()
            if algorithm == "sarsa":
                action = agent.select_action(
                    agent.discretize(state), mask, training=False)
            else:
                action = agent.select_action(state, mask, training=False)
            state, reward, terminated, truncated, info = env.step(action)
            total += reward
            invalid += int(info.get("invalid_action", False))
            if terminated or truncated:
                completed.append(info.get(
                    "completed_count", len(env._completed_ids)))
                break
        rewards.append(total)
    return {
        "mean_reward": float(np.mean(rewards)),
        "std_reward": float(np.std(rewards)),
        "mean_completed": float(np.mean(completed)) if completed else 0.0,
        "invalid_actions": invalid,
    }


def validation_selection_score(metrics):
    """Reward-scale-independent checkpoint score.

    Reward profiles intentionally use different coefficients, so raw return
    cannot fairly compare them. Completion dominates, illegal actions are a
    hard regression, and normalized return only breaks near ties.
    """
    return (100.0 * float(metrics["mean_completed"])
            - 100.0 * float(metrics["invalid_actions"])
            + 0.01 * float(metrics["mean_reward"]))


def train(args):
    env_config = RLEnvironmentConfig()
    reward_config = RewardConfig(
        completion=args.reward_completion,
        valid_assignment=args.reward_assignment,
        empty_distance=args.penalty_distance,
        wait_increment=args.penalty_waiting,
        age_rescue=args.reward_age_bonus,
        invalid_action=args.penalty_invalid,
        avoidable_wait=args.penalty_no_op,
        collision=args.penalty_collision)
    probe = SchedulingEnvironment(env_config, reward_config)
    if args.algorithm == "sarsa":
        agent = SarsaAgent(
            probe.action_dim, probe.no_op_action,
            SarsaConfig(learning_rate=args.lr, gamma=args.gamma,
                        epsilon_start=args.epsilon_start,
                        epsilon_end=args.epsilon_end,
                        epsilon_decay=args.epsilon_decay), seed=args.seed)
        extension = "json"
    else:
        agent = DQNAgent(
            probe.observation_dim, probe.action_dim, probe.no_op_action,
            DQNConfig(hidden_size=args.hidden_size,
                      learning_rate=args.lr, gamma=args.gamma,
                      batch_size=args.batch_size,
                      replay_capacity=args.replay_capacity,
                      warmup_steps=args.warmup_steps,
                      target_update_interval=args.target_update,
                      epsilon_start=args.epsilon_start,
                      epsilon_end=args.epsilon_end,
                      epsilon_decay_steps=args.epsilon_decay_steps),
            seed=args.seed)
        extension = "pkl"
    if args.resume:
        agent.load(args.resume)
    output = Path(args.checkpoint_dir)
    output.mkdir(parents=True, exist_ok=True)
    best_score = -float("inf")
    best_validation_metrics = None
    history = []
    started = time.perf_counter()
    for episode in range(args.episodes):
        seed = args.seed * 100000 + episode  # training seed domain
        progress = episode / max(1, args.episodes)
        if progress < 0.30:
            curriculum_robots, curriculum_tasks = 3, 5
        elif progress < 0.60:
            curriculum_robots, curriculum_tasks = 5, 8
        else:
            curriculum_robots, curriculum_tasks = 8, 12
        env = SchedulingEnvironment(env_config, reward_config,
                                    simulation_mode="abstract")
        state, _ = env.reset(*scenario(
            seed, max_robots=curriculum_robots,
            max_tasks=curriculum_tasks), seed=seed)
        total, losses = 0.0, []
        if args.algorithm == "sarsa":
            key = agent.discretize(state)
            action = agent.select_action(key, env.get_action_mask(), True)
        for _ in range(env_config.max_steps_per_episode):
            if args.algorithm == "dqn":
                action = agent.select_action(
                    state, env.get_action_mask(), True)
            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            next_mask = env.get_action_mask()
            if args.algorithm == "sarsa":
                next_key = agent.discretize(next_state)
                next_action = agent.select_action(next_key, next_mask, True)
                agent.update(
                    key, action, reward, next_key, next_action, done,
                    bootstrap_discount=(0.0 if done else
                                        env.last_bootstrap_discount))
                key, action = next_key, next_action
            else:
                agent.remember(
                    state, action, reward, next_state, done, next_mask,
                    0.0 if done else env.last_bootstrap_discount)
                loss = agent.train_step()
                if loss is not None:
                    losses.append(loss)
            state, total = next_state, total + reward
            if done:
                break
        if args.algorithm == "sarsa":
            agent.end_episode()
        else:
            agent.episode += 1
        row = {"episode": episode + 1, "reward": total,
               "loss": float(np.mean(losses)) if losses else None,
               "epsilon": agent.epsilon}
        history.append(row)
        if (episode + 1) % args.evaluation_interval == 0:
            validation = evaluate(
                agent, args.algorithm,
                range(200000 + args.seed * 100, 200005 + args.seed * 100),
                env_config, reward_config)
            row["validation"] = validation
            selection_score = validation_selection_score(validation)
            row["validation_selection_score"] = selection_score
            if selection_score > best_score:
                best_score = selection_score
                best_validation_metrics = dict(validation)
                agent.save(str(output / f"best_validation.{extension}"))
    agent.save(str(output / f"latest.{extension}"))
    report = {
        "algorithm": args.algorithm.upper(), "episodes": args.episodes,
        "seed": args.seed, "environment_mode": args.environment_mode,
        "training_geometry": "factory-grid-astar-v3-online",
        "curriculum": "A-like(3/5)->B-like(5/8)->C-like(8/12)",
        "training_seconds": time.perf_counter() - started,
        "best_validation_score": best_score, "history": history,
        "best_validation_reward": (
            best_validation_metrics["mean_reward"]
            if best_validation_metrics else None),
        "best_validation_metrics": best_validation_metrics,
        "reward_config": reward_config.__dict__,
        "test_not_used_for_selection": True,
    }
    (output / "training_metrics.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items()
                      if key != "history"}, indent=2))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algorithm", choices=("dqn", "sarsa"), required=True)
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--checkpoint-dir", required=True)
    parser.add_argument("--resume")
    parser.add_argument("--evaluation-interval", type=int, default=20)
    parser.add_argument("--environment-mode", choices=("abstract",),
                        default="abstract")
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--epsilon-start", type=float, default=1.0)
    parser.add_argument("--epsilon-end", type=float, default=0.02)
    parser.add_argument("--epsilon-decay", type=float, default=0.999)
    parser.add_argument("--epsilon-decay-steps", type=int, default=50000)
    parser.add_argument("--hidden-size", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--replay-capacity", type=int, default=100000)
    parser.add_argument("--warmup-steps", type=int, default=5000)
    parser.add_argument("--target-update", type=int, default=500)
    parser.add_argument("--reward-completion", type=float, default=5.0)
    parser.add_argument("--reward-assignment", type=float, default=0.0)
    parser.add_argument("--reward-priority", type=float, default=0.0,
                        help="Deprecated; raw priority is not rewarded")
    parser.add_argument("--reward-age-bonus", type=float, default=0.2)
    parser.add_argument("--penalty-distance", type=float, default=-0.10)
    parser.add_argument("--penalty-waiting", type=float, default=-0.10)
    parser.add_argument("--penalty-invalid", type=float, default=-5.0)
    parser.add_argument("--penalty-no-op", type=float, default=-1.0)
    parser.add_argument("--penalty-collision", type=float, default=-100.0)
    return parser.parse_args()


if __name__ == "__main__":
    train(parse_args())
