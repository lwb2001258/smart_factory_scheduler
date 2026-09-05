"""Train advanced RL schedulers against the shared abstract/Webots contract."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "controllers" / "factory_supervisor"
sys.path.insert(0, str(SUPERVISOR))

from advanced_rl_agents import (A2CAgent, DiscreteSACAgent, QRDQNAgent,
                                RainbowConfig, RainbowDQNAgent,
                                SarsaLambdaAgent)
from rl_environment import RLEnvironmentConfig, SchedulingEnvironment
from training_scenarios import manifest_scenario


AGENT_TYPES = {"A2C": A2CAgent, "DISCRETE_SAC": DiscreteSACAgent,
               "QR_DQN": QRDQNAgent, "RAINBOW_DQN": RainbowDQNAgent}


def build_agent(algorithm, env, seed, batch_size):
    if algorithm == "SARSA_LAMBDA":
        return SarsaLambdaAgent(env.action_dim, env.no_op_action, seed=seed)
    if algorithm == "RAINBOW_DQN":
        config = RainbowConfig(replay_capacity=max(64, batch_size*4))
        return RainbowDQNAgent(env.observation_dim, env.action_dim,
                               env.no_op_action, config, seed)
    return AGENT_TYPES[algorithm](env.observation_dim, env.action_dim,
                                  env.no_op_action, seed=seed)


def run_episode(agent, algorithm, env, seed, batch_size, training=True):
    robots, tasks, context = manifest_scenario(seed, "A", max_tasks=5)
    state, _ = env.reset(robots, tasks, context, seed)
    total, updates = 0.0, []
    key = agent.discretize(state) if algorithm == "SARSA_LAMBDA" else None
    action = (agent.select_action(key, env.get_action_mask(), training)
              if key is not None else None)
    for _ in range(env.config.max_steps_per_episode):
        mask = env.get_action_mask()
        if action is None:
            # Off-policy value agents receive explicit exploration without
            # changing deterministic deployment semantics.
            if training and algorithm == "QR_DQN" and agent.rng.random() < 0.1:
                action = int(agent.rng.choice(np.flatnonzero(mask)))
            else:
                action = agent.select_action(state, mask, training=training)
        next_state, reward, terminated, truncated, _ = env.step(action)
        done, next_mask = terminated or truncated, env.get_action_mask()
        if algorithm == "SARSA_LAMBDA":
            next_key = agent.discretize(next_state)
            next_action = agent.select_action(next_key, next_mask, training)
            if training: updates.append(abs(agent.update(
                key, action, reward, next_key, next_action, done,
                bootstrap_discount=(0.0 if done else
                                    env.last_bootstrap_discount))))
            key, action = next_key, next_action
        elif training and algorithm == "RAINBOW_DQN":
            agent.observe(
                state, action, reward, next_state, done, next_mask,
                bootstrap_discount=(0.0 if done else
                                    env.last_bootstrap_discount))
            if len(agent.replay) >= batch_size:
                updates.append(agent.train_batch(batch_size)["distributional_loss"])
            action = None
        elif training:
            result = agent.update(
                state, action, reward, next_state, done, mask, next_mask,
                bootstrap_discount=(0.0 if done else
                                    env.last_bootstrap_discount))
            updates.append(float(next(iter(result.values())))); action = None
        else:
            action = None
        state, total = next_state, total+reward
        if done: break
    if training and algorithm == "SARSA_LAMBDA": agent.end_episode()
    return {"reward": total, "updates": len(updates),
            "mean_update": float(np.mean(updates)) if updates else None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algorithm", required=True,
                        choices=("SARSA_LAMBDA", "RAINBOW_DQN", "A2C",
                                 "DISCRETE_SAC", "QR_DQN"))
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--checkpoint-dir", required=True)
    args = parser.parse_args()
    env = SchedulingEnvironment(RLEnvironmentConfig(max_steps_per_episode=64),
                                simulation_mode="abstract")
    agent = build_agent(args.algorithm, env, args.seed, args.batch_size)
    history = [run_episode(agent, args.algorithm, env,
                           args.seed*100000+episode, args.batch_size)
               for episode in range(args.episodes)]
    output = Path(args.checkpoint_dir); output.mkdir(parents=True, exist_ok=True)
    checkpoint = output / ("model.json" if args.algorithm == "SARSA_LAMBDA"
                           else "model.npz")
    agent.save(checkpoint)
    validation = run_episode(agent, args.algorithm, env,
                             200000+args.seed, args.batch_size, training=False)
    report = {"algorithm": args.algorithm, "seed": args.seed,
              "episodes": args.episodes, "checkpoint": str(checkpoint),
              "environment_mode": "abstract", "manifest_scenario": "A",
              "validation": validation, "history": history,
              "test_not_used_for_selection": True}
    (output / "training_metrics.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items()
                      if key != "history"}, indent=2))


if __name__ == "__main__":
    main()
