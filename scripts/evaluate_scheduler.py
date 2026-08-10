"""Evaluate a frozen DQN/SARSA checkpoint on held-out seed domains."""

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from rl_agents import DQNAgent, SarsaAgent, dqn_config_from_checkpoint
from rl_environment import RLEnvironmentConfig, RewardConfig, SchedulingEnvironment
from train_scheduler import evaluate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algorithm", choices=("dqn", "sarsa"), required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--split", choices=("validation", "test", "ood"),
                        default="test")
    args = parser.parse_args()
    cfg = RLEnvironmentConfig()
    reward = RewardConfig()
    env = SchedulingEnvironment(cfg, reward)
    if args.algorithm == "dqn":
        agent = DQNAgent(
            env.observation_dim, env.action_dim, env.no_op_action,
            dqn_config_from_checkpoint(args.checkpoint))
    else:
        agent = SarsaAgent(env.action_dim, env.no_op_action)
    agent.load(args.checkpoint)
    base = {"validation": 200000, "test": 300000, "ood": 400000}[args.split]
    metrics = evaluate(agent, args.algorithm, range(base, base + args.seeds),
                       cfg, reward)
    metrics.update({"algorithm": args.algorithm.upper(), "split": args.split,
                    "checkpoint": args.checkpoint, "exploration": False})
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
