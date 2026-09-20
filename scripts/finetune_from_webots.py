"""Safely fine-tune a v8 DQN checkpoint from audited Webots JSONL."""

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from rl_agents import (DQNAgent, SarsaAgent, SarsaConfig,
                       dqn_config_from_checkpoint)
from advanced_rl_agents import advanced_agent_from_checkpoint
from rl_contract import RL_SCHEDULING_CONTRACT
from schedulers import PPONetwork
from webots_dataset import DATASET_VERSION, load_jsonl


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def audit_transitions(rows):
    if not rows:
        raise ValueError("fine-tuning dataset is empty")
    for row in rows:
        if row.get("dataset_version") != DATASET_VERSION:
            raise ValueError("dataset version mismatch")
        state = np.asarray(row.get("state"), np.float32)
        next_state = np.asarray(row.get("next_state"), np.float32)
        current_mask = np.asarray(row.get("action_mask"), bool)
        mask = np.asarray(row.get("next_action_mask"), bool)
        action = int(row.get("action", -1))
        discount = float(row.get("bootstrap_discount", -1.0))
        done = bool(row.get("done", False))
        if state.shape != (RL_SCHEDULING_CONTRACT.observation_dim,) or \
                next_state.shape != state.shape:
            raise ValueError("transition state dimension mismatch")
        if (mask.shape != (RL_SCHEDULING_CONTRACT.action_dim,)
                or current_mask.shape != mask.shape):
            raise ValueError("transition mask dimension mismatch")
        if not 0 <= action < RL_SCHEDULING_CONTRACT.action_dim:
            raise ValueError("transition action out of range")
        if not current_mask[action]:
            raise ValueError("transition action is masked")
        if not np.isfinite(state).all() or not np.isfinite(next_state).all():
            raise ValueError("transition contains non-finite state")
        if not np.isfinite(float(row.get("reward"))) or not 0 <= discount <= 1:
            raise ValueError("transition reward/discount invalid")
        if done and discount != 0.0:
            raise ValueError("terminal transition must have zero bootstrap")
        if not done and not mask.any():
            raise ValueError("non-terminal transition has no legal next action")
        if not done:
            next_action = int(row.get("next_action", -1))
            if not 0 <= next_action < RL_SCHEDULING_CONTRACT.action_dim or \
                    not mask[next_action]:
                raise ValueError("non-terminal next action is invalid or masked")
    return len(rows)


def finetune(base_checkpoint, dataset_paths, output_dir, *,
             algorithm="DQN", learning_rate=1e-5, updates=100,
             proximal=0.001, seed=42):
    if learning_rate <= 0 or updates <= 0 or not 0 <= proximal <= 1:
        raise ValueError("invalid fine-tuning hyperparameters")
    rows = load_jsonl(dataset_paths)
    audit_transitions(rows)
    base_path = Path(base_checkpoint).resolve()
    output = Path(output_dir).resolve()
    algorithm = str(algorithm).upper()
    supported = {"DQN", "SARSA", "PPO", "SARSA_LAMBDA", "RAINBOW_DQN",
                 "A2C", "DISCRETE_SAC", "QR_DQN"}
    if algorithm not in supported:
        raise ValueError(f"unsupported fine-tuning algorithm: {algorithm}")
    suffix = ".json" if algorithm in {"SARSA", "SARSA_LAMBDA"} else \
        (".pkl" if algorithm == "DQN" else ".npz")
    candidate = output / f"candidate{suffix}"
    if candidate == base_path or output == base_path.parent:
        raise ValueError("output must not overwrite the base checkpoint directory")
    agent, losses = _build_agent(algorithm, base_path, learning_rate, seed)
    base_weights = _parameter_snapshot(algorithm, agent)
    losses = _offline_updates(algorithm, agent, rows, int(updates))
    _apply_proximal(algorithm, agent, base_weights, float(proximal))
    if algorithm == "DQN":
        agent.target.copy_from(agent.online)
    output.mkdir(parents=True, exist_ok=True)
    agent.save(str(candidate))
    report = {
        "status": "candidate_requires_webots_validation",
        "algorithm": algorithm, "base_checkpoint": str(base_path),
        "base_sha256": sha256(base_path), "candidate": str(candidate),
        "candidate_sha256": sha256(candidate), "transitions": len(rows),
        "updates_requested": int(updates), "updates_applied": len(losses),
        "mean_loss": float(np.mean(losses)) if losses else None,
        "learning_rate": float(learning_rate), "proximal": float(proximal),
        "adaptation_method": ({
            "PPO": "offline_clipped_importance_actor_critic",
            "SARSA": "logged_next_action_td",
            "SARSA_LAMBDA": "logged_next_action_td_lambda",
        }.get(algorithm, "native_offline_update")),
        "dataset_sha256": sha256(dataset_paths[0]) if len(dataset_paths) == 1 else None,
        "dataset_version": DATASET_VERSION,
        "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint(),
    }
    (output / "finetune_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    return report


def _build_agent(algorithm, base_path, learning_rate, seed):
    dims = (RL_SCHEDULING_CONTRACT.observation_dim,
            RL_SCHEDULING_CONTRACT.action_dim,
            RL_SCHEDULING_CONTRACT.no_op_action)
    if algorithm == "DQN":
        config = dqn_config_from_checkpoint(str(base_path))
        config = replace(config, learning_rate=float(learning_rate),
                         batch_size=1, warmup_steps=1,
                         epsilon_start=0.0, epsilon_end=0.0)
        agent = DQNAgent(*dims, config, seed=seed)
        agent.load(str(base_path)); agent.config = config
        return agent, []
    if algorithm == "SARSA":
        payload = json.loads(base_path.read_text(encoding="utf-8"))
        config = replace(SarsaConfig(**payload["config"]),
                         learning_rate=float(learning_rate),
                         epsilon_start=0.0, epsilon_end=0.0)
        agent = SarsaAgent(dims[1], dims[2], config, seed=seed)
        agent.load(str(base_path)); agent.config = config
        return agent, []
    if algorithm == "PPO":
        with np.load(base_path, allow_pickle=False) as data:
            agent = PPONetwork(int(data["state_dim"][0]),
                               int(data["action_dim"][0]),
                               int(data["hidden_size"][0]))
        agent.load(str(base_path))
        agent._offline_learning_rate = float(learning_rate)
        return agent, []
    agent = advanced_agent_from_checkpoint(
        algorithm, str(base_path), *dims, seed=seed)
    if hasattr(agent, "optimizer"):
        agent.optimizer.learning_rate = float(learning_rate)
    if hasattr(agent.config, "learning_rate"):
        agent.config = replace(agent.config, learning_rate=float(learning_rate))
    return agent, []


def _parameter_snapshot(algorithm, agent):
    if algorithm == "DQN":
        return {key: value.copy() for key, value in agent.online.params.items()}
    if algorithm in {"SARSA", "SARSA_LAMBDA"}:
        return {key: value.copy() for key, value in agent.q_table.items()}
    if algorithm == "PPO":
        return {key: getattr(agent, key).copy() for key in (
            "W1", "b1", "W2", "b2", "W_policy", "b_policy",
            "W_value", "b_value")}
    return {key: value.copy() for key, value in agent.params.items()}


def _apply_proximal(algorithm, agent, base, proximal):
    if not proximal:
        return
    if algorithm == "DQN": current = agent.online.params
    elif algorithm in {"SARSA", "SARSA_LAMBDA"}: current = agent.q_table
    elif algorithm == "PPO":
        for key, old in base.items():
            value = getattr(agent, key)
            value += proximal * (old-value)
        return
    else: current = agent.params
    for key, value in current.items():
        old = base.get(key)
        if old is None:
            old = np.zeros_like(value)
        value += proximal * (old-value)


def _ppo_update(agent, row, old_probability, old_advantage):
    state = np.asarray(row["state"], np.float64)
    next_state = np.asarray(row["next_state"], np.float64)
    mask = np.asarray(row["action_mask"], bool)
    probs, value = agent.forward(state)
    masked = np.where(mask, probs, 0.0); masked /= max(masked.sum(), 1e-12)
    target = value + old_advantage
    advantage = old_advantage
    h1 = np.clip(np.maximum(0, state@agent.W1+agent.b1), 0, 50)
    h2 = np.clip(np.maximum(0, h1@agent.W2+agent.b2), 0, 50)
    action = int(row["action"])
    ratio = float(masked[action])/max(float(old_probability), 1e-8)
    clipped = ((advantage >= 0 and ratio > 1.2)
               or (advantage < 0 and ratio < 0.8))
    grad = masked.copy(); grad[action] -= 1.0
    grad *= (0.0 if clipped else ratio)
    lr = agent._offline_learning_rate
    agent.W_policy -= lr*np.outer(h2, advantage*grad)
    agent.b_policy -= lr*advantage*grad
    value_error = float(np.clip(value-target, -10.0, 10.0))
    agent.W_value -= lr*np.outer(h2, [value_error])
    agent.b_value -= lr*value_error
    return abs(value_error)


def _offline_updates(algorithm, agent, rows, updates):
    losses = []
    if algorithm == "DQN":
        for row in rows:
            agent.remember(row["state"], row["action"], row["reward"],
                           row["next_state"], row["done"],
                           row["next_action_mask"], row["bootstrap_discount"])
        for _ in range(updates):
            loss = agent.train_step()
            if loss is not None: losses.append(float(loss))
        return losses
    if algorithm == "RAINBOW_DQN":
        for row in rows:
            agent.observe(row["state"], row["action"], row["reward"],
                          row["next_state"], row["done"],
                          row["next_action_mask"], row["bootstrap_discount"])
        for _ in range(updates):
            if len(agent.replay):
                losses.append(float(agent.train_batch(
                    min(32, len(agent.replay)))["distributional_loss"]))
        return losses
    ppo_reference = None
    if algorithm == "PPO":
        ppo_reference = []
        for row in rows:
            probabilities, value = agent.forward(np.asarray(row["state"]))
            current_mask = np.asarray(row["action_mask"], bool)
            probabilities = np.where(current_mask, probabilities, 0.0)
            probabilities /= max(probabilities.sum(), 1e-12)
            _, next_value = agent.forward(np.asarray(row["next_state"]))
            target = (float(row["reward"])
                      + float(row["bootstrap_discount"])*next_value)
            ppo_reference.append((float(probabilities[int(row["action"])]),
                                  float(np.clip(target-value, -10.0, 10.0))))
    for index in range(updates):
        row = rows[index % len(rows)]
        if algorithm == "PPO":
            reference = ppo_reference[index % len(rows)]
            losses.append(_ppo_update(agent, row, *reference)); continue
        state = np.asarray(row["state"], np.float32)
        next_state = np.asarray(row["next_state"], np.float32)
        if algorithm in {"SARSA", "SARSA_LAMBDA"}:
            key, next_key = agent.discretize(state), agent.discretize(next_state)
            next_action = (agent.no_op_action if row["done"] else
                           int(row["next_action"]))
            loss = agent.update(key, row["action"], row["reward"], next_key,
                                next_action, row["done"],
                                row["bootstrap_discount"])
            losses.append(abs(float(loss))); continue
        result = agent.update(
            state, row["action"], row["reward"], next_state, row["done"],
            row["action_mask"], row["next_action_mask"],
            row["bootstrap_discount"])
        losses.append(abs(float(next(iter(result.values())))))
    return losses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algorithm", choices=[
        "dqn", "sarsa", "ppo", "sarsa_lambda", "rainbow_dqn", "a2c",
        "discrete_sac", "qr_dqn"], default="dqn")
    parser.add_argument("--base-checkpoint", required=True)
    parser.add_argument("--dataset", nargs="+", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--learning-rate", type=float, default=1e-5)
    parser.add_argument("--updates", type=int, default=100)
    parser.add_argument("--proximal", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    print(json.dumps(finetune(
        args.base_checkpoint, args.dataset, args.output_dir,
        algorithm=args.algorithm,
        learning_rate=args.learning_rate, updates=args.updates,
        proximal=args.proximal, seed=args.seed), indent=2))


if __name__ == "__main__":
    main()
