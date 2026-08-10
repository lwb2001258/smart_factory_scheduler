"""Two-stage hyperparameter search and promotion for all proposed schedulers.

Validation seeds are used for selection. Test seeds and Webots evaluation are
deliberately left to the outer pipeline after a configuration is frozen.
"""

import argparse
import itertools
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "controllers" / "factory_supervisor"
sys.path.insert(0, str(SUPERVISOR))

from rl_environment import SchedulingEnvironment
from schedulers import (GeneticScheduler, GreedyScheduler,
                        HungarianScheduler, NearestNeighbourScheduler,
                        SimulatedAnnealingScheduler)
from training_scenarios import factory_scenario


REWARD_PROFILES = {
    "balanced": dict(completion=10.0, assignment=0.5, priority=0.5,
                     age=0.5, distance=-0.08, waiting=-0.03),
    "throughput": dict(completion=12.0, assignment=0.75, priority=0.4,
                       age=0.3, distance=-0.05, waiting=-0.02),
    "efficiency": dict(completion=10.0, assignment=0.4, priority=0.5,
                       age=0.6, distance=-0.12, waiting=-0.05),
}


def run(command):
    print("[SEARCH]", subprocess.list2cmdline([str(x) for x in command]), flush=True)
    subprocess.run([str(x) for x in command], cwd=ROOT, check=True)


def synthetic_case(seed):
    robots, tasks, context = factory_scenario(
        seed, min_robots=3, max_generated_tasks=12)
    return tasks, robots, context


def score_metaheuristic(algorithm, config, seeds):
    completion_rates, waiting_times, distances = [], [], []
    latencies, failures = [], 0
    for seed in seeds:
        tasks, robots, context = synthetic_case(seed)
        scheduler = (GeneticScheduler(seed=seed, **config) if algorithm == "ga"
                     else SimulatedAnnealingScheduler(seed=seed, **config))
        fallbacks = [HungarianScheduler(), GreedyScheduler(),
                     NearestNeighbourScheduler()]
        env = SchedulingEnvironment(simulation_mode="abstract")
        env.reset(robots, tasks, context, seed=seed)
        for _ in range(env.config.max_steps_per_episode):
            mask = env.get_action_mask()
            legal = np.flatnonzero(mask[:-1])
            action = env.no_op_action
            if legal.size:
                visible = list(env._task_slots)
                decision_latency_ms = 0.0
                result = scheduler.assign(
                    visible, env._robots, env._context)
                decision_latency_ms += result.computation_time * 1000.0
                if not result.is_feasible or not result.assignments:
                    failures += 1
                    for fallback in fallbacks:
                        result = fallback.assign(
                            visible, env._robots, env._context)
                        decision_latency_ms += (
                            result.computation_time * 1000.0)
                        if result.is_feasible and result.assignments:
                            break
                latencies.append(decision_latency_ms)
                if result.is_feasible and result.assignments:
                    chosen = result.assignments[0]
                    try:
                        rslot = env._robot_slots.index(chosen.robot_id)
                        tslot = next(
                            index for index, task in enumerate(env._task_slots)
                            if task.task_id == chosen.task.task_id)
                        action = env.encode_action(rslot, tslot)
                    except (ValueError, StopIteration):
                        failures += 1
                        action = int(legal[0])
            _, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                break
        completed = [task for task in env._tasks
                     if task.status == "completed"]
        completion_rates.append(len(completed) / max(1, len(env._tasks)))
        waiting_times.extend(
            task.waiting_time for task in completed
            if task.waiting_time is not None)
        distances.append(sum(float(state.get("total_distance", 0.0))
                             for state in env._robots.values()))
    mean_completion = float(np.mean(completion_rates)) if completion_rates else 0.0
    mean_wait = float(np.mean(waiting_times)) if waiting_times else 1e6
    mean_distance = float(np.mean(distances)) if distances else 1e6
    p95_latency = float(np.percentile(latencies, 95)) if latencies else 1e6
    # Safety/completion dominate; waiting, travel, latency and native failures
    # break ties among configurations that complete the same online workload.
    score = (10000.0 * mean_completion - mean_wait
             - 0.1 * mean_distance - 0.02 * p95_latency
             - 1000.0 * failures)
    return {"score": score, "completion_rate": mean_completion,
            "mean_waiting_time": mean_wait,
            "mean_distance": mean_distance,
            "p95_latency_ms": p95_latency,
            "native_failures": failures}


def reward_args(profile):
    p = REWARD_PROFILES[profile]
    return ["--reward-completion", p["completion"],
            "--reward-assignment", p["assignment"],
            "--reward-priority", p["priority"],
            "--reward-age-bonus", p["age"],
            "--penalty-distance", p["distance"],
            "--penalty-waiting", p["waiting"]]


def train_value_candidate(algorithm, config, episodes, directory, resume=None):
    command = [sys.executable, ROOT / "scripts" / "train_scheduler.py",
               "--algorithm", algorithm, "--episodes", episodes,
               "--seed", config["seed"], "--checkpoint-dir", directory,
               "--evaluation-interval", max(10, episodes // 5),
               "--lr", config["lr"], "--gamma", config["gamma"],
               "--epsilon-end", config["epsilon_end"]]
    if algorithm == "sarsa":
        command += ["--epsilon-decay", config["epsilon_decay"]]
        extension = "json"
    else:
        command += ["--hidden-size", config["hidden_size"],
                    "--batch-size", config["batch_size"],
                    "--warmup-steps", config["warmup_steps"],
                    "--target-update", config["target_update"],
                    "--epsilon-decay-steps", config["epsilon_decay_steps"]]
        extension = "pkl"
    command += reward_args(config["reward_profile"])
    if resume:
        command += ["--resume", resume]
    run(command)
    metrics = json.loads((directory / "training_metrics.json").read_text())
    return float(metrics["best_validation_score"]), directory / f"best_validation.{extension}"


def train_ppo_candidate(config, episodes, directory, resume=None):
    command = [sys.executable, ROOT / "scripts" / "train_ppo.py",
               "--episodes", episodes, "--save-dir", directory,
               "--seed", config["seed"], "--lr", config["lr"],
               "--gamma", config["gamma"], "--gae-lambda", config["gae_lambda"],
               "--clip", config["clip"], "--entropy", config["entropy"],
               "--epochs", config["epochs"], "--batch-size", config["batch_size"],
               "--buffer-size", config["buffer_size"],
               "--hidden-size", config["hidden_size"]]
    command += reward_args(config["reward_profile"])
    if resume:
        command += ["--resume", resume]
    run(command)
    history = json.loads((directory / "training_history.json").read_text())
    score = float(history.get("best_validation_score", -math.inf))
    return score, directory / "ppo_model_best_validation.npz"


def promote(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / source.name
    shutil.copy2(source, target)
    return target


def spread_sample(items, limit):
    """Select deterministic candidates across the whole Cartesian space."""
    if limit >= len(items):
        return list(items)
    indices = np.linspace(0, len(items) - 1, num=limit, dtype=int)
    return [items[int(index)] for index in indices]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("quick", "full"), default="full")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "results" / "hyperparameter_search")
    parser.add_argument("--algorithms", nargs="+",
                        choices=("sa", "ga", "sarsa", "dqn", "ppo"),
                        default=("sa", "ga", "sarsa", "dqn", "ppo"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    quick = args.profile == "quick"
    report = {"profile": args.profile, "selection_split": "validation",
              "test_not_used_for_selection": True, "algorithms": {}}

    meta_spaces = {
        "sa": [dict(initial_temperature=t, cooling_rate=c,
                    minimum_temperature=0.01, max_iterations=i,
                    time_budget_ms=b)
               for t, c, i, b in itertools.product(
                   (5.0, 10.0), (0.97, 0.985, 0.995),
                   ((500,) if quick else (1000, 2000)),
                   ((2.0,) if quick else (5.0, 10.0)))],
        "ga": [dict(population_size=p, max_generations=g, time_budget_ms=b)
               for p, g, b in itertools.product(
                   ((24, 48) if quick else (24, 48, 96)),
                   ((20,) if quick else (20, 50, 100)),
                   ((5.0,) if quick else (5.0, 10.0, 20.0)))],
    }
    for algorithm in ("sa", "ga"):
        if algorithm not in args.algorithms:
            continue
        rows = []
        for config in meta_spaces[algorithm]:
            metrics = score_metaheuristic(
                algorithm, config, range(210000, 210008 if quick else 210040))
            rows.append({"config": config, "metrics": metrics})
        rows.sort(key=lambda row: row["metrics"]["score"], reverse=True)
        report["algorithms"][algorithm] = {"best": rows[0], "candidates": rows}

    value_spaces = {
        "sarsa": [dict(seed=42, lr=lr, gamma=gamma, epsilon_end=0.02,
                       epsilon_decay=decay, reward_profile=reward)
                  for lr, gamma, decay, reward in itertools.product(
                      (0.03, 0.05, 0.1), (0.95, 0.98, 0.99),
                      (0.997, 0.999), REWARD_PROFILES)],
        "dqn": [dict(seed=42, lr=lr, gamma=0.99, epsilon_end=0.02,
                     hidden_size=hidden, batch_size=batch,
                     warmup_steps=256 if quick else 2000,
                     target_update=500, epsilon_decay_steps=50000,
                     reward_profile=reward)
                for lr, hidden, batch, reward in itertools.product(
                    (1e-4, 3e-4), (128, 256), (64, 128), REWARD_PROFILES)],
    }
    # Deterministic subsampling keeps Quick useful and Full computationally bounded.
    limits = {"sarsa": 6 if quick else 18, "dqn": 6 if quick else 16}
    budgets = ((30, 120) if quick else (500, 3000))
    champion_budgets = {"sarsa": 300 if quick else 10000,
                        "dqn": 300 if quick else 5000}
    for algorithm in ("sarsa", "dqn"):
        if algorithm not in args.algorithms:
            continue
        candidates = spread_sample(value_spaces[algorithm], limits[algorithm])
        stage1 = []
        for index, config in enumerate(candidates):
            directory = args.output / algorithm / f"candidate_{index:02d}_stage1"
            score, model = train_value_candidate(
                algorithm, config, budgets[0], directory)
            stage1.append({"index": index, "config": config,
                           "stage1_score": score, "model": str(model)})
        stage1.sort(key=lambda row: row["stage1_score"], reverse=True)
        finalists = stage1[:max(2, len(stage1) // 3)]
        final_rows = []
        for row in finalists:
            directory = args.output / algorithm / f"candidate_{row['index']:02d}_stage2"
            score, model = train_value_candidate(
                algorithm, row["config"], budgets[1], directory, row["model"])
            final_rows.append({**row, "stage2_score": score, "model": str(model)})
        final_rows.sort(key=lambda row: row["stage2_score"], reverse=True)
        winner = final_rows[0]
        champion_dir = args.output / algorithm / "champion_training"
        champion_score, champion_model = train_value_candidate(
            algorithm, winner["config"], champion_budgets[algorithm],
            champion_dir, winner["model"])
        winner["champion_score"] = champion_score
        winner["champion_model"] = str(champion_model)
        promoted = promote(champion_model, args.output / "best" / algorithm)
        winner["promoted_model"] = str(promoted)
        report["algorithms"][algorithm] = {
            "best": winner, "stage1": stage1, "stage2": final_rows}

    if "ppo" in args.algorithms:
        ppo_space = [dict(seed=42, lr=lr, gamma=0.99, gae_lambda=gae,
                          clip=clip, entropy=entropy, epochs=epochs,
                          batch_size=256, buffer_size=2048, hidden_size=256,
                          reward_profile=reward)
                     for lr, gae, clip, entropy, epochs, reward in itertools.product(
                         (1e-4, 3e-4), (0.9, 0.95), (0.1, 0.2),
                         (0.01, 0.02), (5, 8), REWARD_PROFILES)]
        ppo_space = spread_sample(ppo_space, 4 if quick else 12)
        ppo_budgets = (5, 20) if quick else (200, 1500)
        ppo_champion_budget = 50 if quick else 5000
        stage1 = []
        for index, config in enumerate(ppo_space):
            directory = args.output / "ppo" / f"candidate_{index:02d}_stage1"
            score, model = train_ppo_candidate(config, ppo_budgets[0], directory)
            stage1.append({"index": index, "config": config,
                           "stage1_score": score, "model": str(model)})
        stage1.sort(key=lambda row: row["stage1_score"], reverse=True)
        finalists = stage1[:max(2, len(stage1) // 3)]
        final_rows = []
        for row in finalists:
            directory = args.output / "ppo" / f"candidate_{row['index']:02d}_stage2"
            score, model = train_ppo_candidate(
                row["config"], ppo_budgets[1], directory, row["model"])
            final_rows.append({**row, "stage2_score": score, "model": str(model)})
        final_rows.sort(key=lambda row: row["stage2_score"], reverse=True)
        winner = final_rows[0]
        champion_dir = args.output / "ppo" / "champion_training"
        champion_score, champion_model = train_ppo_candidate(
            winner["config"], ppo_champion_budget, champion_dir,
            winner["model"])
        winner["champion_score"] = champion_score
        winner["champion_model"] = str(champion_model)
        promoted = promote(champion_model, args.output / "best" / "ppo")
        winner["promoted_model"] = str(promoted)
        report["algorithms"]["ppo"] = {
            "best": winner, "stage1": stage1, "stage2": final_rows,
            "selection_note": "PPO is selected on fixed held-out deterministic validation episodes; Webots test follows promotion."}

    report["completed_at_unix"] = time.time()
    report_path = args.output / "search_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({name: data["best"] for name, data in
                      report["algorithms"].items()}, indent=2))
    print(f"Search report: {report_path}")


if __name__ == "__main__":
    main()
