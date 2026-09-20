"""End-to-end collect, fine-tune, validate and promote workflow for all RL."""

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import traceback
import time

ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "controllers" / "factory_supervisor"
sys.path.insert(0, str(SUPERVISOR))
sys.path.insert(0, str(ROOT / "scripts"))

from model_promotion import evaluate_promotion
from run_experiments import run_single_experiment
from rl_contract import RL_SCHEDULING_CONTRACT
from webots_dataset import (DATASET_VERSION, build_transitions, load_jsonl,
                            write_jsonl)
from finetune_from_webots import audit_transitions, finetune, sha256


RESUME_MANIFEST_VERSION = "webots-finetune-resume-v1"


def scheduler_name(algorithm):
    return "PPO_RL" if str(algorithm).upper() == "PPO" else str(algorithm).upper()


def _collection_identity(result):
    info = result["experiment_info"]
    provenance = info["provenance"]
    metrics = result.get("summary_metrics", {})
    return {
        "scenario": info["scenario"], "seed": int(provenance["seed"]),
        "run_mode": provenance["run_mode"],
        "checkpoint_sha256": provenance["sha256"],
        "sim_duration": float(info["sim_duration"]),
        "manifest_fingerprint": provenance.get("manifest_fingerprint"),
        "rl_fallback_decisions": int(metrics.get("rl_fallback_decisions", 0)),
        "fallback_scheduler_commits": int(
            metrics.get("fallback_scheduler_commits", 0)),
        "rl_decisions": len(result.get("rl_decisions", [])),
    }


def write_resume_manifest(path, *, algorithm, base_checkpoint, dataset,
                          scenario, duration, collection_seeds, collected):
    payload = {
        "version": RESUME_MANIFEST_VERSION,
        "algorithm": algorithm.upper(),
        "base_checkpoint": str(Path(base_checkpoint).resolve()),
        "base_sha256": sha256(base_checkpoint),
        "dataset": str(Path(dataset).resolve()),
        "dataset_sha256": sha256(dataset),
        "dataset_version": DATASET_VERSION,
        "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint(),
        "scenario": scenario, "duration": float(duration),
        "collection_seeds": [int(seed) for seed in collection_seeds],
        "collections": [_collection_identity(row) for row in collected],
    }
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def audit_resume(output, *, algorithm, base_checkpoint, dataset, report_path,
                 scenario, duration, collection_seeds):
    manifest_path = output / "collection_manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError("resume rejected: collection manifest is missing")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected = {
        "version": RESUME_MANIFEST_VERSION,
        "algorithm": algorithm.upper(),
        "base_sha256": sha256(base_checkpoint),
        "dataset_sha256": sha256(dataset),
        "dataset_version": DATASET_VERSION,
        "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint(),
        "scenario": scenario, "duration": float(duration),
        "collection_seeds": [int(seed) for seed in collection_seeds],
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise RuntimeError(f"resume rejected: {key} mismatch")
    collections = manifest.get("collections", [])
    if len(collections) != len(expected["collection_seeds"]):
        raise RuntimeError("resume rejected: collection count mismatch")
    if sorted(int(row.get("seed", -1)) for row in collections) != sorted(
            expected["collection_seeds"]):
        raise RuntimeError("resume rejected: collection seed set mismatch")
    for row in collections:
        if (row.get("run_mode") != "webots"
                or row.get("scenario") != scenario
                or abs(float(row.get("sim_duration", -1))-duration) >= 0.1
                or row.get("checkpoint_sha256") != expected["base_sha256"]
                or int(row.get("rl_fallback_decisions", -1)) != 0
                or int(row.get("fallback_scheduler_commits", -1)) != 0
                or int(row.get("rl_decisions", 0)) <= 0):
            raise RuntimeError("resume rejected: collection provenance invalid")
    audit_transitions(load_jsonl([dataset]))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    candidate = Path(report["candidate"])
    if report.get("algorithm") != algorithm.upper():
        raise RuntimeError("resume rejected: candidate algorithm mismatch")
    if report.get("base_sha256") != expected["base_sha256"]:
        raise RuntimeError("resume rejected: candidate base mismatch")
    if (report.get("dataset_sha256") != expected["dataset_sha256"]
            or report.get("dataset_version") != DATASET_VERSION
            or report.get("contract_fingerprint") != expected[
                "contract_fingerprint"]):
        raise RuntimeError("resume rejected: candidate dataset/contract mismatch")
    if not candidate.is_file() or sha256(candidate) != report.get(
            "candidate_sha256"):
        raise RuntimeError("resume rejected: candidate checkpoint mismatch")
    return report


def find_completed_validation(scenario, seed, duration, checkpoint,
                              algorithm="DQN"):
    """Recover a completed pre-resume run using strict immutable identity."""
    expected_hash = sha256(checkpoint)
    pattern = f"experiment_{scenario}_{scheduler_name(algorithm)}_*.json"
    for path in sorted((ROOT / "results").glob(pattern),
                       key=lambda item: item.stat().st_mtime, reverse=True):
        try:
            result = json.loads(path.read_text(encoding="utf-8-sig"))
            info = result["experiment_info"]
            provenance = info["provenance"]
            if (int(provenance.get("seed", -1)) == int(seed)
                    and info.get("scheduler") == scheduler_name(algorithm)
                    and provenance.get("run_mode") == "webots"
                    and provenance.get("sha256") == expected_hash
                    and abs(float(info.get("sim_duration", -1))-duration) < 0.1
                    and int(result.get("summary_metrics", {}).get(
                        "rl_fallback_decisions", 0)) == 0
                    and int(result.get("summary_metrics", {}).get(
                        "fallback_scheduler_commits", 0)) == 0):
                return result
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            continue
    return None


def audit_validation_result(result, *, scenario, seed, duration, checkpoint,
                            algorithm):
    try:
        info, metrics = result["experiment_info"], result["summary_metrics"]
        provenance = info["provenance"]
        valid = (
            info.get("scenario") == scenario
            and info.get("scheduler") == scheduler_name(algorithm)
            and int(provenance.get("seed", -1)) == int(seed)
            and provenance.get("run_mode") == "webots"
            and provenance.get("sha256") == sha256(checkpoint)
            and abs(float(info.get("sim_duration", -1))-duration) < 0.1
            and int(metrics.get("rl_fallback_decisions", 0)) == 0
            and int(metrics.get("fallback_scheduler_commits", 0)) == 0)
    except (KeyError, TypeError, ValueError):
        valid = False
    if not valid:
        raise RuntimeError(
            f"resume validation rejected for {algorithm}/seed {seed}")
    return result


def run_webots_with_retry(args, seed, checkpoint, stage):
    last_error = None
    for attempt in range(1, args.webots_retries + 1):
        try:
            result = run_single_experiment(
                args.scenario, scheduler_name(args.algorithm), seed,
                args.webots, model_path=checkpoint)
        except RuntimeError as exc:
            result, last_error = None, exc
        if result is not None:
            return result
        if attempt < args.webots_retries:
            detail = f": {last_error}" if last_error else ""
            print(f"[{stage}] seed {seed} failed on attempt {attempt}"
                  f"{detail}; retrying")
            time.sleep(3.0)
    message = (
        f"{stage} Webots run failed after {args.webots_retries} attempts "
        f"for seed {seed}")
    if last_error is not None:
        raise RuntimeError(f"{message}: {last_error}") from last_error
    raise RuntimeError(message)


def run(args):
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ["SMART_FACTORY_SIM_DURATION"] = str(args.duration)
    os.environ["RL_SCHEDULER_TIMEOUT_SECONDS"] = str(
        args.rl_timeout_seconds)
    dataset = output / "webots_train.jsonl"
    report_path = output / "candidate" / "finetune_report.json"
    if args.resume and dataset.exists() and report_path.exists():
        report = audit_resume(
            output, algorithm=args.algorithm, base_checkpoint=args.base_checkpoint,
            dataset=dataset, report_path=report_path, scenario=args.scenario,
            duration=args.duration, collection_seeds=args.collection_seeds)
        candidate_path = report["candidate"]
        print(f"[Resume] Reusing dataset and candidate: {candidate_path}")
    else:
        os.environ["WEBOTS_RL_COLLECT"] = "1"
        collected = [run_webots_with_retry(
            args, seed, args.base_checkpoint, "collection")
                     for seed in args.collection_seeds]
        transitions = []
        for result in collected:
            transitions.extend(build_transitions(result))
        dataset = write_jsonl(transitions, dataset)
        report = finetune(args.base_checkpoint, [dataset], output / "candidate",
                          algorithm=args.algorithm,
                          learning_rate=args.learning_rate,
                          updates=args.updates, proximal=args.proximal,
                          seed=args.training_seed)
        report["dataset_sha256"] = sha256(dataset)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        write_resume_manifest(
            output / "collection_manifest.json", algorithm=args.algorithm,
            base_checkpoint=args.base_checkpoint, dataset=dataset,
            scenario=args.scenario, duration=args.duration,
            collection_seeds=args.collection_seeds, collected=collected)
        candidate_path = report["candidate"]
    os.environ["WEBOTS_RL_COLLECT"] = "0"
    base_validation, candidate_validation = [], []
    validation_dir = output / "validation"
    validation_dir.mkdir(parents=True, exist_ok=True)
    for seed in args.validation_seeds:
        for label, checkpoint, destination in (
                ("base", args.base_checkpoint, base_validation),
                ("candidate", candidate_path, candidate_validation)):
            snapshot = validation_dir / f"{label}_seed_{seed}.json"
            if args.resume and snapshot.exists():
                result = json.loads(snapshot.read_text(encoding="utf-8-sig"))
                audit_validation_result(
                    result, scenario=args.scenario, seed=seed,
                    duration=args.duration, checkpoint=checkpoint,
                    algorithm=args.algorithm)
                print(f"[Resume] Reusing {label} validation seed {seed}")
            else:
                result = (find_completed_validation(
                    args.scenario, seed, args.duration, checkpoint,
                    args.algorithm)
                          if args.resume else None)
                if result is not None:
                    print(f"[Resume] Recovered {label} validation seed {seed}")
                else:
                    result = run_webots_with_retry(
                        args, seed, checkpoint, f"validation/{label}")
                if result is not None:
                    snapshot.write_text(json.dumps(result, indent=2),
                                        encoding="utf-8")
            destination.append(result)
    if any(row is None for row in base_validation + candidate_validation):
        raise RuntimeError("Webots validation run failed")
    promotion = evaluate_promotion(base_validation, candidate_validation)
    if promotion["promote"]:
        champion = output / f"champion{Path(candidate_path).suffix}"
        shutil.copy2(candidate_path, champion)
        promotion["champion"] = str(champion)
    (output / "promotion_report.json").write_text(
        json.dumps(promotion, indent=2), encoding="utf-8")
    return promotion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-checkpoint", required=True)
    parser.add_argument("--algorithm", choices=(
        "DQN", "SARSA", "PPO", "SARSA_LAMBDA", "RAINBOW_DQN", "A2C",
        "DISCRETE_SAC", "QR_DQN"), default="DQN")
    parser.add_argument("--webots", required=True)
    parser.add_argument("--scenario", choices=list("ABC"), default="C")
    parser.add_argument("--collection-seeds", nargs="+", type=int, required=True)
    parser.add_argument("--validation-seeds", nargs="+", type=int, required=True)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--learning-rate", type=float, default=1e-5)
    parser.add_argument("--updates", type=int, default=100)
    parser.add_argument("--proximal", type=float, default=0.001)
    parser.add_argument("--training-seed", type=int, default=42)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--webots-retries", type=int, default=2)
    parser.add_argument("--rl-timeout-seconds", type=float, default=0.5)
    args = parser.parse_args()
    overlap = set(args.collection_seeds) & set(args.validation_seeds)
    if overlap:
        parser.error(f"collection and validation seeds overlap: {sorted(overlap)}")
    if args.webots_retries < 1:
        parser.error("--webots-retries must be positive")
    if args.rl_timeout_seconds <= 0:
        parser.error("--rl-timeout-seconds must be positive")
    try:
        print(json.dumps(run(args), indent=2))
    except Exception as exc:
        output = Path(args.output_dir).resolve()
        output.mkdir(parents=True, exist_ok=True)
        failure = {"status": "failed", "error_type": type(exc).__name__,
                   "message": str(exc), "traceback": traceback.format_exc()}
        (output / "workflow_error.json").write_text(
            json.dumps(failure, indent=2), encoding="utf-8")
        print(json.dumps(failure, indent=2), file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
