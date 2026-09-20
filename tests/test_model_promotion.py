import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from model_promotion import evaluate_promotion


def result(seed, throughput=5.0, completion=100.0):
    return {"experiment_info": {"scenario": "C", "scheduler": "DQN",
            "sim_duration": 120.0, "provenance": {"seed": seed,
            "run_mode": "webots", "manifest_fingerprint": f"m-{seed}",
            "manifest_version": "v2"}}, "summary_metrics": {
            "total_tasks_generated": 10, "total_tasks_completed": 10,
            "throughput_per_minute": throughput,
            "avg_task_completion_time": completion, "avg_waiting_time": 10,
            "weighted_tardiness": 0, "hard_deadline_breaches": 0,
            "pair_distance_violations": 0, "scheduler_fallbacks": 0,
            "rl_fallback_decisions": 0, "invalid_scheduler_outputs": 0,
            "unauthorized_route_writes": 0,
            "total_distance_all_robots": 30,
            "scheduling_latency_p95_ms": 1}}


def test_promotes_paired_safe_candidate_with_business_gain():
    base = [result(seed) for seed in (1, 2)]
    candidate = [result(seed, throughput=5.2, completion=94) for seed in (1, 2)]
    assert evaluate_promotion(base, candidate)["promote"]


def test_rejects_safety_regression_even_when_faster():
    base = [result(1)]
    candidate = [result(1, throughput=6.0, completion=80)]
    candidate[0]["summary_metrics"]["pair_distance_violations"] = 1
    decision = evaluate_promotion(base, candidate)
    assert not decision["promote"]
    assert any("pair_violations" in item for item in decision["failures"])


def test_rejects_duplicate_validation_seed():
    with pytest.raises(ValueError, match="duplicate"):
        evaluate_promotion([result(1), result(1)], [result(1)])
