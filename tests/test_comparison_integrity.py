import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from comparison_integrity import validate_paired_results


def result(scheduler, seed, mode="webots", manifest=None, duration=60.0):
    return {"experiment_info": {
        "scenario": "A", "scheduler": scheduler,
        "sim_duration": duration,
        "provenance": {"seed": seed, "run_mode": mode,
                       "manifest_version": "v2",
                       "manifest_fingerprint": manifest or f"manifest-{seed}"}},
        "summary_metrics": {"throughput_per_minute": 1.0}}


def test_accepts_paired_algorithms_with_identical_provenance():
    data = {"A": {name: [result(name, seed) for seed in (1, 2)]
                  for name in ("DQN", "SARSA")}}
    audit = validate_paired_results(data)
    assert audit["A"]["paired_seeds"] == [1, 2]
    assert audit["A"]["run_mode"] == "webots"


@pytest.mark.parametrize("mutation,match", [
    (lambda data: data["A"]["DQN"].pop(), "unpaired seed"),
    (lambda data: data["A"]["DQN"][0]["experiment_info"]["provenance"].update(
        run_mode="standalone"), "mixed run_mode"),
    (lambda data: data["A"]["DQN"][0]["experiment_info"]["provenance"].update(
        manifest_fingerprint="different"), "mixed manifest"),
    (lambda data: data["A"]["DQN"][0]["experiment_info"].update(
        sim_duration=120.0), "mixed sim_duration"),
])
def test_rejects_unfair_comparison_cohorts(mutation, match):
    data = {"A": {name: [result(name, seed) for seed in (1, 2)]
                  for name in ("DQN", "SARSA")}}
    mutation(data)
    with pytest.raises(ValueError, match=match):
        validate_paired_results(data)


def test_report_emits_paired_cohort_audit(tmp_path):
    sys.path.insert(0, str(ROOT / "scripts"))
    import run_experiments
    original = run_experiments.RESULTS_DIR
    run_experiments.RESULTS_DIR = str(tmp_path)
    try:
        data = {"A": {name: [result(name, seed) for seed in (1, 2)]
                      for name in ("DQN", "SARSA")}}
        for rows in data["A"].values():
            for row in rows:
                row["summary_metrics"].update({
                    "avg_task_completion_time": 2.0,
                    "avg_waiting_time": 1.0,
                    "avg_robot_idle_pct": 3.0,
                    "total_conflicts_resolved": 0,
                    "scheduling_latency_p95_ms": 0.1,
                    "scheduling_latency_p99_ms": 0.2,
                })
        run_experiments.generate_comparison_report(data)
        report = (tmp_path / "comparison_report.txt").read_text()
        assert "Paired seeds: [1, 2] | mode: webots" in report
    finally:
        run_experiments.RESULTS_DIR = original
