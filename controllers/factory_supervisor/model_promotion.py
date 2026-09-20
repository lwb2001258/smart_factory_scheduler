"""Paired Webots champion/candidate promotion gate."""

import statistics

from comparison_integrity import result_identity
from evaluation_objective import SelectionMetrics, algorithm_selection_score


def _metrics(result):
    values = result["summary_metrics"]
    generated = max(int(values.get("total_tasks_generated", 0)), 1)
    return {
        "completion_rate": float(values.get("total_tasks_completed", 0))/generated,
        "throughput": float(values.get("throughput_per_minute", 0.0)),
        "completion_time": float(values.get("avg_task_completion_time", 0.0)),
        "weighted_tardiness": float(values.get("weighted_tardiness", 0.0)),
        "hard_breaches": int(values.get("hard_deadline_breaches", 0)),
        "pair_violations": int(values.get("pair_distance_violations", 0)),
        "fallbacks": int(values.get("scheduler_fallbacks", 0))
                     + int(values.get("rl_fallback_decisions", 0)),
        "invalid": int(values.get("invalid_scheduler_outputs", 0)),
        "unauthorized": int(values.get("unauthorized_route_writes", 0)),
        "score": algorithm_selection_score(SelectionMetrics(
            completion_rate=float(values.get("total_tasks_completed", 0))/generated,
            mean_completion_time=float(values.get("avg_task_completion_time", 0)),
            mean_waiting_time=float(values.get("avg_waiting_time", 0)),
            mean_makespan=float(result["experiment_info"]["sim_duration"]),
            mean_distance=float(values.get("total_distance_all_robots", 0)),
            invalid_actions=int(values.get("invalid_scheduler_outputs", 0)),
            native_failures=int(values.get("scheduler_fallbacks", 0)),
            p95_latency_ms=float(values.get("scheduling_latency_p95_ms", 0))))}


def evaluate_promotion(base_results, candidate_results):
    def index(rows):
        indexed = {}
        for row in rows:
            seed = int(result_identity(row)["seed"])
            if seed in indexed:
                raise ValueError(f"duplicate validation seed: {seed}")
            indexed[seed] = row
        return indexed
    base, candidate = index(base_results), index(candidate_results)
    if not base or set(base) != set(candidate):
        raise ValueError("base and candidate seed sets are not paired")
    paired = []
    for seed in sorted(base):
        left, right = result_identity(base[seed]), result_identity(candidate[seed])
        for field in ("scenario", "run_mode", "manifest_fingerprint",
                      "manifest_version", "sim_duration"):
            if left[field] != right[field]:
                raise ValueError(f"base/candidate {field} mismatch for seed {seed}")
        if left["run_mode"] != "webots":
            raise ValueError("promotion requires Webots results")
        paired.append((seed, _metrics(base[seed]), _metrics(candidate[seed])))
    hard_failures = []
    for seed, old, new in paired:
        for field in ("fallbacks", "invalid", "unauthorized"):
            if new[field] != 0:
                hard_failures.append(f"seed {seed}: candidate {field}={new[field]}")
        for field in ("hard_breaches", "pair_violations"):
            if new[field] > old[field]:
                hard_failures.append(f"seed {seed}: {field} regressed")
        if new["completion_rate"] < old["completion_rate"]:
            hard_failures.append(f"seed {seed}: completion rate regressed")
    mean = lambda side, field: statistics.fmean(
        pair[side][field] for pair in paired)
    base_mean = {field: mean(1, field) for field in paired[0][1]}
    candidate_mean = {field: mean(2, field) for field in paired[0][2]}
    throughput_gain = ((candidate_mean["throughput"] /
                        max(base_mean["throughput"], 1e-9)) - 1.0)
    completion_gain = ((base_mean["completion_time"] -
                        candidate_mean["completion_time"]) /
                       max(base_mean["completion_time"], 1e-9))
    business_improved = throughput_gain >= 0.02 or completion_gain >= 0.05
    if candidate_mean["weighted_tardiness"] > base_mean["weighted_tardiness"]:
        hard_failures.append("mean weighted tardiness regressed")
    if candidate_mean["score"] <= base_mean["score"]:
        hard_failures.append("canonical selection score did not improve")
    if not business_improved:
        hard_failures.append("neither throughput nor completion-time gate improved")
    return {"promote": not hard_failures, "paired_seeds": sorted(base),
            "base_mean": base_mean, "candidate_mean": candidate_mean,
            "throughput_gain": throughput_gain,
            "completion_time_gain": completion_gain,
            "failures": hard_failures}
