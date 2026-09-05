"""Machine-enforced publication gate for RL experiment results."""


REQUIRED_RL_PROVENANCE = {
    "sha256", "environment_version", "contract_fingerprint",
    "manifest_fingerprint", "manifest_version", "run_mode", "seed",
}


def validate_evaluation_result(result: dict, algorithm: str) -> dict:
    if not isinstance(result, dict):
        raise ValueError("missing experiment result")
    info = result.get("experiment_info", {})
    provenance = info.get("provenance", {}) or {}
    missing = REQUIRED_RL_PROVENANCE.difference(provenance)
    if missing:
        raise ValueError(f"missing RL provenance: {sorted(missing)}")
    if info.get("scheduler") != algorithm:
        raise ValueError("result scheduler does not match requested algorithm")
    if provenance.get("run_mode") not in {"standalone", "webots"}:
        raise ValueError("invalid run mode")
    if provenance.get("checkpoint_contract_verified") is not True:
        raise ValueError("checkpoint does not embed the verified RL contract")
    metrics = result.get("summary_metrics", {})
    commits = metrics.get("scheduler_commits_by_algorithm", {}) or {}
    if int(commits.get(algorithm, 0)) <= 0:
        raise ValueError("no native algorithm commits")
    if any("FALLBACK" in str(name).upper() and int(count) > 0
           for name, count in commits.items()):
        raise ValueError("fallback-contaminated result")
    if int(metrics.get("rl_fallback_decisions", 0)) > 0:
        raise ValueError("runtime RL fallback detected")
    return {
        "valid": True,
        "algorithm": algorithm,
        "run_mode": provenance["run_mode"],
        "manifest_fingerprint": provenance["manifest_fingerprint"],
        "checkpoint_sha256": provenance["sha256"],
    }
