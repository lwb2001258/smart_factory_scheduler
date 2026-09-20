"""Fair-comparison gates for paired scheduler experiments."""


def result_identity(result: dict) -> dict:
    info = result.get("experiment_info", {})
    provenance = info.get("provenance", {}) or {}
    duration = info.get("sim_duration")
    required = {
        "scenario": info.get("scenario"),
        "scheduler": info.get("scheduler"),
        "seed": provenance.get("seed"),
        "run_mode": provenance.get("run_mode"),
        "manifest_fingerprint": provenance.get("manifest_fingerprint"),
        "manifest_version": provenance.get("manifest_version"),
        "sim_duration": duration,
    }
    missing = [key for key, value in required.items() if value is None]
    if missing:
        raise ValueError(f"comparison result missing identity: {missing}")
    return required


def validate_paired_results(all_results: dict) -> dict:
    """Require every scheduler in a scenario to share the same paired runs."""
    audit = {}
    for scenario, scheduler_results in all_results.items():
        indexes = {}
        for scheduler, results in scheduler_results.items():
            index = {}
            for result in results:
                if not result or "summary_metrics" not in result:
                    continue
                identity = result_identity(result)
                if identity["scenario"] != scenario:
                    raise ValueError("result scenario does not match report group")
                if identity["scheduler"] != scheduler:
                    raise ValueError("result scheduler does not match report group")
                seed = int(identity["seed"])
                if seed in index:
                    raise ValueError(f"duplicate result for {scenario}/{scheduler}/{seed}")
                index[seed] = (result, identity)
            indexes[scheduler] = index
        seed_sets = {scheduler: set(index) for scheduler, index in indexes.items()}
        expected = next(iter(seed_sets.values()), set())
        if not expected or any(seeds != expected for seeds in seed_sets.values()):
            raise ValueError(f"unpaired seed sets for scenario {scenario}: {seed_sets}")
        for seed in sorted(expected):
            identities = [index[seed][1] for index in indexes.values()]
            for field in ("run_mode", "manifest_fingerprint",
                          "manifest_version", "sim_duration"):
                values = {identity[field] for identity in identities}
                if len(values) != 1:
                    raise ValueError(
                        f"mixed {field} for scenario {scenario}, seed {seed}: "
                        f"{sorted(values, key=str)}")
        audit[scenario] = {
            "paired_seeds": sorted(expected),
            "schedulers": sorted(indexes),
            "run_mode": next(iter(indexes.values()))[min(expected)][1]["run_mode"],
        }
    return audit
