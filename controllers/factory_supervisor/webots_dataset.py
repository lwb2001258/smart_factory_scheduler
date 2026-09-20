"""Audit Webots decision traces and build offline-training transitions."""

import json
from pathlib import Path

import numpy as np

from config import RL_ENVIRONMENT_VERSION
from rl_contract import RL_SCHEDULING_CONTRACT
from rl_environment import RewardConfig
from rl_event_ledger import RLEvent, reward_for_event
from time_discount import elapsed_bootstrap_discount


DATASET_VERSION = "webots-transition-v2-current-mask"


def _event_from_dict(row):
    return RLEvent(str(row["event_type"]), float(row["sim_time"]),
                   int(row.get("robot_id", 0)), int(row.get("task_id", 0)),
                   dict(row.get("values", {})))


def audit_result(result: dict) -> dict:
    info = result.get("experiment_info", {})
    provenance = info.get("provenance", {}) or {}
    if provenance.get("run_mode") != "webots":
        raise ValueError("fine-tuning data must come from Webots")
    expected = RL_SCHEDULING_CONTRACT.fingerprint()
    if provenance.get("contract_fingerprint") != expected:
        raise ValueError("result contract fingerprint mismatch")
    if provenance.get("environment_version") != RL_ENVIRONMENT_VERSION:
        raise ValueError("result environment version mismatch")
    metrics = result.get("summary_metrics", {})
    fallback_count = (int(metrics.get("rl_fallback_decisions", 0))
                      + int(metrics.get("fallback_scheduler_commits", 0)))
    if fallback_count:
        raise ValueError(
            f"fine-tuning result is fallback-contaminated: {fallback_count}")
    decisions = result.get("rl_decisions", [])
    if not decisions:
        raise ValueError("result contains no committed RL decisions")
    ids = set()
    for row in decisions:
        decision_id = row.get("decision_id")
        if not decision_id or decision_id in ids:
            raise ValueError("missing or duplicate decision_id")
        ids.add(decision_id)
        state = np.asarray(row.get("state"), dtype=np.float32)
        mask = np.asarray(row.get("action_mask"), dtype=bool)
        action = int(row.get("action", -1))
        if state.shape != (RL_SCHEDULING_CONTRACT.observation_dim,):
            raise ValueError("decision state dimension mismatch")
        if mask.shape != (RL_SCHEDULING_CONTRACT.action_dim,):
            raise ValueError("decision action-mask dimension mismatch")
        if not np.isfinite(state).all() or not 0 <= action < len(mask):
            raise ValueError("invalid decision state/action")
        if not mask[action]:
            raise ValueError("recorded action was masked")
        contract = row.get("contract", {})
        if contract.get("fingerprint") != expected:
            raise ValueError("decision contract fingerprint mismatch")
        if "FALLBACK" in str(row.get("algorithm", "")).upper():
            raise ValueError("fallback decision cannot enter fine-tuning data")
    dangling = sorted({
        event.get("values", {}).get("decision_id")
        for event in result.get("rl_event_ledger", [])
        if event.get("values", {}).get("decision_id") is not None
        and event.get("values", {}).get("decision_id") not in ids})
    if dangling:
        raise ValueError(f"events reference unknown decisions: {dangling}")
    return {"decisions": len(decisions), "events": len(
        result.get("rl_event_ledger", [])), "contract_fingerprint": expected}


def build_transitions(result: dict, reward_config=None):
    audit_result(result)
    reward_config = reward_config or RewardConfig()
    decisions = sorted(result["rl_decisions"],
                       key=lambda row: (float(row["sim_time"]),
                                        str(row["decision_id"])))
    rewards = {str(row["decision_id"]): 0.0 for row in decisions}
    unscoped = []
    for raw in result.get("rl_event_ledger", []):
        event = _event_from_dict(raw)
        value = reward_for_event(event, reward_config)
        owner = event.values.get("decision_id")
        if owner in rewards:
            rewards[owner] += value
        elif owner is None:
            unscoped.append((event.sim_time, value))
    for event_time, value in unscoped:
        eligible = [row for row in decisions
                    if float(row["sim_time"]) <= event_time + 1e-9]
        if eligible:
            rewards[str(eligible[-1]["decision_id"])] += value
    transitions = []
    for index, row in enumerate(decisions):
        terminal = index == len(decisions) - 1
        following = decisions[index + 1] if not terminal else row
        elapsed = max(0.0, float(following["sim_time"])
                      - float(row["sim_time"]))
        transitions.append({
            "dataset_version": DATASET_VERSION,
            "decision_id": str(row["decision_id"]),
            "state": row["state"], "action": int(row["action"]),
            "action_mask": row["action_mask"],
            "reward": float(rewards[str(row["decision_id"])]),
            "next_state": following["state"],
            "done": terminal,
            "next_action": (None if terminal else int(following["action"])),
            "next_action_mask": following["action_mask"],
            "elapsed_seconds": elapsed,
            "bootstrap_discount": elapsed_bootstrap_discount(
                elapsed, terminal=terminal),
        })
    return transitions


def load_result(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_jsonl(rows, path):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    return target


def load_jsonl(paths):
    rows = []
    for path in paths:
        with Path(path).open("r", encoding="utf-8") as stream:
            rows.extend(json.loads(line) for line in stream if line.strip())
    return rows
