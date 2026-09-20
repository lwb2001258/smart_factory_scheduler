import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from config import RL_ENVIRONMENT_VERSION
from rl_contract import RL_SCHEDULING_CONTRACT
from rl_environment import RewardConfig
from webots_dataset import audit_result, build_transitions


def sample_result():
    contract = RL_SCHEDULING_CONTRACT.metadata()
    contract["fingerprint"] = RL_SCHEDULING_CONTRACT.fingerprint()
    state = [0.0] * RL_SCHEDULING_CONTRACT.observation_dim
    mask = [False] * RL_SCHEDULING_CONTRACT.action_dim
    mask[0] = True
    decision = lambda name, time: {
        "decision_id": name, "sim_time": time, "state": state,
        "action": 0, "action_mask": mask, "contract": contract}
    return {
        "experiment_info": {"provenance": {
            "run_mode": "webots",
            "environment_version": RL_ENVIRONMENT_VERSION,
            "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint()}},
        "summary_metrics": {"rl_fallback_decisions": 0,
                            "fallback_scheduler_commits": 0},
        "rl_decisions": [decision("d1", 1.0), decision("d2", 11.0)],
        "rl_event_ledger": [
            {"event_type": "assignment_committed", "sim_time": 1.0,
             "values": {"decision_id": "d1", "waiting_seconds": 0,
                        "empty_distance": 0}},
            {"event_type": "task_completed_on_time", "sim_time": 20.0,
             "values": {"decision_id": "d1", "on_time": True,
                        "business_weight": 1}},
            {"event_type": "queue_wait_advanced", "sim_time": 12.0,
             "values": {"pending_wait_increment": 120}},
        ]}


def test_builds_time_aware_transitions_and_delayed_reward():
    rows = build_transitions(sample_result())
    assert len(rows) == 2
    assert rows[0]["reward"] == pytest.approx(
        RewardConfig().completion + RewardConfig().on_time)
    assert rows[0]["bootstrap_discount"] == pytest.approx(0.99)
    assert rows[1]["reward"] == pytest.approx(RewardConfig().wait_increment)
    assert rows[1]["done"] and rows[1]["bootstrap_discount"] == 0.0


def test_audit_rejects_standalone_and_masked_actions():
    result = sample_result()
    result["experiment_info"]["provenance"]["run_mode"] = "standalone"
    with pytest.raises(ValueError, match="Webots"):
        audit_result(result)
    result = sample_result()
    result["rl_decisions"][0]["action_mask"][0] = False
    with pytest.raises(ValueError, match="masked"):
        audit_result(result)


def test_audit_rejects_dangling_reward_attribution():
    result = sample_result()
    result["rl_event_ledger"][0]["values"]["decision_id"] = "missing"
    with pytest.raises(ValueError, match="unknown decisions"):
        audit_result(result)


def test_audit_rejects_fallback_contaminated_collection():
    result = sample_result()
    result["summary_metrics"]["rl_fallback_decisions"] = 1
    with pytest.raises(ValueError, match="fallback-contaminated"):
        audit_result(result)
