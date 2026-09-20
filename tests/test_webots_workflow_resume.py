import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))
sys.path.insert(0, str(ROOT / "scripts"))

import run_webots_finetune_workflow as workflow
from finetune_from_webots import sha256
from webots_dataset import DATASET_VERSION, write_jsonl
from rl_contract import RL_SCHEDULING_CONTRACT


def test_resume_recovers_only_matching_completed_validation(tmp_path):
    checkpoint = tmp_path / "model.pkl"
    checkpoint.write_bytes(b"checkpoint")
    results = tmp_path / "results"
    results.mkdir()
    matching = {"experiment_info": {"scenario": "C", "scheduler": "DQN",
        "sim_duration": 1800.00000001, "provenance": {
            "seed": 310001, "run_mode": "webots",
            "sha256": sha256(checkpoint)}}}
    (results / "experiment_C_DQN_1.json").write_text(json.dumps(matching))
    original = workflow.ROOT
    workflow.ROOT = tmp_path
    try:
        assert workflow.find_completed_validation(
            "C", 310001, 1800.0, checkpoint) == matching
        assert workflow.find_completed_validation(
            "C", 310002, 1800.0, checkpoint) is None
    finally:
        workflow.ROOT = original


def test_webots_retry_recovers_transient_launch_failure(monkeypatch):
    calls = []
    def fake_run(*args, **kwargs):
        calls.append(1)
        return None if len(calls) == 1 else {"ok": True}
    monkeypatch.setattr(workflow, "run_single_experiment", fake_run)
    monkeypatch.setattr(workflow.time, "sleep", lambda seconds: None)
    args = type("Args", (), {"webots_retries": 2, "scenario": "A",
                              "algorithm": "DQN",
                              "webots": "webots"})()
    assert workflow.run_webots_with_retry(
        args, 1, "model.pkl", "validation") == {"ok": True}
    assert len(calls) == 2


def test_webots_retry_recovers_rejected_native_result(monkeypatch):
    calls = []
    def fake_run(*args, **kwargs):
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError("fallback-contaminated")
        return {"ok": True}
    monkeypatch.setattr(workflow, "run_single_experiment", fake_run)
    monkeypatch.setattr(workflow.time, "sleep", lambda seconds: None)
    args = type("Args", (), {"webots_retries": 2, "scenario": "C",
                              "algorithm": "DQN", "webots": "webots"})()
    assert workflow.run_webots_with_retry(
        args, 310003, "model.pkl", "validation") == {"ok": True}
    assert len(calls) == 2


def _transition():
    return {"dataset_version": DATASET_VERSION, "decision_id": "d1",
            "state": [0.0] * RL_SCHEDULING_CONTRACT.observation_dim,
            "action": 0, "reward": 1.0,
            "action_mask": [True] * RL_SCHEDULING_CONTRACT.action_dim,
            "next_state": [0.0] * RL_SCHEDULING_CONTRACT.observation_dim,
            "done": True,
            "next_action_mask": [True] * RL_SCHEDULING_CONTRACT.action_dim,
            "elapsed_seconds": 1.0, "bootstrap_discount": 0.0}


def test_resume_strictly_reaudits_dataset_and_candidate(tmp_path):
    base = tmp_path / "base.pkl"; base.write_bytes(b"base")
    candidate = tmp_path / "candidate.pkl"; candidate.write_bytes(b"candidate")
    dataset = write_jsonl([_transition()], tmp_path / "webots_train.jsonl")
    report_path = tmp_path / "report.json"
    report = {"algorithm": "DQN", "base_sha256": sha256(base),
              "candidate": str(candidate), "candidate_sha256": sha256(candidate),
              "dataset_sha256": sha256(dataset), "dataset_version": DATASET_VERSION,
              "contract_fingerprint": RL_SCHEDULING_CONTRACT.fingerprint()}
    report_path.write_text(json.dumps(report))
    result = {"experiment_info": {"scenario": "C", "sim_duration": 120.0,
              "provenance": {"seed": 1, "run_mode": "webots",
                             "sha256": sha256(base),
                             "manifest_fingerprint": "m"}},
              "summary_metrics": {"rl_fallback_decisions": 0,
                                  "fallback_scheduler_commits": 0},
              "rl_decisions": [{"decision_id": "d1"}]}
    workflow.write_resume_manifest(
        tmp_path / "collection_manifest.json", algorithm="DQN",
        base_checkpoint=base, dataset=dataset, scenario="C", duration=120,
        collection_seeds=[1], collected=[result])
    assert workflow.audit_resume(
        tmp_path, algorithm="DQN", base_checkpoint=base, dataset=dataset,
        report_path=report_path, scenario="C", duration=120,
        collection_seeds=[1]) == report
    candidate.write_bytes(b"tampered")
    import pytest
    with pytest.raises(RuntimeError, match="candidate checkpoint mismatch"):
        workflow.audit_resume(
            tmp_path, algorithm="DQN", base_checkpoint=base, dataset=dataset,
            report_path=report_path, scenario="C", duration=120,
            collection_seeds=[1])


def test_resume_rejects_legacy_directory_without_manifest(tmp_path):
    import pytest
    base = tmp_path / "base.pkl"; base.write_bytes(b"base")
    dataset = write_jsonl([_transition()], tmp_path / "webots_train.jsonl")
    report = tmp_path / "report.json"; report.write_text("{}")
    with pytest.raises(RuntimeError, match="manifest is missing"):
        workflow.audit_resume(
            tmp_path, algorithm="DQN", base_checkpoint=base, dataset=dataset,
            report_path=report, scenario="C", duration=120,
            collection_seeds=[1])
