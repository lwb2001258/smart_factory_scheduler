import math
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from audit_collision_workflow import (
    AuditError, assert_result_fresh, audit_result, capture_snapshot,
    write_evidence)


def result():
    return {
        "experiment_info": {
            "scenario": "C", "scheduler": "FCFS", "sim_duration": 90.0,
            "timestamp": "2026-09-09 12:00:00",
            "provenance": {
                "seed": 42, "run_mode": "webots", "sha256": "a",
                "contract_fingerprint": "b", "manifest_fingerprint": "c",
                "motion_safety_fingerprint": "d" * 64,
                "motion_safety_config": {
                    "version": "motion-safety-v1", "fingerprint": "d" * 64},
            },
        },
        "summary_metrics": {
            "audited_replans": 2, "physical_escapes": 1,
            "route_dispatches": 2, "unauthorized_route_writes": 0,
            "pair_distance_violations": 0, "min_pair_distance": 0.61,
            "nonphysical_recoveries": 0,
            "joint_cell_traversal_samples": 2,
            "joint_cell_traversal_p99_seconds": 2.9,
            "joint_window_gaps": 0,
            "reservation_expired_movements": 0,
        },
        "replan_events": [
            {"source": "_joint_stall_recovery"},
            {"source": "_command_reverse"},
        ],
        "escape_events": [{"source": "_joint_escape_robot"}],
        "route_dispatch_events": [
            {"source": "joint_grid_transaction"},
            {"source": "_joint_stall_recovery"},
        ],
        "nonphysical_recovery_events": [],
        "unauthorized_route_write_events": [],
        "joint_cell_traversal_seconds": [2.5, 2.9],
        "joint_window_gap_events": [],
        "reservation_expired_movement_events": [],
        "execution_metrics": {
            "webots_wall_seconds": 45.0,
            "sim_to_wall_ratio": 2.0,
        },
    }


def audit(data, **overrides):
    options = dict(scenario="C", scheduler="FCFS", seed=42,
                   minimum_duration=90.0)
    options.update(overrides)
    return audit_result(data, **options)


def test_valid_result_is_audited_and_replans_are_categorized():
    report = audit(result())
    assert report["status"] == "PASS"
    assert report["replans"]["by_category"] == {
        "escape_triggered": 1, "stall_recovery": 1}
    assert report["route_dispatches"]["by_category"] == {
        "rolling_joint": 1, "stall_recovery": 1}


def test_webots_performance_gate_is_fail_closed():
    data = result()
    data["summary_metrics"]["scheduling_latency_p99_ms"] = 20.0
    report = audit(
        data, require_webots_performance=True,
        minimum_sim_to_wall_ratio=1.0,
        maximum_scheduling_p99_ms=50.0)
    assert report["webots_performance"]["sim_to_wall_ratio"] == 2.0
    data["execution_metrics"]["sim_to_wall_ratio"] = 0.5
    with pytest.raises(AuditError, match="sim/wall ratio"):
        audit(data, require_webots_performance=True,
              minimum_sim_to_wall_ratio=1.0)


@pytest.mark.parametrize("field,value", [
    ("scenario", "A"), ("scheduler", "DQN"),
])
def test_result_identity_mismatch_fails_closed(field, value):
    data = result()
    data["experiment_info"][field] = value
    with pytest.raises(AuditError, match="identity mismatch"):
        audit(data)


def test_seed_and_run_mode_mismatch_fail_closed():
    for key, value in (("seed", 43), ("run_mode", "standalone")):
        data = result()
        data["experiment_info"]["provenance"][key] = value
        with pytest.raises(AuditError, match="identity mismatch"):
            audit(data)


def test_short_run_and_missing_provenance_fail_closed():
    data = result()
    data["experiment_info"]["sim_duration"] = 89.0
    with pytest.raises(AuditError, match="short Webots result"):
        audit(data)
    data = result()
    del data["experiment_info"]["provenance"]["manifest_fingerprint"]
    with pytest.raises(AuditError, match="provenance.manifest_fingerprint"):
        audit(data)


def test_checkpoint_hash_is_required_only_for_learning_scheduler():
    data = result()
    del data["experiment_info"]["provenance"]["sha256"]
    del data["experiment_info"]["provenance"]["contract_fingerprint"]
    assert audit(data)["status"] == "PASS"
    data["experiment_info"]["scheduler"] = "DQN"
    with pytest.raises(AuditError, match="provenance.sha256"):
        audit_result(data, scenario="C", scheduler="DQN", seed=42,
                     minimum_duration=90.0)


def test_partial_motion_safety_provenance_is_rejected():
    data = result()
    del data["experiment_info"]["provenance"]["motion_safety_config"]
    with pytest.raises(AuditError, match="motion_safety_config"):
        audit(data)


def test_motion_safety_can_be_required_and_pinned():
    data = result()
    assert audit_result(
        data, scenario="C", scheduler="FCFS", seed=42,
        minimum_duration=90, require_motion_safety=True,
        expected_motion_safety_fingerprint="d" * 64)["status"] == "PASS"
    data["experiment_info"]["provenance"][
        "motion_safety_config"]["fingerprint"] = "e" * 64
    with pytest.raises(AuditError, match="config/fingerprint mismatch"):
        audit(data)


def test_missing_motion_safety_is_rejected_when_required():
    data = result()
    del data["experiment_info"]["provenance"]["motion_safety_fingerprint"]
    del data["experiment_info"]["provenance"]["motion_safety_config"]
    with pytest.raises(AuditError, match="provenance is required"):
        audit_result(data, scenario="C", scheduler="FCFS", seed=42,
                     minimum_duration=90, require_motion_safety=True)


def test_joint_timing_gate_requires_samples_and_enforces_p99():
    data = result()
    assert audit_result(
        data, scenario="C", scheduler="FCFS", seed=42,
        minimum_duration=90, require_joint_timing=True,
        maximum_traversal_p99=3.0)["joint_timing"]["samples"] == 2
    data["summary_metrics"]["joint_cell_traversal_p99_seconds"] = 3.1
    with pytest.raises(AuditError, match="exceeds"):
        audit_result(data, scenario="C", scheduler="FCFS", seed=42,
                     minimum_duration=90, require_joint_timing=True,
                     maximum_traversal_p99=3.0)


def test_joint_timing_gate_rejects_gap_and_expired_movement():
    for metric, events in (
        ("joint_window_gaps", "joint_window_gap_events"),
        ("reservation_expired_movements",
         "reservation_expired_movement_events"),
    ):
        data = result()
        data["summary_metrics"][metric] = 1
        data[events].append({"robot_id": 1})
        with pytest.raises(AuditError, match="timing safety failure"):
            audit_result(data, scenario="C", scheduler="FCFS", seed=42,
                         minimum_duration=90, require_joint_timing=True)


def test_nonfinite_and_counter_mismatch_fail_closed():
    data = result()
    data["summary_metrics"]["min_pair_distance"] = math.nan
    with pytest.raises(AuditError, match="non-finite"):
        audit(data)
    data = result()
    data["summary_metrics"]["audited_replans"] = 3
    with pytest.raises(AuditError, match="replan count mismatch"):
        audit(data)


def test_route_and_recovery_counters_are_reconciled():
    data = result()
    data["summary_metrics"]["route_dispatches"] = 3
    with pytest.raises(AuditError, match="route dispatch count mismatch"):
        audit(data)
    data = result()
    data["unauthorized_route_write_events"].append({"source": "bad"})
    with pytest.raises(AuditError, match="unauthorized write count mismatch"):
        audit(data)


def test_unsafe_result_requires_explicit_baseline_mode():
    data = result()
    data["summary_metrics"]["pair_distance_violations"] = 1
    data["summary_metrics"]["min_pair_distance"] = 0.49
    with pytest.raises(AuditError, match="unsafe Webots result"):
        audit(data)
    report = audit(data, allow_unsafe_baseline=True)
    assert report["status"] == "CAPTURED_UNSAFE_BASELINE"


def test_nonphysical_recovery_is_rejected_even_for_unsafe_baseline():
    data = result()
    data["summary_metrics"]["nonphysical_recoveries"] = 1
    data["nonphysical_recovery_events"].append({"source": "teleport"})
    with pytest.raises(AuditError, match="nonphysical_recoveries"):
        audit(data)
    with pytest.raises(AuditError, match="nonphysical_recoveries"):
        audit(data, allow_unsafe_baseline=True)


def test_result_older_than_launch_is_rejected(tmp_path):
    path = tmp_path / "result.json"
    path.write_text("{}", encoding="utf-8")
    with pytest.raises(AuditError, match="stale result"):
        assert_result_fresh(
            path, result(), datetime.now(timezone.utc) + timedelta(seconds=1))


def test_recorded_experiment_time_must_also_be_fresh(tmp_path):
    path = tmp_path / "result.json"
    path.write_text("{}", encoding="utf-8")
    with pytest.raises(AuditError, match="experiment timestamp"):
        assert_result_fresh(
            path, result(), datetime(2026, 9, 9, 5, tzinfo=timezone.utc))


def test_evidence_file_cannot_be_overwritten(tmp_path):
    path = tmp_path / "audit.json"
    write_evidence(path, "first")
    with pytest.raises(AuditError, match="refusing to overwrite"):
        write_evidence(path, "second")
    assert path.read_text(encoding="utf-8") == "first\n"


def test_snapshot_contains_dirty_state_and_required_hashes():
    snapshot = capture_snapshot()
    assert snapshot["schema"] == "collision-workflow-snapshot-v1"
    assert isinstance(snapshot["git_dirty"], bool)
    assert snapshot["resolved_config"]["ENABLE_JOINT_RUNTIME"] is True
    assert snapshot["world_hash"] == snapshot["files"]["worlds/smart_factory.wbt"]
    assert all(len(value) == 64 for value in snapshot["files"].values())
