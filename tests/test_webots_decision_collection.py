import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from metrics_collector import MetricsCollector


def test_metrics_serializes_committed_rl_decisions(tmp_path):
    collector = MetricsCollector("A", "DQN", 3)
    collector.output_dir = str(tmp_path)
    collector.output_path = str(tmp_path / "result.json")
    collector.record_rl_decision({"decision_id": "d1", "state": [0.0],
                                  "action": 0, "action_mask": [True]})
    assert collector.rl_decisions == [{"decision_id": "d1", "state": [0.0],
                                       "action": 0,
                                       "action_mask": [True]}]


def test_non_mapping_decision_is_ignored():
    collector = MetricsCollector("A", "DQN", 3)
    collector.record_rl_decision(None)
    assert collector.rl_decisions == []


def test_decision_record_is_copied_from_caller():
    collector = MetricsCollector("A", "DQN", 3)
    decision = {"decision_id": "d1", "action": 4}
    collector.record_rl_decision(decision)
    decision["action"] = 9
    assert collector.rl_decisions[0]["action"] == 4
