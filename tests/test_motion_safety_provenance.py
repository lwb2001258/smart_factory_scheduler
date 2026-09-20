import sys
from pathlib import Path


SUPERVISOR = (Path(__file__).resolve().parents[1] /
              "controllers" / "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR))

from motion_safety import MOTION_SAFETY


def test_motion_safety_contract_is_stable_and_json_serializable():
    resolved = MOTION_SAFETY.resolved()
    assert resolved["fingerprint"] == MOTION_SAFETY.fingerprint()
    assert resolved["stopping_distance_m"] > 0
    assert len(resolved["fingerprint"]) == 64
