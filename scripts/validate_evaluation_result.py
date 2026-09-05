import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from evaluation_gate import validate_evaluation_result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result")
    parser.add_argument("--algorithm", choices=("SARSA", "DQN", "PPO_RL", "SARSA_LAMBDA",
                                                "RAINBOW_DQN", "A2C", "DISCRETE_SAC", "QR_DQN"),
                        required=True)
    args = parser.parse_args()
    try:
        payload = json.loads(Path(args.result).read_text(encoding="utf-8-sig"))
        report = validate_evaluation_result(payload, args.algorithm)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({"valid": False, "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
