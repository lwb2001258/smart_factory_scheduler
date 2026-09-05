"""Validate RL checkpoints and emit machine-readable provenance metadata."""

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from rl_model_registry import audit_checkpoint, registry_snapshot


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--algorithm", choices=("SARSA", "DQN", "PPO_RL", "SARSA_LAMBDA",
                                                "RAINBOW_DQN", "A2C", "DISCRETE_SAC", "QR_DQN"))
    parser.add_argument("--checkpoint")
    parser.add_argument("--allow-legacy-ppo", action="store_true")
    parser.add_argument("--registry", action="store_true")
    args = parser.parse_args()
    if args.registry:
        print(json.dumps(registry_snapshot(), indent=2))
        return 0
    if not args.algorithm or not args.checkpoint:
        parser.error("--algorithm and --checkpoint are required unless --registry is used")
    try:
        result = audit_checkpoint(
            args.algorithm, args.checkpoint,
            allow_legacy_ppo=args.allow_legacy_ppo)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({"valid": False, "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(result.to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
