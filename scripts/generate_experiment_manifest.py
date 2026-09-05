import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from experiment_manifest import generate_experiment_manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("A", "B", "C"), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--duration", type=float, default=1800.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    manifest = generate_experiment_manifest(
        args.scenario, args.seed, args.duration)
    payload = manifest.to_dict()
    payload["fingerprint"] = manifest.fingerprint()
    text = json.dumps(payload, indent=2)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    else:
        print(text)


if __name__ == "__main__":
    main()

