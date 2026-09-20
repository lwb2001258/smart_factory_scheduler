"""Convert one or more Webots experiment results to audited JSONL."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "factory_supervisor"))

from webots_dataset import build_transitions, load_result, write_jsonl


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rows = []
    for path in args.input:
        rows.extend(build_transitions(load_result(path)))
    write_jsonl(rows, args.output)
    print(json.dumps({"output": args.output, "transitions": len(rows)}))


if __name__ == "__main__":
    main()
