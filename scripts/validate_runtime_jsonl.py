#!/usr/bin/env python3
"""Validate and summarize runtime JSONL files before simulation import."""

import argparse
import collections
import glob
import json
import os
import sys

from runtime_jsonl_logger import SCHEMA_VERSION


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "paths",
        nargs="*",
        default=[os.path.expanduser(
            "~/doosan_ws/src/e0509_gripper_description/logs/runtime/**/*.jsonl")],
    )
    args = parser.parse_args()

    files = []
    for pattern in args.paths:
        matches = glob.glob(os.path.expanduser(pattern), recursive=True)
        files.extend(matches or [pattern])
    files = sorted(set(files))

    if not files:
        print("No runtime JSONL files found.", file=sys.stderr)
        return 1

    events = collections.Counter()
    nodes = collections.Counter()
    bad = 0
    total = 0
    for path in files:
        with open(path, "r", encoding="utf-8") as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    record = json.loads(line)
                    required = {"schema_version", "timestamp", "run_id", "node", "event", "data"}
                    missing = required - set(record)
                    if missing:
                        raise ValueError(f"missing={sorted(missing)}")
                    if record["schema_version"] != SCHEMA_VERSION:
                        raise ValueError(
                            f"schema={record['schema_version']} expected={SCHEMA_VERSION}")
                    events[record["event"]] += 1
                    nodes[record["node"]] += 1
                    total += 1
                except Exception as exc:
                    bad += 1
                    print(f"INVALID {path}:{line_no}: {exc}", file=sys.stderr)

    print(f"files={len(files)} events={total} invalid={bad}")
    print("nodes:")
    for node, count in nodes.most_common():
        print(f"  {node}: {count}")
    print("events:")
    for event, count in events.most_common():
        print(f"  {event}: {count}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
