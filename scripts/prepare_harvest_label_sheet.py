#!/usr/bin/env python3
"""Create/update a CSV sheet where humans fill only non-automatable observations."""

import argparse
import collections
import csv
import glob
import json
import os
from pathlib import Path


REPO_ROOT = Path(os.path.expanduser("~/doosan_ws/src/e0509_gripper_description"))
RUNTIME_GLOB = str(REPO_ROOT / "logs/runtime/*/curobo_planner_node_*.jsonl")
TERMINALS = {
    "pick_sequence_complete", "pick_sequence_stopped", "pick_sequence_hold_latched"}
FIELDS = [
    "source_runtime_jsonl", "source_run_id", "source_attempt_index", "cell",
    "scene_id", "automatic_terminal", "automatic_grasp_result", "duration_sec",
    "stem_grasp", "detach", "retention", "non_target_contact",
    "human_intervention", "place", "notes",
]


def _attempt_rows(paths, cell):
    rows = []
    for path in paths:
        records = [
            json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        starts = [i for i, record in enumerate(records)
                  if record.get("event") == "pick_sequence_start"]
        for attempt_index, start_index in enumerate(starts, 1):
            end_index = (
                starts[attempt_index]
                if attempt_index < len(starts)
                else len(records)
            )
            attempt = records[start_index:end_index]
            terminal = next(
                (r for r in attempt if r.get("event") in TERMINALS), None)
            if terminal is None:
                continue
            verify = next(
                (r for r in attempt if r.get("event") == "verify_grasp"), {})
            context = attempt[0].get("experiment_context", {})
            if cell and context.get("cell") != cell:
                continue
            rows.append({
                "source_runtime_jsonl": os.path.abspath(path),
                "source_run_id": attempt[0].get("run_id", ""),
                "source_attempt_index": attempt_index,
                "cell": context.get("cell", ""),
                "scene_id": context.get("scene_id", ""),
                "automatic_terminal": terminal.get("data", {}).get(
                    "result_code", terminal.get("event", "")),
                "automatic_grasp_result": verify.get("data", {}).get(
                    "result_code", "NOT_RECORDED"),
                "duration_sec": round(
                    float(terminal.get("monotonic_sec", 0.0))
                    - float(attempt[0].get("monotonic_sec", 0.0)), 3),
            })
    return rows


def _key(row):
    return (
        row.get("source_runtime_jsonl", ""),
        str(row.get("source_run_id", "")),
        str(row.get("source_attempt_index", "")),
    )


def main():
    parser = argparse.ArgumentParser(description="사람 판정용 CSV 시트 생성/갱신")
    parser.add_argument("--cell", help="예: root/nw")
    parser.add_argument("--output")
    args = parser.parse_args()
    suffix = (args.cell or "all").replace("/", "_")
    output = Path(os.path.expanduser(
        args.output or str(REPO_ROOT / f"reports/harvest_kpi/manual_labels_{suffix}.csv")))
    output.parent.mkdir(parents=True, exist_ok=True)

    existing = {}
    if output.exists():
        with output.open("r", encoding="utf-8-sig", newline="") as stream:
            existing = {_key(row): row for row in csv.DictReader(stream)}

    rows = _attempt_rows(sorted(glob.glob(RUNTIME_GLOB)), args.cell)
    merged = []
    for row in rows:
        old = existing.get(_key(row), {})
        merged.append({field: old.get(field, row.get(field, "")) for field in FIELDS})

    with output.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(merged)

    required = ("stem_grasp", "detach", "retention", "human_intervention")
    complete = sum(all(row.get(field) for field in required) for row in merged)
    print(output)
    print(f"rows={len(merged)} labeled={complete} unlabeled={len(merged) - complete}")
    print("입력값: yes/no/unknown, non_target_contact=none/leaf_or_stem/other_fruit/structure/multiple, place=not_attempted/success/fail/unknown")


if __name__ == "__main__":
    main()
