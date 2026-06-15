#!/usr/bin/env python3
"""Report whether automatic runtime logs and required human labels are current."""

import argparse
import csv
import os
from pathlib import Path


REPO_ROOT = Path(os.path.expanduser("~/doosan_ws/src/e0509_gripper_description"))


def main():
    parser = argparse.ArgumentParser(description="수확 실험 기록 누락 확인")
    parser.add_argument("--cell", default="root/nw")
    args = parser.parse_args()
    suffix = args.cell.replace("/", "_")
    sheet = REPO_ROOT / f"reports/harvest_kpi/manual_labels_{suffix}.csv"
    if not sheet.exists():
        raise SystemExit(
            f"라벨 시트 없음: {sheet}\n"
            f"python3 scripts/prepare_harvest_label_sheet.py --cell {args.cell}")
    with sheet.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = ("stem_grasp", "detach", "retention", "human_intervention")
    missing = [
        row for row in rows if not all(row.get(field, "").strip() for field in required)]
    print(f"cell={args.cell} attempts={len(rows)} labeled={len(rows)-len(missing)} missing={len(missing)}")
    for row in missing[-10:]:
        print(
            f"- run={row['source_run_id']} attempt={row['source_attempt_index']} "
            f"automatic={row['automatic_grasp_result']} duration={row['duration_sec']}s")


if __name__ == "__main__":
    main()
