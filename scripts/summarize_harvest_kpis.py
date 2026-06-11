#!/usr/bin/env python3
"""Summarize the six core harvest KPIs from human-label JSONL files."""

import argparse
import glob
import json
import os
import statistics


REPO_ROOT = os.path.expanduser("~/doosan_ws/src/e0509_gripper_description")
DEFAULT_GLOB = os.path.join(
    REPO_ROOT, "logs/human_labels/*/harvest_attempt_labels.jsonl")


def _load(paths):
    records = []
    for path in paths:
        with open(path, "r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    records.append(json.loads(line))
    return records


def _rate(records, getter, success_value="success", valid_values=None):
    valid_values = valid_values or {"success", "fail"}
    values = [getter(record) for record in records]
    evaluated = [value for value in values if value in valid_values]
    successes = sum(value == success_value for value in evaluated)
    return successes, len(evaluated)


def _print_rate(label, successes, evaluated):
    if evaluated:
        print(f"{label}: {successes}/{evaluated} = {100.0 * successes / evaluated:.1f}%")
    else:
        print(f"{label}: 측정 데이터 없음")


def _print_times(label, values):
    if not values:
        print(f"{label}: 측정 데이터 없음")
        return
    print(
        f"{label}: 평균 {statistics.mean(values):.1f}s "
        f"(n={len(values)}, 최소={min(values):.1f}s, 최대={max(values):.1f}s)")


def main():
    parser = argparse.ArgumentParser(description="수확 실험 핵심 KPI 6종 요약")
    parser.add_argument("paths", nargs="*", help="label JSONL 경로. 생략하면 전체 사용")
    args = parser.parse_args()
    paths = args.paths or sorted(glob.glob(DEFAULT_GLOB))
    if not paths:
        raise SystemExit(f"label JSONL not found: {DEFAULT_GLOB}")
    records = _load(paths)

    stem_ok, stem_n = _rate(
        records,
        lambda r: r.get("human_label", {}).get("stem_grasp"),
        success_value="yes",
        valid_values={"yes", "no"},
    )
    pick_ok, pick_n = _rate(
        records, lambda r: r.get("derived", {}).get("pick_success"))
    place_ok, place_n = _rate(
        records,
        lambda r: r.get("human_label", {}).get("place"),
        valid_values={"success", "fail"},
    )
    intervention_no, intervention_n = _rate(
        records,
        lambda r: r.get("human_label", {}).get("human_intervention"),
        success_value="no",
        valid_values={"yes", "no"},
    )
    pick_times = [
        float(r["automatic"]["duration_sec"]) for r in records
        if r.get("automatic", {}).get("duration_sec") is not None
    ]
    total_times = [
        float(r["human_measurement"]["total_task_time_sec"]) for r in records
        if r.get("human_measurement", {}).get("total_task_time_sec") is not None
    ]

    print(f"대상 라벨: {len(records)}건 ({len(paths)} file(s))\n")
    _print_rate("1. 실제 줄기 파지 성공률", stem_ok, stem_n)
    _print_rate("2. 최종 Pick 성공률(줄기 파지+분리+후퇴 유지)", pick_ok, pick_n)
    _print_times("3. 평균 Pick 시퀀스 시간", pick_times)
    _print_rate("4. Place 성공률", place_ok, place_n)
    _print_times("5. 전체 작업 시간", total_times)
    if intervention_n:
        interventions = intervention_n - intervention_no
        print(
            f"6. 사람 개입률: {interventions}/{intervention_n} = "
            f"{100.0 * interventions / intervention_n:.1f}%")
    else:
        print("6. 사람 개입률: 측정 데이터 없음")


if __name__ == "__main__":
    main()
