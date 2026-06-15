#!/usr/bin/env python3
"""Automatically summarize planning, execution and grasp-verifier runtime KPIs."""

import argparse
import collections
import glob
import json
import os
import statistics


REPO_ROOT = os.path.expanduser("~/doosan_ws/src/e0509_gripper_description")
DEFAULT_GLOB = os.path.join(
    REPO_ROOT, "logs/runtime/*/curobo_planner_node_*.jsonl")
TERMINAL_EVENTS = {
    "pick_sequence_complete",
    "pick_sequence_stopped",
    "pick_sequence_hold_latched",
}


def _load(paths):
    records = []
    for path in paths:
        with open(path, "r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    record = json.loads(line)
                    record["_source_path"] = path
                    records.append(record)
    return records


def _attempts(records):
    grouped = collections.defaultdict(list)
    for record in records:
        grouped[(record["_source_path"], record.get("run_id"))].append(record)

    attempts = []
    for run_records in grouped.values():
        run_records.sort(key=lambda r: float(r.get("monotonic_sec", 0.0)))
        current = None
        for record in run_records:
            if record.get("event") == "pick_sequence_start":
                if current:
                    attempts.append(current)
                current = [record]
            elif current is not None:
                current.append(record)
                if record.get("event") in TERMINAL_EVENTS:
                    attempts.append(current)
                    current = None
        if current:
            attempts.append(current)
    return attempts


def _rate(label, numerator, denominator):
    if denominator:
        print(f"{label}: {numerator}/{denominator} = {100.0 * numerator / denominator:.1f}%")
    else:
        print(f"{label}: 측정 데이터 없음")


def _stats(label, values, unit):
    if not values:
        print(f"{label}: 측정 데이터 없음")
        return
    print(
        f"{label}: 평균={statistics.mean(values):.1f}{unit}, "
        f"표준편차={statistics.pstdev(values):.1f}{unit}, "
        f"최소={min(values):.1f}{unit}, 최대={max(values):.1f}{unit}, n={len(values)}")


def main():
    parser = argparse.ArgumentParser(description="runtime JSONL 자동 KPI 요약")
    parser.add_argument("paths", nargs="*", help="생략하면 전체 cuRobo runtime 사용")
    parser.add_argument("--cell", help="experiment_context.cell 필터, 예: root/nw")
    args = parser.parse_args()
    paths = args.paths or sorted(glob.glob(DEFAULT_GLOB))
    if not paths:
        raise SystemExit(f"runtime JSONL not found: {DEFAULT_GLOB}")

    records = _load(paths)
    if args.cell:
        records = [
            record for record in records
            if record.get("experiment_context", {}).get("cell") == args.cell
        ]
    attempts = _attempts(records)

    plan_success = [r for r in records if r.get("event") == "curobo_plan_success"]
    plan_fail = [r for r in records if r.get("event") == "curobo_plan_fail"]
    plan_reject = [r for r in records if r.get("event") == "curobo_plan_rejected"]
    plan_total = len(plan_success) + len(plan_fail) + len(plan_reject)
    plan_latencies = [
        float(r["data"]["planning_latency_ms"]) for r in plan_success + plan_fail
        if r.get("data", {}).get("planning_latency_ms") is not None
    ]

    verify = [r for r in records if r.get("event") == "verify_grasp"]
    valid_verify = [
        r for r in verify
        if r.get("data", {}).get("result_code") in {
            "GRASP_CONTACT_DETECTED", "GRASP_EMPTY"}
    ]
    contact = sum(
        r.get("data", {}).get("result_code") == "GRASP_CONTACT_DETECTED"
        for r in valid_verify)
    empty = sum(
        r.get("data", {}).get("result_code") == "GRASP_EMPTY"
        for r in valid_verify)

    durations = []
    terminal_counts = collections.Counter()
    for attempt in attempts:
        start = attempt[0]
        terminal = next(
            (r for r in attempt if r.get("event") in TERMINAL_EVENTS),
            attempt[-1])
        duration = (
            float(terminal.get("monotonic_sec", 0.0))
            - float(start.get("monotonic_sec", 0.0)))
        if duration >= 0:
            durations.append(duration)
        terminal_counts[terminal.get("event", "unknown")] += 1

    contexts = {
        json.dumps(r.get("experiment_context", {}), ensure_ascii=False, sort_keys=True)
        for r in records if r.get("experiment_context")
    }
    print(f"runtime 파일: {len(paths)}개")
    print(f"수확 시도: {len(attempts)}건")
    if args.cell:
        print(f"cell filter: {args.cell}")
    if contexts:
        print(f"실험 조건 종류: {len(contexts)}개")
    print()
    _rate("1. cuRobo 후보 계획 통과율", len(plan_success), plan_total)
    _stats("2. cuRobo 계획 지연시간", plan_latencies, "ms")
    print(f"3. 계획 거부/실패: reject={len(plan_reject)}, fail={len(plan_fail)}")
    _rate("4. 자동 파지 판정 가능률", len(valid_verify), len(verify))
    _rate("5. 접촉 후보 감지율", contact, len(valid_verify))
    _rate("6. 빈 파지 감지율", empty, len(valid_verify))
    _stats("7. Pick 시퀀스 시간", durations, "s")
    print("8. 종료 이벤트: " + ", ".join(
        f"{key}={value}" for key, value in sorted(terminal_counts.items())))
    print("\n주의: 접촉 후보 감지는 실제 줄기 파지 성공이 아닙니다.")
    print("실제 줄기 파지·분리·후퇴 유지 성공률은 사람/영상 라벨과 결합해 계산합니다.")


if __name__ == "__main__":
    main()
