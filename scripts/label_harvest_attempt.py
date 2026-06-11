#!/usr/bin/env python3
"""Attach compact human-observation labels to the latest harvest attempt."""

import argparse
import glob
import json
import os
import subprocess
from datetime import datetime


REPO_ROOT = os.path.expanduser("~/doosan_ws/src/e0509_gripper_description")
RUNTIME_GLOB = os.path.join(REPO_ROOT, "logs/runtime/*/curobo_planner_node_*.jsonl")
LABEL_ROOT = os.path.join(REPO_ROOT, "logs/human_labels")


def _git_commit():
    try:
        return subprocess.check_output(
            ["git", "-C", REPO_ROOT, "rev-parse", "--short", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return "unknown"


def _latest_runtime_path():
    paths = glob.glob(RUNTIME_GLOB)
    if not paths:
        raise FileNotFoundError(f"runtime JSONL not found: {RUNTIME_GLOB}")
    return max(paths, key=os.path.getmtime)


def _read_jsonl(path):
    records = []
    with open(path, "r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON ({exc})") from exc
    return records


def _latest_attempt(records):
    starts = [
        index for index, record in enumerate(records)
        if record.get("event") == "pick_sequence_start"
    ]
    if not starts:
        raise ValueError("pick_sequence_start event not found")
    start_index = starts[-1]
    attempt = records[start_index:]
    terminal_events = {
        "pick_sequence_complete",
        "pick_sequence_stopped",
        "pick_sequence_hold_latched",
    }
    terminal = next(
        (record for record in attempt if record.get("event") in terminal_events),
        attempt[-1],
    )
    start = attempt[0]
    duration_sec = max(
        0.0,
        float(terminal.get("monotonic_sec", 0.0))
        - float(start.get("monotonic_sec", 0.0)),
    )
    verify_grasp = next(
        (record for record in attempt if record.get("event") == "verify_grasp"),
        None,
    )
    return {
        "records": attempt,
        "start": start,
        "terminal": terminal,
        "duration_sec": duration_sec,
        "verify_grasp": verify_grasp,
        "attempt_index": len(starts),
    }


def _choose(prompt, choices):
    print(f"\n{prompt}")
    for key, (_, description) in choices.items():
        print(f"  {key}. {description}")
    while True:
        selected = input("> ").strip()
        if selected in choices:
            return choices[selected][0]
        print("표시된 번호 중 하나를 입력하세요.")


def _yes_no_unknown(prompt):
    return _choose(prompt, {
        "1": ("yes", "성공 / 예"),
        "2": ("no", "실패 / 아니오"),
        "3": ("unknown", "확인 불가"),
    })


def _optional_positive_float(prompt):
    while True:
        raw = input(f"\n{prompt} (모르면 Enter): ").strip()
        if not raw:
            return None
        try:
            value = float(raw)
            if value >= 0.0:
                return value
        except ValueError:
            pass
        print("0 이상의 초 단위 숫자 또는 Enter를 입력하세요.")


def _derive_result(labels):
    required = (
        labels["stem_grasp"],
        labels["detach"],
        labels["retention"],
    )
    if all(value == "yes" for value in required):
        pick_success = "success"
    elif any(value == "no" for value in required):
        pick_success = "fail"
    else:
        pick_success = "unknown"

    place = labels["place"]
    if pick_success == "success" and place == "success":
        end_to_end = "success"
    elif pick_success == "fail" or place == "fail":
        end_to_end = "fail"
    else:
        end_to_end = "not_evaluable"
    return pick_success, end_to_end


def _write_label(record):
    day_dir = os.path.join(LABEL_ROOT, datetime.now().strftime("%Y-%m-%d"))
    os.makedirs(day_dir, exist_ok=True)
    path = os.path.join(day_dir, "harvest_attempt_labels.jsonl")
    with open(path, "a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    return path


def main():
    parser = argparse.ArgumentParser(
        description="최신 수확 시도에 사람 관찰 라벨을 연결합니다.")
    parser.add_argument(
        "--runtime", help="대상 curobo runtime JSONL. 생략하면 최신 파일 사용")
    args = parser.parse_args()

    runtime_path = os.path.abspath(os.path.expanduser(
        args.runtime or _latest_runtime_path()))
    records = _read_jsonl(runtime_path)
    attempt = _latest_attempt(records)
    start_data = attempt["start"].get("data", {})
    terminal = attempt["terminal"]
    verify_data = (
        attempt["verify_grasp"].get("data", {})
        if attempt["verify_grasp"] is not None else {}
    )

    print("\n=== 최신 수확 시도 사람 판정 ===")
    print(f"runtime: {runtime_path}")
    print(f"run_id:  {attempt['start'].get('run_id', 'unknown')}")
    print(f"attempt: {attempt['attempt_index']}")
    print(f"target:  {start_data.get('input_target_m', 'unknown')}")
    print(f"자동 종료 이벤트: {terminal.get('event')} "
          f"({terminal.get('data', {}).get('result_code', 'no result code')})")
    print(f"자동 파지 판정: {verify_data.get('result_code', 'not recorded')}")
    print(f"현재까지 걸린 시간: {attempt['duration_sec']:.1f}s")

    labels = {
        "stem_grasp": _yes_no_unknown("그리퍼가 실제 딸기 줄기를 잡았습니까?"),
        "detach": _yes_no_unknown("딸기가 줄기/고정부에서 분리됐습니까?"),
        "retention": _yes_no_unknown("후퇴가 끝날 때까지 딸기를 유지했습니까?"),
        "non_target_contact": _choose("진입/후퇴 중 원하지 않은 접촉이 있었습니까?", {
            "1": ("none", "없음"),
            "2": ("leaf_or_stem", "잎 또는 목표가 아닌 줄기"),
            "3": ("other_fruit", "다른 딸기"),
            "4": ("structure", "보드, 테이블, tray 등 구조물"),
            "5": ("multiple", "둘 이상"),
            "6": ("unknown", "확인 불가"),
        }),
        "human_intervention": _choose("이번 시도 중 사람이 개입했습니까?", {
            "1": ("no", "없음"),
            "2": ("yes", "정지, 복구, 위치 조정 등 개입함"),
        }),
        "place": _choose("Place 결과는 무엇입니까?", {
            "1": ("not_attempted", "Place를 시도하지 않음"),
            "2": ("success", "목표 slot에 배치 성공"),
            "3": ("fail", "배치 실패 또는 낙하"),
            "4": ("unknown", "확인 불가"),
        }),
    }
    total_task_time_sec = _optional_positive_float(
        "전체 작업 시간[scan 시작→pick→place→다음 작업 준비 완료]을 측정했다면 초 입력")
    note = input("\n짧은 메모(없으면 Enter): ").strip()
    pick_success, end_to_end = _derive_result(labels)

    record = {
        "schema_version": "strawberry_harvest_human_label.v1",
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit": _git_commit(),
        "source_runtime_jsonl": runtime_path,
        "source_run_id": attempt["start"].get("run_id"),
        "source_attempt_index": attempt["attempt_index"],
        "source_target_m": start_data.get("input_target_m"),
        "automatic": {
            "terminal_event": terminal.get("event"),
            "terminal_result_code": terminal.get("data", {}).get("result_code"),
            "grasp_result_code": verify_data.get("result_code"),
            "duration_sec": attempt["duration_sec"],
        },
        "human_label": labels,
        "human_measurement": {
            "total_task_time_sec": total_task_time_sec,
        },
        "derived": {
            "pick_success": pick_success,
            "end_to_end_success": end_to_end,
        },
        "note": note,
    }
    output_path = _write_label(record)
    print("\n저장 완료")
    print(f"  파일: {output_path}")
    print(f"  최종 Pick 판정: {pick_success}")
    print(f"  Pick+Place 최종 판정: {end_to_end}")


if __name__ == "__main__":
    main()
