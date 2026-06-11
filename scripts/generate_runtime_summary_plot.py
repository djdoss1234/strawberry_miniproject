#!/usr/bin/env python3
"""Generate a Notion-friendly planning latency and pick timeline summary."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager


MILESTONES = {
    "pick_sequence_start": "Pick 시작",
    "grasp_pose_reached": "Grasp pose 도달",
    "verify_grasp": "Grasp 판정",
    "detach_pull_down": "Detach pull",
    "verify_detach": "Detach 판정",
    "pick_sequence_complete": "Scan pose 복귀",
}


def configure_korean_font():
    candidates = [
        "Noto Sans CJK KR",
        "Noto Sans CJK JP",
        "Noto Serif CJK KR",
        "Noto Serif CJK JP",
        "NanumGothic",
        "DejaVu Sans",
    ]
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in candidates:
        if candidate in installed:
            plt.rcParams["font.family"] = candidate
            break
    plt.rcParams["axes.unicode_minus"] = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    configure_korean_font()

    records = [json.loads(line) for line in args.jsonl.read_text().splitlines()]
    start = next(r["monotonic_sec"] for r in records if r["event"] == "pick_sequence_start")

    plans = []
    rejected = 0
    for record in records:
        event = record["event"]
        data = record["data"]
        if event in ("curobo_plan_success", "curobo_plan_fail"):
            plans.append(
                (
                    "success" if event.endswith("success") else "fail",
                    float(data["planning_latency_ms"]),
                )
            )
        elif event == "curobo_plan_rejected":
            rejected += 1

    milestones = [
        (MILESTONES[r["event"]], r["monotonic_sec"] - start)
        for r in records
        if r["event"] in MILESTONES
    ]

    motion_results = [
        r["data"]
        for r in records
        if r["event"] == "motion_result"
    ]
    spline_results = [
        r for r in motion_results if r.get("controller") == "doosan_move_spline_joint"
    ]
    line_results = [
        r for r in motion_results if r.get("controller") == "doosan_move_line"
    ]
    detach_results = [
        r["data"] for r in records if r["event"] == "detach_pull_down"
    ]
    execution_labels = [
        "MoveSplineJoint\nPre-approach + 복귀",
        "MoveLine TOOL\n진입 + 추가진입 + 후퇴",
        "MoveLine BASE\nDetach pull",
    ]
    execution_success = [
        sum(bool(r.get("success")) for r in spline_results),
        sum(bool(r.get("success")) for r in line_results),
        sum(bool(r.get("success")) for r in detach_results),
    ]
    execution_total = [len(spline_results), len(line_results), len(detach_results)]

    fig, (ax_plan, ax_exec, ax_time) = plt.subplots(
        3, 1, figsize=(13, 10), gridspec_kw={"height_ratios": [1.35, 0.8, 1.0]}
    )
    plan_roles = [
        "Pre-approach\n실행 경로",
        "Endpoint 후보\nIK 실패",
        "Endpoint 후보\nIK 실패",
        "Endpoint 검증\n실행 안 함",
        "Scan pose 복귀\n실행 경로",
    ]
    colors = ["#2e8b57", "#d9534f", "#d9534f", "#f0ad4e", "#2e8b57"]
    values = [value / 1000.0 for _, value in plans]
    labels = [f"{i + 1}. {role}" for i, role in enumerate(plan_roles)]
    ax_plan.bar(labels, values, color=colors)
    ax_plan.set_ylabel("계획 계산 소요시간 (초)")
    ax_plan.set_title("1. cuRobo Planning: 후보 검사 결과와 실제 실행 여부")
    ax_plan.grid(axis="y", alpha=0.25)
    for i, value in enumerate(values):
        ax_plan.text(i, value + max(values) * 0.02, f"{value:.2f}초", ha="center")
    ax_plan.text(
        0.01,
        0.94,
        "막대 높이 = cuRobo 계산 시간 (로봇 이동 속도 아님)\n초록: 실제 실행 / 주황: endpoint 검증용 / 빨강: IK 실패",
        transform=ax_plan.transAxes,
        ha="left",
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#fff8dc", "edgecolor": "#c9a227"},
    )
    ax_plan.text(
        0.99,
        0.94,
        f"Spline-jump reject {rejected}건\n위험 후보를 실행 전에 별도 차단",
        transform=ax_plan.transAxes,
        ha="right",
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#e8f4ff", "edgecolor": "#4c78a8"},
    )

    exec_colors = [
        "#2e8b57" if success == total and total > 0 else "#d9534f"
        for success, total in zip(execution_success, execution_total)
    ]
    ax_exec.bar(execution_labels, execution_total, color="#d9e2f3", label="실행 명령 수")
    ax_exec.bar(execution_labels, execution_success, color=exec_colors, label="성공 수")
    ax_exec.set_ylim(0, max(execution_total) + 1)
    ax_exec.set_ylabel("건수")
    ax_exec.set_title("2. Robot Execution: 선택된 모션 명령의 실제 실행 결과")
    ax_exec.legend(loc="upper right")
    ax_exec.grid(axis="y", alpha=0.25)
    for i, (success, total) in enumerate(zip(execution_success, execution_total)):
        ax_exec.text(i, total + 0.12, f"{success}/{total} 성공", ha="center")

    times = [time for _, time in milestones]
    names = [name for name, _ in milestones]
    ax_time.hlines(0, min(times), max(times), color="#777777", linewidth=2)
    ax_time.scatter(times, [0] * len(times), s=90, color="#1f77b4", zorder=3)
    for index, (name, time) in enumerate(milestones):
        y = 0.18 if index % 2 == 0 else -0.22
        ax_time.annotate(
            f"{name}\n{time:.1f}s",
            (time, 0),
            xytext=(time, y),
            ha="center",
            va="center",
            arrowprops={"arrowstyle": "-", "color": "#888888"},
        )
    ax_time.set_xlabel("수확 시작 후 경과 시간 (초)")
    ax_time.set_yticks([])
    ax_time.set_ylim(-0.4, 0.4)
    ax_time.set_title(
        f"3. Task Timeline: Pick 요청부터 Scan pose 복귀까지 총 {max(times):.1f}초 (Place 미포함)"
    )
    ax_time.grid(axis="x", alpha=0.25)

    fig.suptitle(
        "SW 단일딸기 Pick 실행 요약: Planning → Robot Execution → Task Timeline",
        fontsize=14,
    )
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
