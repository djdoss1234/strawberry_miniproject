#!/usr/bin/env python3
"""Generate a Notion-friendly planning latency and pick timeline summary."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


MILESTONES = {
    "pick_sequence_start": "Pick start",
    "grasp_pose_reached": "Grasp pose",
    "verify_grasp": "Verify grasp",
    "detach_pull_down": "Detach pull",
    "verify_detach": "Verify detach",
    "pick_sequence_complete": "Scan pose return",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

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

    fig, (ax_plan, ax_time) = plt.subplots(2, 1, figsize=(12, 7))
    colors = ["#2e8b57" if status == "success" else "#d9534f" for status, _ in plans]
    values = [value for _, value in plans]
    labels = [f"{i + 1}: {status}" for i, (status, _) in enumerate(plans)]
    ax_plan.bar(labels, values, color=colors)
    ax_plan.set_ylabel("Planning latency (ms)")
    ax_plan.set_title(
        f"cuRobo planning attempts: {len(plans)} timed, {rejected} spline-jump rejects"
    )
    ax_plan.grid(axis="y", alpha=0.25)
    for i, value in enumerate(values):
        ax_plan.text(i, value + max(values) * 0.02, f"{value:.0f}", ha="center")

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
    ax_time.set_xlabel("Elapsed time from pick start (s)")
    ax_time.set_yticks([])
    ax_time.set_ylim(-0.4, 0.4)
    ax_time.set_title(f"Pick timeline: total {max(times):.1f}s")
    ax_time.grid(axis="x", alpha=0.25)

    fig.suptitle(f"SW Single-Fruit Runtime Summary | {args.jsonl.stem}", fontsize=13)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")
    print(args.output)


if __name__ == "__main__":
    main()
