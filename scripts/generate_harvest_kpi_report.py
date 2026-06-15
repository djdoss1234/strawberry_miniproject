#!/usr/bin/env python3
"""Generate an automatic harvest KPI dashboard, JSON summary and Markdown report."""

import argparse
import collections
import glob
import json
import os
import statistics
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager


REPO_ROOT = Path(os.path.expanduser("~/doosan_ws/src/e0509_gripper_description"))
RUNTIME_GLOB = str(REPO_ROOT / "logs/runtime/*/curobo_planner_node_*.jsonl")
LABEL_GLOB = str(REPO_ROOT / "logs/human_labels/*/harvest_attempt_labels.jsonl")
TERMINAL_EVENTS = {
    "pick_sequence_complete",
    "pick_sequence_stopped",
    "pick_sequence_hold_latched",
}


def _configure_font():
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in (
        "Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic", "DejaVu Sans"
    ):
        if candidate in installed:
            plt.rcParams["font.family"] = candidate
            break
    plt.rcParams["axes.unicode_minus"] = False


def _read_jsonl(paths):
    records = []
    for path in paths:
        with open(path, "r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    record = json.loads(line)
                    record["_source_path"] = os.path.abspath(path)
                    records.append(record)
    return records


def _read_label_csv(path):
    import csv
    labels = []
    if not path.exists():
        return labels
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            stem = row.get("stem_grasp", "").strip()
            detach = row.get("detach", "").strip()
            retention = row.get("retention", "").strip()
            place = row.get("place", "").strip()
            if not any((stem, detach, retention, place)):
                continue
            if all(value == "yes" for value in (stem, detach, retention)):
                pick_success = "success"
            elif any(value == "no" for value in (stem, detach, retention)):
                pick_success = "fail"
            else:
                pick_success = "unknown"
            labels.append({
                "source_runtime_jsonl": row.get("source_runtime_jsonl", ""),
                "automatic": {
                    "grasp_result_code": row.get("automatic_grasp_result", "")},
                "human_label": {
                    "stem_grasp": stem,
                    "detach": detach,
                    "retention": retention,
                    "non_target_contact": row.get("non_target_contact", "").strip(),
                    "human_intervention": row.get("human_intervention", "").strip(),
                    "place": place,
                },
                "derived": {"pick_success": pick_success},
            })
    return labels


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


def _mean(values):
    return statistics.mean(values) if values else None


def _rate(success, total):
    return 100.0 * success / total if total else None


def _human_metrics(labels):
    def count(path, success_value, valid_values):
        values = []
        for record in labels:
            value = record
            for key in path:
                value = value.get(key, {}) if isinstance(value, dict) else None
            if value in valid_values:
                values.append(value)
        return values.count(success_value), len(values)

    stem = count(("human_label", "stem_grasp"), "yes", {"yes", "no"})
    pick = count(("derived", "pick_success"), "success", {"success", "fail"})
    place = count(("human_label", "place"), "success", {"success", "fail"})
    no_intervention = count(
        ("human_label", "human_intervention"), "no", {"yes", "no"})
    verifier_pairs = []
    for record in labels:
        actual = record.get("human_label", {}).get("stem_grasp")
        predicted = record.get("automatic", {}).get("grasp_result_code")
        if (
            actual in {"yes", "no"}
            and predicted in {"GRASP_CONTACT_DETECTED", "GRASP_EMPTY"}
        ):
            verifier_pairs.append((actual == "yes", predicted == "GRASP_CONTACT_DETECTED"))
    tp = sum(actual and predicted for actual, predicted in verifier_pairs)
    fp = sum(not actual and predicted for actual, predicted in verifier_pairs)
    fn = sum(actual and not predicted for actual, predicted in verifier_pairs)
    precision = _rate(tp, tp + fp)
    recall = _rate(tp, tp + fn)
    return {
        "stem_grasp": {"success": stem[0], "total": stem[1], "rate_pct": _rate(*stem)},
        "pick_success": {"success": pick[0], "total": pick[1], "rate_pct": _rate(*pick)},
        "place_success": {"success": place[0], "total": place[1], "rate_pct": _rate(*place)},
        "human_intervention": {
            "success": no_intervention[1] - no_intervention[0],
            "total": no_intervention[1],
            "rate_pct": _rate(no_intervention[1] - no_intervention[0], no_intervention[1]),
        },
        "grasp_verifier_precision": {
            "success": tp, "total": tp + fp, "rate_pct": precision},
        "grasp_verifier_recall": {
            "success": tp, "total": tp + fn, "rate_pct": recall},
    }


def _summarize(records, labels):
    attempts = _attempts(records)
    plan_success = [r for r in records if r.get("event") == "curobo_plan_success"]
    plan_fail = [r for r in records if r.get("event") == "curobo_plan_fail"]
    plan_reject = [r for r in records if r.get("event") == "curobo_plan_rejected"]
    latencies = [
        float(r["data"]["planning_latency_ms"]) for r in plan_success + plan_fail
        if r.get("data", {}).get("planning_latency_ms") is not None
    ]
    verify = [r for r in records if r.get("event") == "verify_grasp"]
    verify_counts = collections.Counter(
        r.get("data", {}).get("result_code", "NOT_RECORDED") for r in verify)
    valid_verify = (
        verify_counts["GRASP_CONTACT_DETECTED"] + verify_counts["GRASP_EMPTY"])
    durations = []
    terminal_counts = collections.Counter()
    for attempt in attempts:
        terminal = next(
            (r for r in attempt if r.get("event") in TERMINAL_EVENTS),
            attempt[-1])
        duration = float(terminal.get("monotonic_sec", 0.0)) - float(
            attempt[0].get("monotonic_sec", 0.0))
        if duration >= 0:
            durations.append(duration)
        terminal_counts[terminal.get("event", "unknown")] += 1
    plan_total = len(plan_success) + len(plan_fail) + len(plan_reject)
    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "automatic": {
            "attempts": len(attempts),
            "plan_success": len(plan_success),
            "plan_fail": len(plan_fail),
            "plan_reject": len(plan_reject),
            "plan_acceptance_rate_pct": _rate(len(plan_success), plan_total),
            "planning_latency_ms": {
                "mean": _mean(latencies),
                "min": min(latencies) if latencies else None,
                "max": max(latencies) if latencies else None,
                "values": latencies,
            },
            "grasp_verification": {
                "total": len(verify),
                "valid": valid_verify,
                "coverage_pct": _rate(valid_verify, len(verify)),
                "outcomes": dict(verify_counts),
            },
            "pick_cycle_sec": {
                "mean": _mean(durations),
                "min": min(durations) if durations else None,
                "max": max(durations) if durations else None,
                "values": durations,
            },
            "terminal_events": dict(terminal_counts),
        },
        "ground_truth": _human_metrics(labels),
    }


def _fmt(value, suffix=""):
    return "측정 없음" if value is None else f"{value:.1f}{suffix}"


def _plot(summary, output):
    auto = summary["automatic"]
    truth = summary["ground_truth"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    ax = axes[0, 0]
    plan_values = [auto["plan_success"], auto["plan_fail"], auto["plan_reject"]]
    ax.bar(["통과", "실패", "안전 거부"], plan_values,
           color=["#2e8b57", "#d9534f", "#f0ad4e"])
    ax.set_title("1. cuRobo 후보 계획 결과")
    ax.set_ylabel("후보 수")
    for i, value in enumerate(plan_values):
        ax.text(i, value + 0.05, str(value), ha="center")

    ax = axes[0, 1]
    latencies = auto["planning_latency_ms"]["values"]
    if latencies:
        ax.hist([v / 1000.0 for v in latencies], bins=min(10, len(latencies)),
                color="#4c78a8", edgecolor="white")
        ax.axvline(statistics.mean(latencies) / 1000.0, color="#d9534f",
                   linestyle="--", label="평균")
        ax.legend()
    else:
        ax.text(0.5, 0.5, "측정 데이터 없음", ha="center", va="center")
    ax.set_title("2. Planning latency 분포")
    ax.set_xlabel("계획 계산 시간 (초, 로봇 이동속도 아님)")
    ax.set_ylabel("건수")

    ax = axes[1, 0]
    outcomes = auto["grasp_verification"]["outcomes"]
    grasp_labels = ["접촉 후보", "빈 파지", "판정 불가"]
    grasp_values = [
        outcomes.get("GRASP_CONTACT_DETECTED", 0),
        outcomes.get("GRASP_EMPTY", 0),
        outcomes.get("GRASP_UNVERIFIED", 0),
    ]
    ax.bar(grasp_labels, grasp_values, color=["#2e8b57", "#d9534f", "#999999"])
    ax.set_title(
        "3. 그리퍼 자동 판정\n"
        f"판정 가능률 {_fmt(auto['grasp_verification']['coverage_pct'], '%')}")
    ax.set_ylabel("시도 수")

    ax = axes[1, 1]
    metric_labels = [
        "실제 줄기 파지", "최종 Pick", "Place", "사람 개입",
        "판정 Precision", "판정 Recall",
    ]
    metric_keys = [
        "stem_grasp", "pick_success", "place_success", "human_intervention",
        "grasp_verifier_precision", "grasp_verifier_recall",
    ]
    metric_values = [
        truth[key]["rate_pct"] if truth[key]["rate_pct"] is not None else 0.0
        for key in metric_keys
    ]
    colors = [
        "#2e8b57" if truth[key]["rate_pct"] is not None else "#d9d9d9"
        for key in metric_keys
    ]
    ax.bar(metric_labels, metric_values, color=colors)
    ax.set_ylim(0, 105)
    ax.set_ylabel("비율 (%)")
    ax.set_title("4. 사람/영상 정답 기반 태스크 KPI")
    for i, key in enumerate(metric_keys):
        value = truth[key]["rate_pct"]
        text = "미측정" if value is None else f"{value:.1f}%"
        ax.text(i, metric_values[i] + 2, text, ha="center")

    fig.suptitle(
        f"Strawberry Harvest KPI Dashboard | attempts={auto['attempts']} | "
        f"평균 Pick time={_fmt(auto['pick_cycle_sec']['mean'], 's')}",
        fontsize=14,
    )
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _markdown(summary, cell):
    auto = summary["automatic"]
    truth = summary["ground_truth"]
    lines = [
        "# 수확 KPI 자동 보고서",
        "",
        f"- 생성 시각: `{summary['generated_at']}`",
        f"- cell filter: `{cell or 'all'}`",
        f"- 수확 시도: `{auto['attempts']}`건",
        "",
        "## 자동 집계",
        "",
        "| 지표 | 값 | 의미 |",
        "| --- | ---: | --- |",
        f"| 후보 계획 통과율 | {_fmt(auto['plan_acceptance_rate_pct'], '%')} | 실행 가능한 후보 / 전체 계획 후보 |",
        f"| 평균 planning latency | {_fmt(auto['planning_latency_ms']['mean'], 'ms')} | cuRobo 계산 시간 |",
        f"| 자동 파지 판정 가능률 | {_fmt(auto['grasp_verification']['coverage_pct'], '%')} | 유효 position/current 판독 비율 |",
        f"| 평균 Pick 시퀀스 시간 | {_fmt(auto['pick_cycle_sec']['mean'], 's')} | target 수신부터 종료 이벤트까지 |",
        "",
        "## 사람/영상 정답이 필요한 KPI",
        "",
        "| KPI | 현재 값 | 표본 수 |",
        "| --- | ---: | ---: |",
    ]
    for label, key in (
        ("실제 줄기 파지 성공률", "stem_grasp"),
        ("최종 Pick 성공률", "pick_success"),
        ("Place 성공률", "place_success"),
        ("사람 개입률", "human_intervention"),
        ("자동 파지 판정 Precision", "grasp_verifier_precision"),
        ("자동 파지 판정 Recall", "grasp_verifier_recall"),
    ):
        item = truth[key]
        lines.append(f"| {label} | {_fmt(item['rate_pct'], '%')} | {item['total']} |")
    lines += [
        "",
        "> `GRASP_CONTACT_DETECTED`는 접촉 후보이며 실제 줄기 파지 성공이 아니다.",
        "> 최종 수확 성공률은 줄기 파지 + 분리 + 후퇴 유지의 정답 라벨로 계산한다.",
        "",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="수확 KPI PNG/JSON/Markdown 보고서 생성")
    parser.add_argument("--cell", help="experiment_context.cell 필터, 예: root/nw")
    parser.add_argument("--runtime", nargs="*", help="runtime JSONL 경로")
    parser.add_argument("--labels", nargs="*", help="human-label JSONL 경로")
    parser.add_argument(
        "--output-dir", default=str(REPO_ROOT / "reports/harvest_kpi"))
    args = parser.parse_args()
    _configure_font()

    runtime_paths = args.runtime or sorted(glob.glob(RUNTIME_GLOB))
    label_paths = args.labels or sorted(glob.glob(LABEL_GLOB))
    records = _read_jsonl(runtime_paths)
    if args.cell:
        records = [
            r for r in records
            if r.get("experiment_context", {}).get("cell") == args.cell
        ]
    allowed_sources = {r["_source_path"] for r in records}
    labels = _read_jsonl(label_paths)
    sheet_suffix = (args.cell or "all").replace("/", "_")
    labels.extend(_read_label_csv(
        REPO_ROOT / f"reports/harvest_kpi/manual_labels_{sheet_suffix}.csv"))
    if args.cell or args.runtime:
        labels = [
            r for r in labels
            if os.path.abspath(r.get("source_runtime_jsonl", "")) in allowed_sources
        ]

    summary = _summarize(records, labels)
    output_dir = Path(os.path.expanduser(args.output_dir))
    output_dir.mkdir(parents=True, exist_ok=True)
    suffix = (args.cell or "all").replace("/", "_")
    png_path = output_dir / f"kpi_dashboard_{suffix}.png"
    json_path = output_dir / f"kpi_summary_{suffix}.json"
    md_path = output_dir / f"kpi_report_{suffix}.md"
    _plot(summary, png_path)
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path.write_text(_markdown(summary, args.cell), encoding="utf-8")
    print(png_path)
    print(json_path)
    print(md_path)


if __name__ == "__main__":
    main()
