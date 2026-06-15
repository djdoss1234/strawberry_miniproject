#!/usr/bin/env python3
"""Store one experiment context reused by subsequent runtime JSONL logs."""

import argparse
import json
import os
from datetime import datetime


DEFAULT_OUTPUT = os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/logs/experiment_context.json")


def main():
    parser = argparse.ArgumentParser(
        description="다음 로봇 실행들에 공통으로 붙일 실험 조건을 저장합니다.")
    parser.add_argument("--cell", required=True, help="예: root/nw")
    parser.add_argument("--scene-id", required=True, help="예: nw_leaf_occlusion_v1")
    parser.add_argument(
        "--occlusion", default="unknown",
        choices=("none", "leaf", "stem", "leaf_and_stem", "unknown"))
    parser.add_argument(
        "--stem-shape", default="unknown",
        choices=("straight", "bent", "mixed", "unknown"))
    parser.add_argument("--notes", default="")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    record = {
        "schema_version": "strawberry_experiment_context.v1",
        "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "cell": args.cell,
        "scene_id": args.scene_id,
        "occlusion": args.occlusion,
        "stem_shape": args.stem_shape,
        "notes": args.notes,
    }
    path = os.path.abspath(os.path.expanduser(args.output))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(path)
    print(json.dumps(record, ensure_ascii=False))


if __name__ == "__main__":
    main()
