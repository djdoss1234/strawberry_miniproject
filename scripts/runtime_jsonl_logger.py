#!/usr/bin/env python3
"""Small append-only JSONL logger for runtime replay and simulation import."""

import json
import os
import subprocess
import sys
import threading
import time
import uuid
from datetime import datetime, timezone

import numpy as np


SCHEMA_VERSION = "strawberry_runtime_event.v1"


def _json_safe(value):
    if hasattr(value, "detach") and hasattr(value, "cpu"):
        return value.detach().cpu().tolist()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "tolist"):
        return value.tolist()
    try:
        return float(value)
    except (TypeError, ValueError):
        return repr(value)


def _default_log_root():
    configured = os.environ.get("STRAWBERRY_RUNTIME_LOG_ROOT")
    if configured:
        return os.path.expanduser(configured)
    source_root = os.path.expanduser(
        "~/doosan_ws/src/e0509_gripper_description/logs/runtime")
    return source_root


def _git_commit():
    try:
        repo = os.path.expanduser("~/doosan_ws/src/e0509_gripper_description")
        return subprocess.check_output(
            ["git", "-C", repo, "rev-parse", "--short", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return "unknown"


class RuntimeJsonlLogger:
    def __init__(self, node_name, log_root=None):
        self.node_name = str(node_name)
        self.run_id = (
            f"{datetime.now().strftime('%Y%m%dT%H%M%S')}-"
            f"{uuid.uuid4().hex[:8]}"
        )
        day_dir = os.path.join(
            log_root or _default_log_root(), datetime.now().strftime("%Y-%m-%d"))
        os.makedirs(day_dir, exist_ok=True)
        self.path = os.path.join(day_dir, f"{self.node_name}_{self.run_id}.jsonl")
        self.git_commit = _git_commit()
        self._lock = threading.Lock()
        self._write_error_reported = False

    def log(self, event, **data):
        record = {
            "schema_version": SCHEMA_VERSION,
            "timestamp": datetime.now(timezone.utc).astimezone().isoformat(
                timespec="milliseconds"),
            "monotonic_sec": time.monotonic(),
            "run_id": self.run_id,
            "node": self.node_name,
            "git_commit": self.git_commit,
            "event": str(event),
            "data": _json_safe(data),
        }
        try:
            with self._lock:
                with open(self.path, "a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception as exc:
            # Logging must never interrupt a physical robot motion sequence.
            if not self._write_error_reported:
                self._write_error_reported = True
                print(
                    f"[RuntimeJsonlLogger] write failed ({self.path}): {exc}",
                    file=sys.stderr,
                )
        return record
