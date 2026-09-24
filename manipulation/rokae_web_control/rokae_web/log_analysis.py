"""Offline export only. This module never imports the robot backend or SDK.

python3 -m rokae_web.log_analysis logs/motion-2026-09-17.jsonl --operation ID --output logs/analysis/ID
Omit --operation to export the whole file; request IDs may also be selected.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
from pathlib import Path


def export(log_path, output, selected=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    counts, durations, phases = collections.Counter(), {}, []
    rows, invalid = 0, 0
    base_columns = ["timestamp", "monotonic_ns", "session_id", "request_id", "operation_id", "stage", "event"]
    columns = base_columns + [f"{module}_j{axis}_deg" for module, count in
                             (("left_arm", 7), ("right_arm", 7), ("trunk", 4), ("head", 2))
                             for axis in range(1, count + 1)]
    columns += [f"{module}_{axis}" for module in ("left_arm", "right_arm", "trunk")
                for axis in ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg")]
    columns += [f"{module}_{name}" for module in ("left_arm", "right_arm") for name in ("elbow_deg", "pose_frame")]
    columns += [f"{module}_operation_state" for module in ("left_arm", "right_arm", "trunk")]
    # Never replace an earlier analysis accidentally.
    with (output / "events.jsonl").open("x", encoding="utf-8") as events, \
         (output / "states.csv").open("x", encoding="utf-8", newline="") as states:
        writer = csv.DictWriter(states, fieldnames=columns)
        writer.writeheader()
        with Path(log_path).open(encoding="utf-8") as source:
            for line in source:
                try:
                    entry = json.loads(line)
                except json.JSONDecodeError:
                    invalid += 1
                    continue
                related = [entry.get("operation_id"), entry.get("request_id"),
                           *entry.get("related_operation_ids", []), *entry.get("related_request_ids", [])]
                if selected and selected not in related:
                    continue
                events.write(json.dumps(entry, ensure_ascii=False) + "\n")
                event = entry.get("event", "unknown")
                counts[event] += 1
                if event == "sdk_call":
                    item = durations.setdefault(entry.get("action", "unknown"), {"calls": 0, "total_ms": 0})
                    item["calls"] += 1
                    item["total_ms"] += entry.get("duration_ms", 0)
                if event == "phase_finished":
                    phases.append({k: entry.get(k) for k in (*base_columns, "phase_name", "duration_ms", "ok", "error")})
                state = entry.get("state")
                if not isinstance(state, dict) or "joints_deg" not in state:
                    continue
                row = {k: entry.get(k) for k in base_columns}
                for module, values in state["joints_deg"].items():
                    row.update({f"{module}_j{i}_deg": v for i, v in enumerate(values, 1)})
                for module, values in state.get("poses", {}).items():
                    row.update({f"{module}_{axis}": v for axis, v in zip(
                        ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"), values)})
                for module in ("left_arm", "right_arm"):
                    row[f"{module}_elbow_deg"] = state.get("arm_elbow_deg", {}).get(module)
                    row[f"{module}_pose_frame"] = state.get("pose_frames", {}).get(module)
                for module, value in state.get("operation_state", {}).items():
                    row[f"{module}_operation_state"] = value
                writer.writerow({k: v for k, v in row.items() if k in columns})
                rows += 1
    summary = {"source": str(log_path), "selected": selected, "state_rows": rows,
               "invalid_lines": invalid, "events": counts, "sdk_actions": durations, "phases": phases,
               "note": "Times are observations, not servo samples. No automatic robot replay."}
    with (output / "summary.json").open("x", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--operation")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.log, args.output, args.operation), ensure_ascii=False, indent=2))
