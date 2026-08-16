#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo "Build Phase 2 first; install/setup.bash is missing." >&2
  exit 2
fi
source install/setup.bash
set -u

pose_config="ros2_ws/src/experiment_runner/config/nominal_poses.yaml"
mapfile -t pose_rows < <(
  python3 - "${pose_config}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
for pose in config["poses"]:
    print(pose["id"], pose["x"], pose["y"])
PY
)

logs=()
failures=0
for row in "${pose_rows[@]}"; do
  read -r pose_id peg_x peg_y <<< "${row}"
  echo "RUN ${pose_id}: vision peg=(${peg_x}, ${peg_y})"
  launch_output="$(
    ros2 launch experiment_runner phase2_task.launch.py \
      peg_x:="${peg_x}" peg_y:="${peg_y}" 2>&1 || true
  )"
  log_path="$(sed -n 's/.*PHASE2_LOG=//p' <<< "${launch_output}" | tail -n 1)"
  if [[ -z "${log_path}" || ! -f "${log_path}" ]]; then
    echo "FAIL ${pose_id}: runner produced no Phase 2 JSON log" >&2
    failures=$((failures + 1))
    continue
  fi
  logs+=("${log_path}")
  if python3 - "${log_path}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    run = json.load(stream)
perception = run.get("perception_evaluation_only") or {}
objects = perception.get("objects", {})
valid = (
    run.get("success") is True
    and run.get("planning_input", {}).get("source") == "perception_snapshot"
    and perception.get("ready") is True
    and perception.get("frozen") is True
    and all(
        objects.get(name) is not None
        and objects[name].get("valid") is True
        and objects[name]["translation_error_m"] <= 0.015
        and objects[name]["measurement_age_sec"] <= 0.5
        for name in ("peg", "fixture")
    )
)
raise SystemExit(0 if valid else 1)
PY
  then
    echo "PASS ${pose_id}: ${log_path}"
  else
    echo "FAIL ${pose_id}: task or perception gate failed (${log_path})" >&2
    failures=$((failures + 1))
  fi
done

python3 - "${logs[@]}" <<'PY'
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

runs = [json.loads(Path(name).read_text(encoding="utf-8")) for name in sys.argv[1:]]
objects = [
    run["perception_evaluation_only"]["objects"][name]
    for run in runs
    for name in ("peg", "fixture")
]
task_successes = sum(bool(run["task_success"]) for run in runs)
force_available = bool(runs) and all(run["force_monitor_available"] for run in runs)
force_violations = sum(run["force_violation"] is True for run in runs)
pose_gate = bool(objects) and all(
    item is not None
    and item["valid"]
    and item["translation_error_m"] <= 0.015
    and item["measurement_age_sec"] <= 0.5
    for item in objects
)
vision_source_gate = bool(runs) and all(
    run["planning_input"]["source"] == "perception_snapshot" for run in runs
)
task_gate = len(runs) == 5 and task_successes / len(runs) >= 0.95
force_gate = force_available and force_violations == 0
exit_gate = pose_gate and vision_source_gate and task_gate and force_gate
report = {
    "schema_version": 1,
    "phase": 2,
    "created_at": datetime.now(timezone.utc).isoformat(),
    "task": "task_b",
    "requested_trials": 5,
    "recorded_trials": len(runs),
    "successful_trials": task_successes,
    "success_rate": task_successes / len(runs) if runs else 0.0,
    "translation_error_limit_m": 0.015,
    "latency_limit_sec": 0.5,
    "maximum_translation_error_m": max(
        (item["translation_error_m"] for item in objects if item), default=None
    ),
    "maximum_measurement_age_sec": max(
        (item["measurement_age_sec"] for item in objects if item), default=None
    ),
    "pose_and_latency_gate": pose_gate,
    "vision_source_gate": vision_source_gate,
    "task_gate": task_gate,
    "force_monitor_available": force_available,
    "force_violation_trials": force_violations,
    "peak_contact_force_n": max(
        (run["contact_force"]["peak_force_n"] or 0.0 for run in runs),
        default=0.0,
    ),
    "force_gate": force_gate,
    "phase2_exit_gate": exit_gate,
    "phase2_exit_gate_blocker": (
        None if exit_gate else "pose/latency, vision-source, task, or force gate failed"
    ),
    "run_logs": sys.argv[1:],
}
output_dir = Path("experiments/summaries/phase2")
output_dir.mkdir(parents=True, exist_ok=True)
stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
output = output_dir / f"vision_task_b_{stamp}.json"
output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print(f"PHASE2_BENCHMARK={output}")
print(f"TASK_SUCCESS={task_successes}/{len(runs)}")
print(f"POSE_LATENCY_GATE={'PASS' if pose_gate else 'FAIL'}")
print(f"FORCE_GATE={'PASS' if force_gate else 'FAIL'}")
print(f"PHASE2_EXIT_GATE={'PASS' if exit_gate else 'FAIL'}")
raise SystemExit(0 if exit_gate else 1)
PY
benchmark_status=$?

if (( failures > 0 )); then
  exit 1
fi
exit "${benchmark_status}"
