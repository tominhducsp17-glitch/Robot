#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo "Build Phase 1 first; install/setup.bash is missing." >&2
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
  echo "RUN ${pose_id}: peg=(${peg_x}, ${peg_y})"
  launch_output="$(
    ros2 launch experiment_runner phase1_task.launch.py \
      task:=task_b peg_x:="${peg_x}" peg_y:="${peg_y}" 2>&1 || true
  )"
  log_path="$(sed -n 's/.*PHASE1_LOG=//p' <<< "${launch_output}" | tail -n 1)"
  if [[ -z "${log_path}" || ! -f "${log_path}" ]]; then
    echo "FAIL ${pose_id}: runner produced no JSON log" >&2
    failures=$((failures + 1))
    continue
  fi
  logs+=("${log_path}")
  if python3 - "${log_path}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    raise SystemExit(0 if json.load(stream)["success"] else 1)
PY
  then
    echo "PASS ${pose_id}: ${log_path}"
  else
    echo "FAIL ${pose_id}: ${log_path}" >&2
    failures=$((failures + 1))
  fi
done

python3 - "${logs[@]}" <<'PY'
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

runs = [json.loads(Path(name).read_text(encoding="utf-8")) for name in sys.argv[1:]]
successes = sum(bool(run["success"]) for run in runs)
task_successes = sum(bool(run["task_success"]) for run in runs)
rate = successes / len(runs) if runs else 0.0
task_rate = task_successes / len(runs) if runs else 0.0
force_available = bool(runs) and all(run["force_monitor_available"] for run in runs)
force_violations = sum(run["force_violation"] is True for run in runs)
force_gate = force_available and force_violations == 0
success_rate_gate = task_rate >= 0.95 and len(runs) == 5
phase1_exit_gate = success_rate_gate and force_gate
report = {
    "schema_version": 1,
    "phase": 1,
    "created_at": datetime.now(timezone.utc).isoformat(),
    "task": "task_b",
    "requested_trials": 5,
    "recorded_trials": len(runs),
    "successful_trials": successes,
    "trajectory_and_physical_successful_trials": task_successes,
    "trajectory_and_physical_success_rate": task_rate,
    "overall_success_rate": rate,
    "success_rate_gate": success_rate_gate,
    "force_monitor_available": force_available,
    "force_violation_trials": force_violations,
    "peak_contact_force_n": max(
        (run["contact_force"]["peak_force_n"] or 0.0 for run in runs),
        default=0.0,
    ),
    "force_gate": force_gate,
    "phase1_exit_gate": phase1_exit_gate,
    "phase1_exit_gate_blocker": (
        None
        if phase1_exit_gate
        else "nominal success-rate or contact-force gate failed"
    ),
    "run_logs": sys.argv[1:],
}
output_dir = Path("experiments/summaries/phase1")
output_dir.mkdir(parents=True, exist_ok=True)
stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
output = output_dir / f"nominal_task_b_{stamp}.json"
output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print(f"PHASE1_BENCHMARK={output}")
print(f"SUCCESS_RATE={successes}/{len(runs)} ({rate:.1%})")
print(f"FORCE_GATE={'PASS' if force_gate else 'FAIL'}")
print(f"PHASE1_EXIT_GATE={'PASS' if phase1_exit_gate else 'FAIL'}")
PY

exit "${failures}"
