#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
export ROS_DOMAIN_ID="${PHASE0_GZ_CONTROL_ROS_DOMAIN_ID:-44}"
export GZ_PARTITION="${PHASE0_GZ_CONTROL_PARTITION:-closed_loop_phase0_control_${BASHPID}}"

SMOKE_LOG="$(mktemp)"
LAUNCH_PID=""
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  rm -f -- "${SMOKE_LOG}"
}
trap cleanup EXIT

setsid ros2 launch gz_ros2_control_demos cart_example_position.launch.py gz_args:=-s \
  >"${SMOKE_LOG}" 2>&1 &
LAUNCH_PID=$!

for _ in {1..60}; do
  if ros2 topic list 2>/dev/null | grep -qx '/joint_states' && \
     timeout 3 ros2 control list_controllers 2>/dev/null | grep -q 'joint_state_broadcaster.*active' && \
     timeout 3 ros2 control list_controllers 2>/dev/null | grep -q 'joint_trajectory_controller.*active'; then
    timeout 5 ros2 topic echo /joint_states --once --field name
    timeout 5 ros2 control list_controllers
    timeout 30 ros2 run gz_ros2_control_demos example_position
    echo "PASS: /joint_states published, controller manager responded, and a position command ran."
    exit 0
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    break
  fi
  sleep 1
done

echo "FAIL: gz_ros2_control demo was not ready within 60 seconds." >&2
sed -n '1,200p' "${SMOKE_LOG}" >&2
exit 1
