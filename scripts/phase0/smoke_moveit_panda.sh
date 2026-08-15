#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
export ROS_DOMAIN_ID="${PHASE0_MOVEIT_ROS_DOMAIN_ID:-43}"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "FAIL: no DISPLAY or WAYLAND_DISPLAY; RViz GUI cannot be tested." >&2
  exit 1
fi

SMOKE_LOG="$(mktemp)"
JOINT_STATE_LOG="$(mktemp)"
CONTROLLERS_LOG="$(mktemp)"
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
  rm -f -- "${SMOKE_LOG}" "${JOINT_STATE_LOG}" "${CONTROLLERS_LOG}"
}
trap cleanup EXIT

setsid ros2 launch moveit_resources_panda_moveit_config demo.launch.py \
  >"${SMOKE_LOG}" 2>&1 &
LAUNCH_PID=$!

for _ in {1..60}; do
  if ros2 node list 2>/dev/null | grep -qx '/move_group' && \
     ros2 topic list 2>/dev/null | grep -qx '/monitored_planning_scene' && \
     pgrep -f '[r]viz2' >/dev/null; then
    if timeout 3 ros2 topic echo /joint_states --once >"${JOINT_STATE_LOG}" 2>&1 && \
       timeout 3 ros2 service call \
         /controller_manager/list_controllers \
         controller_manager_msgs/srv/ListControllers '{}' \
         >"${CONTROLLERS_LOG}" 2>&1 && \
       grep -q "name='joint_state_broadcaster', state='active'" "${CONTROLLERS_LOG}" && \
       grep -q "name='panda_arm_controller', state='active'" "${CONTROLLERS_LOG}"; then
      echo "PASS (automated checks): Panda move_group, planning scene, RViz,"
      echo "joint states, and arm controller are running."
      echo "MANUAL CHECK: set a collision-free goal in RViz, click Plan, and confirm"
      echo "that the trajectory appears and the Panda/planning scene render correctly."
      read -r -p "Press Enter only after the RViz plan and rendering checks PASS: "
      echo "PASS: user confirmed Panda model, planning scene, and a displayed plan in RViz."
      exit 0
    fi
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    break
  fi
  sleep 1
done

echo "FAIL: Panda MoveIt demo was not ready within 60 seconds." >&2
sed -n '1,160p' "${SMOKE_LOG}" >&2
sed -n '1,80p' "${JOINT_STATE_LOG}" >&2
sed -n '1,120p' "${CONTROLLERS_LOG}" >&2
exit 1
