#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

SMOKE_DIR="$(mktemp -d)"
TALKER_PID=""
LISTENER_PID=""
cleanup() {
  [[ -z "${TALKER_PID}" ]] || kill "${TALKER_PID}" 2>/dev/null || true
  [[ -z "${LISTENER_PID}" ]] || kill "${LISTENER_PID}" 2>/dev/null || true
  rm -rf -- "${SMOKE_DIR}"
}
trap cleanup EXIT

ros2 run demo_nodes_cpp talker >"${SMOKE_DIR}/talker.log" 2>&1 &
TALKER_PID=$!
ros2 run demo_nodes_py listener >"${SMOKE_DIR}/listener.log" 2>&1 &
LISTENER_PID=$!

for _ in {1..20}; do
  if grep -q "I heard" "${SMOKE_DIR}/listener.log"; then
    echo "PASS: ROS 2 listener received a talker message."
    grep -m1 "I heard" "${SMOKE_DIR}/listener.log"
    exit 0
  fi
  sleep 0.5
done

echo "FAIL: no talker/listener exchange within 10 seconds." >&2
sed -n '1,80p' "${SMOKE_DIR}/talker.log" >&2
sed -n '1,80p' "${SMOKE_DIR}/listener.log" >&2
exit 1
