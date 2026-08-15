#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
export GZ_PARTITION="${PHASE0_GZ_PARTITION:-closed_loop_phase0_gazebo_${BASHPID}}"

if ! command -v gz >/dev/null; then
  echo "FAIL: the Gazebo 'gz' command is missing." >&2
  exit 1
fi

run_server_check() {
  local log_file pid
  log_file="$(mktemp)"
  setsid gz sim -r -s empty.sdf >"${log_file}" 2>&1 &
  pid=$!
  for _ in {1..30}; do
    if gz service -l 2>/dev/null | grep -q '/world/empty/control'; then
      echo "PASS: Gazebo Harmonic empty-world server advertised world services."
      kill -TERM -- "-${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
      rm -f -- "${log_file}"
      return 0
    fi
    sleep 0.5
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
  sed -n '1,160p' "${log_file}" >&2
  rm -f -- "${log_file}"
  return 1
}

run_server_check || {
  echo "FAIL: Gazebo headless empty-world smoke test failed." >&2
  exit 1
}

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "FAIL: headless simulation passed, but no WSLg display exists for GUI test." >&2
  exit 1
fi

echo "Starting Gazebo GUI. Confirm the empty world renders and the window remains responsive."
echo "Close the Gazebo window to finish this smoke test."
gz sim empty.sdf
echo "PASS: Gazebo GUI exited normally after the user-visible WSLg check."
