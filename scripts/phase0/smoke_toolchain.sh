#!/usr/bin/env bash
set -Eeuo pipefail

gcc --version | head -n 1
g++ --version | head -n 1
cmake --version | head -n 1
ninja --version
git --version
git lfs version
python3 --version
colcon version-check
rosdep --version

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "FAIL: WSLg display variables are absent." >&2
  exit 1
fi

glxinfo -B
echo "PASS: D1 executables and OpenGL query ran successfully."
