#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-resources-panda-description \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-rviz-visual-tools

echo "PASS: MoveIt 2 and Panda binary resources are installed."
echo "Run scripts/phase0/smoke_moveit_panda.sh for the RViz planning smoke test."
