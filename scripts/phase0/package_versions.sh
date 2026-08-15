#!/usr/bin/env bash
set -Eeuo pipefail

PACKAGES=(
  build-essential cmake ninja-build git git-lfs ros-dev-tools
  python3 python3-pytest python3-pip mesa-utils
  ros-jazzy-desktop ros-jazzy-moveit
  ros-jazzy-moveit-resources-panda-description
  ros-jazzy-moveit-resources-panda-moveit-config
  ros-jazzy-joint-state-broadcaster
  ros-jazzy-joint-trajectory-controller
  ros-jazzy-rviz-visual-tools
  ros-jazzy-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers
  ros-jazzy-gz-ros2-control ros-jazzy-gz-ros2-control-demos
)

for package in "${PACKAGES[@]}"; do
  version="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
  if [[ -n "${version}" ]]; then
    printf '%s\t%s\n' "${package}" "${version}"
  else
    printf '%s\tNOT_INSTALLED\n' "${package}"
  fi
done
