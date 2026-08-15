#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "$(. /etc/os-release && printf '%s' "${VERSION_CODENAME}")" != "noble" ]]; then
  echo "FAIL: this Phase 0 matrix requires Ubuntu Noble (24.04)." >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  build-essential cmake ninja-build git git-lfs \
  python3-pytest python3-pip mesa-utils curl software-properties-common
sudo add-apt-repository -y universe

ROS_APT_SOURCE_VERSION="$(
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p'
)"
if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
  echo "FAIL: could not resolve the official ros-apt-source release." >&2
  exit 1
fi

ROS_APT_SOURCE_DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
curl -fL -o "${ROS_APT_SOURCE_DEB}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i "${ROS_APT_SOURCE_DEB}"
sudo apt update
sudo apt install -y ros-dev-tools

git lfs install --local
echo "PASS: D1 packages installed. Run scripts/phase0/smoke_toolchain.sh next."
