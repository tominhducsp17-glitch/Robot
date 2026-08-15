#!/usr/bin/env bash
set -Eeuo pipefail

if ! apt-cache show ros-jazzy-desktop >/dev/null 2>&1; then
  echo "FAIL: ros-jazzy-desktop is unavailable; run install_d1.sh to configure the official ROS apt source." >&2
  exit 1
fi

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# ROS-generated setup files may probe optional variables that are unset.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy

echo "PASS: ROS 2 Jazzy Desktop is installed and rosdep is initialized."
echo "Run scripts/phase0/smoke_ros_graph.sh to validate DDS discovery and messages."
