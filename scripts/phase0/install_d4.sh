#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos

echo "PASS: Gazebo/ROS, ros2_control, and gz_ros2_control Jazzy binaries are installed."
echo "Run the two D4 smoke scripts before marking the gate PASS."
