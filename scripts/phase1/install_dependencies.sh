#!/usr/bin/env bash
set -eo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as your normal user; it invokes sudo only for apt." >&2
  exit 2
fi

sudo apt update
sudo apt install -y --no-install-recommends \
  python3-yaml \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-capabilities \
  ros-jazzy-moveit-task-constructor-msgs \
  ros-jazzy-moveit-task-constructor-visualization

source /opt/ros/jazzy/setup.bash
set -u
ros2 pkg prefix moveit_task_constructor_core
ros2 pkg prefix moveit_task_constructor_capabilities
ros2 pkg prefix moveit_task_constructor_visualization
