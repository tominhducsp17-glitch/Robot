#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "FAIL: /opt/ros/jazzy/setup.bash is missing; D2 is not installed." >&2
  exit 1
fi

# ROS-generated setup files may probe optional variables that are unset. Source
# them with nounset temporarily disabled, then restore strict mode for our code.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DISTRO=jazzy
export RCUTILS_COLORIZED_OUTPUT=0
