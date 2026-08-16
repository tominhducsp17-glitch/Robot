#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u

gz topic \
  -t /phase1/peg/detach \
  -m gz.msgs.Empty \
  -p 'unused: true'

# Gazebo transport is asynchronous; wait until the fixed grasp joint is gone.
sleep 0.5

gz service \
  -s /world/phase1_baseline/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'name: "peg", position: {x: 0.38, y: 0.20, z: 0.812}, orientation: {w: 1.0}'
