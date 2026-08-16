#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
gz topic \
  -t /phase1/peg/detach \
  -m gz.msgs.Empty \
  -p 'unused: true'
