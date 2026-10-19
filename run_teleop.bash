#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/environment.bash"
exec ros2 run super_sim_dynamics teleop_node.py "$@"
