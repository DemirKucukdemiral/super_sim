#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/environment.bash"
exec ros2 launch super_sim_dynamics super_sim.launch.py "$@"
