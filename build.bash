#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
[[ -f "$setup" ]] || { echo "Missing $setup. Run inside ROS 2 Linux." >&2; exit 1; }
source "$setup"
cd "$root/ros2_ws"
exec colcon build --symlink-install --packages-select super_sim_dynamics "$@"
