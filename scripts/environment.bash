#!/usr/bin/env bash
# Shared setup; source from the project's launchers.
SUPER_SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ ! -f "$setup" ]]; then
  echo "ROS setup not found: $setup. Run inside your ROS 2 Linux environment." >&2
  return 1
fi
source "$setup"
if [[ ! -f "$SUPER_SIM_ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "Build first: $SUPER_SIM_ROOT/build.bash" >&2
  return 1
fi
source "$SUPER_SIM_ROOT/ros2_ws/install/setup.bash"
