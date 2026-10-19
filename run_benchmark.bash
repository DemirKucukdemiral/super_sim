#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$root/ros2_ws/src/super_sim_dynamics${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$root/benchmark_iso_validation.py" "$@"
