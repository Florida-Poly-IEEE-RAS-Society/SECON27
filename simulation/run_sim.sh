#!/usr/bin/env bash
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

exec ros2 launch "${SIM_DIR}/launch/sim.launch.py"
