#!/usr/bin/env bash
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

# Forced software rendering crashes Gazebo on native Linux / VMs:
# QT_QUICK_BACKEND=software breaks the 3D view, and LIBGL_ALWAYS_SOFTWARE
# crashes the camera sensor's EGL setup.
unset QT_QUICK_BACKEND LIBGL_ALWAYS_SOFTWARE

exec ros2 launch "${SIM_DIR}/launch/sim.launch.py"
