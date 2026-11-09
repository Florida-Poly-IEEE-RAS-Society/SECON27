#!/usr/bin/env bash
# Install dependencies and run the SoutheastCon 2027 race simulation.
#
#   ./simulation/run_sim.sh --install         install what's missing (asks for sudo)
#   ./simulation/run_sim.sh                   run with a random field
#   ./simulation/run_sim.sh seed:=42          any launch option, see README.md
#   ./simulation/run_sim.sh --tui             control panel (second terminal)
#   ./simulation/run_sim.sh --batch --runs 5 --cmd "ros2 run ..."
#                                             test robot code on many fields
#   ./simulation/run_sim.sh --no-gui          no Gazebo window (faster)
#   ./simulation/run_sim.sh --stop            stop everything the sim started
#
# Works on native Ubuntu 24.04, a VM, or WSL. WSL is detected automatically.
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
APT_PACKAGES=(ros-jazzy-ros-gz python3-pil)

is_wsl() {
    grep -qi microsoft /proc/version 2>/dev/null
}

missing_packages() {
    local pkg
    for pkg in "${APT_PACKAGES[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || echo "$pkg"
    done
}

install() {
    if [[ ! -f "$ROS_SETUP" ]]; then
        echo "ROS 2 Jazzy is not installed. Install it first:"
        echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
        exit 1
    fi
    local missing
    missing=$(missing_packages)
    if [[ -z "$missing" ]]; then
        echo "All simulation packages are already installed."
        return
    fi
    echo "Installing: $missing"
    sudo apt-get update
    # shellcheck disable=SC2086
    sudo apt-get install -y $missing
    echo "Done. Start the sim with: ./simulation/run_sim.sh"
}

# Everything a sim session can leave running: the launch (and the ROS nodes
# it started), Gazebo server + window, the ROS-Gazebo bridge, and our scripts
# (field manager, control panel, batch test). Each pattern is anchored to the
# start of the command line, so it matches the program itself and never a
# shell, editor or grep that merely mentions these names.
PY='[^ ]*python3[^ ]*'
SIM_PATTERNS=(
    "^${PY} [^ ]*/ros2 launch .*sim\\.launch\\.py"
    "^gz sim( |\$)"
    "^(/bin/sh -c )?ruby [^ ]*/gz sim( |\$)"
    "^[^ ]*/parameter_bridge( |\$)"
    "^[^ ]*/robot_state_publisher( |\$)"
    "^${PY} (-B )?${SIM_DIR}/scripts/[a-z_]+\\.py"
)

sim_pids() {
    local pattern
    for pattern in "${SIM_PATTERNS[@]}"; do
        pgrep -f "$pattern" || true
    done | sort -u
}

stop_all() {
    local pids sig
    pids=$(sim_pids)
    if [[ -z "$pids" ]]; then
        echo "Nothing is running."
        return
    fi
    echo "Stopping the simulation..."
    # Ask nicely first (Ctrl+C), so the launch shuts its nodes down and the
    # batch test cleans up; then escalate for anything that hangs.
    for sig in INT TERM KILL; do
        # shellcheck disable=SC2086
        kill -s "$sig" $pids 2>/dev/null || true
        for _ in $(seq 1 60); do
            pids=$(sim_pids)
            [[ -z "$pids" ]] && { echo "Stopped."; return; }
            sleep 0.5
        done
    done
    echo "Could not stop: $(echo $pids)"
    exit 1
}

if [[ "${1:-}" == "--stop" ]]; then
    stop_all
    exit 0
fi

if [[ "${1:-}" == "--install" ]]; then
    install
    exit 0
fi
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 0
fi

# Control panel for a sim that is already running in another terminal.
if [[ "${1:-}" == "--tui" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
    exec python3 -B "${SIM_DIR}/scripts/sim_tui.py"
fi

# Batch test: run robot code against many random fields, headless.
if [[ "${1:-}" == "--batch" ]]; then
    shift
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
    exec python3 -B "${SIM_DIR}/scripts/batch_test.py" "$@"
fi

# --no-gui: server only, no window. Same as the launch option gui:=false.
if [[ "${1:-}" == "--no-gui" ]]; then
    shift
    set -- "$@" gui:=false
fi

if [[ ! -f "$ROS_SETUP" ]] || [[ -n "$(missing_packages)" ]]; then
    echo "Missing simulation packages. Run this once first:"
    echo "  ./simulation/run_sim.sh --install"
    exit 1
fi

# A second Gazebo with the same world makes the field commands go to the
# wrong instance, so refuse to start one.
if pgrep -f "^gz sim server" >/dev/null; then
    echo "A Gazebo simulation is already running. Close it first:"
    echo "  ./simulation/run_sim.sh --stop"
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

if ! is_wsl; then
    # Forced software rendering crashes Gazebo on native Linux / VMs:
    # QT_QUICK_BACKEND=software breaks the 3D view, and LIBGL_ALWAYS_SOFTWARE
    # crashes the camera sensor's EGL setup. WSL needs them, so keep them there.
    unset QT_QUICK_BACKEND LIBGL_ALWAYS_SOFTWARE
fi

exec ros2 launch "${SIM_DIR}/launch/sim.launch.py" "$@"
