#!/usr/bin/env bash
# =============================================================================
#  F1TENTH Sim — workspace setup inside the RoboRacer dev container
#
#  The container already provides: Ubuntu 22.04, ROS 2 Humble, and the
#  virtual desktop (DISPLAY=:99, set in devcontainer.json).
#  This script installs the sim pieces and builds the workspace.
#
#  Usage (inside the container, from roboracer_ws/):
#     ./setup.sh
#  Safe to re-run.
# =============================================================================
set -eo pipefail          # no "set -u": ROS setup scripts use unset variables

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
GYM_DIR="${WS_DIR}/f1tenth_gym"          # backend gets cloned here (gitignored)

banner() { echo -e "\n\033[1;34m==> $1\033[0m"; }
fail()   { echo -e "\033[1;31mERROR:\033[0m $1"; exit 1; }

# -----------------------------------------------------------------------------
banner "0. Pre-flight checks"
# -----------------------------------------------------------------------------
[[ -f /opt/ros/humble/setup.bash ]] || \
    fail "/opt/ros/humble not found. Run this inside the dev container (Reopen in Container)."
[[ -z "${VIRTUAL_ENV:-}${CONDA_DEFAULT_ENV:-}" ]] || \
    fail "a virtualenv/conda env is active. ROS 2 needs system Python — deactivate it and retry."
[[ "${EUID}" -ne 0 ]] || fail "run as a normal user, not root/sudo."
[[ -d "${WS_DIR}/src" ]] || fail "no src/ folder next to setup.sh. Put setup.sh in roboracer_ws/."

# Catch the empty-folder problem (package committed as a broken git pointer)
[[ -f "${WS_DIR}/src/f1tenth_gym_ros/package.xml" ]] || \
    fail "src/f1tenth_gym_ros is missing or empty. Pull the latest branch, or see the setup guide."

# -----------------------------------------------------------------------------
banner "1. Install any missing sim dependencies"
# -----------------------------------------------------------------------------
SIM_PKGS=(
    python3-pip git build-essential python3-dev
    python3-colcon-common-extensions python3-rosdep
    ros-humble-navigation2 ros-humble-nav2-bringup
    ros-humble-ackermann-msgs ros-humble-xacro
)
MISSING=()
for pkg in "${SIM_PKGS[@]}"; do
    dpkg -s "$pkg" &>/dev/null || MISSING+=("$pkg")
done
if (( ${#MISSING[@]} )); then
    echo "Installing: ${MISSING[*]}"
    sudo apt update
    sudo apt install -y "${MISSING[@]}"
else
    echo "All sim dependencies already installed."
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

# -----------------------------------------------------------------------------
banner "2. Install the f1tenth_gym Python backend (editable, system Python)"
# -----------------------------------------------------------------------------
if [[ ! -d "${GYM_DIR}" ]]; then
    git clone https://github.com/f1tenth/f1tenth_gym.git "${GYM_DIR}"
else
    echo "f1tenth_gym already present, skipping clone."
fi
pip3 install -e "${GYM_DIR}"

# -----------------------------------------------------------------------------
banner "3. Python dependency fixes (transforms3d + numba/coverage)"
# -----------------------------------------------------------------------------
pip3 install transforms3d
pip3 install --upgrade coverage   # fixes 'module coverage has no attribute types'

# -----------------------------------------------------------------------------
banner "4. Resolve ROS deps + build the workspace"
# -----------------------------------------------------------------------------
cd "${WS_DIR}"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y || true
rm -rf build install log
colcon build

# -----------------------------------------------------------------------------
banner "5. Auto-source ROS + workspace in new terminals (idempotent)"
# -----------------------------------------------------------------------------
add_line() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }
add_line "source /opt/ros/humble/setup.bash"
add_line "source ${WS_DIR}/install/local_setup.bash"
# Display settings (DISPLAY, LIBGL_ALWAYS_SOFTWARE, QT_QPA_PLATFORM) come from
# devcontainer.json containerEnv — not set here. Remove any stale overrides:
sed -i '/^export DISPLAY=/d' ~/.bashrc

# -----------------------------------------------------------------------------
banner "6. Verify"
# -----------------------------------------------------------------------------
python3 -c "import f110_gym" && echo "f110_gym: OK" || fail "f110_gym did not import."
source "${WS_DIR}/install/local_setup.bash"
ros2 pkg list | grep -qx f1tenth_gym_ros && echo "f1tenth_gym_ros: OK" || fail "f1tenth_gym_ros not built."

cat <<MSG

Setup complete. Open a NEW terminal, then:

  1. Desktop:   http://localhost:6080/vnc.html   (password: vscode)
  2. Sim:       ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  3. Drive:     python3 ${WS_DIR}/scripts/key_drive.py   (second terminal)
MSG