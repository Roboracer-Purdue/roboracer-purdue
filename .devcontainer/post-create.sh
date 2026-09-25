#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /workspaces/$(basename $PWD)   # dynamic
# If you didn't add submodules, you may want to run the original setup.sh here:
# bash ./setup.sh   # but ensure it doesn't re-install apt packages unnecessarily
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
echo "Workspace built successfully!"