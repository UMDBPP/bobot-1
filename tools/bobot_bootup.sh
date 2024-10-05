#!/bin/bash

# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|

# Script to run on startup - should source the workspace, (possible) build the repository, and the launch the main code

_path=$(realpath "$0")
_DIR=$(dirname $_path)

cd ${_DIR}
cd ..
source /opt/ros/jazzy/setup.bash
source install/setup.bash

gnome-terminal -- bash -c "ros2 launch bobot_bringup core_launcher.launch.py; exec bash"
