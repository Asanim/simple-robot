#!/bin/bash
set -e
source "/opt/ros/foxy/setup.bash"
source "/isaac-sim/ros2_workspace/install/local_setup.bash"
printenv
exec "${@:1}"
