#!/bin/bash
set -e

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
# source /usr/local/webots_ws/devel/setup.bash
$@