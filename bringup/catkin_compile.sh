#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
cd /usr/local/webots_ws/ && catkin_make -DCMAKE_BUILD_TYPE=Release