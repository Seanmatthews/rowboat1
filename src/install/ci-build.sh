#!/bin/bash
set -ev
source /opt/ros/indigo/setup.bash
# extend this as needed for complete build coverage

echo " ~~~~### BUILDING WORKSPACE ###~~~~"
cd /root/rowboat1/src/rowboat
git submodule update --init --recursive
# VERBOSE=1 catkin build --no-status --no-color -p1 -v controller_translator
catkin_make
