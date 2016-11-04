#!/bin/bash
set -ev
source /opt/ros/indigo/setup.bash
# extend this as needed for complete build coverage

echo " ~~~~### INSTALLING DEPENDENCIES ###~~~~"
# /root/rowboat1/src/install/base-install.sh
# /root/rowboat1/src/rowboat/ros-packages-install.sh

echo " ~~~~### BUILDING WORKSPACE ###~~~~"
cd /root/rowboat1/src/rowboat
git submodule update --init --recursive
catkin build 

