#!/bin/bash
set -ev
source /opt/ros/jade/setup.bash
# extend this as needed for complete build coverage
WORKSPACE="rowboat"
echo " ~~~~### BUILDING WORKSPACE: $WORKSPACE ###~~~~"
cd /home/odroid/rowboat1/src/$WORKSPACE
catkin_make

