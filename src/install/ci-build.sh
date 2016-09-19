#!/bin/bash
set -ev
source /opt/ros/indigo/setup.bash
# extend this as needed for complete build coverage

echo " ~~~~### BUILDING WORKSPACE ###~~~~"
cd /home/odroid/rowboat1/src/rowboat
catkin_make

