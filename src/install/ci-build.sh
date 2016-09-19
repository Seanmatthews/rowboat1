#!/bin/bash
set -ev
source /opt/ros/indigo/setup.bash
# extend this as needed for complete build coverage

echo " ~~~~### BUILDING WORKSPACE ###~~~~"
cd /root/rowboat1/src/rowboat
catkin build --no-color

