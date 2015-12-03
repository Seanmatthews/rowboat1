#!/bin/bash
set -e
source /opt/ros/jade/setup.bash
# extend this as needed for complete build coverage
for d in arbiter navigator operator safety
do
    echo " ~~~~### BUILDING $d ###~~~~"
    cd /home/odroid/rowboat1/src/$d
    catkin_make
done

