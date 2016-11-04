#!/bin/bash
set -ev
source /opt/ros/indigo/setup.bash
# extend this as needed for complete build coverage

echo " ~~~~### INSTALLING DEPENDENCIES ###~~~~"
# /root/rowboat1/src/install/base-install.sh
# /root/rowboat1/src/rowboat/ros-packages-install.sh

tty_fix = \
"if [[ $- == *i* ]]
then 
    NRM=`tput sgr0` 
    BLD=`tput bold` 
    ITL=`tput sitm` 
    UL=`tput smul` 
    RED=`tput setaf 1` 
    GRN=`tput setaf 2` 
    BLU=`tput setaf 4`
    PS1='\n\r${BLD}\u${NRM}|${UL}\h${NRM} [${BLD}${BLU}\W${NRM}] \w \n>> '
fi"

echo "$tty_fix" > /root/.bash_profile 

echo " ~~~~### BUILDING WORKSPACE ###~~~~"
cd /root/rowboat1/src/rowboat
git submodule update --init --recursive
catkin build 

