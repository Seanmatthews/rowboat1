#!/bin/bash
sudo apt-get update

sudo apt-get -y install git
sudo apt-get -y install emacs
sudo apt-get -y install build-essential
sudo apt-get -y install python-dev
sudo apt-get -y install g++
sudo apt-get -y install python-smbus
sudo apt-get -y install i2c-tools

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-jade-desktop-base
sudo rosdep init
rosdep update
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
sudo apt-get -y install python-rosinstall

# External libs
git clone git://git.drogon.net/wiringPi external/wiringPi

exit
