#!/bin/bash
sudo apt-get update

sudo apt-get -y install git
sudo apt-get -y install emacs
sudo apt-get -y install build-essential
sudo apt-get -y install python-dev
sudo apt-get -y install g++
sudo apt-get -y install python-smbus
sudo apt-get -y install i2c-tools

# This fixes roscd (maybe something to do with running ROS as root?)
# http://answers.ros.org/question/53353/autocomplete-not-working-anymore/?comment=72208#comment-72208
echo "export LC_ALL="C"" >> ~/.bashrc

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get -y install ros-indigo-ros-base
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
sudo apt-get -y install python-rosinstall
sudo apt-get -y install python-catkin-tools

# Pololu
sudo apt-get -y install libusb-1.0-0-dev
# maybe not needed 
sudo apt-get -y install mono-gmcs 
sudo apt-get -y install mono-devel 
sudo apt-get -y install libmono-winforms2.0-cil

exit
