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
sudo apt-get -y install ros-jade-ros-base
sudo rosdep init
rosdep update
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
sudo apt-get -y install python-rosinstall

# Pololu
sudo apt-get -y install libusb-1.0-0-dev
# maybe not needed 
sudo apt-get -y install mono-gmcs 
sudo apt-get -y install mono-devel 
sudo apt-get -y install libmono-winforms2.0-cil

# External libs
#if [ ! -d external/wiringPi/.git ]
#then
#    git clone git://git.drogon.net/wiringPi /vagrant/external/wiringPi
#fi

exit
