#!/bin/bash

PKG="sudo apt-get -y --no-install-recommends install"
ROS_VERSION="indigo"

sudo apt-get update

${PKG} git
${PKG} emacs
${PKG} build-essential
${PKG} python-dev
${PKG} g++
${PKG} python-smbus
${PKG} i2c-tools

# This fixes roscd (maybe something to do with running ROS as root?)
# http://answers.ros.org/question/53353/autocomplete-not-working-anymore/?comment=72208#comment-72208
echo "export LC_ALL="C"" >> ~/.bashrc

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
${PKG} "ros-${ROS_VERSION}-ros-base"
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
${PKG} python-rosinstall python-catkin-tools

# Pololu
${PKG} libusb-1.0-0-dev
# maybe not needed 
${PKG} mono-gmcs 
${PKG} mono-devel 
${PKG} libmono-winforms2.0-cil

# FFMpeg -- ffmpeg not in Ubuntu 14.04. Change to apt-get if upgrading to 16.*
$PKG} autoconf automake build-essential libass-dev libfreetype6-dev \
  libsdl1.2-dev libtheora-dev libtool libva-dev libvdpau-dev libvorbis-dev libxcb1-dev \
  libxcb-shm0-dev libxcb-xfixes0-dev pkg-config texinfo zlib1g-dev

git clone https://github.com/FFmpeg/FFmpeg.git
cd FFmpeg
git checkout tags/n3.1.3
./configure
make
sudo make install
cd ..
rm -rf FFmpeg

exit
