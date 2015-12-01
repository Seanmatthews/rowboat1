rowboat1
===

[![Travis CI](https://travis-ci.org/ubergarm/rowboat1.svg)](https://travis-ci.org/ubergarm/rowboat1/)

This is the repository for Brooklyn's Diamond Reef Explorer's autonomous underwater vehicle, Rowboat-1. The project is currently in its planning and design phase. If you'd like help out, no matter what your skills, join our meetup group at http://www.meetup.com/Tech-Tinkerers-NYC. 

# Dev Setup

## Mac/Linux Setup
0. Install VirtualBox from https://www.virtualbox.org/wiki/Downloads
0. Install Vagrant from https://www.vagrantup.com
2. Install Git from https://git-scm.com 
1. Checkout the project with Git
2. Go to src directory
3. `vagrant up` (this might take a few)
4. `vagrant ssh`
5. `/vagrant/base-install.sh`
6. `source ~/.bashrc`
7. If you’re new to ROS, follow the tutorials at http://wiki.ros.org/ROS/Tutorials

## Windows Setup 
1. Install VirtualBox from https://www.virtualbox.org/wiki/Downloads
2. Download Ubuntu 14.04 Desktop from http://www.ubuntu.com
3. Create a new Linux 64-bit VM and follow the steps to install your downloaded Ubuntu image.
4. `sudo apt-get install git`
5. `git clone https://github.com/Seanmatthews/rowboat1.git`
6. From the rowboat1/src/install directory, run `./base-install.sh`
7. `source ~/.bashrc`
8. The ROS tutorials: http://wiki.ros.org/ROS/Tutorials

 
CircleCI Docker Image Automation
===

This project contains files relevent to the automated building of armhf
(armv7) compatible Docker images. These images can be used for dev,
build, test, and deploy to ensure a consistent user experience.

This solution combines:

* Host kernel support for binfmt misc
* qemu-arm-static
* chroot
* Docker armhf images
* CircleCI

## Quick Start

To configure build automation and be able to run ARM Docker containers
from an x86-64 system follow these steps or check out the reference
found at the bottom.

    sudo curl -o /usr/share/binfmts/qemu-arm http://ubergarm.com/dre/qemu-arm
    sudo curl -o /usr/bin/qemu-arm-static http://ubergarm.com/dre/qemu-arm-static
    sudo modprobe binfmt_misc
    sudo apt-get install -y binfmt-support

To test that the host is configured properly:

    curl -O http://ubergarm.com/dre/hello_world-arm-static
    chmod a+x hello_world-arm-static
    ./hello_world-arm-static

## Build ARM Docker Image 

From top level project directory.

    docker build -t ubergarm/rowboat1 .

## Run ARM Docker Image

   docker run --rm -it ubergarm/rowboat1

## Update ubergarm base image

Pull the latest base image and add the qemu-arm-static binary.

    docker pull ioft/armhf-ubuntu:trusty
    CID=$(docker create --name trusty ioft/armhf-ubuntu:trusty)
    docker cp /usr/bin/qemu-arm-static $CID:/usr/bin/qemu-arm-static
    docker commit $CID ubergarm/armhf-ubuntu:trusty
    docker push ubergarm/armhf-ubuntu:trusty
    
## References
[blog.ubergarm.com](http://blog.ubergarm.com/run-arm-docker-images-on-x86_64-hosts/)
