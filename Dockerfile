FROM ubergarm/armhf-ubuntu:trusty
RUN echo "deb http://archive.ubuntu.com/ubuntu trusty universe" >> /etc/apt/sources.list
RUN apt-get update