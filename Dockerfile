FROM ubergarm/armhf-ubuntu:trusty

LABEL description="This repo is always the latest image for the rowboat1 AUV system \
whose software may be found at https://github.com/Seanmatthews/rowboat1. The image is \
based upon ubergarm/armhf-ubuntu:trusty. It runs on an Odroid XU4 (ARM) computer."

#RUN apt-get update && apt-get install -y --no-install-recommends \
#        git \
#	&& rm -rf /var/lib/apt/lists/*

# Base Packages

#RUN apt-get update
#RUN apt-get install -y --no-install-recommends git
#RUN apt-get install -y --no-install-recommends build-essential
#RUN apt-get install -y --no-install-recommends python-dev
#RUN apt-get install -y --no-install-recommends python-smbus
#RUN apt-get install -y --no-install-recommends i2c-tools
#RUN apt-get install -y --no-install-recommends libusb-1.0-0-dev

# Various Specific Source Packages

#RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
#RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
#RUN apt-get update
#RUN apt-get install -y --no-install-recommends ros-indigo-ros-base
#RUN apt-get install -y --no-install-recommends ros-indigo-roslint
#RUN apt-get install -y --no-install-recommends ros-indigo-image-common
#RUN apt-get install -y --no-install-recommends ros-indigo-diagnostics
#RUN apt-get install -y --no-install-recommends python-rosinstall
#RUN apt-get install -y --no-install-recommends python-catkin-tools
#RUN rosdep init

# Copy in git repo
COPY . /root/rowboat1
CMD ["/root/rowboat1/src/install/base-install.sh"]
#RUN /root/rowboat1/src/install/base-install.sh
RUN /root/rowboat1/src/install/ros-packages-install.sh


# Clean up installation files
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Add odroid user (w/ root privileges)
# RUN useradd -ou 0 -g 0 -m odroid

# switch to new user
USER root
WORKDIR /root

# configure ROS (it will give Warning as our UID is same as root, 0)
RUN echo "source /opt/ros/indigo/setup.bash" >> /root/.bash_aliases
RUN bash -c "source /root/.bashrc && rosdep update"


# build everything
#RUN /root/rowboat1/src/install/ci-build.sh

CMD ["/bin/bash"]
