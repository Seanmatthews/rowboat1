FROM ubergarm/armhf-ubuntu:trusty

LABEL description="This repo is always the latest image for the rowboat1 AUV system \
whose software may be found at https://github.com/Seanmatthews/rowboat1. The image is \
based upon ubergarm/armhf-ubuntu:trusty. It runs on an Odroid XU4 (ARM) computer."

#RUN apt-get update && apt-get install -y --no-install-recommends \
#        git \
#	&& rm -rf /var/lib/apt/lists/*

# Base Packages

RUN apt-get update
RUN apt-get install -y --no-install-recommends \
    git \
    emacs \
    build-essential \
    python-dev \
    g++ \
    python-smbus \
    i2c-tools \
    cmake \
    libusb-1.0-0-dev \
    mono-gmcs \
    mono-devel \
    libmono-winforms2.0-cil \
    autoconf \
    automake \
    libass-dev \
    libfreetype6-dev \
    libsdl1.2-dev \
    libtheora-dev \
    libtool \
    libva-dev \
    libvdpau-dev \
    libvorbis-dev \
    libxcb1-dev \
    libxcb-shm0-dev \
    libxcb-xfixes0-dev \
    pkg-config \
    texinfo \
    zlib1g-dev \
    libcurlpp-dev \
    libculpp0

RUN git clone https://github.com/FFmpeg/FFmpeg.git
RUN cd FFmpeg; git checkout tags/n3.1.3
RUN ./configure; make; make install; cd ..; rm -rf FFmpeg

# This fixes roscd (maybe something to do with running ROS as root)
# http://answers.ros.org/question/53353/autocomplete-not-working-anymore/?comment=72208#comment-72208
RUN echo "export LC_ALL="C"" >> ~/.bashrc

# Various Specific Source Packages
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get update
#RUN apt-get install -y --no-install-recommends 
     ros-indigo-ros-base \
     ros-indigo-roslint \
     ros-indigo-image-common \
     ros-indigo-diagnostics \
     python-rosinstall \
     python-catkin-tools
RUN rosdep init


#RUN /root/rowboat1/src/install/base-install.sh
#RUN /root/rowboat1/src/install/ros-packages-install.sh


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

# Copy in git repo
COPY . /root/rowboat1

# build everything
#RUN /root/rowboat1/src/install/ci-build.sh

CMD ["/bin/bash"]
