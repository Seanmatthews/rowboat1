FROM ubergarm/armhf-ubuntu:trusty

ENV QEMU_EXECVE 1
# Force armv6l
# ENV QEMU_CPU arm1176

COPY qemu-arm-static /usr/bin
COPY cross-build-start /usr/bin
COPY cross-build-end /usr/bin
COPY sh-shim /usr/bin

RUN [ "cross-build-start" ]

# Packages
RUN apt-get clean && apt-get update && apt-get install -y --no-install-recommends \
    git \
    emacs \
    build-essential \
    python-dev \
    g++ \
    python-smbus \
    i2c-tools \
    cmake \
    libusb-1.0-0-dev 

RUN apt-get clean && apt-get update && apt-get install -y --no-install-recommends \
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
    libcurlpp0

# FFmpeg
RUN git clone https://github.com/FFmpeg/FFmpeg.git; \
    cd FFmpeg; git checkout tags/n3.1.3; \
    ./configure; make; make install; cd ..; rm -rf FFmpeg;

# This fixes roscd (maybe something to do with running ROS as root)
# http://answers.ros.org/question/53353/autocomplete-not-working-anymore/?comment=72208#comment-72208
RUN echo "export LC_ALL="C"" >> ~/.bashrc

# ROS Packages
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get clean && apt-get update && apt-get install -y --no-install-recommends \
     ros-indigo-ros-base \
     ros-indigo-catkin \
     ros-indigo-roslint \
     ros-indigo-image-common \
     ros-indigo-diagnostics \
     ros-indigo-diagnostic-aggregator \
     ros-indigo-image-transport \
     ros-indigo-teleop-twist-joy \
     ros-indigo-camera-info-manager \
     ros-indigo-razor-imu-9dof \
     python-rosinstall \
     python-catkin-tools 

RUN rosdep init

# Clean up installation files
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# switch to new user
USER root
WORKDIR /root

# configure ROS (it will give Warning as our UID is same as root, 0)
RUN echo "source /opt/ros/indigo/setup.bash" >> /root/.bash_aliases
RUN echo "source /root/rowboat1/src/rowboat/devel/setup.bash" >> /root/.bash_aliases
RUN bash -c "source /root/.bashrc && rosdep update"

# Copy in git repo
COPY src /root/rowboat1
COPY README.md /root/rowboat1
COPY .gitignore /root/rowboat1

RUN [ "cross-build-end" ]


