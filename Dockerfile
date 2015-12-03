FROM ubergarm/armhf-ubuntu:trusty

#RUN apt-get update && apt-get install -y --no-install-recommends \
#        git \
#	&& rm -rf /var/lib/apt/lists/*

# Base Packages
RUN apt-get update
RUN apt-get install -y --no-install-recommends git
RUN apt-get install -y --no-install-recommends build-essential 
RUN apt-get install -y --no-install-recommends python-dev 
RUN apt-get install -y --no-install-recommends python-smbus 
RUN apt-get install -y --no-install-recommends i2c-tools 
RUN apt-get install -y --no-install-recommends libusb-1.0-0-dev 

# Various Specific Source Packages
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get update
RUN apt-get install -y --no-install-recommends ros-jade-ros-base 
RUN rosdep init

# Clean up installation files
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Add odroid user (w/ root privileges)
RUN useradd -ou 0 -g 0 -m odroid

# switch to new user
USER odroid
WORKDIR /home/odroid

# configure ROS (it will give Warning as our UID is same as root, 0)
RUN echo "source /opt/ros/jade/setup.bash" >> /home/odroid/.bash_aliases
RUN bash -c "source /home/odroid/.bashrc && rosdep update"

# Copy in git repo
COPY . /home/odroid/rowboat1

# build everything 
RUN /home/odroid/rowboat1/src/install/ci-build.sh 

CMD ["/bin/bash"]
