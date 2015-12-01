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

# Add odroid user (in root group) and home directory
RUN groupadd -r odroid && \
    useradd -r -g odroid odroid && \
    usermod -a -G root odroid && \
    mkdir -p /home/odroid && \
    chown -R odroid:odroid /home/odroid

# Copy in git repo
COPY . /home/odroid/rowboat1
RUN chown -R odroid:odroid /home/odroid

# switch to new user
USER odroid
WORKDIR /home/odroid

# configure ROS 
RUN echo "source /opt/ros/jade/setup.bash" >> /home/odroid/.bashrc
RUN bash -c "source /opt/ros/jade/setup.bash && \
    rosdep update"

# build everything 
RUN /home/odroid/rowboat1/src/install/ci-build.sh 

CMD ["/bin/bash"]
