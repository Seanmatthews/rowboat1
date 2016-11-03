# Change this for dev install
ROWBOAT_INSTALL_DIR=/root/rowboat1/src

ROSVER="ros-indigo"
PKG="sudo apt-get -y --no-install-recommends install"

# Install ROS packages
${PKG} "${ROSVER}-diagnostic-aggregator"

# GUI & teleop
${PKG} "${ROSVER}-image-transport"
${PKG} "${ROSVER}-teleop-twist-joy"

# pointgrey_camera_driver requirements
${PKG} "${ROSVER}-roslint"
${PKG} "${ROSVER}-camera-info-manager"

# Navigation
${PKG} "${ROSVER}-razor_imu_9dof"

# Using old vision_opencv repo as submodule
# OpenCV + Vision
# ${PKG} "${ROSVER}-vision-opencv"

# Make all local workspaces and add paths to workspace
echo "source $ROWBOAT_INSTALL_DIR/rowboat/devel/setup.bash" >> ~/.bashrc

exit
