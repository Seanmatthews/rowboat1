# Change this for dev install
ROWBOAT_INSTALL_DIR=/root/rowboat1/src

# Install ROS packages
sudo apt-get -y install ros-indigo-image-transport

# Build


# Make all local workspaces and add paths to workspace
echo "source $ROWBOAT_INSTALL_DIR/rowboat/devel/setup.bash" >> ~/.bashrc

exit
