# Change this for dev install
ROWBOAT_INSTALL_DIR=/home/odroid/rowboat1/src

# Install ROS packages

# Make all local workspaces and add paths to workspace
for ws in safety arbiter navigator common
do
    catkin_make -C $ROWBAOT_INSTALL_DIR/$ws
    echo "source $ROWBOAT_INSTALL_DIR/$ws/devel/setup.bash" >> ~/.bashrc
done

# Make sure the user can source the ROS workspaces
# chmod -R 755 /vagrant/*

exit
