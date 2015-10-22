# Install ROS packages

# Make all local workspaces
cd /vagrant/safety
catkin_make
cd /vagrant/arbiter
catkin_make
cd /vagrant/navigator
catkin_make
cd /vagrant/common
catkin_make

# Add all workspaces to the path
echo "source /vagrant/safety/devel/setup.bash" >> ~/.bashrc
echo "source /vagrant/navigator/devel/setup.bash" >> ~/.bashrc
echo "source /vagrant/arbiter/devel/setup.bash" >> ~/.bashrc
echo "source /vagrant/common/devel/setup.bash" >> ~/.bashrc

# Make sure the user can source the ROS workspaces
# chmod -R 755 /vagrant/*

exit
