# Make all workspaces
cd /vagrant/safety
catkin_make
cd /vagrant/arbiter
catkin_make
cd /vagrant/navigator
catkin_make
cd /vagrant/common
catkin_make

# Add all workspaces to the path
echo "/vagrant/safety/devel/setup.bash" >> ~/.bashrc
echo "/vagrant/navigator/devel/setup.bash" >> ~/.bashrc
echo "/vagrant/arbiter/devel/setup.bash" >> ~/.bashrc
echo "/vagrant/common/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc
