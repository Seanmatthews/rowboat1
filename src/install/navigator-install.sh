# Install I2C driver 
modprobe am1_i2c
sudo echo "i2c-dev" >> /etc/modules
sudo echo "am1_i2c" >> /etc/modules

# Add odroid user to i2c group so we don't need route for i2c access
sudo adduser odroid i2c

# ROS packages
sudo apt-get -y install ros-indigo-razor_imu_9dof

# Post-install instructions
echo "REBOOT THE MACHINE!"
echo "IF IN VM, EXIT VM AND `vagrant reload --no-privision`
