# Install I2C driver 
modprobe am1_i2c
sudo echo "i2c-dev" >> /etc/modules
sudo echo "am1_i2c" >> /etc/modules

# Add odroid user to i2c group so we don't need route for i2c access
sudo adduser odroid i2c

sudo reboot
