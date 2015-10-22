# ASSUMES BASE INSTALL HAS BEEN DONE

# Install drivers
sudo modprobe joydev
sudo apt-get -y install joystick
sudo apt-get -y install xboxdrv

# Configure modules to start at boot
XBOXDRV_CONF=/etc/init/xboxdrv.conf
sudo touch $XBOXDRV_CONF
echo "start on filesystem" | sudo tee --append $XBOXDRV_CONF
echo "pre-start script" | sudo tee --append $XBOXDRV_CONF
echo "    rm -f /dev/input/js*" | sudo tee --append $XBOXDRV_CONF
echo "end script" | sudo tee --append $XBOXDRV_CONF
echo "exec xboxdrv -D --detach-kernel-driver --silent --dbus disabled" | sudo tee --append $XBOXDRV_CONF

# Install ROS packages
sudo apt-get -y install ros-jade-joy

