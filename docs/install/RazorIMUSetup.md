# Razor 9 DOF IMU
This is a low/mid-level IMU offered by SparkFun (https://www.sparkfun.com/products/10736). Maybe I’m missing some main repository of documentation for this IMU, but I haven’t found much on setting up this IMU. When I do, more often than not, it handles only Arduino integration. Here, I’ll document the steps I took to set up the IMU specifically for this project, but you still may find the instructions useful for your own.

I am using:
* Razor 9DOF IMU
* ODroid C1+
* Ubuntu 14.04
* ROS

## Serial interface
Serial is the primary interface to the Razor. SparkFun also sells a FTDI breakout board and/or cord so that you can make this a USB interface, but I didn’t see much of a reason to spend the extra cash, or to add another point of failure along the path to the sensor. 


From the sensor to the Odroid C1+, we connected:
* PWR to pin1
* GND to pin6
* RX to pin8
* TX to pin10

Remember that TX and RX pins are swapped because one device is receiving the other’s transmission.

### Minicom
Testing your connected device is simple. `ls /dev/ttyS*` on your host device to get a list of the serial ports. Open ‘’’minicom’’’ from a terminal and set the communication baud rate to 57600 8N1. For each serial port (until you figure out which is the correct one), entering a question mark should display a help menu. Use the keys in the help menu to play around with your device. Further testing software is available through the ROS interface mentioned below.

## ROS
A ROS package exists for the Razor: http://wiki.ros.org/razor_imu_9dof. It provides a visualization, calibration instructions, as well as a testing interface.