{\rtf1\ansi\ansicpg1252\cocoartf1348\cocoasubrtf170
{\fonttbl\f0\fswiss\fcharset0 Helvetica;\f1\froman\fcharset0 Times-Roman;\f2\froman\fcharset0 TimesNewRomanPSMT;
\f3\fswiss\fcharset0 ArialMT;\f4\fnil\fcharset0 HelveticaNeue;}
{\colortbl;\red255\green255\blue255;\red38\green38\blue38;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural

\f0\fs24 \cf0 # IMU\
\
## Requirements\
Due to the high cost of IMUs, these are mostly nice-to-haves\
- ROS supported\
- < $1000\
- Resistant to electromagnetic disturbance\
- GPS-integrated (but does not require GPS for operation)\
\
\
## Resources\
- Comparison list: http://damien.douxchamps.net/research/imu/\
- ROS-supported sensors: http://wiki.ros.org/Sensors\
- http://www.chrobotics.com/library/accel-position-velocity\
- Converting between noise density and angular error: http://www.rcgroups.com/forums/showthread.php?t=1220872\
\
## Pending inquiries\
- http://openrov.dozuki.com/Answers/View/214/Use+IMU+with+other+vehicles\
\
## Potentials\
- http://store.openrov.com/products/openrov-imu-depth-module \'97 $120	\
- http://landing.kvh.com/imu \'97 $???, Probably too much > $3000\
- https://www.sparkfun.com/categories/160 \'97 $20 - $80\
- http://www.x-io.co.uk/products/x-imu/\
- http://www.phidgets.com/products.php?product_id=1044 \'97 $140\
- http://www.microstrain.com/inertial/3dm-gx4-15 \'97 $1700\
- http://www.mouser.com/ProductDetail/Analog-Devices-Inc/ADIS16480BMLZ/ \'97 $2200\
- https://www.spartonnavex.com/product/gedc-6e/ \'97 $1350\
- http://www.robotshop.com/en/6-dof-gyro-accelerometer-imu-mpu6050.html \'97 $10\
- http://store.invensense.com/ProductDetail/MPU9250-InvenSense-Inc/487537/ \'97 < $10\
- http://www.sensonor.com/gyro-products/inertial-measurement-units/stim300.aspx \'97 $???\
- http://www.chrobotics.com/shop/gp9-ahrs \'97 $500\
- https://learn.adafruit.com/adafruit-10-dof-imu-breakout-lsm303-l3gd20-bmp180 \'97 $30\
- http://www.tanenhausassociates.com/advantages-1-1/\
- http://www.siliconsensing.com/products/inertial-modules-systems/dmu30/\
\
### IMUs I like based on not enough research\
- http://www.acmesystems.it/DAISY-7\
- http://www.chrobotics.com/shop/gp9-ahrs\
\'97 looks like velocity is computed using GPS only\
\'97 earlier version is ROS supported\
- http://www.tanenhausassociates.com/advantages-1-1/\
\'97 May not be available for purchase\
- http://www.vectornav.com/products/vn100-rugged/specifications\
\'97 $1200\
- http://www.tanenhausassociates.com/advantages-1-1/\
\'97 Received an email response from this company. They will release their product at the end of Q3.\
\
\
\
## RoboSub Team-used IMUs\
- 
\f1\fs36 \cf2 \expnd0\expndtw0\kerning0
SparkFun 9DOF Razor\
- 
\f2\fs24 \cf0 \expnd0\expndtw0\kerning0
Phidget Spatial Precision Inertial Measurement Unit
\f3 \expnd0\expndtw0\kerning0
\
- 
\f2\fs32 \expnd0\expndtw0\kerning0
VectorNav VN-100 
\f1\fs24 \expnd0\expndtw0\kerning0
\
- KVH 
\f2\fs32 \expnd0\expndtw0\kerning0
DSP-1750 FOG 
\f1\fs24 \expnd0\expndtw0\kerning0
\
- 
\fs32 \expnd0\expndtw0\kerning0
LORD MicroStrain 3DM- GX4-25 AHRS, and a custom team designed compass and IMU. 
\fs24 \expnd0\expndtw0\kerning0
\
- 
\fs32 \expnd0\expndtw0\kerning0
Xsens Mti IMU
\fs24 \expnd0\expndtw0\kerning0
\
- 
\fs30 \expnd0\expndtw0\kerning0
3DM-GX3-25 
\fs24 \expnd0\expndtw0\kerning0
\
- 
\f2\fs20 \expnd0\expndtw0\kerning0
Lord MicroStrain 3DM-GX3-25 + KVH 1775 FOG IMU\
- PNI TRAX\
- 
\fs32 \expnd0\expndtw0\kerning0
ADIS16480 
\f1\fs24 \expnd0\expndtw0\kerning0
\
- 
\fs32 \expnd0\expndtw0\kerning0
MPU6050 6DOF IMU by Invensense 
\fs24 \expnd0\expndtw0\kerning0
\
- 
\fs32 \expnd0\expndtw0\kerning0
Sparton GEDC-6 IMU 
\fs24 \expnd0\expndtw0\kerning0
\
- 
\f2 \expnd0\expndtw0\kerning0
VN-200 Rugged GPS/INS\
- 
\f1\fs32 \expnd0\expndtw0\kerning0
Microstrain 3DMGX1 
\fs24 \expnd0\expndtw0\kerning0
\
- 
\f3\fs32 \expnd0\expndtw0\kerning0
MPU6050 AHRS (Triple Axis Accel- erometer and Gyro Breakout 
\f1\fs24 \expnd0\expndtw0\kerning0
by sparkfun)\
- 
\f2\fs32 \expnd0\expndtw0\kerning0
MPU9250 9\'ad axis IMU \uc0\u8232 - S
\f1 \expnd0\expndtw0\kerning0
parton GEDC-60 AHRS 
\fs24 \expnd0\expndtw0\kerning0
\

\f2\fs32 \expnd0\expndtw0\kerning0
- 20 Sparton AHRS-8 
\f1\fs24 \expnd0\expndtw0\kerning0
\

\f2\fs32 \expnd0\expndtw0\kerning0
- 
\f3\fs30 \expnd0\expndtw0\kerning0
CH Robotics\'92 CHR- 6dm 
\f1\fs24 \expnd0\expndtw0\kerning0
\

\f2\fs32 \expnd0\expndtw0\kerning0
- 
\f1 \expnd0\expndtw0\kerning0
OceanServer OS5000-T accelerometer/compass 
\fs24 \expnd0\expndtw0\kerning0
\

\f2\fs32 \expnd0\expndtw0\kerning0
- 
\f1\fs30 \expnd0\expndtw0\kerning0
MPU-9150 IMU 
\fs24 \expnd0\expndtw0\kerning0
\

\f2\fs32 \expnd0\expndtw0\kerning0
- 
\f4 \expnd0\expndtw0\kerning0
Phidget High Precision unit and a Sparton AHRS-8 
\f1\fs24 \expnd0\expndtw0\kerning0
\

\f2\fs32 \expnd0\expndtw0\kerning0
- Sensonar STIM300 9-axis inertial measurement unit + PNI TCM MB compass \
- 
\f1 \expnd0\expndtw0\kerning0
Xsens MTi Attitude and Hearing Reference System (AHRS), a Honeywell HMC6343 3-axis compass 
\fs24 \expnd0\expndtw0\kerning0
\

\fs32 \expnd0\expndtw0\kerning0
- 
\fs26 \expnd0\expndtw0\kerning0
DFRobot IMU chip 
\fs32 \expnd0\expndtw0\kerning0
\uc0\u8232 \

\f2 \expnd0\expndtw0\kerning0
\

\f1\fs24 \expnd0\expndtw0\kerning0
\
\uc0\u8232 \
}