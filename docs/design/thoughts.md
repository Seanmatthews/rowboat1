Jotting down thoughts and lists here that will eventually mature into their own documents.

- proprioception
-- local positioning (dvl?)
-- internal temp
-- moisture sensor
 
- sensing environmental conditions
-- temp
-- current
-- lumens
 
- software architecture
-- logging
-- ROS
-- video/image compression & storage
—- ubuntu os
—- vagrant dev env, perhaps with docker if it’s gotten easier to use
 
- low-level movement
-- remaining still in currents
-- moving around local 3D axis
 
- vision
-- image processing (filtering deep water images, etc)
-- computer vision
 
- obstacle detection
 
- electronics
-- high level processing, programmable board
-- motor controller boards
-- radio comms -- wifi, LF band, pings for underwater?
-- cameras
-- CF card
 
- simulator
-- perfect world simulation for modeling movements when
current is ignored
 
- mechanical
-- hull integrity
-- ballast system
-- buoyancy control (compressed air)
 
==========================

* The main sbc should do the image processing, as it will be the most powerful. This sbc will also control all the high level behavior, so that will be a risk, for memory especially.

SBC Candidates
- http://www.minnowboard.org/meet-minnowboard-max/
- 
 
ROS components
- camera interface
- thruster motor interface
- thruster controller interface
- ballast interface
- moisture sensor interface
- low level navigation
- high level navigation
- sonar interface
- tactile sensor interface
- sonar fusion
- thruster feedback fusion
- temperature sensor
- arbiter
- environment mapper (external sensor fusion)

 
Hardware
- 1x dvl
- 8x sonar
- 2x thruster (http://bluerobotics.com)
- 1x lexan globe
- 1x lexan tube
- 2x battery pvc stand
- 1x hd camera with light
- 1x temperature sensor
- 1x high frequency pinger
- 25x small co2 container
- 4x moisture sensor
- ?x tactile whisker sensors / bumper sensors
- 1x cell transceiver w/gps
- 1x mini pc104 stack
- 2x raspberry pi (one to handle low-level and high-level navigation, the other for high-update sensors)
- 1x IMU

Primary SBC Required I/O
- eth
- wifi
- pwm? (for ballast?)
- 1x GPIO for temp
- 8x GPIO for sonar
- 1 GPIO per tactile (let’s say ~8)
- 4x GPIO for moisture sensor

=====

-          Add board-specific install script to each board—the script installs package requirements, checks out tagged version of code, and builds code.
-          Add board-specific startup script to each board—the script starts all on-board software
-          Safety board maintains a node monitor
-          Safety monitor has the ability to force kill any other node—if it detects a problem, for example, it should be able to kill high-level behaviors and/or thruster control
-           Safety monitor & control has top priority in the system
-          Ambient temperature sensor to GPIO?
-   https://www.sparkfun.com/products/245
-   https://www.sparkfun.com/products/10988
-   https://www.sparkfun.com/products/11050
-          Moisture sensor to GPIO?
-   http://www.mouser.com/ProductDetail/Gravitech/I2C-HUTMP/?qs=sGAEpiMZZMvoGNntvmgYkHmFdY5RRUxdKjQNVG4q%2fToFUGWtRTp2ug%3d%3d
-          Polycarbonate fabrication: http://custom-division.com/
-          Divers
-   http://www.oasisnyc.net/stewardship/organizationdetails.aspx?id=1529
-   Urbandivers.org

=====

Early testing
If I can manage any of this, here are some early tests that don’t include some of the more difficult-to-design equipment.

- Thrusters on metal frame with imu and computer in watertight case with ethernet port, suspended in the center of the metal frame. Put this in a clear vat of water and try to make the auv suspend itself.