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

Primary SBC Required I/O
- eth
- wifi
- pwm? (for ballast?)
- 1x GPIO for temp
- 8x GPIO for sonar
- 1 GPIO per tactile (let’s say ~8)
- 4x GPIO for moisture sensor