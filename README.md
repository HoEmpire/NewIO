# NewIO
New version of IO for dancing cock
## Pre-Requirement 
+ ROS is a must for IOManagerment.**BUT** ist nicht notwendig fuer ServoIO.

## How to build 
+ This is a ROS workspace use the following code to build
 ``` cpp
  catkin_make
  source devel/setup.zsh
 ```
 
## Tips
+ Servo part has been finished.
+ Servo Read and Write all work well after test.
+ IMU and Footsensor ist not yet erledigt(finished).
+ Read the detail function of the code in  src/dmotion/include/dmotion/IO/SeroIO.h

## Run a test
+ make changes in src/dmotion/test/test_servo.cpp
+ Then use roslaunch to test
+ make sure you have the authority of the USB port
 ``` cpp
  roslaunch dmotion test_servo.launch
 ```
