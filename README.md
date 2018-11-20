# NewIO
New version of IO for dancing cock
## Pre-Requirement 
+ ROS is a must for IOManagerment.**BUT** ist nicht notwendig fuer ServoIO, which means you can make some change of the files then use cmake instead to build files.

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
+ Read the description function of the code in  
  + src/dmotion/include/dmotion/IO/SeroIO.h  
  + src/dmotion/include/dmotion/IO/IOManager.h
+ example show how to use ServoIO and IOManager to read and write Servo 

## Run a test
+ make changes in src/dmotion/test/test_servo.cpp
+ Then use roslaunch to test
+ make sure you have the authority of the USB port
 ``` cpp
  roslaunch dmotion ... #servo_read.launch or servo_write.launch or IOManager_read_write.launch
 ```
