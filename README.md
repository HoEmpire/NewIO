# NewIO
New version of IO for dancing cock
## Pre-Requirement 
+ ROS is a must for IOManager and the other part.**BUT** ist not necessary ServoIO, which means you can make some change of the files then use cmake instead to build files.
+ but ROS is suggested 

## How to build 
+ This is a ROS workspace use the following code to build
 ``` cpp
  catkin_make
  source devel/setup.zsh
 ```
 
## Tips
+ All IO is finished 
+ Read the description function of the code in  
  + src/dmotion/include/dmotion/IO/SeroIO.h  
  + src/dmotion/include/dmotion/IO/IOManager.h
  + src/dmotion/include/dmotion/IO/FeetSensorIO.h
  + src/dmotion/include/dmotion/IO/IMUReader.h
+ example show how to use servo and iomanager and the sensors to read and write Servo 
+ if it's the first time that you used this code, make sure you have checked the Macro definition in the following files is correct(especially the port and baud rate)
  + src/dmotion/include/dmotion/IO/SeroIO.cpp
  + src/dmotion/include/dmotion/IO/IOManager.cpp
  + src/dmotion/include/dmotion/IO/FeetSensorIO.cpp
  + src/dmotion/include/dmotion/IO/IMUReader.cpp
+ **In the issue some solutions of some tricky bugs is recorded.**

## What's new
+ you can find it yourself

## Run a test 
+ There are some examples that you can use roslaunch to test
 ``` sh
  roslaunch dmotion ...    #servo_read.launch or servo_write.launch or IOManager_read_write.launch
 ```
+ make sure you have the authority of the USB port,use the following code in terminal to get authority
```sh
 sudo chmod 666 /dev/ttyUSB0     #sometimes it's not USB0
```
