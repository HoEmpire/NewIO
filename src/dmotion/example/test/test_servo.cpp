// #include "dmotion/IO/IOManager3.h"
// #include <iostream>
// #include <ros/ros.h>
// #include "dmotion/Common/Utility/Utility.h"
// #include "dmotion/Common/Parameters.h"
// #include "../../include/dmotion/IO/IMUReader.h"
// // #include <stdio.h>
// // #include <stdlib.h>
// // #include <pthread.h>
// // #include <sched.h>
// #define PORT_NAME "/dev/IMU"
// #define BAUDRATE  576000
//
//
// using namespace std;
// int main(int argc, char ** argv)
// {
//   dynamixel::PortHandler* imu_port = dynamixel::PortHandler::getPortHandler(PORT_NAME);
//
//   if (!imu_port->setBaudRate(BAUDRATE,false))
//   {
//       ROS_FATAL("IOManager3::_initPort: could not change baudrate");
//       ROS_FATAL("IOManager3::_initPort: are you stupid enough, dass du unfaehig zu eroeffnenung des Port bist?");
//       std::abort();
//   }
//
//   uint8_t byte_buffer;
//   timer a;
//   a.tic();
//   int count;
//   while(1)
//   {
//     INFO("start reading!");
//     count = 0;
//     while(!imu_port->readPort(&byte_buffer, 1))
//     {
//       timer::delay_us(100);
//       count++;
//     }
//     INFO("read success");
//     std::cout << "read times:" << count << std::endl;
//     timer::delay_ms(2);
//     INFO("delay 2ms");
//     imu_port->clearPort();
//     INFO("clear port");
//     a.toc();
//     a.tic();
//   }
//
// }

// #include <iostream>
// #include <vector>
// #include <fstream>
// #include "dmotion/Common/Utility/Utility.h"
// #include <unistd.h>
// #define workplace "/home/tim/NewIO/test_data/test1"
//
// using namespace std;
// int main(int argc, char **argv) {
//
//   if(chdir(workplace))
//         exit(0);  //设置工作路径，也就是B显示的路径
//
//   std::vector<double> v;
//
//   v.push_back(1.2);
//   v.push_back(2.2);
//   int i = 1;
//   std::cout << int(v.size()) << std::endl;
//   std::cout << (i < int(v.size())) << std::endl;
//
//   ofstream out1;
//   out1.open("roll.txt", ios::out|ios::trunc);
//   if(!out1)
//       INFO("FUCK!!!!!!!!");
//   out1 << "fuck!!" << endl;
//   out1.close();
//
//
//
//   return 0;
// }
// #include "dmotion/IO/IOManager3.h"
// #include <iostream>
// #include <ros/ros.h>
// #include "dmotion/Common/Utility/Utility.h"
// #include "dmotion/Common/Parameters.h"
// #define PORT_NAME "/dev/Servo"
// #define BAUDRATE  1000000
// using namespace std;
// int main(int argc, char **argv) {
//   Motion::  FeetSensorIO feet;
//   while(1){
//     feet.readPressureData();
//     timer::delay_ms(1);
//   }
//
//   return 0;
// }
//
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "../../include/dmotion/IO/ServoIO.h"
#include <iostream>

using namespace std;
int main(int argc, char ** argv)
{
    Motion::ServoIO servo_test;
    INFO("FUCKTION WELL!");
    struct Motion::JointConfig _cfg4(5,-1,4096,202,4096,0);

    struct Motion::Joint joints_cfg4(_cfg4);
    servo_test.addJoint("right_knee", joints_cfg4);


    servo_test.initServoPositions();
    sleep(1);
    for(int i = 0; i < 23; i++){
       servo_test.setSingleServoPosition("right_knee", i);
       servo_test.sendServoPositions();
       timer::delay_ms(10);
    }

    for(int i = 23; i > 0; i--){
       servo_test.setSingleServoPosition("right_knee", i);
       servo_test.sendServoPositions();
       timer::delay_ms(10);
    }

    while(1){
        servo_test.setAllServoAcc(200);
        timer::delay_ms(10);
        servo_test.setAllServoSpeed(1000);
        timer::delay_ms(10);
       //servo_test.setAllServoTimeBase();
       servo_test.setSingleServoPosition("right_knee", 90);
       servo_test.sendServoPositions();
       timer::delay_ms(10000);

       servo_test.setSingleServoPosition("right_knee", 0);
       servo_test.sendServoPositions();
       timer::delay_ms(10000);
    }
}
