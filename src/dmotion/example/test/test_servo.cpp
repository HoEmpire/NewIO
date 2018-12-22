#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
#include "../../include/dmotion/IO/IMUReader.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <pthread.h>
// #include <sched.h>
//#define PORT_NAME "/dev/ttyUSB0"
//#define BAUDRATE  1000000


using namespace std;
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "testing");
  ros::NodeHandle nh("~");
  parameters.init(&nh);
  Motion::IOManager3 io;
  std::vector<double> fucking(16);
  ROS_INFO("！！！！！！！！！！！！！FUCK！！！！！！！！！！！！！！");
  //int cunt = 0;
  double zeit = 0;
  //read constantly
  io.ServoPowerOff();
  while(ros::ok())
  {   
    timer a;
    io.readJointValue();
    zeit = a.toc();
    cout << "***********************************" << endl;
    cout << "[read time]:" << zeit << endl << endl;
    cout << "***********************************" << endl;
    sleep(1);
  }
}
