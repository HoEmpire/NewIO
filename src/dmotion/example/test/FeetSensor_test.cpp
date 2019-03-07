#include "dmotion/IO/IOManager3.h"
#include <iostream>
#include <ros/ros.h>
#include "dmotion/Common/Utility/Utility.h"
#include "dmotion/Common/Parameters.h"
using namespace std;
#define workplace "/home/ubuntu/test/NewIO/test_data/test1"
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh("~");
    parameters.init(&nh);
    Motion::FeetSensorIO Feet;

    timer a;
    while(ros::ok()){
      a.tic();
      Feet.readPressureData();
      a.SmartDelayMs(10.0);
    }
}
